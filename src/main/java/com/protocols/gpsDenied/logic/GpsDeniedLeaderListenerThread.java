package com.protocols.gpsDenied.logic;

import com.api.API;
import com.api.ArduSim;
import com.api.GUI;
import com.api.communications.lowLevel.LowLevelCommLink;
import com.api.copter.Copter;
import com.esotericsoftware.kryo.io.Input;
import com.protocols.gpsDenied.pojo.GpsDeniedMessage;
import com.protocols.gpsDenied.pojo.PositionSample;
import com.setup.Param;
import es.upv.grc.mapper.Location2DUTM;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;


class GpsDeniedLeaderListenerThread extends Thread {

    private static final int  RECV_TIMEOUT_MS      = 300;
    /** Sliding window for counting received packets per observer.  Shorter = less lag for a
     *  moving leader (PDR reflects a shorter path) but noisier; longer = smoother but laggier. */
    private static final long PACKET_WINDOW_MS     = 10_000;  // TUNING
    private static final int  MONTE_CARLO_SAMPLES  = 1000;

    // Plausibility gate + bootstrap seed

    /** Calibration estimates aggregated (median) to seed the gate anchor. */
    private static final int    BOOTSTRAP_MIN_SAMPLES   = 20;      // TUNING
    /** Tolerated normal estimation jitter (m): the gate's fixed budget term. */
    private static final double GATE_NOISE_ENVELOPE_M   = 350.0;   // TUNING
    //private static final double GATE_NOISE_ENVELOPE_M   = 1e9;   // DESACTIVADO
    /** Multiplier on leaderSpeed for the max plausible travel between accepted fixes. */
    private static final double GATE_SPEED_MARGIN       = 1.5;     // TUNING
    /** A continuous reject streak ≥ this (ms) with clustered candidates re-anchors the gate. */
    private static final long   GATE_CONSENSUS_MS       = 1_500;   // TUNING
    /** Max spread (m) among recent rejected candidates to treat them as one consistent cluster. */
    private static final double GATE_CONSENSUS_SPREAD_M = 350.0;   // TUNING
    /** Backstop (ms): force a re-anchor if nothing is accepted for this long, ignoring spread. */
    private static final long   GATE_RESET_TIMEOUT_MS   = 8_000;   // TUNING

    //Kalman

    private static final double KF_ACCEL_NOISE  = 0.5;      // TUNING  q (m²/s⁴)
    private static final double KF_MEAS_NOISE   = 22_500.0; // TUNING  R ≈ (150 m)² per-cycle estimate variance
    private static final double KF_INIT_POS_VAR = 22_500.0; // TUNING  initial position variance (m²)
    private static final double KF_INIT_VEL_VAR = 100.0;    // TUNING  initial velocity variance (m²/s²)


    private final ArduSim arduSim;
    private final GUI gui;
    private final Copter leaderCopter;
    private final LowLevelCommLink link;
    private final Input input;

    /** Throttle log recording to at most once per BROADCAST_PERIOD_MS. */
    private long lastSampleTime = 0;

    private final Random rng = new Random();

    private Location2DUTM gateAnchor = null;
    private long gateLastAcceptMs = 0;
    private final List<Location2DUTM> bootstrapBuffer = new ArrayList<>();
    private final ArrayDeque<Location2DUTM> rejectPts   = new ArrayDeque<>();
    private final ArrayDeque<Long>          rejectTimes = new ArrayDeque<>();
    private final GpsDeniedKalmanFilter kalman =
            new GpsDeniedKalmanFilter(KF_ACCEL_NOISE, KF_MEAS_NOISE, KF_INIT_POS_VAR, KF_INIT_VEL_VAR);

    GpsDeniedLeaderListenerThread() {
        super("GpsDenied-LeaderListener");
        this.arduSim      = API.getArduSim();
        this.gui          = API.getGUI(0);
        this.leaderCopter = API.getCopter(0);
        this.link         = LowLevelCommLink.getCommLink(0);
        this.input        = new Input(new byte[LowLevelCommLink.DATAGRAM_MAX_LENGTH]);
    }

    @Override
    public void run() {
        while (!arduSim.isExperimentInProgress()) {
            arduSim.sleep(GpsDeniedParam.STATE_CHANGE_TIMEOUT);
        }
        gui.logVerboseUAV("GpsDenied leader: listener started.");
        while (!GpsDeniedParam.leaderLanded.get()) {
            byte[] data = link.receiveMessage(RECV_TIMEOUT_MS);
            if (data != null) {
                processMessage(data);
            }
            updateEstimatedPosition();
        }
        gui.logVerboseUAV("GpsDenied leader: listener stopped.");
    }

    private void processMessage(byte[] data) {
        long receiveTime = System.currentTimeMillis();
        input.setBuffer(data);
        short type = input.readShort();
        if (type != GpsDeniedMessage.OBSERVER_POSITION) return;

        int    observerNumUAV = input.readInt();
        double x              = input.readDouble();
        double y              = input.readDouble();

        GpsDeniedParam.observerLastHeard[observerNumUAV] = receiveTime;
        GpsDeniedParam.observerLastPos[observerNumUAV]   = new Location2DUTM(x, y);
        GpsDeniedParam.observerPacketTimestamps[observerNumUAV].addLast(receiveTime);

        gui.logVerboseUAV("Leader heard observer " + observerNumUAV
                + " @ (" + String.format("%.1f", x) + ", " + String.format("%.1f", y) + ")");
    }

    private void updateEstimatedPosition() {
        long now    = System.currentTimeMillis();
        int numUAVs = API.getArduSim().getNumUAVs();
        List<Location2DUTM> inRange      = new ArrayList<>();
        List<Long>          packetCounts = new ArrayList<>();
        for (int i = 1; i < numUAVs; i++) {
            long last = GpsDeniedParam.observerLastHeard[i];
            if (last > 0 && now - last <= GpsDeniedParam.OBSERVER_TIMEOUT_MS) {
                inRange.add(GpsDeniedParam.observerLastPos[i]);
                ArrayDeque<Long> ts = GpsDeniedParam.observerPacketTimestamps[i];
                while (!ts.isEmpty() && now - ts.peekFirst() > PACKET_WINDOW_MS) {
                    ts.pollFirst();
                }
                packetCounts.add((long) ts.size());
            }
        }

        Location2DUTM truePos = leaderCopter.getLocationUTM();
        int n = inRange.size();

        if (n < 3) {
            GpsDeniedParam.estimatedLeaderPosition.set(null);
            gui.updateGlobalInformation("Observadores en rango: " + n + " (mínimo 3)");
            logSample(now, null, truePos, n);
            return;
        }

        // FIXED_RANGE: intersection centroid using Param.fixedRange.
        // All other models: multilaterate with the DISTANCE_5GHZ propagation model.
        double  R;
        boolean useMultilateration;
        if (Param.selectedWirelessModel == com.api.communications.WirelessModel.FIXED_RANGE) {
            R = Param.fixedRange;
            useMultilateration = false;
        } else {
            R = GpsDeniedParam.MAX_RANGE_5GHZ_M;
            useMultilateration = true;
        }

        Location2DUTM geomEst = geometricIntersectionCenter(inRange, R);
        if (geomEst == null) {
            GpsDeniedParam.estimatedLeaderPosition.set(null);
            gui.updateGlobalInformation("Observadores en rango: " + n + " (sin intersección)");
            logSample(now, null, truePos, n);
            return;
        }

        Location2DUTM estimate = useMultilateration
                ? multilaterate(inRange, packetCounts, R, geomEst)
                : geomEst;
        if (estimate == null) {   // no usable candidate this cycle
            logSample(now, null, truePos, n);
            return;
        }

        // ── Bootstrap: seed the gate anchor with the robust median of the calibration estimates.
        if (gateAnchor == null) {
            if (useMultilateration && now - arduSim.getExperimentStartTime() < PACKET_WINDOW_MS) {
                gui.updateGlobalInformation("Observadores en rango: " + n + " (calentando ventana PDR)");
                logSample(now, null, truePos, n);
                return;
            }
            bootstrapBuffer.add(estimate);
            if (bootstrapBuffer.size() < BOOTSTRAP_MIN_SAMPLES) {
                gui.updateGlobalInformation("Observadores en rango: " + n
                        + " (sembrando ancla " + bootstrapBuffer.size() + "/" + BOOTSTRAP_MIN_SAMPLES + ")");
                logSample(now, null, truePos, n);
                return;
            }
            gateAnchor = componentwiseMedian(bootstrapBuffer);
            gateLastAcceptMs = now;
            bootstrapBuffer.clear();
            kalman.reset(gateAnchor.x, gateAnchor.y);   // seed the filter at the bootstrap median
            publishEstimate(now, filtered(), truePos, n);
            return;
        }

        // ── Plausibility gate: publish only if the candidate is within the movement budget
        double dtSec   = Math.max(0.0, (now - gateLastAcceptMs) / 1000.0);
        double maxJump = GATE_NOISE_ENVELOPE_M + GATE_SPEED_MARGIN * GpsDeniedParam.leaderSpeed * dtSec;
        double jump    = dist(estimate, gateAnchor);
        if (jump <= maxJump) {
            gateAnchor = estimate;
            rejectPts.clear();
            rejectTimes.clear();
            kalman.predict(dtSec);                  // advance the track over the elapsed gap
            kalman.update(estimate.x, estimate.y);  // fuse the accepted measurement
            gateLastAcceptMs = now;
            publishEstimate(now, filtered(), truePos, n);
            return;
        }

        // Rejected — physically impossible jump.  Hold the last valid estimate: leave
        // estimatedLeaderPosition and lastValidEstimateTimeMs untouched and log a no-estimate
        // sample, so the leader coasts on its last good fix.
        rejectPts.addLast(estimate);
        rejectTimes.addLast(now);
        while (!rejectTimes.isEmpty() && now - rejectTimes.peekFirst() > GATE_CONSENSUS_MS) {
            rejectTimes.pollFirst();
            rejectPts.pollFirst();
        }
        gui.updateGlobalInformation("Observadores en rango: " + n + " (salto implausible "
                + String.format("%.0f", jump) + " m — manteniendo última estimación)");
        logSample(now, null, truePos, n);

        // Escape from a wrong anchor/seed: re-anchor when good estimates are consistently
        // rejected.  Consensus = a reject streak ≥ GATE_CONSENSUS_MS whose recent candidates
        // cluster within GATE_CONSENSUS_SPREAD_M (so a brief outlier never re-anchors).
        // Timeout = backstop so the gate can never freeze on a bad anchor forever.
        long    rejectDurMs = now - gateLastAcceptMs;
        boolean timedOut    = rejectDurMs >= GATE_RESET_TIMEOUT_MS;
        List<Location2DUTM> recent = new ArrayList<>(rejectPts);
        Location2DUTM cluster = componentwiseMedian(recent);
        boolean consensus = rejectDurMs >= GATE_CONSENSUS_MS
                && maxSpread(recent, cluster) <= GATE_CONSENSUS_SPREAD_M;
        if (consensus || timedOut) {
            gateAnchor = cluster;
            gateLastAcceptMs = now;
            rejectPts.clear();
            rejectTimes.clear();
            kalman.reset(cluster.x, cluster.y);     // restart the filter at the new reference
            gui.logVerboseUAV("GpsDenied leader: gate re-anchored ("
                    + (timedOut ? "timeout" : "consensus") + ").");
        }
    }

    /** Appends one PositionSample to the log, throttled to BROADCAST_PERIOD_MS. */
    private void logSample(long now, Location2DUTM estimate,
                           Location2DUTM truePos, int n) {
        if (now - lastSampleTime < GpsDeniedParam.BROADCAST_PERIOD_MS) return;
        lastSampleTime = now;

        if (estimate != null) {
            GpsDeniedParam.leaderPositionLog.add(new PositionSample(
                    now, estimate.x, estimate.y, truePos.x, truePos.y, n));
        } else {
            GpsDeniedParam.leaderPositionLog.add(
                    new PositionSample(now, truePos.x, truePos.y, n));
        }
    }



    private Location2DUTM multilaterate(List<Location2DUTM> observers,
                                        List<Long> packetCounts,
                                        double R, Location2DUTM fallback) {
        int n = observers.size();
        double cMax = (double) PACKET_WINDOW_MS / GpsDeniedParam.BROADCAST_PERIOD_MS;

        // Per-observer range from inverted packet-delivery ratio.
        double[] range = new double[n];
        for (int i = 0; i < n; i++) {
            double pdr = packetCounts.get(i) / cMax;
            if (pdr > 1.0) pdr = 1.0;
            if (pdr < 0.0) pdr = 0.0;
            range[i] = rangeFromPdr(pdr);
        }

        // Linearise |P - p_i|² = r_i² by subtracting the mean equation (symmetric form,
        // no reference-anchor bias). Each observer yields: 2(x_i-x̄)X + 2(y_i-ȳ)Y = g_i - ḡ,
        // with g_i = x_i² + y_i² - r_i².  Solve the 2×2 normal equations directly.
        double mx = 0, my = 0, mg = 0;
        double[] g = new double[n];
        for (int i = 0; i < n; i++) {
            Location2DUTM o = observers.get(i);
            g[i] = o.x * o.x + o.y * o.y - range[i] * range[i];
            mx += o.x; my += o.y; mg += g[i];
        }
        mx /= n; my /= n; mg /= n;

        double Saa = 0, Sab = 0, Sbb = 0, Sac = 0, Sbc = 0;
        for (int i = 0; i < n; i++) {
            Location2DUTM o = observers.get(i);
            double a = 2.0 * (o.x - mx);
            double b = 2.0 * (o.y - my);
            double c = g[i] - mg;
            Saa += a * a; Sab += a * b; Sbb += b * b;
            Sac += a * c; Sbc += b * c;
        }

        double det = Saa * Sbb - Sab * Sab;
        if (Math.abs(det) < 1e-6) return fallback;  // collinear / degenerate geometry

        double x = (Sbb * Sac - Sab * Sbc) / det;
        double y = (Saa * Sbc - Sab * Sac) / det;
        return clampToFeasible(x, y, fallback, observers, R);
    }

    /**
     * Inverts the DISTANCE_5GHZ packet-loss model pLoss = A·d² + B·d to recover the
     * distance d (m) corresponding to a given packet-delivery ratio.
     */
    private double rangeFromPdr(double pdr) {
        double pLoss = 1.0 - pdr;
        return (-GpsDeniedParam.PROP_LOSS_B + Math.sqrt(GpsDeniedParam.PROP_LOSS_B * GpsDeniedParam.PROP_LOSS_B
                + 4.0 * GpsDeniedParam.PROP_LOSS_A * pLoss))
                / (2.0 * GpsDeniedParam.PROP_LOSS_A);
    }

    /**
     * Returns the candidate (cx, cy) if it lies inside every observer's range circle;
     * otherwise binary-searches along the segment from {@code inside} (guaranteed inside)
     * toward the candidate and returns the last point still inside the feasible region.
     */
    private Location2DUTM clampToFeasible(double cx, double cy, Location2DUTM inside,
                                          List<Location2DUTM> observers, double R) {
        if (insideAllCircles(new double[]{cx, cy}, observers, R, -1, -1)) {
            return new Location2DUTM(cx, cy);
        }
        double lo = 0.0, hi = 1.0;  // lo → inside (t=0), hi → candidate (t=1)
        for (int iter = 0; iter < 30; iter++) {
            double mid = (lo + hi) * 0.5;
            double px = inside.x + mid * (cx - inside.x);
            double py = inside.y + mid * (cy - inside.y);
            if (insideAllCircles(new double[]{px, py}, observers, R, -1, -1)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return new Location2DUTM(inside.x + lo * (cx - inside.x),
                                 inside.y + lo * (cy - inside.y));
    }

    private Location2DUTM geometricIntersectionCenter(List<Location2DUTM> observers, double R) {
        // Intersection of axis-aligned bounding boxes: tightest possible sampling domain.
        double xMin = Double.NEGATIVE_INFINITY, xMax = Double.POSITIVE_INFINITY;
        double yMin = Double.NEGATIVE_INFINITY, yMax = Double.POSITIVE_INFINITY;
        for (Location2DUTM o : observers) {
            if (o.x - R > xMin) xMin = o.x - R;
            if (o.x + R < xMax) xMax = o.x + R;
            if (o.y - R > yMin) yMin = o.y - R;
            if (o.y + R < yMax) yMax = o.y + R;
        }
        if (xMin >= xMax || yMin >= yMax) return null;

        double w = xMax - xMin, h = yMax - yMin;
        double sumX = 0, sumY = 0;
        int hits = 0;
        double[] pt = new double[2];
        for (int i = 0; i < MONTE_CARLO_SAMPLES; i++) {
            pt[0] = xMin + rng.nextDouble() * w;
            pt[1] = yMin + rng.nextDouble() * h;
            if (insideAllCircles(pt, observers, R, -1, -1)) {
                sumX += pt[0];
                sumY += pt[1];
                hits++;
            }
        }
        return (hits == 0) ? null : new Location2DUTM(sumX / hits, sumY / hits);
    }

    private boolean insideAllCircles(double[] pt,
                                     List<Location2DUTM> observers, double R,
                                     int skip1, int skip2) {
        double R2 = R * R;
        for (int k = 0; k < observers.size(); k++) {
            if (k == skip1 || k == skip2) continue;
            Location2DUTM o = observers.get(k);
            double dx = pt[0] - o.x;
            double dy = pt[1] - o.y;
            if (dx * dx + dy * dy > R2) return false;
        }
        return true;
    }


    /** Publishes an accepted estimate: updates the shared output + emergency timer, logs, refreshes GUI. */
    private void publishEstimate(long now, Location2DUTM est, Location2DUTM truePos, int n) {
        GpsDeniedParam.estimatedLeaderPosition.set(est);
        GpsDeniedParam.lastValidEstimateTimeMs.set(now);
        logSample(now, est, truePos, n);
        gui.updateGlobalInformation("Observadores en rango: " + n);
    }

    /** Current Kalman-filtered position as a UTM point. */
    private Location2DUTM filtered() {
        return new Location2DUTM(kalman.getX(), kalman.getY());
    }

    /** Component-wise median (median of x, median of y); robust to outliers (breakdown 0.5). */
    private static Location2DUTM componentwiseMedian(List<Location2DUTM> pts) {
        int m = pts.size();
        double[] xs = new double[m];
        double[] ys = new double[m];
        for (int i = 0; i < m; i++) {
            xs[i] = pts.get(i).x;
            ys[i] = pts.get(i).y;
        }
        Arrays.sort(xs);
        Arrays.sort(ys);
        return new Location2DUTM(median(xs), median(ys));
    }

    private static double median(double[] sorted) {
        int m = sorted.length;
        return (m % 2 == 1) ? sorted[m / 2] : 0.5 * (sorted[m / 2 - 1] + sorted[m / 2]);
    }

    /** Largest distance from any point in {@code pts} to {@code center}. */
    private static double maxSpread(List<Location2DUTM> pts, Location2DUTM center) {
        double max = 0.0;
        for (Location2DUTM p : pts) {
            max = Math.max(max, dist(p, center));
        }
        return max;
    }

    /** Euclidean distance between two UTM points. */
    private static double dist(Location2DUTM a, Location2DUTM b) {
        double dx = a.x - b.x, dy = a.y - b.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

}
