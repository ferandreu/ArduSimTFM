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
import java.util.List;
import java.util.Random;


class GpsDeniedLeaderListenerThread extends Thread {

    private static final int  RECV_TIMEOUT_MS      = 300;
    /** Sliding window for counting received packets per observer.  Shorter = less lag for a
     *  moving leader (PDR reflects a shorter path) but noisier; longer = smoother but laggier. */
    private static final long PACKET_WINDOW_MS     = 10_000;  // TUNING
    private static final int  MONTE_CARLO_SAMPLES  = 1000;


    private final ArduSim arduSim;
    private final GUI gui;
    private final Copter leaderCopter;
    private final LowLevelCommLink link;
    private final Input input;

    /** Throttle log recording to at most once per BROADCAST_PERIOD_MS. */
    private long lastSampleTime = 0;

    private final Random rng = new Random();

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

    // ── Message processing ───────────────────────────────────────────────

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

    // ── Position estimation ──────────────────────────────────────────────

    private void updateEstimatedPosition() {
        long now    = System.currentTimeMillis();
        int numUAVs = API.getArduSim().getNumUAVs();

        // Collect observers heard recently.
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
        GpsDeniedParam.estimatedLeaderPosition.set(estimate);
        GpsDeniedParam.lastValidEstimateTimeMs.set(System.currentTimeMillis());
        logSample(now, estimate, truePos, n);
        gui.updateGlobalInformation("Observadores en rango: " + n);
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


    // ── Model-based multilateration (DISTANCE_5GHZ) ──────────────────────

    /**
     * Estimates the leader position by inverting the propagation model: each observer's
     * packet-delivery ratio over the window gives an absolute range estimate, and the set
     * of (position, range) pairs is solved by linear least squares (range-based
     * multilateration).  The result is clamped to the feasibility region (intersection of
     * the observers' max-range circles); {@code fallback} (the geometric centroid, always
     * inside) is used both as the clamp anchor and when the system is degenerate.
     */
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

    // ── Geometric intersection helpers ───────────────────────────────────

    /**
     * Returns the area centroid of the intersection region of N equal-radius circles
     * (N ≥ 3) via Monte Carlo integration.
     *
     * A tight axis-aligned bounding box is derived from the per-circle x/y ranges and
     * used as the sampling domain, which keeps the hit-rate high regardless of observer
     * count.  MONTE_CARLO_SAMPLES points are drawn uniformly; those that lie inside
     * every circle are averaged to produce the area centroid.
     *
     * Returns null when the bounding box is empty (circles provably don't overlap) or
     * when no sampled point falls inside all circles (degenerate geometry).
     */
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

    /**
     * Returns true if point pt lies inside or on every circle in the list,
     * excluding circles at indices skip1 and skip2 (-1 to skip none).
     */
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

}
