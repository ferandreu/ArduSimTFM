package com.protocols.gpsDenied.logic;

import com.api.API;
import com.api.ArduSim;
import com.api.GUI;
import com.api.communications.WirelessModel;
import com.api.communications.lowLevel.LowLevelCommLink;
import com.api.copter.Copter;
import com.esotericsoftware.kryo.io.Input;
import com.protocols.gpsDenied.pojo.GpsDeniedMessage;
import com.protocols.gpsDenied.pojo.PositionSample;
import com.setup.Param;
import es.upv.grc.mapper.Location2DUTM;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs only on the leader drone (numUAV == 0).
 *
 * Receives OBSERVER_POSITION broadcasts and estimates the leader's position:
 *
 *   n < 2  → no estimate (null)
 *   n = 2  → midpoint of the two observers' positions (point on the chord
 *             between the two circle-intersection candidates)
 *   n ≥ 3  → geometric intersection centre: compute all pairwise circle
 *             intersection points, keep those that lie inside every other
 *             circle, return their centroid.  Falls back to simple centroid
 *             of observer positions if no inner intersection point is found
 *             (degenerate geometry).
 *
 * Each broadcast includes the sender's system timestamp so the delay can be
 * recorded for diagnostics, but delay is NOT used as a weight for positioning
 * (simulation delays reflect OS scheduling, not radio propagation distance).
 */
class GpsDeniedLeaderListenerThread extends Thread {

    private static final int RECV_TIMEOUT_MS = 300;

    private final ArduSim arduSim;
    private final GUI gui;
    private final Copter leaderCopter;
    private final LowLevelCommLink link;
    private final Input input;

    /** Throttle log recording to at most once per BROADCAST_PERIOD_MS. */
    private long lastSampleTime = 0;

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
        long   sentTimestamp  = input.readLong();

        long delay = receiveTime - sentTimestamp;
        GpsDeniedParam.observerLastHeard[observerNumUAV] = receiveTime;
        GpsDeniedParam.observerLastPos[observerNumUAV]   = new Location2DUTM(x, y);
        GpsDeniedParam.observerLastDelay[observerNumUAV] = Math.max(delay, 1L);
        GpsDeniedParam.observerPacketCount[observerNumUAV]++;

        gui.logVerboseUAV("Leader heard observer " + observerNumUAV
                + " @ (" + String.format("%.1f", x) + ", " + String.format("%.1f", y) + ")"
                + "  delay=" + delay + " ms");
    }

    // ── Position estimation ──────────────────────────────────────────────

    private void updateEstimatedPosition() {
        long now    = System.currentTimeMillis();
        int numUAVs = API.getArduSim().getNumUAVs();

        // Collect observers heard recently.
        List<Location2DUTM> inRange      = new ArrayList<>();
        List<Long>          delays       = new ArrayList<>();
        List<Long>          packetCounts = new ArrayList<>();
        for (int i = 1; i < numUAVs; i++) {
            long last = GpsDeniedParam.observerLastHeard[i];
            if (last > 0 && now - last <= GpsDeniedParam.OBSERVER_TIMEOUT_MS) {
                inRange.add(GpsDeniedParam.observerLastPos[i]);
                delays.add(GpsDeniedParam.observerLastDelay[i]);
                packetCounts.add(GpsDeniedParam.observerPacketCount[i]);
            }
        }

        Location2DUTM truePos = leaderCopter.getLocationUTM();
        int n       = inRange.size();
        int nInRange = n;   // total observers in range, used for logging

        if (n < 3) {
            GpsDeniedParam.estimatedLeaderPosition.set(null);
            gui.updateGlobalInformation("Observadores en rango: " + n + " (mínimo 3)");
            logSample(now, null, truePos, n, false);
            return;
        }

        // When more than 3 observers are in range, keep only the 3 closest to the
        // current estimated position (centroid of all observers as fallback when
        // no prior estimate exists yet).
        if (n > 3) {
            Location2DUTM ref = GpsDeniedParam.estimatedLeaderPosition.get();
            if (ref == null) {
                double sx = 0, sy = 0;
                for (Location2DUTM o : inRange) { sx += o.x; sy += o.y; }
                ref = new Location2DUTM(sx / n, sy / n);
            }
            boolean[] used = new boolean[n];
            List<Location2DUTM> sel        = new ArrayList<>(3);
            List<Long>          selDelays  = new ArrayList<>(3);
            List<Long>          selCounts  = new ArrayList<>(3);
            for (int s = 0; s < 3; s++) {
                int best = -1; double bestDist = Double.MAX_VALUE;
                for (int k = 0; k < n; k++) {
                    if (used[k]) continue;
                    double d = squaredDist(inRange.get(k), ref);
                    if (d < bestDist) { bestDist = d; best = k; }
                }
                used[best] = true;
                sel.add(inRange.get(best));
                selDelays.add(delays.get(best));
                selCounts.add(packetCounts.get(best));
            }
            inRange      = sel;
            delays       = selDelays;
            packetCounts = selCounts;
            n = 3;
        }

        double R = (Param.selectedWirelessModel == WirelessModel.FIXED_RANGE)
                ? Param.fixedRange : GpsDeniedParam.estimationRadius;

        Location2DUTM geomEst = geometricIntersectionCenter(inRange, R);
        Location2DUTM estimate;
        if (geomEst != null) {
            //estimate = applyDelayCorrection(geomEst, inRange, delays, R);
            estimate = applyPacketRateCorrection(geomEst, inRange, packetCounts, R);
        } else {
            GpsDeniedParam.estimatedLeaderPosition.set(null);
            gui.updateGlobalInformation("Observadores en rango: " + nInRange + " (sin intersección)");
            logSample(now, null, truePos, nInRange, false);
            return;
        }

        GpsDeniedParam.estimatedLeaderPosition.set(estimate);
        GpsDeniedParam.lastValidEstimateTimeMs.set(System.currentTimeMillis());
        logSample(now, estimate, truePos, nInRange, false);
        gui.updateGlobalInformation("Observadores en rango: " + nInRange);
    }

    /** Appends one PositionSample to the log, throttled to BROADCAST_PERIOD_MS. */
    private void logSample(long now, Location2DUTM estimate,
                           Location2DUTM truePos, int n, boolean inArea) {
        if (now - lastSampleTime < GpsDeniedParam.BROADCAST_PERIOD_MS) return;
        lastSampleTime = now;

        if (estimate != null) {
            GpsDeniedParam.leaderPositionLog.add(new PositionSample(
                    now, estimate.x, estimate.y, truePos.x, truePos.y, n, inArea));
        } else {
            GpsDeniedParam.leaderPositionLog.add(
                    new PositionSample(now, truePos.x, truePos.y, n, inArea));
        }
    }

    // ── Delay-based correction ───────────────────────────────────────────

    /**
     * Maximum displacement (metres) applied by the delay-based repulsion.
     * The net repulsion vector is normalised before scaling, so this value
     * directly controls how far the estimate can move from the geometric centre,
     * regardless of the absolute delay values.
     * Set to 0 to disable the correction entirely.
     */
    private static final double REPULSION_STRENGTH = 200.0;

    /**
     * Repels the geometric intersection centre away from the observers with the
     * SMALLEST delay (closest observers).
     *
     * For each observer i:
     *   - compute unit vector FROM obs_i TOWARD geomCenter (repulsion direction)
     *   - weight = 1/delay_i  (smaller delay → closer → stronger repulsion)
     *
     * Weights are NORMALISED so their sum equals 1 before scaling by
     * REPULSION_STRENGTH.  This ensures the displacement is always in the
     * range [0, REPULSION_STRENGTH] metres regardless of absolute delay values.
     *
     * If the displaced point falls outside the intersection area, a binary
     * search clamps it back to the boundary in the same direction.
     */
    private Location2DUTM applyDelayCorrection(Location2DUTM geomCenter,
                                                List<Location2DUTM> observers,
                                                List<Long> delays, double R) {
        // Raw weights and their sum (used for normalisation).
        double[] w   = new double[observers.size()];
        double   sumW = 0;
        for (int i = 0; i < observers.size(); i++) {
            w[i]  = 1.0 / delays.get(i);
            sumW += w[i];
        }

        // Normalised repulsion vector: each observer contributes its direction
        // scaled by its normalised weight (weights sum to 1).
        double repX = 0, repY = 0;
        for (int i = 0; i < observers.size(); i++) {
            Location2DUTM obs  = observers.get(i);
            double        vx   = geomCenter.x - obs.x;
            double        vy   = geomCenter.y - obs.y;
            double        dist = Math.sqrt(vx * vx + vy * vy);
            if (dist < 1e-6) continue;
            double wn = w[i] / sumW;   // normalised weight ∈ [0,1], sum = 1
            repX += wn * vx / dist;
            repY += wn * vy / dist;
        }

        double dx = REPULSION_STRENGTH * repX;
        double dy = REPULSION_STRENGTH * repY;
        double cx = geomCenter.x + dx;
        double cy = geomCenter.y + dy;

        if (insideAllCircles(new double[]{cx, cy}, observers, R, -1, -1)) {
            return new Location2DUTM(cx, cy);
        }

        // Clamp to the intersection boundary along the displacement direction.
        double lo = 0.0, hi = 1.0;
        for (int iter = 0; iter < 20; iter++) {
            double mid = (lo + hi) * 0.5;
            if (insideAllCircles(new double[]{geomCenter.x + mid * dx,
                                              geomCenter.y + mid * dy},
                                 observers, R, -1, -1)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return new Location2DUTM(geomCenter.x + lo * dx, geomCenter.y + lo * dy);
    }

    // ── Packet-rate-based correction ─────────────────────────────────────

    private Location2DUTM applyPacketRateCorrection(Location2DUTM geomCenter,
                                                     List<Location2DUTM> observers,
                                                     List<Long> packetCounts,
                                                     double R) {
        int n = observers.size();

        long maxCount = 0;
        for (long c : packetCounts) if (c > maxCount) maxCount = c;

        // weight_i: high when observer i has few packets (is far away).
        double[] w    = new double[n];
        double   sumW = 0;
        for (int i = 0; i < n; i++) {
            w[i]  = (double)(maxCount - packetCounts.get(i)) / (maxCount + 1);
            sumW += w[i];
        }

        // All observers equally sampled → no asymmetry to exploit.
        if (sumW < 1e-9) return geomCenter;

        // Repulsion direction: push away from far observers (low packet count).
        double repX = 0, repY = 0;
        for (int i = 0; i < n; i++) {
            Location2DUTM obs  = observers.get(i);
            double        vx   = geomCenter.x - obs.x;
            double        vy   = geomCenter.y - obs.y;
            double        dist = Math.sqrt(vx * vx + vy * vy);
            if (dist < 1e-6) continue;
            double wn = w[i] / sumW;
            repX += wn * vx / dist;
            repY += wn * vy / dist;
        }

        // Correction proportional to packet-count asymmetry; (repX,repY) magnitude ∈ [0,1]
        // encodes how coherently asymmetric the counts are, so the actual displacement
        // scales naturally between 0 and CORRECTION_FACTOR*R.
        // The boundary clamp below is a safety net, not the intended target.
        final double CORRECTION_FACTOR = 0.2;
        double dx = CORRECTION_FACTOR * R * repX;
        double dy = CORRECTION_FACTOR * R * repY;
        double cx = geomCenter.x + dx;
        double cy = geomCenter.y + dy;

        if (insideAllCircles(new double[]{cx, cy}, observers, R, -1, -1)) {
            return new Location2DUTM(cx, cy);
        }

        // Safety clamp to intersection boundary (only if correction pushed outside).
        double lo = 0.0, hi = 1.0;
        for (int iter = 0; iter < 20; iter++) {
            double mid = (lo + hi) * 0.5;
            if (insideAllCircles(new double[]{geomCenter.x + mid * dx,
                                              geomCenter.y + mid * dy},
                                 observers, R, -1, -1)) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return new Location2DUTM(geomCenter.x + lo * dx, geomCenter.y + lo * dy);
        
    }

    // ── Distance helpers ─────────────────────────────────────────────────

    private static double squaredDist(Location2DUTM a, Location2DUTM b) {
        double dx = a.x - b.x, dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    // ── Geometric intersection helpers ───────────────────────────────────

    /**
     * Returns the centre of the intersection region of N equal-radius circles
     * (N ≥ 3).
     *
     * For every pair (i,j) the two intersection points of circles i and j are
     * computed.  Those that lie inside ALL other circles are the vertices of
     * the curved intersection polygon.  Their centroid is returned.
     *
     * Returns null when no inner intersection point exists (degenerate case —
     * caller should fall back to centroid of observer positions).
     */
    private Location2DUTM geometricIntersectionCenter(List<Location2DUTM> observers, double R) {
        int n = observers.size();
        List<double[]> innerPoints = new ArrayList<>();

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double[] pts = circleIntersectionPoints(observers.get(i), observers.get(j), R);
                if (pts == null) continue;

                double[] p1 = {pts[0], pts[1]};
                double[] p2 = {pts[2], pts[3]};

                if (insideAllCircles(p1, observers, R, i, j)) innerPoints.add(p1);
                if (insideAllCircles(p2, observers, R, i, j)) innerPoints.add(p2);
            }
        }

        if (innerPoints.isEmpty()) return null;

        double sumX = 0, sumY = 0;
        for (double[] p : innerPoints) { sumX += p[0]; sumY += p[1]; }
        int m = innerPoints.size();
        return new Location2DUTM(sumX / m, sumY / m);
    }

    /**
     * Finds the two intersection points of two circles with equal radius R.
     *
     * Math (equal radii):
     *   d  = distance between centres
     *   a  = d/2  (foot of perpendicular from intersection point to centre-line)
     *   h  = √(R² − a²)  (half-chord length)
     *   Intersection points = midpoint ± h × perpendicular_unit_vector
     *
     * Returns {x1, y1, x2, y2} or null when circles don't intersect or are
     * concentric (d > 2R or d < ε).
     */
    private double[] circleIntersectionPoints(Location2DUTM c1, Location2DUTM c2, double R) {
        double dx = c2.x - c1.x;
        double dy = c2.y - c1.y;
        double d  = Math.sqrt(dx * dx + dy * dy);

        if (d < 1e-6 || d > 2.0 * R) return null;

        double a  = d / 2.0;
        double h  = Math.sqrt(R * R - a * a);
        double mx = (c1.x + c2.x) / 2.0;
        double my = (c1.y + c2.y) / 2.0;
        double ux = dx / d, uy = dy / d;
        double vx = -uy,    vy = ux;

        return new double[]{ mx + h * vx, my + h * vy, mx - h * vx, my - h * vy };
    }

    /**
     * Returns true if point pt lies inside or on every circle in the list,
     * excluding circles at indices skip1 and skip2.
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
