package com.protocols.gpsDenied.logic;

import com.api.API;
import com.api.ArduSim;
import com.api.GUI;
import com.api.copter.Copter;
import es.upv.grc.mapper.Location2DUTM;
import es.upv.grc.mapper.Location3DUTM;

class GpsDeniedDroneThread extends Thread {

    private static final int    LOOP_MS               = 200;
    private static final long   CALIBRATION_MS        = 30_000;  // hover at start to collect observer packets
    private static final long   EMERGENCY_MS          = 70_000;  // abort after 70 s with no estimate at all
    /** Along-track distance-to-go (m) below which the drone counts as "near" the waypoint.
     *  Measured by projecting the estimate onto the planned segment axis, so lateral
     *  estimation error does not prevent arrival. */
    private static final double ARRIVAL_THRESHOLD_M   = 250.0;   // TUNING
    /** Consecutive near (smoothed) samples required to confirm arrival by proximity. */
    private static final int    ARRIVAL_SAMPLES       = 6;       // TUNING
    /** EMA factor for the estimate used ONLY in arrival detection (not steering):
     *  higher = more responsive, lower = smoother.  Filters spikes that reset the counter. */
    private static final double ESTIMATE_EMA_ALPHA    = 0.30;    // TUNING
    /** Min fraction of a segment's nominal flight time (active flight only) before arrival
     *  may be declared — prevents early arrival from estimation bias near the segment start. */
    private static final double MIN_ARRIVAL_FRACTION  = 0.9;    // TUNING
    /** Max fraction of a segment's nominal flight time after which arrival is forced even if
     *  proximity never confirms — prevents flying past a waypoint forever on a bad estimate. */
    private static final double MAX_ARRIVAL_FRACTION  = 1.20;    // TUNING

    private final int     numUAV;
    private final ArduSim arduSim;
    private final GUI     gui;
    private final Copter  copter;

    private long activeFlightMs = 0;

    GpsDeniedDroneThread(int numUAV) {
        this.numUAV  = numUAV;
        this.arduSim = new ArduSim();
        this.gui     = API.getGUI(numUAV);
        this.copter  = API.getCopter(numUAV);
    }

    @Override
    public void run() {
        gui.updateProtocolState(GpsDeniedText.START);

        // Drones are already airborne (setupActionPerformed took them off).
        // Wait for the Start button.
        while (!arduSim.isExperimentInProgress()) {
            arduSim.sleep(GpsDeniedParam.STATE_CHANGE_TIMEOUT);
        }

        if (numUAV == 0) {
            flyToTarget();
            GpsDeniedParam.leaderLanded.set(true);   // signal observers
        } else {
            hoverUntilLeaderDone();
        }

        land();
    }


    private void flyToTarget() {
        gui.updateProtocolState(GpsDeniedText.CALIBRATING);
        copter.setPlannedSpeed(GpsDeniedParam.leaderSpeed);

        // Hover for CALIBRATION_MS so the listener thread accumulates packets
        // before the leader starts moving.
        gui.logUAV("GpsDenied leader: calibrating for " + (CALIBRATION_MS / 1000) + " s.");
        long calibrationEnd = System.currentTimeMillis() + CALIBRATION_MS;
        while (System.currentTimeMillis() < calibrationEnd) {
            copter.moveTo(0, 0, 0);
            arduSim.sleep(LOOP_MS);
        }
        gui.logUAV("GpsDenied leader: calibration done — starting route.");

        gui.updateProtocolState(GpsDeniedText.FLYING);

        GpsDeniedParam.lastValidEstimateTimeMs.set(System.currentTimeMillis());

        Location2DUTM nominalLeaderStart = GpsDeniedParam.leaderStartUTM;
        double        msPerMeter         = 1000.0 / GpsDeniedParam.leaderSpeed;

        double minMsIntermediate = -1, maxMsIntermediate = -1;
        Location2DUTM nominalMid = null;
        double minMsFinal, maxMsFinal;

        if (GpsDeniedParam.intermediateTarget != null) {
            double d1 = dist2D(nominalLeaderStart, GpsDeniedParam.intermediateTarget);
            minMsIntermediate = d1 * MIN_ARRIVAL_FRACTION * msPerMeter;
            maxMsIntermediate = d1 * MAX_ARRIVAL_FRACTION * msPerMeter;
            nominalMid = new Location2DUTM(GpsDeniedParam.intermediateTarget.x,
                                            GpsDeniedParam.intermediateTarget.y);
            double d2 = dist2D(nominalMid, GpsDeniedParam.flyingTargets[0]);
            minMsFinal = minMsIntermediate + d2 * MIN_ARRIVAL_FRACTION * msPerMeter;
            maxMsFinal = maxMsIntermediate + d2 * MAX_ARRIVAL_FRACTION * msPerMeter;
        } else {
            double d = dist2D(nominalLeaderStart, GpsDeniedParam.flyingTargets[0]);
            minMsFinal = d * MIN_ARRIVAL_FRACTION * msPerMeter;
            maxMsFinal = d * MAX_ARRIVAL_FRACTION * msPerMeter;
        }

        boolean ok = true;
        if (GpsDeniedParam.intermediateTarget != null) {
            ok = flyToWaypoint(GpsDeniedParam.intermediateTarget, "intermediate waypoint",
                               nominalLeaderStart, minMsIntermediate, maxMsIntermediate);
        }
        if (ok) {
            Location2DUTM nomStart = (nominalMid != null) ? nominalMid : nominalLeaderStart;
            flyToWaypoint(GpsDeniedParam.flyingTargets[0], "final target", nomStart, minMsFinal, maxMsFinal);
        }

        copter.moveTo(0, 0, 0);
        gui.logUAV("GpsDenied leader: route completed.");
        gui.updateProtocolState(GpsDeniedText.FINISHED);
    }


    private boolean flyToWaypoint(Location3DUTM target, String label,
                                   Location2DUTM nominalStart, double minFlightMs, double maxFlightMs) {
        // Planned segment axis (config geometry, noise-free).
        double segDx   = target.x - nominalStart.x;
        double segDy   = target.y - nominalStart.y;
        double segDist = Math.sqrt(segDx * segDx + segDy * segDy);
        double ux = (segDist > 1.0) ? segDx / segDist : 0.0;
        double uy = (segDist > 1.0) ? segDy / segDist : 0.0;

        int     consecutiveNear = 0;
        boolean smoothInit      = false;
        double  smX = 0, smY = 0;   // EMA of the estimate, used only for detection

        gui.logUAV(String.format(
                "GpsDenied leader: flying to %s (active flight window: %.0f–%.0f ms).",
                label, minFlightMs, maxFlightMs));

        while (true) {
            Location2DUTM estimated = GpsDeniedParam.estimatedLeaderPosition.get();

            if (estimated == null) {
                long noEstimateMs = System.currentTimeMillis()
                        - GpsDeniedParam.lastValidEstimateTimeMs.get();
                if (noEstimateMs >= EMERGENCY_MS) {
                    gui.logUAV("GpsDenied leader: EMERGENCY STOP — no estimate for >"
                            + (EMERGENCY_MS / 1000) + " s.");
                    return false;
                }
                arduSim.sleep(LOOP_MS);
                continue;
            }

            // Smooth the estimate for detection only (filters spikes that reset the counter).
            if (!smoothInit) {
                smX = estimated.x; smY = estimated.y; smoothInit = true;
            } else {
                smX += ESTIMATE_EMA_ALPHA * (estimated.x - smX);
                smY += ESTIMATE_EMA_ALPHA * (estimated.y - smY);
            }

            // Along-track distance-to-go on the planned axis (robust to lateral error).
            double progress  = (smX - nominalStart.x) * ux + (smY - nominalStart.y) * uy;
            double remaining = segDist - progress;   // < 0 means overshoot past the target
            boolean near     = remaining <= ARRIVAL_THRESHOLD_M;

            if (activeFlightMs >= maxFlightMs) {
                gui.logUAV(String.format(
                        "GpsDenied leader: %s reached (time ceiling, active=%.0f/%.0f ms, to-go=%.0f m).",
                        label, (double) activeFlightMs, maxFlightMs, remaining));
                break;
            }
            if (activeFlightMs >= minFlightMs && near) {
                if (++consecutiveNear >= ARRIVAL_SAMPLES) {
                    gui.logUAV(String.format(
                            "GpsDenied leader: %s reached (proximity, active=%.0f ms, to-go=%.0f m).",
                            label, (double) activeFlightMs, remaining));
                    break;
                }
            } else {
                consecutiveNear = 0;
            }

            // Steer using the raw estimate; keep the easting/northing swap (intentional frame map).
            double dx   = target.x - estimated.x;
            double dy   = target.y - estimated.y;
            double dist = Math.sqrt(dx * dx + dy * dy);
            if (dist > 1e-6) {
                double vNorth = (dy / dist) * GpsDeniedParam.leaderSpeed;
                double vEast  = (dx / dist) * GpsDeniedParam.leaderSpeed;
                copter.moveTo(vNorth, vEast, 0.0);
            }
            activeFlightMs += LOOP_MS;   // advances only while moving (estimate non-null)
            arduSim.sleep(LOOP_MS);
        }
        return true;
    }

    private static double dist2D(Location2DUTM a, Location3DUTM b) {
        double dx = b.x - a.x, dy = b.y - a.y;
        return Math.sqrt(dx * dx + dy * dy);
    }


    private void hoverUntilLeaderDone() {
        gui.updateProtocolState(GpsDeniedText.HOVERING);
        while (!GpsDeniedParam.leaderLanded.get()) {
            arduSim.sleep(GpsDeniedParam.HOVER_CHECK_PERIOD);
        }
    }


    private void land() {
        gui.updateProtocolState(GpsDeniedText.LANDING);
        copter.land();
        while (copter.isFlying()) {
            arduSim.sleep(500);
        }
        gui.updateProtocolState(GpsDeniedText.FINISHED);
    }
}
