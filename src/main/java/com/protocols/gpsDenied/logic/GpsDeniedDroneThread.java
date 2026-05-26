package com.protocols.gpsDenied.logic;

import com.api.API;
import com.api.ArduSim;
import com.api.GUI;
import com.api.copter.Copter;
import es.upv.grc.mapper.Location2DUTM;
import es.upv.grc.mapper.Location3DUTM;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * One thread per UAV.  Takeoff has already completed in setupActionPerformed()
 * so drones are in Guided_armed state when isExperimentInProgress() becomes true.
 *
 * Leader (numUAV == 0):
 *   Sends velocity commands every 200 ms toward the pre-computed target —
 *   same mechanism as MagneticsAvoidance.moveUAV() / copter.moveTo(vx,vy,vz).
 *   When within 2 m of the target, stops, signals observers, and lands.
 *
 * Observer (numUAV >= 1):
 *   Hovers (GUIDED position hold — no velocity commands needed) until the
 *   leader sets leaderLanded, then lands.
 */
class GpsDeniedDroneThread extends Thread {

    private static final int    LOOP_MS          = 200;
    private static final long   NULL_STOP_MS     = 2_000;   // hover after 2 s without estimate
    private static final long   EMERGENCY_MS     = 30_000;  // abort after 30 s without estimate
    private static final double ARRIVAL_CHECK_M  = 100.0;   // stop if averaged estimate is within this distance of target
    private static final int    ARRIVAL_AVG_COUNT = 3;     // number of recent estimates to average for arrival check

    private final int     numUAV;
    private final ArduSim arduSim;
    private final GUI     gui;
    private final Copter  copter;

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

    // ── Leader: velocity loop ─────────────────────────────────────────────

    private void flyToTarget() {
        gui.updateProtocolState(GpsDeniedText.FLYING);
        copter.setPlannedSpeed(GpsDeniedParam.leaderSpeed);

        Location3DUTM        target          = GpsDeniedParam.flyingTargets[0];
        Deque<Location2DUTM> recentEstimates = new ArrayDeque<>(ARRIVAL_AVG_COUNT);

        // Seed timer so the first null period is measured from flight start.
        GpsDeniedParam.lastValidEstimateTimeMs.set(System.currentTimeMillis());

        while (true) {
            Location2DUTM estimated = GpsDeniedParam.estimatedLeaderPosition.get();

            if (estimated == null) {
                long noEstimateMs = System.currentTimeMillis()
                        - GpsDeniedParam.lastValidEstimateTimeMs.get();

                if (noEstimateMs >= EMERGENCY_MS) {
                    gui.logUAV("GpsDenied leader: EMERGENCY STOP — no estimate for >"
                            + (EMERGENCY_MS / 1000) + " s.");
                    break;
                } else if (noEstimateMs >= NULL_STOP_MS) {
                    // Hover — no estimate available.
                    copter.moveTo(0, 0, 0);
                }
                arduSim.sleep(LOOP_MS);
                continue;
            }

            // Update rolling buffer.
            if (recentEstimates.size() == ARRIVAL_AVG_COUNT) recentEstimates.poll();
            recentEstimates.add(estimated);

            // Arrival check: average of the last ARRIVAL_AVG_COUNT estimates within ARRIVAL_CHECK_M.
            if (recentEstimates.size() == ARRIVAL_AVG_COUNT && targetReachedByPosition(target, recentEstimates)){
                gui.logUAV("GpsDenied leader: target reached based on average of recent estimates positions.");
                break;
            }

            // Fin basado en posicion real -> necesario eliminar
            if (targetReachedByPosition(target, copter.getLocationUTM())) {
                gui.logUAV("GpsDenied leader: target reached based on real position.");
                break;
            }
            double dx   = target.x - estimated.x;
            double dy   = target.y - estimated.y;
            double dist = Math.sqrt(dx * dx + dy * dy);

            if (dist < 0.1) break;  // safety: avoid division by near-zero

            copter.moveTo((dy / dist) * GpsDeniedParam.leaderSpeed,
                          (dx / dist) * GpsDeniedParam.leaderSpeed, 0.0);
            arduSim.sleep(LOOP_MS);
        }

        copter.moveTo(0, 0, 0);
        gui.logUAV("GpsDenied leader: target reached.");
        gui.updateProtocolState(GpsDeniedText.FINISHED);
    }

    /** Returns true when the average of the last ARRIVAL_AVG_COUNT estimates is within ARRIVAL_CHECK_M of the target. */
    private boolean targetReachedByPosition(Location3DUTM target, Deque<Location2DUTM> recentEstimates) {
        double sumX = 0, sumY = 0;
        for (Location2DUTM e : recentEstimates) { sumX += e.x; sumY += e.y; }
        double avgX = sumX / recentEstimates.size();
        double avgY = sumY / recentEstimates.size();
        double dx = target.x - avgX;
        double dy = target.y - avgY;
        return Math.sqrt(dx * dx + dy * dy) <= ARRIVAL_CHECK_M;
    }

    /** Returns true when a single position is within ARRIVAL_CHECK_M of the target. */
    private boolean targetReachedByPosition(Location3DUTM target, Location2DUTM pos) {
        double dx = target.x - pos.x;
        double dy = target.y - pos.y;
        return Math.sqrt(dx * dx + dy * dy) <= ARRIVAL_CHECK_M;
    }

    // ── Observer: hover until leader arrives ─────────────────────────────

    private void hoverUntilLeaderDone() {
        gui.updateProtocolState(GpsDeniedText.HOVERING);
        while (!GpsDeniedParam.leaderLanded.get()) {
            arduSim.sleep(GpsDeniedParam.HOVER_CHECK_PERIOD);
        }
    }

    // ── Land ─────────────────────────────────────────────────────────────

    private void land() {
        gui.updateProtocolState(GpsDeniedText.LANDING);
        copter.land();
        while (copter.isFlying()) {
            arduSim.sleep(500);
        }
        gui.updateProtocolState(GpsDeniedText.FINISHED);
    }
}
