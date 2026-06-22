package com.protocols.gpsDenied.logic;

import com.protocols.gpsDenied.pojo.PositionSample;
import es.upv.grc.mapper.DrawableCircleGeo;
import es.upv.grc.mapper.Location2DUTM;
import es.upv.grc.mapper.Location3DUTM;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class GpsDeniedParam {

    public static final String PROTOCOL_TEXT = "GpsDenied";

    // ── Configurable parameters (set via GUI / .properties) ─────────────
    /** Starting latitude of the leader (degrees). Also the geometric reference for observers. */
    public static volatile double centerLatitude  = 39.482594;
    /** Starting longitude of the leader (degrees). */
    public static volatile double centerLongitude = -0.346265;
    /** Heading direction the leader flies, in DEGREES in the GUI; stored in RADIANS internally.
     *  0 = North, 90 = East (clockwise). */
    public static volatile double centerYaw = 0.0;  // radians

    /** (X) Distance the leader flies in a straight line (metres). */
    public static volatile double leaderFlightDistance = 2000.0;
    /** (Y) Perpendicular distance of observer drones from the leader's path (metres). */
    public static volatile double observerSideDistance = 500.0;
    /** Distance along the heading to the optional intermediate waypoint (metres). 0 = disabled. */
    public static volatile double midpointAlong = 0.0;
    /** Perpendicular offset of the intermediate waypoint (metres, positive = right, negative = left). */
    public static volatile double midpointRight = 0.0;
    /** Cruise altitude for all UAVs (metres, relative to takeoff point). */
    public static volatile double altitude = 20.0;
    /** Leader cruise speed (m/s). */
    public static volatile double leaderSpeed = 10.0;
    // ── DISTANCE_5GHZ propagation model constants ────────────────────────
    // Packet-loss probability vs distance d (m): pLoss = A·d² + B·d
    // (must match RangeCalculusThread.isInRange).
    public static final double PROP_LOSS_A = 5.335e-7;   // 1/m²
    public static final double PROP_LOSS_B = 3.395e-5;   // 1/m
    /** Distance where pLoss = 1: the model's true maximum range (~1370 m). */
    public static final double MAX_RANGE_5GHZ_M =
            (-PROP_LOSS_B + Math.sqrt(PROP_LOSS_B * PROP_LOSS_B + 4.0 * PROP_LOSS_A)) / (2.0 * PROP_LOSS_A);

    // ── Timing (milliseconds) ────────────────────────────────────────────
    public static final long STATE_CHANGE_TIMEOUT = 250;
    public static final long HOVER_CHECK_PERIOD   = 500;
    /** How often each observer broadcasts its position (ms). */
    public static final long BROADCAST_PERIOD_MS  = 250;
    /** An observer is considered out of range if not heard for this long (ms). */
    public static final long OBSERVER_TIMEOUT_MS  = 750;


    public static Location3DUTM[] flyingTargets;

    public static volatile Location3DUTM intermediateTarget = null;

    /** UTM position of the leader's takeoff point (route origin). Computed in setStartingLocation(). */
    public static volatile Location2DUTM leaderStartUTM = null;

    /** Set to true by the leader thread once it has landed; observers poll this. */
    public static final AtomicBoolean leaderLanded = new AtomicBoolean(false);

    /** Per-UAV generic state counter (available for future extensions). */
    public static AtomicInteger[] state;

    public static long[] observerLastHeard;

    public static Location2DUTM[] observerLastPos;

    @SuppressWarnings("unchecked")
    public static ArrayDeque<Long>[] observerPacketTimestamps = new ArrayDeque[0];

    /**
     * Leader's estimated UTM position.  Null when no observers are in range.
     */
    public static final AtomicReference<Location2DUTM> estimatedLeaderPosition =
            new AtomicReference<>(null);

    public static final AtomicLong lastValidEstimateTimeMs = new AtomicLong(0);

    public static List<PositionSample> leaderPositionLog;

    public static DrawableCircleGeo[] observerRangeCircles;
}
