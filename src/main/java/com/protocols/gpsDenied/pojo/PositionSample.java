package com.protocols.gpsDenied.pojo;

/**
 * One snapshot of the leader's estimated vs. true position.
 * Recorded by GpsDeniedLeaderListenerThread each time the estimate is refreshed.
 *
 * When no estimate is available (estX/estY are NaN), the sample still records
 * the true position and the intersection-area flag so the CSV is complete.
 */
public class PositionSample {

    /** Wall-clock time of this sample (System.currentTimeMillis()). */
    public final long timeMs;

    /** Estimated UTM position. NaN when no estimate could be computed. */
    public final double estX;
    public final double estY;

    /** True GPS position of the leader at the same instant. */
    public final double trueX;
    public final double trueY;

    /** 2-D horizontal error between estimate and truth (metres). NaN when no estimate. */
    public final double error2D;

    /** Number of observers whose last broadcast arrived within OBSERVER_TIMEOUT_MS. */
    public final int observersInRange;

    /**
     * True when the leader's true position lies inside every in-range observer's
     * coverage circle (radius = observerSideDistance + 75 m for 802.11a, or
     * fixedRange for FIXED_RANGE model).
     */;

    /** Sample with a valid position estimate. */
    public PositionSample(long timeMs,
                          double estX, double estY,
                          double trueX, double trueY,
                          int observersInRange) {
        this.timeMs             = timeMs;
        this.estX               = estX;
        this.estY               = estY;
        this.trueX              = trueX;
        this.trueY              = trueY;
        double dx = estX - trueX;
        double dy = estY - trueY;
        this.error2D            = Math.sqrt(dx * dx + dy * dy);
        this.observersInRange   = observersInRange;
    }

    /** Sample without a position estimate — true position and area flag only. */
    public PositionSample(long timeMs,
                          double trueX, double trueY,
                          int observersInRange) {
        this.timeMs             = timeMs;
        this.estX               = Double.NaN;
        this.estY               = Double.NaN;
        this.trueX              = trueX;
        this.trueY              = trueY;
        this.error2D            = Double.NaN;
        this.observersInRange   = observersInRange;
    }

    public boolean hasEstimate() {
        return !Double.isNaN(estX);
    }
}
