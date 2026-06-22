package com.protocols.gpsDenied.logic;

import java.util.Arrays;


class GpsDeniedKalmanFilter {

    private final double accelNoise;   // q : acceleration-noise spectral density (m²/s⁴)
    private final double measNoise;    // r : position-measurement variance (m²)
    private final double initPosVar;   // initial position variance on reset (m²)
    private final double initVelVar;   // initial velocity variance on reset (m²/s²)

    private final double[]   x = new double[4];     // state [px, py, vx, vy]
    private final double[][] P = new double[4][4];  // state covariance
    private boolean initialized = false;

    GpsDeniedKalmanFilter(double accelNoise, double measNoise,
                          double initPosVar, double initVelVar) {
        this.accelNoise = accelNoise;
        this.measNoise  = measNoise;
        this.initPosVar = initPosVar;
        this.initVelVar = initVelVar;
    }

    boolean isInitialized() { return initialized; }

    /** Seeds the filter at (px, py) with zero velocity and a fresh diagonal covariance. */
    void reset(double px, double py) {
        x[0] = px; x[1] = py; x[2] = 0.0; x[3] = 0.0;
        for (double[] row : P) Arrays.fill(row, 0.0);
        P[0][0] = initPosVar; P[1][1] = initPosVar;
        P[2][2] = initVelVar; P[3][3] = initVelVar;
        initialized = true;
    }

    /** Time update over {@code dt} seconds (constant-velocity prediction). */
    void predict(double dt) {
        if (!initialized || dt <= 0.0) return;
        // x = F·x   (F advances position by velocity·dt; velocity unchanged)
        x[0] += x[2] * dt;
        x[1] += x[3] * dt;
        // P = F·P·Fᵀ + Q
        double[][] F = {
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };
        double[][] FPFt = mul(mul(F, P), transpose(F));
        addInPlace(FPFt, processNoise(dt));
        copyInto(FPFt, P);
    }

    /** Measurement update with a position observation z = (zx, zy). */
    void update(double zx, double zy) {
        if (!initialized) { reset(zx, zy); return; }
        // Innovation y = z − H·x   (H selects px, py)
        double y0 = zx - x[0];
        double y1 = zy - x[1];
        // S = H·P·Hᵀ + R   (top-left 2×2 block of P plus measurement noise)
        double s00 = P[0][0] + measNoise, s01 = P[0][1];
        double s10 = P[1][0],             s11 = P[1][1] + measNoise;
        double det = s00 * s11 - s01 * s10;
        if (Math.abs(det) < 1e-9) return;   // singular innovation covariance; skip update
        // S⁻¹
        double i00 =  s11 / det, i01 = -s01 / det;
        double i10 = -s10 / det, i11 =  s00 / det;
        // K = P·Hᵀ·S⁻¹   (P·Hᵀ is the first two columns of P; K is 4×2)
        double[][] K = new double[4][2];
        for (int r = 0; r < 4; r++) {
            double ph0 = P[r][0];   // (P·Hᵀ)[r][0]
            double ph1 = P[r][1];   // (P·Hᵀ)[r][1]
            K[r][0] = ph0 * i00 + ph1 * i10;
            K[r][1] = ph0 * i01 + ph1 * i11;
        }
        // x = x + K·y
        for (int r = 0; r < 4; r++) {
            x[r] += K[r][0] * y0 + K[r][1] * y1;
        }
        // P = (I − K·H)·P   (K·H is 4×4 with columns 0,1 = K, the rest zero)
        double[][] ImKH = new double[4][4];
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                ImKH[r][c] = (r == c ? 1.0 : 0.0);
            }
            ImKH[r][0] -= K[r][0];
            ImKH[r][1] -= K[r][1];
        }
        copyInto(mul(ImKH, P), P);
    }

    double getX()  { return x[0]; }
    double getY()  { return x[1]; }
    double getVx() { return x[2]; }
    double getVy() { return x[3]; }

    /** DWNA process-noise covariance Q for a step of {@code dt} seconds. */
    private double[][] processNoise(double dt) {
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double pp = accelNoise * dt4 / 4.0;   // position–position
        double pv = accelNoise * dt3 / 2.0;   // position–velocity
        double vv = accelNoise * dt2;         // velocity–velocity
        return new double[][] {
            {pp, 0,  pv, 0},
            {0,  pp, 0,  pv},
            {pv, 0,  vv, 0},
            {0,  pv, 0,  vv}
        };
    }

    // ── small dense-matrix helpers (sizes are fixed and tiny) ────────────
    private static double[][] mul(double[][] A, double[][] B) {
        int n = A.length, m = B[0].length, k = B.length;
        double[][] C = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                double s = 0.0;
                for (int t = 0; t < k; t++) s += A[i][t] * B[t][j];
                C[i][j] = s;
            }
        }
        return C;
    }

    private static double[][] transpose(double[][] A) {
        int n = A.length, m = A[0].length;
        double[][] T = new double[m][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) T[j][i] = A[i][j];
        }
        return T;
    }

    private static void addInPlace(double[][] A, double[][] B) {
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) A[i][j] += B[i][j];
        }
    }

    private static void copyInto(double[][] src, double[][] dst) {
        for (int i = 0; i < src.length; i++) {
            System.arraycopy(src[i], 0, dst[i], 0, src[i].length);
        }
    }
}
