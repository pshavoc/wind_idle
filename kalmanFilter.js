// --- Kalman Filter Logic (Refactored with math.js) ---
function runKalmanFilter(dt) {
    const I6 = math.identity(6);

    // Note: kf, boat, controls, WATER_DRAG_COEFFICIENT, WIND_FORCE_COEFFICIENT, 
    // MAX_THROTTLE_FORCE, GPS_NOISE, gaussianRandom, and math are expected to be global.

    // --- 1. PREDICT ---
    // State transition matrix F
    const F = math.matrix([
        [1, 0, dt, 0, 0, 0],
        [0, 1, 0, dt, 0, 0],
        [0, 0, 1 - WATER_DRAG_COEFFICIENT * dt, 0, WIND_FORCE_COEFFICIENT * dt, 0],
        [0, 0, 0, 1 - WATER_DRAG_COEFFICIENT * dt, 0, WIND_FORCE_COEFFICIENT * dt],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]
    ]);
    
    // Control input model B_u
    const engineForce = (controls.throttle / 100) * MAX_THROTTLE_FORCE;
    const B_u = math.matrix([
        0, 0,
        (Math.cos(boat.angle) * engineForce * dt),
        (Math.sin(boat.angle) * engineForce * dt),
        0, 0
    ]);

    // Predict state: x_pred = F * x + B*u
    const x_pred = math.add(math.multiply(F, kf.x), B_u);

    // Predict error covariance: P_pred = F * P * F_T + Q
    const P_pred = math.add(math.multiply(math.multiply(F, kf.P), math.transpose(F)), kf.Q);
    
    // --- 2. UPDATE ---
    // Measurement z (noisy GPS position)
    const z = math.matrix([boat.x + gaussianRandom() * GPS_NOISE, boat.y + gaussianRandom() * GPS_NOISE]);
    
    // Innovation: y = z - H * x_pred
    const y = math.subtract(z, math.multiply(kf.H, x_pred));
    
    // Innovation covariance: S = H * P_pred * H_T + R
    const S = math.add(math.multiply(math.multiply(kf.H, P_pred), math.transpose(kf.H)), kf.R);
    
    // Kalman gain: K = P_pred * H_T * S^-1
    const K = math.multiply(math.multiply(P_pred, math.transpose(kf.H)), math.inv(S));

    // Update state estimate: x = x_pred + K * y
    kf.x = math.add(x_pred, math.multiply(K, y));
    
    // Update error covariance: P = (I - K * H) * P_pred
    const I_KH = math.subtract(I6, math.multiply(K, kf.H));
    kf.P = math.multiply(I_KH, P_pred);
}
