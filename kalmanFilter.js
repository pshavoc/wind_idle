class KalmanFilter {
    constructor() {
        // State: [x, y, vx, vy, wx, wy]
        // x, y: boat position
        // vx, vy: boat velocity
        // wx, wy: wind velocity
        this.x = math.matrix(math.zeros(6, 1));

        // State covariance matrix
        this.P = math.diag([1, 1, 1, 1, 10, 10]); // Initial uncertainty

        // Process noise covariance - models uncertainty in the physics model
        this.Q = math.diag([0.01, 0.01, 0.1, 0.1, 0.5, 0.5]);

        // Measurement noise covariance
        this.R = math.diag([10, 10, 0.5]); // For x, y, and angle

        this.initialized = false;
    }

    run(dt, boat, controls) {
        if (!this.initialized) {
            this.x.set([0, 0], boat.pos.x);
            this.x.set([1, 0], boat.pos.y);
            this.initialized = true;
            return;
        }

        // --- Prediction Step ---
        const F = this.getStateTransitionMatrix(dt);
        const B = this.getControlMatrix(dt, boat.angle);
        const u = math.matrix([
            [(controls.throttle / 100) * MAX_THROTTLE_FORCE / BOAT_MASS]
        ]);

        // Predict state: x_pred = F * x + B * u
        const x_pred = math.add(math.multiply(F, this.x), math.multiply(B, u));
        
        // Predict covariance: P_pred = F * P * F' + Q
        const P_pred = math.add(math.multiply(F, this.P, math.transpose(F)), this.Q);

        // --- Measurement ---
        // Simulate noisy GPS and compass reading
        const GPS_NOISE = 0.5; // Standard deviation for GPS noise
        const ANGLE_NOISE = 0.01; // Standard deviation for angle noise

        const noisy_pos_x = boat.pos.x + gaussianRandom() * GPS_NOISE;
        const noisy_pos_y = boat.pos.y + gaussianRandom() * GPS_NOISE;
        const noisy_angle = boat.angle + gaussianRandom() * ANGLE_NOISE;
        const z = math.matrix([[noisy_pos_x], [noisy_pos_y], [noisy_angle]]);

        // --- Update Step (EKF Style for non-linear measurement) ---
        const H = this.getMeasurementJacobian(x_pred);
        const h_x = this.getMeasurementPrediction(x_pred);
        const y = math.subtract(z, h_x); // Innovation

        // Normalize angle error to be within [-PI, PI]
        let angle_error = y.get([2, 0]);
        while (angle_error > Math.PI) angle_error -= 2 * Math.PI;
        while (angle_error < -Math.PI) angle_error += 2 * Math.PI;
        y.set([2, 0], angle_error);

        const S = math.add(math.multiply(H, P_pred, math.transpose(H)), this.R); // Innovation covariance
        const K = math.multiply(P_pred, math.transpose(H), math.inv(S)); // Kalman gain

        // Update state estimate
        this.x = math.add(x_pred, math.multiply(K, y));

        // Update covariance estimate
        const I = math.identity(6);
        this.P = math.multiply(math.subtract(I, math.multiply(K, H)), P_pred);
    }

    getStateTransitionMatrix(dt) {
        // Defines how state evolves without controls or noise.
        // x_new = x + vx*dt, v_new = v, w_new = w (random walk)
        return math.matrix([
            [1, 0, dt, 0, 0, 0],
            [0, 1, 0, dt, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ]);
    }

    getControlMatrix(dt, angle) {
        // Maps control input (engine acceleration) to the state.
        const cosA = Math.cos(angle);
        const sinA = Math.sin(angle);
        return math.matrix([
            [0.5 * cosA * dt * dt], // Effect on x
            [0.5 * sinA * dt * dt], // Effect on y
            [cosA * dt],           // Effect on vx
            [sinA * dt],           // Effect on vy
            [0],
            [0]
        ]);
    }

    getMeasurementPrediction(x) {
        const vx = x.get([2, 0]);
        const vy = x.get([3, 0]);
        const wx = x.get([4, 0]);
        const wy = x.get([5, 0]);

        // Model: boat heading aligns with apparent wind direction. A simplification.
        const apparent_wind_x = wx - vx;
        const apparent_wind_y = wy - vy;
        const predicted_angle = Math.atan2(apparent_wind_y, apparent_wind_x);

        return math.matrix([
            [x.get([0, 0])],
            [x.get([1, 0])],
            [predicted_angle]
        ]);
    }

    getMeasurementJacobian(x) {
        const vx = x.get([2, 0]);
        const vy = x.get([3, 0]);
        const wx = x.get([4, 0]);
        const wy = x.get([5, 0]);

        const apparent_wind_x = wx - vx;
        const apparent_wind_y = wy - vy;
        const len_sq = apparent_wind_x * apparent_wind_x + apparent_wind_y * apparent_wind_y;

        if (len_sq < 0.001) { // Avoid division by zero
            return math.matrix([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0] // No info from angle if no apparent wind
            ]);
        }

        const d_angle_dvx = apparent_wind_y / len_sq;
        const d_angle_dvy = -apparent_wind_x / len_sq;
        const d_angle_dwx = -apparent_wind_y / len_sq;
        const d_angle_dwy = apparent_wind_x / len_sq;

        return math.matrix([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, d_angle_dvx, d_angle_dvy, d_angle_dwx, d_angle_dwy]
        ]);
    }

    getWindVelocityEstimate() {
        return new Vector2D(this.x.get([4, 0]), this.x.get([5, 0]));
    }
}
