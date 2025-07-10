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
        const B = this.getControlMatrix(dt, boat);
        const u = math.matrix([
            [controls.throttle]
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

        const vx = this.x.get([2, 0]);
        const vy = this.x.get([3, 0]);
        const wx = this.x.get([4, 0]);
        const wy = this.x.get([5, 0]);

        const apparent_wind_x = (wx - vx) * dt;
        const apparent_wind_y = (wy - vy) * dt;
        

        return math.matrix([
            [1, 0, dt, 0, 0, 0],
            [0, 1, 0, dt, 0, 0],
            [0, 0, 1, 0, apparent_wind_x, 0],
            [0, 0, 0, 1, 0, apparent_wind_y],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ]);
    }

    getControlMatrix(dt, boat) {
        // Maps control input to the state.

        const cosA = Math.cos(boat.angle);
        const sinA = Math.sin(boat.angle);

        return math.matrix([
            [0],
            [0],
            [cosA * dt],
            [sinA * dt],
            [0],
            [0]
        ]);
    }

    getMeasurementPrediction(x) {
        const vx = x.get([2, 0]);
        const vy = x.get([3, 0]);
        const wx = x.get([4, 0]);
        const wy = x.get([5, 0]);

        
        const predicted_angle = 0.0;

        return math.matrix([
            [x.get([0, 0])],
            [x.get([1, 0])],
            [predicted_angle]
        ]);
    }

    getMeasurementJacobian(x) {
        return math.matrix([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
            ]);
    }

    getWindVelocityEstimate() {
        return new Vector2D(this.x.get([4, 0]), this.x.get([5, 0]));
    }
}
