class KalmanFilter {
    constructor() {
        // State: [x, y, vx, vy, a, av, wx, wy]
        // x, y: boat position
        // vx, vy: boat velocity
        // a : boat angle
        // av: boat angular velocity
        // wx, wy: wind velocity

        // State vector
        this.x = math.matrix(math.zeros(8, 1));

        // State covariance matrix
        this.P = math.diag([10, 10, 10, 10, 10, 10, 1000, 1000]); // Initial uncertainty, high for wind

        

        // Control input matrix
        this.B = math.matrix([
            [0.5 * dt * dt, 0],
            [0, 0.5 * dt * dt],
            [dt, 0],
            [0, dt],
            [0, 0],
            [0, 0]
        ]);

        // Observation matrix (we measure position)
        this.H = math.matrix([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]
        ]);

        // Process noise covariance (model uncertainty)
        // Higher noise for velocity to account for unmodeled physics (drag, etc.)
        const pos_noise = 0.01, vel_noise = 0.5, wind_noise = 0.1;
        this.Q = math.diag([pos_noise, pos_noise, vel_noise, vel_noise, wind_noise, wind_noise]);

        // Measurement noise covariance (sensor uncertainty)
        const measurement_noise = 2.0; // GPS noise in meters
        this.R = math.diag([measurement_noise, measurement_noise]);
        
        // Identity matrix
        this.I = math.identity(6);

        // Constants from the simulation
        this.MAX_THROTTLE_FORCE = 2000.0;
        this.BOAT_MASS = 300;
    }

    getStateTransitionMatrix(dt) {

        // x, y, vx, vy, a, av, wx, wy

        const WIND_EFFECT = 0.1; // Adjust this based on how much wind affects the boat

        return math.matrix([
            [1, 0, dt, 0, 0, 0, 0, 0],
            [0, 1, 0, dt, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, dt * WIND_EFFECT, 0],
            [0, 0, 0, 1, 0, 0, 0, dt * WIND_EFFECT],
            [0, 0, 0, 0, 1, dt, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ]);
    }

    transitionFunction(dt, controls) {
        let pos = new Vector2D(this.x.get([0, 0]), this.x.get([1, 0]));
        let velocity = new Vector2D(this.x.get([2, 0]), this.x.get([3, 0]));
        let boatAngle = this.x.get([4, 0]);
        let boatAngularVelocity = this.x.get([5, 0]);
        const boatDirection = new Vector2D(Math.cos(boatAngle), Math.sin(boatAngle));
        const windEst = new Vector2D(this.x.get([6, 0]), this.x.get([7, 0]));

        let force = new Vector2D(0, 0);

        const engineForce = boatDirection.clone().multiplyScalar((controls.throttle / 100) * this.MAX_THROTTLE_FORCE);
        force.add(engineForce);

        
        if (velocity.length() > 0.1) { // Only apply keel forces if moving to prevent division by zero
            // Angle of attack (alpha) is the angle between the boat's heading and its velocity vector.
            
            const velocityNormal = velocity.clone().normalize();
            
            let alpha = boatDirection.angleTo(velocity);

            // Use the 2D cross product to determine the sign of the angle (leeway direction).
            const crossProduct = boatDirection.x * velocityNormal.y - boatDirection.y * velocityNormal.x;
            if (crossProduct < 0) {
                alpha = -alpha;
            }

            // Simplified airfoil physics for the keel
            const LIFT_COEFFICIENT = 1.0; // Adjusted for noticeable effect
            const DRAG_COEFFICIENT = 0.7;  // Adjusted for noticeable effect
            const KEEL_AREA = 0.08;       // m^2, effective area of the keel
            const WATER_DENSITY = 1000;  // kg/m^3

            const speedSq = velocity.lengthSq();
            const dynamicPressure = 0.5 * WATER_DENSITY * speedSq * KEEL_AREA;

            // Lift and Drag coefficients as a function of angle of attack (alpha)
            const cl = LIFT_COEFFICIENT * Math.sin(2 * alpha);
            const cd = DRAG_COEFFICIENT * (1 - Math.cos(2 * alpha)) + 1.0;

            const liftForceMag = dynamicPressure * cl;
            const dragForceMag = dynamicPressure * cd;

            // Lift force is perpendicular to the direction of water flow (velocity).
            const liftAngle = velocity.angle() + -Math.PI / 2;
            const liftForce = new Vector2D(Math.cos(liftAngle), Math.sin(liftAngle)).multiplyScalar(liftForceMag);

            // Drag force is opposite to the direction of velocity.
            const dragForce = velocityNormal.multiplyScalar(-dragForceMag);

            force.add(liftForce);
            force.add(dragForce);
        }

        let apparentWind = windEst.clone().add(velocity.clone().multiplyScalar(-1)); // Apparent wind is the true wind minus the boat's velocity
        const AIR_DENSITY = 1.225; // kg/m^3, density of air at sea level
        const HULL_AREA = 5.0; // m^2, effective area of the hull
        const WIND_DRAG_COEFFICIENT = 1.0; // Adjusted for noticeable effect
        const dynamicPressure = 0.5 * AIR_DENSITY * apparentWind.lengthSq() * HULL_AREA;
        const windDragForceMag = dynamicPressure * WIND_DRAG_COEFFICIENT;
        let windForce = apparentWind.clone().normalize().multiplyScalar(windDragForceMag);
        force.add(windForce);

        // wind angle of attack
        let alpha = boatDirection.angleTo(apparentWind);
        const windNormal = apparentWind.clone().normalize();
        // Use the 2D cross product to determine the sign of the angle (leeway direction).
        const crossProduct = boatDirection.x * windNormal.y - boatDirection.y * windNormal.x;
        if (crossProduct < 0) {
            alpha = -alpha;
        }
        const windTorque = Math.sin(alpha) * windForce.length() * 2.0;

        velocity.add( force.multiplyScalar(dt / BOAT_MASS) )
        pos.add( velocity.clone().multiplyScalar(dt) );

        const motorTorque = Math.sin(degreesToRadians(controls.rudder)) * engineForce.length() * 1.0;
        const angularDamping = -boatAngularVelocity * 500.0;

        const totalTorque = motorTorque + windTorque + angularDamping;
        const angularAcceleration = totalTorque / MOMENT_OF_INERTIA;
        boatAngularVelocity += angularAcceleration * dt;
        boatAngle += boatAngularVelocity * dt;

        return math.matrix([
            pos.x, pos.y,
            velocity.x, velocity.y,
            boatAngle, boatAngularVelocity,
            windEst.x, windEst.y
        ]);
    }

    getStateTransitionJacobian(dt, controls) {

        // x, y, vx, vy, a, av, wx, wy
        
        let jacobian = math.identity(8);
        jacobian.set([0, 2], dt);
        jacobian.set([1, 3], dt);
        jacobian.set([4, 5], dt);


        jacobian.set([5, 6], )
        
        return jacobian;
    }

    predict(dt, controls, boat) {

        let x_k = this.transitionFunction(dt, controls);


        const F = getStateTransitionMatrix(dt);

        // Control input vector
        const engineForceMag = (controls.throttle / 100) * this.MAX_THROTTLE_FORCE;
        const acc_x = (engineForceMag * Math.cos(boatAngle)) / this.BOAT_MASS;
        const acc_y = (engineForceMag * Math.sin(boatAngle)) / this.BOAT_MASS;
        const u = math.matrix([[acc_x], [acc_y]]);

        // Predict state: x_pred = F * x + B * u
        this.x = math.add(math.multiply(F, this.x), math.multiply(this.B, u));

        // Predict covariance: P_pred = F * P * F^T + Q
        this.P = math.add(math.multiply(F, this.P, math.transpose(F)), this.Q);
    }

    update(boat) {
        // Measurement vector from GPS
        const z = math.matrix([[boat.pos.x], [boat.pos.y]]);

        // Innovation (measurement residual): y = z - H * x
        const y = math.subtract(z, math.multiply(this.H, this.x));

        // Innovation covariance: S = H * P * H^T + R
        const S = math.add(math.multiply(this.H, this.P, math.transpose(this.H)), this.R);

        // Kalman gain: K = P * H^T * S^-1
        const K = math.multiply(this.P, math.transpose(this.H), math.inv(S));

        // Update state estimate: x = x + K * y
        this.x = math.add(this.x, math.multiply(K, y));

        // Update covariance estimate: P = (I - K * H) * P
        this.P = math.multiply(math.subtract(this.I, math.multiply(K, this.H)), this.P);
    }

    run(dt, boat, controls) {
        // Initialize state with first measurement if not initialized
        if (this.x.get([0,0]) === 0 && this.x.get([1,0]) === 0) {
            this.x.set([0, 0], boat.pos.x);
            this.x.set([1, 0], boat.pos.y);
            this.x.set([2, 0], boat.velocity.x);
            this.x.set([3, 0], boat.velocity.y);
        }
        this.predict(dt, controls, boat);
        this.update(boat);
    }

    getWindVelocityEstimate() {
        const wind_x = this.x.get([4, 0]);
        const wind_y = this.x.get([5, 0]);
        return new Vector2D(wind_x, wind_y);
    }
}