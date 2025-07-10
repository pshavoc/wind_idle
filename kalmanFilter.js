class KalmanFilter {
    constructor() {
        // State: [x, y, vx, vy, wx, wy]
        // x, y: boat position
        // vx, vy: boat velocity
        // wx, wy: wind velocity
        this.x = math.matrix(math.zeros(6, 1));

        
    }

    run(dt, boat, controls) {
        
    }

}
