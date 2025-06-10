class PIDController {
    constructor(kp, ki, kd, maxIntegral) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.maxIntegral = maxIntegral;

        this.integralError = 0;
        this.previousError = 0;
    }

    update(targetValue, currentValue, dt) {
        const error = targetValue - currentValue;

        // Proportional term
        const p_term = this.kp * error;

        // Integral term
        this.integralError += error * dt;
        if (this.maxIntegral !== null) {
            this.integralError = clamp(this.integralError, -this.maxIntegral, this.maxIntegral);
        }
        const i_term = this.ki * this.integralError;

        // Derivative term
        const derivativeError = (error - this.previousError) / dt;
        const d_term = this.kd * derivativeError;
        this.previousError = error;

        // Calculate output
        let output = p_term + i_term + d_term;

        return output;
    }

    reset() {
        this.integralError = 0;
        this.previousError = 0;
    }
}