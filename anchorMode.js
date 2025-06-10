const AP_MAX_THROTTLE = 40;
const THROTTLE_P_GAIN = 0.5;
const AP_KI_GAIN = 0.05;
const AP_MAX_INTEGRAL_ANCHOR = 400;
const AP_STOPPING_DISTANCE = 5;
const RUDDER_P_GAIN = 0.8;

function runAnchorMode() {
    const dx = anchorTarget.x - boat.x, dy = anchorTarget.y - boat.y, dist = Math.sqrt(dx*dx + dy*dy);
    const targetAngle = Math.atan2(dy, dx);
    let angleError = Math.atan2(Math.sin(targetAngle - boat.angle), Math.cos(targetAngle - boat.angle));
    controls.rudder = Math.max(-30, Math.min(30, (angleError * (180/Math.PI)) * RUDDER_P_GAIN));

    let p_throttle = dist * THROTTLE_P_GAIN;
    // Calculate integral contribution only if AP_KI_GAIN is defined and non-zero
    let i_throttle = anchorIntegralError * AP_KI_GAIN;

    if(Math.abs(angleError) < (Math.PI/1.5)) { // If pointing somewhat towards target
        anchorIntegralError += dist * DT; // DT is the physics time step
        // Clamp integral term to prevent windup
        anchorIntegralError = Math.max(-AP_MAX_INTEGRAL_ANCHOR, Math.min(AP_MAX_INTEGRAL_ANCHOR, anchorIntegralError));
        controls.throttle = Math.min(AP_MAX_THROTTLE, Math.max(0, p_throttle + i_throttle));
    } else {
        controls.throttle = 5; // Fixed throttle for turning, maintain heading
        // When turning sharply, we might not want to accumulate integral,
        // or even decay it. For now, accumulation only happens if angleError is small.
    }
}
