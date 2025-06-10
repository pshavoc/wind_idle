const WIND_VANE_THROTTLE = 20;

function runWindVaneMode() {
    const est_wind_dir = Math.atan2(kf.x.get([5]), kf.x.get([4]));
    const targetAngle = est_wind_dir + Math.PI; // Point into wind
    let angleError = Math.atan2(Math.sin(targetAngle - boat.angle), Math.cos(targetAngle - boat.angle));
    controls.rudder = Math.max(-30, Math.min(30, (angleError * (180/Math.PI)) * RUDDER_P_GAIN));
    // controls.throttle = WIND_VANE_THROTTLE; // This was commented out, keeping it so
}
