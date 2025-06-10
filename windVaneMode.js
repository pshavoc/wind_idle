(function() {
const THROTTLE_P_GAIN = 2.0;
const THROTTLE_I_GAIN = 0.0;
const RUDDER_P_GAIN = 0.8;

const throttlePID = new PIDController(THROTTLE_P_GAIN, THROTTLE_I_GAIN, 0, 30);

function runWindVaneMode() {
    const dx = anchorTarget.x - boat.x, dy = anchorTarget.y - boat.y, dist = Math.sqrt(dx*dx + dy*dy);
    // Update debug display
    document.getElementById('debug-display').style.display = 'block';
    document.getElementById('dx-value').textContent = dx.toFixed(2);
    document.getElementById('dy-value').textContent = dy.toFixed(2);
    document.getElementById('dist-value').textContent = dist.toFixed(2);

    const est_wind_dir = Math.atan2(kf.x.get([5]), kf.x.get([4]));
    const targetAngle = est_wind_dir + Math.PI; // Point into wind
    let angleError = Math.atan2(Math.sin(targetAngle - boat.angle), Math.cos(targetAngle - boat.angle));
    controls.rudder = Math.max(-30, Math.min(30, (angleError * (180/Math.PI)) * RUDDER_P_GAIN));


    let throttleOutput = throttlePID.update(0, dy, DT);

    
    controls.throttle = clamp(throttleOutput, 0, 50);

    
}
window.runWindVaneMode = runWindVaneMode; // Expose to global scope
})();
