function degreesToRadians(d) { return d * (Math.PI / 180); }
function radiansToDegrees(r) { return (r * (180 / Math.PI) + 360) % 360; }
function clamp(value, min, max) { return Math.min(Math.max(value, min), max); }
function norm(x, y) {
    const mag = Math.sqrt(x * x + y * y);
    if (mag == 0) {
        return {
            x: 0,
            y: 0,
        };
    }
    return {
        x: x / mag,
        y: y / mag,
    };
}

function normalDistribution(mu = 0, sigma = 1) {
    let u1 = 0, u2 = 0;
    // Convert [0, 1) to (0, 1) to avoid log(0) which is -Infinity
    while (u1 === 0) u1 = Math.random();
    while (u2 === 0) u2 = Math.random();

    // Box-Muller transform
    const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
    // z1 is the second value, which can be stored for the next call if needed for efficiency
    // const z1 = Math.sqrt(-2.0 * Math.log(u1)) * Math.sin(2.0 * Math.PI * u2);

    // Scale and translate to the desired mean (mu) and standard deviation (sigma)
    return z0 * sigma + mu;
}

export { degreesToRadians, radiansToDegrees, clamp, norm, normalDistribution };