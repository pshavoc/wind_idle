import init, { Windpark, MotorboatDynamicsKalmanFilter } from './windpark/pkg/windpark.js';
import { manualModeControl, acroModeControl } from './lsm_mode.js';
import { degreesToRadians, radiansToDegrees, normalDistribution } from './utils.js';

let sim = null;
let ekf = null;

init().then(() => {
    console.log('Windpark module initialized');
    // You can now use the functions from the windpark module
    sim = new Windpark();
    ekf = new MotorboatDynamicsKalmanFilter();

    initialize();

}).catch(err => {
    console.error('Error initializing Windpark module:', err);
});

const canvas = document.getElementById('simulationCanvas');
const ctx = canvas.getContext('2d');

// UI Elements
const throttleSlider = document.getElementById('throttle'), rudderSlider = document.getElementById('rudder');
const throttleValue = document.getElementById('throttleValue'), rudderValue = document.getElementById('rudderValue');
const speedDisplay = document.getElementById('speedDisplay'), windSpeedDisplay = document.getElementById('windSpeedDisplay');
const windDirDisplay = document.getElementById('windDirDisplay'), estWindSpeedDisplay = document.getElementById('estWindSpeedDisplay');
const estWindDirDisplay = document.getElementById('estWindDirDisplay'), autopilotBtn = document.getElementById('autopilotBtn');
const autopilotStatus = document.getElementById('autopilotStatus');
const newWindBtn = document.getElementById('newWindBtn');
const gpsBtn = document.getElementById('gpsBtn');
const compassBtn = document.getElementById('compassBtn');



let perlinGenerator = new PerlinNoise(); // Instantiate PerlinNoise class

// let kalmanFilter = new KalmanFilter();



// --- Main Simulation Functions ---
function initialize() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;

    // Initialize boat state
    boat.x = 0;
    boat.y = 0;

    // initialize display scale
    screen_m_to_pix.m = 10.0;
    screen_m_to_pix.x = canvas.width / 2; // Center the boat in the middle of the screen
    screen_m_to_pix.y = canvas.height / 2; // Center the boat in the middle of the screen


    generateNewWind();
    updateTrueWindWithNoise(0); // Initialize trueWind with noise (at time 0)

    // --- Event Listeners ---
    throttleSlider.addEventListener('input', (e) => { controls.throttle = parseInt(e.target.value, 10); });
    rudderSlider.addEventListener('input', (e) => { if (autopilotMode === 'OFF') controls.rudder = parseInt(e.target.value, 10); });
    autopilotBtn.addEventListener('click', cycleAutopilotMode);
    newWindBtn.addEventListener('click', generateNewWind); // Add event listener for new wind button
    gpsBtn.addEventListener('click', toggleGps);
    compassBtn.addEventListener('click', toggleCompass);
    window.addEventListener('keydown', (e) => {
        if (autopilotMode === 'OFF') keysPressed[e.key] = true;
        if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(e.key)) e.preventDefault();
    });
    window.addEventListener('keyup', (e) => { keysPressed[e.key] = false; });
    window.addEventListener('resize', () => { canvas.width = window.innerWidth; canvas.height = window.innerHeight; });

    requestAnimationFrame(animate);
}

function cycleAutopilotMode() {
    keysPressed = {};
    switch (autopilotMode) {
        case MODE_MANUAL:
            autopilotMode = MODE_ACRO;
            anchorTarget = { x: 0, y: 0 };
            anchorIntegralError = 0; // Reset integral error for anchor mode
            autopilotBtn.textContent = autopilotMode;
            autopilotBtn.classList.replace('bg-blue-500', 'bg-green-500');
            autopilotBtn.classList.replace('hover:bg-blue-600', 'hover:bg-green-600');
            autopilotStatus.textContent = autopilotMode;
            autopilotStatus.classList.replace('text-red-600', 'text-green-600');
            break;
        case MODE_ACRO:
            autopilotMode = MODE_MANUAL;
            anchorTarget = { x: 0, y: 0 };
            autopilotBtn.textContent = autopilotMode;
            autopilotBtn.classList.replace('bg-green-500', 'bg-yellow-500');
            autopilotBtn.classList.replace('hover:bg-green-600', 'hover:bg-yellow-600');
            autopilotStatus.textContent = autopilotMode;
            autopilotStatus.classList.replace('text-green-600', 'text-yellow-800');
            break;
        case 'WIND_VANE':
            autopilotMode = 'LSM';
            anchorTarget = null; // Clear anchor target
            autopilotBtn.textContent = 'Autopilot OFF';
            autopilotBtn.classList.replace('bg-yellow-500', 'bg-blue-500');
            autopilotBtn.classList.replace('hover:bg-yellow-600', 'hover:bg-blue-600');
            autopilotStatus.textContent = 'LSM';
            autopilotStatus.classList.replace('text-yellow-800', 'text-red-600');
            break;
        case 'LSM':
            autopilotMode = 'OFF';
            anchorTarget = null; // Clear anchor target
            autopilotBtn.textContent = 'Low Speed Mode';
            autopilotBtn.classList.replace('bg-yellow-500', 'bg-blue-500');
            autopilotBtn.classList.replace('hover:bg-yellow-600', 'hover:bg-blue-600');
            autopilotStatus.textContent = 'OFF';
            autopilotStatus.classList.replace('text-yellow-800', 'text-red-600');
            break;
    }
}

function toggleGps() {
    gpsUpdateEnabled = !gpsUpdateEnabled;
    if (gpsUpdateEnabled) {
        gpsBtn.textContent = 'GPS: ON';
        gpsBtn.classList.replace('bg-red-500', 'bg-green-500');
        gpsBtn.classList.replace('hover:bg-red-600', 'hover:bg-green-600');
    } else {
        gpsBtn.textContent = 'GPS: OFF';
        gpsBtn.classList.replace('bg-green-500', 'bg-red-500');
        gpsBtn.classList.replace('hover:bg-green-600', 'hover:bg-red-600');
    }
}

function toggleCompass() {
    compassUpdateEnabled = !compassUpdateEnabled;
    if (compassUpdateEnabled) {
        compassBtn.textContent = 'Compass: ON';
        compassBtn.classList.replace('bg-red-500', 'bg-green-500');
        compassBtn.classList.replace('hover:bg-red-600', 'hover:bg-green-600');
    } else {
        compassBtn.textContent = 'Compass: OFF';
        compassBtn.classList.replace('bg-green-500', 'bg-red-500');
        compassBtn.classList.replace('hover:bg-green-600', 'hover:bg-red-600');
    }
}

function generateNewWind() {
    baseWindSpeed = Math.random() * 8; // Speed in m/s
    baseWindDirRadians = Math.random() * 2 * Math.PI;
    // trueWind.vx = Math.cos(dir) * speed; // Old direct update
    // trueWind.vy = Math.sin(dir) * speed; // Old direct update
    // Initial trueWind calculation will be handled by updateTrueWindWithNoise
}

// --- Control Logic ---
// Keyboard adjustment rates (added for reverse throttle support)
const THROTTLE_KEY_RATE = 2; // percent per frame when key held
const RUDDER_KEY_RATE = 1;    // degrees per frame when key held
function handleKeyboardControls() {
    if (keysPressed['ArrowUp']) controls.throttle = Math.min(100, controls.throttle + THROTTLE_KEY_RATE);
    if (keysPressed['ArrowDown']) controls.throttle = Math.max(-100, controls.throttle - THROTTLE_KEY_RATE);
    if (keysPressed['ArrowLeft']) controls.rudder = Math.max(-15, controls.rudder - RUDDER_KEY_RATE);
    if (keysPressed['ArrowRight']) controls.rudder = Math.min(15, controls.rudder + RUDDER_KEY_RATE);
}

function runAutopilot() {
    if (autopilotMode === 'ANCHOR') runAnchorMode();
    else if (autopilotMode === 'WIND_VANE') runWindVaneMode();
    else if (autopilotMode === 'LSM') lsmModeControl();
}

// --- Main Loop ---
let physicsTime = 0.0;
let filter_time = 0.0;
let filter_counter = 0;

function animate(now) {
    requestAnimationFrame(animate);


    while (physicsTime < now) {
        update_physics(DT);
        physicsTime += DT * 1000; // Convert to milliseconds
    }

    if (now - filter_time > 100) {
        ekf.predict(degreesToRadians(controls.rudder), controls.throttle);
        filter_counter += 1;

        if (filter_counter % 5 == 0) {
            if (compassUpdateEnabled) {
                const compass_reading = normalDistribution(boat.angle, 0.1);
                ekf.update_compass(compass_reading, 0.1);
            }
        }

        if (filter_counter % 10 == 0) {
            if (gpsUpdateEnabled) {

                const gps_x = normalDistribution(boat.pos.x, 1.0);
                const gps_y = normalDistribution(boat.pos.y, 1.0);

                ekf.update_gps(gps_x, gps_y, 1.0);
            }

            let pos = ekf.pos();
            let vel = ekf.velocity();
            let angular_velocity = ekf.angular_velocity();
            let wind = ekf.wind_velocity();
            let wind_vector = new Vector2D(wind[0], wind[1]);
            let wind_dir = wind_vector.angle();
            let wind_speed = wind_vector.length();
            console.log("EKF velocity:", vel[0].toFixed(2), vel[1].toFixed(2));
            console.log("EKF wind dir:", wind_dir.toFixed(2), "wind speed:", wind_speed.toFixed(2));
            console.log("EKF angular velocity:", angular_velocity.toFixed(2), "r/s");

            const wind_dir_error = radiansToDegrees(Math.abs(wind_dir - trueWind.angle()));
            console.log("Wind dir error:", wind_dir_error.toFixed(2));
        }
        filter_time = now;
    }


    pilot();
    draw();
}

function pilot() {
    switch (autopilotMode) {
        case MODE_MANUAL:
            manualModeControl();
            break;
        case MODE_ACRO:
            acroModeControl();
            break;
        default:
            break;
    }

    handleKeyboardControls();

}

function updateTrueWindWithNoise(dt) {
    perlinNoiseTime += dt;
    const noise = perlinGenerator.noise1D(perlinNoiseTime * PERLIN_WIND_FREQUENCY); // Use class method
    const directionOffsetDegrees = noise * MAX_WIND_DIR_CHANGE_DEGREES;
    const directionOffsetRadians = degreesToRadians(directionOffsetDegrees);

    const currentWindDirRadians = baseWindDirRadians + directionOffsetRadians;

    const windSpeedNoise = perlinGenerator.noise1D(perlinNoiseTime * 0.1) * 2.0;

    trueWind.x = Math.cos(currentWindDirRadians) * (baseWindSpeed + windSpeedNoise);
    trueWind.y = Math.sin(currentWindDirRadians) * (baseWindSpeed + windSpeedNoise);
}

function update_physics(dt) {
    updateTrueWindWithNoise(dt); // Update wind with Perlin noise

    sim.set_wind(trueWind.x, trueWind.y);
    sim.step(degreesToRadians(controls.rudder), controls.throttle, dt);
    const boat_pos = sim.pos();

    boat.pos.x = boat_pos[0];
    boat.pos.y = boat_pos[1];
    boat.angle = sim.theta();
    boat.speed = sim.speed();
    boat.angularVelocity = sim.angular_velocity();
    let velocity = sim.velocity();
    boat.velocity.x = velocity[0];
    boat.velocity.y = velocity[1];


    // const boatDirection = new Vector2D(Math.cos(boat.angle), Math.sin(boat.angle));

    // let force = new Vector2D(0, 0);

    // const engineForce = new Vector2D(Math.cos(boat.angle), Math.sin(boat.angle)).multiplyScalar( (controls.throttle / 100) * MAX_THROTTLE_FORCE);

    // force.add(engineForce);

    // // --- Keel Physics (Symetric Airfoil Model) ---
    // if (boat.velocity.length() > 0.1) { // Only apply keel forces if moving to prevent division by zero
    //     // Angle of attack (alpha) is the angle between the boat's heading and its velocity vector.

    //     const velocityNormal = boat.velocity.clone().normalize();

    //     let alpha = boatDirection.angleTo(boat.velocity);

    //     // Use the 2D cross product to determine the sign of the angle (leeway direction).
    //     const crossProduct = boatDirection.x * velocityNormal.y - boatDirection.y * velocityNormal.x;
    //     if (crossProduct < 0) {
    //         alpha = -alpha;
    //     }

    //     // Simplified airfoil physics for the keel
    //     const LIFT_COEFFICIENT = 1.0; // Adjusted for noticeable effect
    //     const DRAG_COEFFICIENT = 0.7;  // Adjusted for noticeable effect
    //     const KEEL_AREA = 0.08;       // m^2, effective area of the keel
    //     const WATER_DENSITY = 1000;  // kg/m^3

    //     const speedSq = boat.velocity.lengthSq();
    //     const dynamicPressure = 0.5 * WATER_DENSITY * speedSq * KEEL_AREA;

    //     // Lift and Drag coefficients as a function of angle of attack (alpha)
    //     const cl = LIFT_COEFFICIENT * Math.sin(2 * alpha);
    //     const cd = DRAG_COEFFICIENT * (1 - Math.cos(2 * alpha)) + 1.0;

    //     const liftForceMag = dynamicPressure * cl;
    //     const dragForceMag = dynamicPressure * cd;

    //     // Lift force is perpendicular to the direction of water flow (velocity).
    //     const liftAngle = boat.velocity.angle() + -Math.PI / 2;
    //     const liftForce = new Vector2D(Math.cos(liftAngle), Math.sin(liftAngle)).multiplyScalar(liftForceMag);

    //     // Drag force is opposite to the direction of velocity.
    //     const dragForce = velocityNormal.multiplyScalar(-dragForceMag);

    //     force.add(liftForce);
    //     force.add(dragForce);
    // }

    // let apparentWind = trueWind.clone().add(boat.velocity.clone().multiplyScalar(-1)); // Apparent wind is the true wind minus the boat's velocity
    // const AIR_DENSITY = 1.225; // kg/m^3, density of air at sea level
    // const HULL_AREA = 5.0; // m^2, effective area of the hull
    // const WIND_DRAG_COEFFICIENT = 1.0; // Adjusted for noticeable effect
    // const dynamicPressure = 0.5 * AIR_DENSITY * apparentWind.lengthSq() * HULL_AREA;
    // const windDragForceMag = dynamicPressure * WIND_DRAG_COEFFICIENT;
    // let windForce = apparentWind.clone().normalize().multiplyScalar(windDragForceMag);
    // force.add(windForce);

    // // wind angle of attack
    // let alpha = boatDirection.angleTo(trueWind);
    // const trueWindNormal = trueWind.clone().normalize();
    // // Use the 2D cross product to determine the sign of the angle (leeway direction).
    // const crossProduct = boatDirection.x * trueWindNormal.y - boatDirection.y * trueWindNormal.x;
    // if (crossProduct < 0) {
    //     alpha = -alpha;
    // }
    // const windTorque = Math.sin(alpha) * windForce.length() * 2.0;

    // boat.velocity.add( force.multiplyScalar(dt / BOAT_MASS) )
    // boat.pos.add( boat.velocity.clone().multiplyScalar(dt) );


    // const motorTorque = Math.sin(degreesToRadians(controls.rudder)) * engineForce.length() * 1.0;
    // const angularDamping = -boat.angularVelocity * 500.0;

    // const totalTorque = motorTorque + windTorque + angularDamping;
    // const angularAcceleration = totalTorque / MOMENT_OF_INERTIA;
    // boat.angularVelocity += angularAcceleration * dt;
    // boat.angle += boat.angularVelocity * dt;



    // boat.speed = boat.velocity.length();

    // --- Bubble generation ---
    // Bubble generation now uses absolute throttle so reverse still shows prop wash
    let genbubble = Math.random() < Math.abs(controls.throttle) * 0.05;
    if (genbubble) {
        const spread = (Math.random() - 0.5) * 2.0; // Spread bubbles behind the boat
        const angleOffset = (Math.random() - 0.5) * 0.1; // Slight random angle for bubbles
        const reversing = controls.throttle < 0;
        // If reversing, thrust vector is forward (no +PI flip)
        const angle = boat.angle + degreesToRadians(-controls.rudder) + (reversing ? 0 : Math.PI) + angleOffset;
        const forceMag = Math.abs(controls.throttle) * 0.1; // Bubbles move slower than the boat

        let body_origin_x = -3.0;
        let body_origin_y = 0.0;

        // origin_x = Math.cos(boat.angle) * origin_x - Math.sin(boat.angle) * origin_y;
        // origin_y = Math.sin(boat.angle) * origin_x + Math.cos(boat.angle) * origin_y;

        let origin_x = boat.pos.x;
        let origin_y = boat.pos.y;

        origin_x += Math.cos(boat.angle) * body_origin_x - Math.sin(boat.angle) * body_origin_y;
        origin_y += Math.sin(boat.angle) * body_origin_x + Math.cos(boat.angle) * body_origin_y;

        // rotate origin_x 

        bubbles.push({
            x: origin_x,
            y: origin_y,
            vx: Math.cos(angle) * forceMag, // Bubbles move opposite to thrust, influenced by boat speed
            vy: Math.sin(angle) * forceMag,
            life: 1 + Math.random() * 10, // Lifespan in seconds
            size: 1 + Math.random() * 3
        });
    }

    // --- Update existing bubbles ---
    for (let i = bubbles.length - 1; i >= 0; i--) {
        let b = bubbles[i];
        b.x += b.vx * dt;
        b.y += b.vy * dt;
        b.life -= dt;
        b.size *= 0.98; // Bubbles shrink slightly

        if (b.life <= 0 || b.size < 0.5) {
            bubbles.splice(i, 1); // Remove dead bubbles
        }
    }
}

function draw() {
    ctx.fillStyle = '#3b82f6'; ctx.fillRect(0, 0, canvas.width, canvas.height);
    drawBubbles(); // Draw bubbles before the boat
    if (anchorTarget) drawAnchorMarker();
    drawBoat();
    drawGhostBoat();
    drawWindIndicator();

    let x = screen_m_to_pix.m * boat.pos.x + screen_m_to_pix.x;
    let y = screen_m_to_pix.m * boat.pos.y + screen_m_to_pix.y;

    if (x > canvas.width - 50) {
        screen_m_to_pix.x -= canvas.width;
    } else if (x < 50) {
        screen_m_to_pix.x += canvas.width;
    } else if (y > canvas.height - 50) {
        screen_m_to_pix.y -= canvas.height;
    } else if (y < 50) {
        screen_m_to_pix.y += canvas.height;
    }

    updateUI();
}

function drawAnchorMarker() {
    let x = screen_m_to_pix.m * anchorTarget.x + screen_m_to_pix.x;
    let y = screen_m_to_pix.m * anchorTarget.y + screen_m_to_pix.y;

    ctx.save(); ctx.translate(x, y); ctx.strokeStyle = '#f59e0b'; ctx.lineWidth = 3; ctx.globalAlpha = 0.9;
    const s = 15; ctx.beginPath(); ctx.moveTo(-s, -s); ctx.lineTo(s, s); ctx.moveTo(s, -s); ctx.lineTo(-s, s); ctx.stroke();
    ctx.beginPath(); ctx.arc(0, 0, s * 1.5, 0, 2 * Math.PI); ctx.stroke(); ctx.restore();
}

function drawBoat() {
    let x = screen_m_to_pix.m * boat.pos.x + screen_m_to_pix.x;
    let y = screen_m_to_pix.m * boat.pos.y + screen_m_to_pix.y;

    ctx.save(); ctx.translate(x, y); ctx.rotate(boat.angle);
    ctx.fillStyle = '#4B5563'; ctx.fillRect(-BOAT_LENGTH / 2, -BOAT_WIDTH / 2, BOAT_LENGTH, BOAT_WIDTH);
    ctx.fillStyle = '#D1D5DB'; ctx.fillRect(-BOAT_LENGTH / 4, -BOAT_WIDTH / 2 + 2, BOAT_LENGTH / 2, BOAT_WIDTH - 4);
    ctx.fillStyle = '#4B5563'; ctx.beginPath();
    ctx.moveTo(BOAT_LENGTH / 2, -BOAT_WIDTH / 2); ctx.lineTo(BOAT_LENGTH / 2 + 15, 0); ctx.lineTo(BOAT_LENGTH / 2, BOAT_WIDTH / 2);
    ctx.closePath(); ctx.fill(); ctx.restore();
}

function drawGhostBoat() {
    let pos = ekf.pos();
    let angle = ekf.theta();
    let x = screen_m_to_pix.m * pos[0] + screen_m_to_pix.x;
    let y = screen_m_to_pix.m * pos[1] + screen_m_to_pix.y;

    ctx.save(); ctx.translate(x, y); ctx.rotate(angle);

    // the ghost boat should be drawn with a lower opacity
    ctx.globalAlpha = 0.3;

    ctx.fillStyle = '#4B5563'; ctx.fillRect(-BOAT_LENGTH / 2, -BOAT_WIDTH / 2, BOAT_LENGTH, BOAT_WIDTH);
    ctx.fillStyle = '#D1D5DB'; ctx.fillRect(-BOAT_LENGTH / 4, -BOAT_WIDTH / 2 + 2, BOAT_LENGTH / 2, BOAT_WIDTH - 4);
    ctx.fillStyle = '#4B5563'; ctx.beginPath();
    ctx.moveTo(BOAT_LENGTH / 2, -BOAT_WIDTH / 2); ctx.lineTo(BOAT_LENGTH / 2 + 15, 0); ctx.lineTo(BOAT_LENGTH / 2, BOAT_WIDTH / 2);
    ctx.closePath(); ctx.fill(); ctx.restore();
}

function drawBubbles() {
    ctx.fillStyle = 'rgba(255, 255, 255, 0.6)';
    for (const b of bubbles) {
        let x = screen_m_to_pix.m * b.x + screen_m_to_pix.x;
        let y = screen_m_to_pix.m * b.y + screen_m_to_pix.y;
        ctx.beginPath();
        ctx.arc(x, y, b.size * screen_m_to_pix.m * 0.2, 0, Math.PI * 2); // Scale bubble size with screen zoom
        ctx.fill();
    }
}

function drawWindIndicator() {
    const windSpeed = trueWind.length();
    const windDir = trueWind.angle();
    const indX = canvas.width - 60, indY = 60, arrLen = 25 + windSpeed * 2;
    ctx.save(); ctx.translate(indX, indY); ctx.rotate(windDir);
    ctx.strokeStyle = 'rgba(255,255,255,0.7)'; ctx.lineWidth = 3; ctx.fillStyle = 'rgba(255,255,255,0.7)';
    ctx.beginPath(); ctx.moveTo(-arrLen / 2, 0); ctx.lineTo(arrLen / 2, 0); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(arrLen / 2, 0); ctx.lineTo(arrLen / 2 - 10, -5); ctx.lineTo(arrLen / 2 - 10, 5); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.font = '16px sans-serif'; ctx.fillStyle = 'rgba(255,255,255,0.7)';
    ctx.fillText('True', indX - 20, indY + 25);
}

function updateUI() {
    throttleSlider.value = controls.throttle; rudderSlider.value = controls.rudder;
    speedDisplay.textContent = `${boat.speed.toFixed(1)} m/s`; // m/s to knots
    headingDisplay.textContent = `${radiansToDegrees(boat.angle).toFixed(0)} °`;
    throttleValue.textContent = `${Math.round(controls.throttle)}%`; rudderValue.textContent = `${Math.round(controls.rudder)}°`;
    // True Wind
    const trueSpeed = trueWind.length();
    const trueDir = radiansToDegrees(trueWind.angle());
    windSpeedDisplay.textContent = `${trueSpeed.toFixed(1)} m/s`; windDirDisplay.textContent = `${Math.round(trueDir)}°`;
    // Estimated Wind from kalman filter
    const wind = ekf.wind_velocity();
    const wind_dir = radiansToDegrees(Math.atan2(wind[1], wind[0]));
    const wind_speed = Math.sqrt(wind[0] * wind[0] + wind[1] * wind[1]);
    estWindSpeedDisplay.textContent = `${wind_speed.toFixed(1)} m/s`; estWindDirDisplay.textContent = `${Math.round(wind_dir)}°`;
}

const MODE_MANUAL = 'MANUAL';
const MODE_ACRO = 'ACRO';


let boat = {
    pos: new Vector2D(0, 0), // Position in meters
    velocity: new Vector2D(0, 0), // Velocity in m/s
    angle: 0, // Angle in radians
    angularVelocity: 0, // Angular velocity in radians/s
};
let trueWind = new Vector2D(0, 0); // Wind vector in m/s
let controls = { throttle: 0, rudder: 0 };
let keysPressed = {};
let autopilotMode = MODE_MANUAL;
let gpsUpdateEnabled = true;
let compassUpdateEnabled = true;
let anchorTarget = null;
let anchorIntegralError = 0; // Accumulated integral error for anchor mode
let bubbles = []; // Add this line to store bubble particles

// --- Simulation Constants ---
const DT = 1 / 20; // 50Hz
const BOAT_LENGTH = 40, BOAT_WIDTH = 15, MAX_THROTTLE_FORCE = 2000.0;
const BOAT_MASS = 300; // Mass of the boat in kg
const WATER_DRAG_COEFFICIENT = 0.08, WIND_FORCE_COEFFICIENT = 0.001;
const MOMENT_OF_INERTIA = 511; // Moment of inertia for the boat (kg*m^2)

let screen_m_to_pix = {
    m: 0.1,
    x: 0.0,
    y: 0.0,
};

// --- Simulation State ---

// Wind noise state
let baseWindSpeed = 0;
let baseWindDirRadians = 0;
let perlinNoiseTime = 0;
const PERLIN_WIND_FREQUENCY = 0.01; // Hz
const MAX_WIND_DIR_CHANGE_DEGREES = 60; // +/- 30 degrees


export { ekf, controls };