from controller import Robot, Keyboard
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

MAX_MOTOR_VEL = 20.0

def clamp(val, limit):
    return max(-limit, min(limit, val))

keyboard = Keyboard()
keyboard.enable(timestep)

imu = robot.getDevice('imu')
imu.enable(timestep)

motor1 = robot.getDevice('motor_1')
motor2 = robot.getDevice('motor_2')
motor3 = robot.getDevice('motor_3')
motor4 = robot.getDevice('motor_4')

for m in [motor1, motor2, motor3, motor4]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# --- GPS for speed feedback ---
gps = robot.getDevice('gps')
gps.enable(timestep)

# --- tuning constants ---
THRUST_STEP     = 0.5
YAW_STEP        = 0.3
MAX_THRUST      = 20.0
MAX_YAW         = 8.0
SMOOTHING       = 0.08
YAW_RETURN_TIME = 1.5

# --- speed PID constants ---
KP = 2
KI = 0.8
KD = 0.1

# --- adaptive feedforward ---
FF              = 0.0
FF_LEARN_RATE   = 0.01
FF_THRESHOLD    = 0.05

# --- boost constants ---
BOOST_THRESHOLD = 0.3       # boost until this fraction of target speed is reached
BOOST_POWER     = 1.0       # fraction of MAX_MOTOR_VEL during boost

YAW_DECAY = 1.0 - (timestep / 1000.0) / YAW_RETURN_TIME

# --- state ---
target_speed     = 0.0
yaw_input        = 0.0
thrust           = 0.0

# --- PID state ---
prev_pos         = None
speed_integral   = 0.0
prev_speed_error = 0.0

print("Controls:")
print("  W / S     -> Increase / decrease target speed")
print("  A / D     -> Yaw left / right (returns to 0 after release)")
print("  SPACE     -> Full stop reset")
print("  P         -> Set target speed to 3.2 m/s")
print("  O         -> Set target speed to -3.2 m/s")

while robot.step(timestep) != -1:
    key = keyboard.getKey()

    # --- target speed ---
    if key == ord('W'):
        target_speed = min(target_speed + THRUST_STEP, MAX_THRUST)
    elif key == ord('S'):
        target_speed = max(target_speed - THRUST_STEP, -MAX_THRUST)

    if key == ord('P'):
        target_speed = 3.2
    if key == ord('O'):
        target_speed = -3.2

    # --- yaw ---
    if key == ord('D'):
        yaw_input = min(yaw_input + YAW_STEP, MAX_YAW)
    elif key == ord('A'):
        yaw_input = max(yaw_input - YAW_STEP, -MAX_YAW)
    else:
        yaw_input *= YAW_DECAY
        if abs(yaw_input) < 0.01:
            yaw_input = 0.0

    # --- reset ---
    if key == ord(' '):
        target_speed     = 0.0
        yaw_input        = 0.0
        thrust           = 0.0
        speed_integral   = 0.0
        prev_speed_error = 0.0
        FF               = 0.0

    # --- compute actual speed and direction ---
    pos = gps.getValues()
    rpy = imu.getRollPitchYaw()  # [roll, pitch, yaw]
    yaw = rpy[2] 
    
    if prev_pos is not None:
        # 1. World-space velocity vector
        vx = (pos[0] - prev_pos[0]) / dt
        vz = (pos[2] - prev_pos[2]) / dt
        
        # 2. Heading vector from IMU (NUE system: X=pitch, Y=yaw, Z=roll)
        # Since your robot's thrust is on the Z axis in Webots:
        head_x = math.sin(yaw)
        head_z = math.cos(yaw)
        
        # 3. Dot product to find direction
        # Positive result = Forward, Negative = Backward
        dot_product = (vx * head_x) + (vz * head_z)
        
        # 4. Signed speed
        magnitude = math.sqrt(vx**2 + vz**2)
        actual_speed = math.copysign(magnitude, dot_product)
    else:
        actual_speed = 0.0
    
    prev_pos = list(pos)

    # --- PID speed controller with boost ---
    speed_error      = target_speed - actual_speed
    speed_integral   = clamp(speed_integral + speed_error * dt, MAX_MOTOR_VEL)
    speed_derivative = (speed_error - prev_speed_error) / dt
    prev_speed_error = speed_error

    # adaptive feedforward — learns drag offset at current speed
    if abs(speed_error) < FF_THRESHOLD and abs(target_speed) > 0.1:
        FF += FF_LEARN_RATE * speed_error
        FF  = clamp(FF, MAX_MOTOR_VEL)

    pid_correction = (
        KP * speed_error +
        KI * speed_integral +
        KD * speed_derivative
    )

    # boost phase: full power until BOOST_THRESHOLD fraction of target reached
    # PID phase: precise control once close to target
    if abs(target_speed) > 0.1:
        proximity = abs(actual_speed) / abs(target_speed)
        if proximity < BOOST_THRESHOLD:
            thrust = clamp(math.copysign(MAX_MOTOR_VEL * BOOST_POWER, target_speed), MAX_MOTOR_VEL)
        else:
            thrust = clamp(FF + pid_correction, MAX_MOTOR_VEL)
    else:
        thrust = clamp(FF + pid_correction, MAX_MOTOR_VEL)

    if key in (ord('W'), ord('S'), ord('A'), ord('D'), ord(' '), ord('P'), ord('O')):
        print(f"target: {target_speed:.2f}  actual: {actual_speed:.2f}  thrust: {thrust:.2f}  yaw: {yaw_input:.2f}  FF: {FF:.3f}")

    if robot.getTime() % 0.1 < dt:
        print(f"speed: {actual_speed:.3f} m/s  target: {target_speed:.2f}  thrust: {thrust:.2f}  FF: {FF:.3f}")

    # --- mixing ---
    v1_target = clamp(thrust,     MAX_MOTOR_VEL)
    v2_target = clamp(thrust,     MAX_MOTOR_VEL)
    v3_target = clamp( yaw_input, MAX_MOTOR_VEL)
    v4_target = clamp(-yaw_input, MAX_MOTOR_VEL)

    # --- low-pass smoothing ---
    v1 = motor1.getVelocity() * (1 - SMOOTHING) + v1_target * SMOOTHING
    v2 = motor2.getVelocity() * (1 - SMOOTHING) + v2_target * SMOOTHING
    v3 = motor3.getVelocity() * (1 - SMOOTHING) + v3_target * SMOOTHING
    v4 = motor4.getVelocity() * (1 - SMOOTHING) + v4_target * SMOOTHING

    motor1.setVelocity(clamp(v1, MAX_MOTOR_VEL))
    motor2.setVelocity(clamp(v2, MAX_MOTOR_VEL))
    motor3.setVelocity(clamp(v3, MAX_MOTOR_VEL))
    motor4.setVelocity(clamp(v4, MAX_MOTOR_VEL))