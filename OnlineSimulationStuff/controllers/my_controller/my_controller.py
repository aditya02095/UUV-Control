"""my_controller controller (FIXED)."""

from controller import Robot, Keyboard

# Create the Robot instance
robot = Robot()

# Get simulation timestep
timestep = int(robot.getBasicTimeStep())

# Enable keyboard (MUST match timestep)
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# Get motors (NAMES MUST MATCH .wbt FILE EXACTLY)
yawMotor   = robot.getMotor('yaw motor')
swayMotor  = robot.getMotor('sway motor')
surgeMotor = robot.getMotor('surge motor')
heaveMotor = robot.getMotor('heave motor')

# Set motors to velocity control mode
for motor in [yawMotor, swayMotor, surgeMotor, heaveMotor]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# Velocity variables
yawVel   = 0.0
swayVel  = 0.0
surgeVel = 0.0
heaveVel = 0.0

# Constants
VEL_STEP = 0.25
MAX_VEL  = 5.0

# Helper function to clamp velocity
def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))

# Instructions
print('"UP"    : Yaw +')
print('"DOWN"  : Yaw -')
print('"LEFT"  : Sway +')
print('"RIGHT" : Sway -')
print('"W"     : Heave +')
print('"S"     : Heave -')
print('"A"     : Surge +')
print('"D"     : Surge -')

# Main loop
while robot.step(timestep) != -1:

    key = keyboard.getKey()
    while key != -1:

        if key == Keyboard.UP:
            yawVel += VEL_STEP
            print(f"Yaw + | {yawVel:.2f}")

        elif key == Keyboard.DOWN:
            yawVel -= VEL_STEP
            print(f"Yaw - | {yawVel:.2f}")

        elif key == Keyboard.LEFT:
            swayVel += VEL_STEP
            print(f"Sway + | {swayVel:.2f}")

        elif key == Keyboard.RIGHT:
            swayVel -= VEL_STEP
            print(f"Sway - | {swayVel:.2f}")

        elif key == ord('W') or key == ord('w'):
            heaveVel += VEL_STEP
            print(f"Heave + | {heaveVel:.2f}")

        elif key == ord('S') or key == ord('s'):
            heaveVel -= VEL_STEP
            print(f"Heave - | {heaveVel:.2f}")

        elif key == ord('A') or key == ord('a'):
            surgeVel += VEL_STEP
            print(f"Surge + | {surgeVel:.2f}")

        elif key == ord('D') or key == ord('d'):
            surgeVel -= VEL_STEP
            print(f"Surge - | {surgeVel:.2f}")

        key = keyboard.getKey()

    # Clamp velocities
    yawVel   = clamp(yawVel,   -MAX_VEL, MAX_VEL)
    swayVel  = clamp(swayVel,  -MAX_VEL, MAX_VEL)
    surgeVel = clamp(surgeVel, -MAX_VEL, MAX_VEL)
    heaveVel = clamp(heaveVel, -MAX_VEL, MAX_VEL)

    # Apply velocities
    yawMotor.setVelocity(yawVel)
    swayMotor.setVelocity(swayVel)
    surgeMotor.setVelocity(surgeVel)
    heaveMotor.setVelocity(heaveVel)