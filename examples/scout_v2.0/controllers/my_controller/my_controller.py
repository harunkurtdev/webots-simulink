from controller import Robot, Keyboard

TIME_STEP = 64

# Create the Robot instance
robot = Robot()

# Get time step
time_step = int(robot.getBasicTimeStep())

# Initialize motors
wheel_names = ['front_right_wheel', 'front_left_wheel', 'rear_left_wheel', 'rear_right_wheel']
wheels = []

for name in wheel_names:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))  # Velocity control mode
    motor.setVelocity(0.0)
    wheels.append(motor)

# Enable keyboard
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# Movement speeds
MAX_SPEED = 3.0

# Main control loop
while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()

    # Default speeds (stop)
    left_speed = 0.0
    right_speed = 0.0

    # Check key pressed
    if key == ord('W'):  # forward
    
        left_speed = -MAX_SPEED
        right_speed = MAX_SPEED
    elif key == ord('S'):  # backward
    
        left_speed = MAX_SPEED
        right_speed = -MAX_SPEED
    elif key == ord('A'):  # turn left
    
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
    elif key == ord('D'):  # turn right
    
        left_speed = -MAX_SPEED
        right_speed = -MAX_SPEED

    # Apply speeds to wheels
    wheels[0].setVelocity(right_speed)  # front right
    wheels[1].setVelocity(left_speed)   # front left
    wheels[2].setVelocity(left_speed)   # rear left
    wheels[3].setVelocity(right_speed)  # rear right
