from controller import Robot, Keyboard

robot = Robot()
timestep = int(robot.getBasicTimeStep())

rot_motor = robot.getDevice("rotational_motor")
rot_sensor = robot.getDevice("position_sensor")

propeller = robot.getDevice("rotational_motor_propeller")


propeller.setPosition(float('inf'))

rot_sensor.enable(timestep)

rot_motor.setVelocity(1.0) 
rot_motor.setPosition(0.0) 


keyboard = robot.getKeyboard()
keyboard.enable(timestep)

thrust = 0.0
rot_angle = 0.0   

print("Kontroller: W=Thrust+, S=Thrust-, A=Yaw Left, D=Yaw Right")

while robot.step(timestep) != -1:
    key = keyboard.getKey()

    if key == ord('W'):
        thrust += 0.1
    elif key == ord('S'):
        thrust -= 0.1
    elif key == ord('A'):
        rot_angle += 0.01   
    elif key == ord('D'):
        rot_angle -= 0.01   

    if thrust < 0:
        thrust = 0
    if thrust > 4:
        thrust = 4
    if rot_angle > 0.5:   
        rot_angle = 0.5
    if rot_angle < -0.5:
        rot_angle = -0.5

    rot_motor.setPosition(rot_angle)

    propeller.setVelocity(thrust)

    pos_val = rot_sensor.getValue()
    print(f"Angle: {pos_val:.2f} rad | Thrust: {thrust:.2f} N")
