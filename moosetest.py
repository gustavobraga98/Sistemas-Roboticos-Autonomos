from controller import Robot


if __name__ == "__main__":

    robot = Robot()
    
    timestep = 64
    max_speed = 7.0
    
    left_motor1 = robot.getDevice('left motor 1')
    left_motor2 = robot.getDevice('left motor 2')
    left_motor3 = robot.getDevice('left motor 3')
    left_motor4 = robot.getDevice('left motor 4')
    left_side = [left_motor1, left_motor2,left_motor3,left_motor4]
    right_motor1 = robot.getDevice('right motor 1')
    right_motor2 = robot.getDevice('right motor 2')
    right_motor3 = robot.getDevice('right motor 3')
    right_motor4 = robot.getDevice('right motor 4')
    right_side = [right_motor1, right_motor2,right_motor3,right_motor4]
    for motor in left_side:
        motor.setPosition(float('inf'))
    for motor in right_side:
        motor.setPosition(float('inf'))

    def set_robot_speed(sides = [0,0], velocity = 0.0):
        if sides == [1,0]:
            for motor in left_side:
                motor.setVelocity(velocity)
        if sides == [1,1]:
            for motor in left_side:
                motor.setVelocity(velocity)
            for motor in right_side:
                motor.setVelocity(velocity)
        
    
while robot.step(timestep) != -1:
    set_robot_speed([1,0],max_speed)