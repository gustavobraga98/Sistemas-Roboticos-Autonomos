from controller import Robot
from controller import Camera
import numpy as np

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
    camera = robot.getDevice("camera")
    camera.enable(1)

    def set_robot_speed(sides = [0,0]):
            for motor in left_side:
                motor.setVelocity(sides[0])
            for motor in right_side:
                motor.setVelocity(sides[1])
        
    def getImage():
        image = camera.getImageArray()
        image = np.array(image)
        return(image)
    
while robot.step(timestep) != -1:
    set_robot_speed([1,1])
    image = getImage()
    print(image.shape)