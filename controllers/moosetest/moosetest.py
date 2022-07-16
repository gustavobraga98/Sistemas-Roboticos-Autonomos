from controller import Robot
from controller import Camera
import numpy as np
import cv2
import time

if __name__ == "__main__":
    robot = Robot()
    
    timestep = 64
    max_speed = 7.0
    set_speed = 7.0
    Centro = 32
    margem = 4
    
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

    def threshold_image(image):
        image = np.uint8(image)
        image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        ret,image = cv2.threshold(image,150,255,cv2.THRESH_BINARY)
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        image = cv2.flip(image,1)
        contours,hierarchy = cv2.findContours(image,1,cv2.CHAIN_APPROX_NONE)
        return(image,contours)

    def keep_line(image,contours):
        if len(contours)>0:
            c = max(contours,key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00']!=0:
                cx=int(M['m10']/M['m00'])
                cy=int(M['m01']/M['m00'])
                if cx>=Centro+margem:
                    set_robot_speed([set_speed,0])
                if cx <Centro+margem and cx > Centro-margem:
                    set_robot_speed([max_speed,max_speed])
                if cx <=Centro-margem:
                    set_robot_speed([0,set_speed])
        image = cv2.circle(image,(cx,cy),5,(0,255,0),-1)
        cv2.imshow("image",image)
        cv2.waitKey(1)
                    
    set_robot_speed([0,0])
while robot.step(timestep) != -1:
    image = getImage()
    image,contours = threshold_image(image)
    try:
        keep_line(image,contours)
    except:
        print("test")
    timestep=64