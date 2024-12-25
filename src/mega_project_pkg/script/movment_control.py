#!/usr/bin/env python3
import rospy
from mega_project_pkg.msg import controller
import mdds10
import math

DEADZONE = 5
class movment_control:
    def __init__(self):
        rospy.init_node('movment_control' , anonymous=False)#create node
        self.sub = rospy.Subscriber('webtoros' , controller , self.callback)#create subscriber
        self.__motor_driver1 = mdds10(11,12,13,14)#create motor driver object
        self.motor_driver2 = mdds10(15,16,17,18)#create motor driver object
        self.LX = 0 
        self.LY = 0 

    def move_robot(self) :
        """
        callback function for subscriber
        msg is a controller message
        The function converts the message to a motor speed and direction command
        and sends it to the motor drivers
        """
        if (DEADZONE<self.LY<100 and self.LX<DEADZONE):
            self.__motor_driver1.move_forward(self.LY,self.LY)
        elif (-100<self.LY < -DEADZONE and self.LX > -DEADZONE):
            self.__motor_driver1.move_backward(self.LY,self.LY)
        elif (DEADZONE<self.LX<100 and self.LY<DEADZONE):
            self.__motor_driver1.move_right(self.LX,self.LX)
        elif (-DEADZONE>self.LX>-100 and self.LY > -DEADZONE):
            self.__motor_driver1.move_left(self.LX,self.LX)
        else :
            speed = math.sqrt((abs(self.LY)**2 + abs(self.LY)**2))
            if (self.LX>DEADZONE):
                self.__motor_driver1.move_right(speed,speed)
            elif(self.LX<-DEADZONE):
                self.__motor_driver1.move_left(speed,speed)
            else :
                self.__motor_driver1.stop()


        
        
    def callback(self , msg):
        self.LX = msg.L3X*100
        self.LY = msg.L3Y*100
        
            
            
            
    def spin(self):
        rospy.spin()
    
if __name__ == "__main__":
    try:
        movment_control = movment_control()
        movment_control.move_robot() 
        movment_control.spin()
    except rospy.ROSInitException:
        pass
    finally:
        movment_control.motor_driver.cleanup()