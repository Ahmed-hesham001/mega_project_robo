import RPi.GPIO as GPIO
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
import time

class EncoderNode:
    def __init__(self,countsPerRev,encoder_pins,wheel_radius):
        self.pins=encoder_pins
        self.radius=wheel_radius
        self.counts_per_rev=countsPerRev
        self.yaw=0.0
        self.x=0.0
        self.y=0.0
        self.counts=0

        rospy.init_node('encoder_node',anonymous=False)
        self.pub_odom=rospy.Publisher('/odom',Odometry,queue_size=10)
        rospy.Subscriber('/imu/yaw',Float64,self.callback_for_yaw)

        GPIO.setmode(GPIO.BCM)

        for pin in self.pins:
            GPIO.setup(pin,GPIO.IN)
            GPIO.add_event_detect(pin,GPIO.HIGH,bouncetime=2,callback=self.inc_count)

    def inc_count(self):
        self.counts+=1

    def get_distance(self):
        circumference = 2 * math.pi * self.radius
        return (self.counts / self.counts_per_rev) * circumference

    def update_odom(self,msg):
        distance=self.get_distance()

        self.yaw = math.radians(msg.data)  # Convert yaw to radians

        self.x=distance * math.cos(self.yaw)
        self.y=distance * math.sin(self.yaw)

        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = Quaternion()  # Replace with real orientation
        rospy.loginfo(f"Odometry Published: x={self.x}, y={self.y}")
        self.pub_odom.publish(odom_msg)

    def spin():
        rospy.spin()

    def cleanup():
        GPIO.cleanup()


if __name__=="__main__":
    try:
        node=EncoderNode(99,(12,13,14),13)
        EncoderNode.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        # Clean All GPIO PIns
        node.cleanup()
        pass