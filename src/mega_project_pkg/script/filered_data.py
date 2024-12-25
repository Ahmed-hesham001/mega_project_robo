import rospy
from nav_msgs.msg import Odometry
from ..msg import positioning
import math

class published_pos:

    def __init__(self):
    
        rospy.init_node('filtered_data',anonymous=False)

        self.__position_publisher=rospy.Publisher('/webtoros',positioning,queue_size=10)

        self.__getting_filtered_data=rospy.Subscriber('/odometry/filtered',Odometry,self.callback)

        self.__actual_x=0.0
        self.__actual_y=0.0
        self.__actual_z=0.0

    def callback(self,msg):
        # Extract position data
        self.__actual_x = msg.pose.pose.position.x
        self.__actual_y = msg.pose.pose.position.y
        self.__actual_z = msg.pose.pose.position.z

        # Extract orientation as a quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        roll,pitch,yaw=self.quaternion_to_euler(qx,qy,qz)

        final_data=positioning()

        final_data.x=self.__actual_x
        final_data.y=self.__actual_y
        final_data.z=self.__actual_z

        final_data.roll=roll
        final_data.pitch=pitch
        final_data.yaw=yaw

        self.__position_publisher.publish(final_data)

    def quaternion_to_euler(qx, qy, qz, qw):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw).

        Parameters:
            qx, qy, qz, qw: float
                Components of the quaternion.

        Returns:
            tuple: (roll, pitch, yaw) in radians
        """
        # Roll (x-axis rotation)
        t0 = +2.0 * (qw * qx + qy * qz)
        t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (qw * qy - qz * qx)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (qw * qz + qx * qy)
        t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def spin():
        rospy.spin()

if __name__=="__main__":
    try:
        node=published_pos()
        published_pos.spin()

    except rospy.ROSInterruptException:
        pass
   