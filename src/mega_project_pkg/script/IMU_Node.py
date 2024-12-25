import rospy
from mpu6050 import mpu6050
from std_msgs.msg import Float64
from mega_project.msg import imu
import time

class  mpu_node:

	def __init__(self,address):
		rospy.init_node('imu_node' , anonymous=False)#create node
		self.pub=rospy.Publisher('/imu/data',imu,queue_size=10)
		self.pub_yaw = rospy.Publisher('/imu/yaw', Float64, queue_size=10)
		self.__address=address
		self.__mpuObject=mpu6050(address)
		self.yaw=0.0
		self.rate=rospy.Rate(10)


	def get_address(self):
		return self.__address
	
	def get_acceleration_data(self):
		acceleration_data=self.__mpuObject.get_accel_data()
		Acc_x,Acc_y,Acc_z=acceleration_data['x'],acceleration_data['y'],acceleration_data['z']

		return Acc_x,Acc_y,Acc_z

	def get_gyroscope_data(self):
		gyro_data=self.__mpuObject.get_gyro_data()

		angle_x,angle_y,angle_z=gyro_data['x'],gyro_data['y'],gyro_data['z']

		return angle_x,angle_y,angle_z
	
	def get_temperature(self):
		return self.__mpuObject.get_temp()

	def Publish_msg(self,msg):
		msg=imu()
		old_t=0.0
			
		while not rospy.is_shutdown():
			cur_t=time.time()
			delta_t=cur_t-old_t

			angle_x,angle_y,angle_z=self.get_gyroscope_data()
			Acc_x,Acc_y,Acc_z=self.get_acceleration_data()

			self.yaw+=angle_z*delta_t

			msg.angle_x=angle_x
			msg.angle_y=angle_y
			msg.angle_z=angle_z

			msg.Acc_x=Acc_x
			msg.Acc_y=Acc_y
			msg.Acc_z=Acc_z

			rospy.loginfo(msg) #debugging line to know if we entered this method or not
			self.pub.publish(msg)
			self.pub_yaw.publish(self.pub_yaw)
			self.rate.sleep()# to send the data with the rate we gave at the __init__ of the Publisher Node


if __name__=="__main__":

	try:
		node=mpu_node()
		node.Publish_msg()

	except rospy.ROSInterruptException:
		pass
