import rospy
from mpu6050 import mpu6050
import time
from mega_project.msg import imu




class  mpu_node:

	def __init__(self,address):
		rospy.init_node('imu_node' , anonymous=False)#create node
		self.pub=rospy.Publisher("imu/data",imu,self.callback)
		self.__address=address
		self.__mpuObject=mpu6050(address)

	def get_address(self):
		return self.__address
	
	def get_acceleration_data(self):
		acceleration_data=self.__mpuObject.get_accel_data()
		Acc_x,Acc_y,Acc_z=acceleration_data['x'],acceleration_data['y'],acceleration_data['z']

		return Acc_x,Acc_y,Acc_z

	def get_gyroscope_data(self):
		gyro_data=self.__mpuObject.get_gyro_data()

		angle_x,angel_y,angle_z=gyro_data['x'],gyro_data['y'],gyro_data['z']

		return angle_x,angel_y,angle_z
	
	def get_temperature(self):
		return self.__mpuObject.get_temp()
		

if __name__=="__main__":

	pass
