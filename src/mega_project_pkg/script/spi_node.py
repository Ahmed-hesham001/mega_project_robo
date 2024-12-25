#!/usr/bin/env python3

import rospy
import spidev
from std_msgs.msg import Bool

class SPINode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('spi_node', anonymous=False)

        # ROS publisher
        self.pub = rospy.Publisher('spi/boolean_flag', Bool, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Initialize SPI
        try:
            self.spi = spidev.SpiDev()  # Create SPI object
            self.spi.open(0, 0)        # Open bus 0, device 0
            self.spi.max_speed_hz = 1000000  # 1 MHz
            self.spi.mode = 0               # CPOL=0, CPHA=0
            rospy.loginfo("SPI device initialized successfully.")
        except IOError as e:
            rospy.logerr(f"SPI initialization failed: {e}")
            raise

    def get_flag(self):
        """Read a single byte from SPI and interpret it as a boolean."""
        try:
            data = self.spi.readbytes(1)
            return bool(data[0])
        except spidev.SpiDevError as e:
            rospy.logerr(f"SPI error: {e}")
            return False
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
            return False

    def publish_msg(self):
        """Continuously publish SPI boolean values."""
        rospy.loginfo("SPI Node started and publishing messages.")
        while not rospy.is_shutdown():
            boolean_value = self.get_flag()
            self.pub.publish(boolean_value)
            #rospy.loginfo(f"Published: {boolean_value}") # lao e7tagnaha
            self.rate.sleep()

    def close(self):
        """Close the SPI connection."""
        self.spi.close()
        rospy.loginfo("SPI device closed.")

def main():
    spi_node = SPINode()
    try:
        spi_node.publish_msg()
    except rospy.ROSInterruptException:
        pass
    finally:
        spi_node.close()

if __name__ == '__main__':
    main()