#!/usr/bin/env python3

import rospy
import spidev
from std_msgs.msg import Bool

class SPINode:
    def __init__(self):
        rospy.init_node('spi_node', anonymous=False)
        
        # Configuration parameters
        self.bus = rospy.get_param('~spi_bus', 0)
        self.device = rospy.get_param('~spi_device', 0)
        self.speed = rospy.get_param('~spi_speed', 1000000)
        self.rate_hz = rospy.get_param('~rate', 10)
        
        # ROS publisher
        self.pub = rospy.Publisher('spi/boolean_flag', Bool, queue_size=10)
        self.rate = rospy.Rate(self.rate_hz)
        
        # Initialize SPI
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.bus, self.device)
            self.spi.max_speed_hz = self.speed
            self.spi.mode = 0
            self._configure_spi()
            rospy.loginfo("SPI device initialized: bus=%d, device=%d, speed=%d", 
                         self.bus, self.device, self.speed)
        except IOError as e:
            rospy.logerr("SPI initialization failed: %s", e)
            raise

    def _configure_spi(self):
        """Additional SPI configuration settings"""
        self.spi.bits_per_word = 8
        self.spi.lsbfirst = False
        self.spi.threewire = False

    def get_flag(self):
        try:
            # Send dummy byte to receive data
            data = self.spi.xfer2([0x00])
            return bool(data[0])
        except (spidev.SpiDevError, IOError) as e:
            rospy.logerr("SPI read error: %s", e)
            return False
        except Exception as e:
            rospy.logerr("Unexpected error: %s", e)
            return False

    def publish_msg(self):
        rospy.loginfo("Starting SPI message publication at %d Hz", self.rate_hz)
        last_value = None
        
        while not rospy.is_shutdown():
            try:
                boolean_value = self.get_flag()
                
                # Only publish if value changed
                if boolean_value != last_value:
                    self.pub.publish(boolean_value)
                    rospy.logdebug("Value changed: %s", boolean_value)
                    last_value = boolean_value
                    
                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

    def close(self):
        try:
            self.spi.close()
            rospy.loginfo("SPI device closed successfully")
        except Exception as e:
            rospy.logerr("Error closing SPI device: %s", e)

def main():
    spi_node = None
    try:
        spi_node = SPINode()
        spi_node.publish_msg()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr("Node failed: %s", e)
    finally:
        if spi_node:
            spi_node.close()

if __name__ == '__main__':
    main()