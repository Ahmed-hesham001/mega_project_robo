#!/usr/bin/env python3

import rospy
import spidev
from std_msgs.msg import Int32

class SPINode:
    def __init__(self):
        rospy.init_node('spi_node', anonymous=False)
        
        # Configuration parameters
        self.bus = rospy.get_param('~spi_bus', 0)
        self.device = rospy.get_param('~spi_device', 0)
        self.speed = rospy.get_param('~spi_speed', 1000000)
        self.rate_hz = rospy.get_param('~rate', 10)
        
        # ROS publisher
        self.int_pub = rospy.Publisher('rostoweb/mine', Int32, queue_size=10)
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

    def read_spi_data(self):
        try:
            # Send dummy byte to receive data
            data = self.spi.xfer2([0x00])
            return data[0]  # Return single byte as integer
            
        except (spidev.SpiDevError, IOError) as e:
            rospy.logerr("SPI read error: %s", e)
            return 0
        except Exception as e:
            rospy.logerr("Unexpected error: %s", e)
            return 0

    def publish_msg(self):
        rospy.loginfo("Starting SPI message publication at %d Hz", self.rate_hz)
        last_int = None
        
        while not rospy.is_shutdown():
            try:
                int_value = self.read_spi_data()
                
                # Publish integer value if changed
                if int_value != last_int:
                    self.int_pub.publish(int_value)
                    rospy.logdebug("Integer value changed: %d", int_value)
                    last_int = int_value
                    
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