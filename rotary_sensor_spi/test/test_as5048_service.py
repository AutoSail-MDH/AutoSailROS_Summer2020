#!/usr/bin/env python
import sys
import rospy
from unittest import TestCase
from unittest.mock import patch
from std_srvs.srv import Trigger

PKG = 'rotary_sensor_'
NAME = 'test_as5048_service'

class TestAS5048Service(TestCase):

    @patch('rotary_sensor_spi.as5048.SPI')
    def test_write_zero_position_service(self, mock_spi):
        mock_spi.transfer.return_value = [0, 0]
        rospy.wait_for_service('windvane_write_zero_position')
        write_zero_position = rospy.ServiceProxy('windvane_write_zero_position', Trigger)
        response = write_zero_position()
        self.assertTrue(response.success)
        self.assertTrue(response.message.startswith('Successfully '))
        self.assertEqual(mock_spi.transfer.call_count, 12)

        mock_spi.reset_mock()
        mock_spi.transfer.return_value = [0xff, 0xff]
        write_zero_position = rospy.ServiceProxy('windvane_write_zero_position', Trigger)
        response = write_zero_position()
        self.assertFalse(response.success)
        self.assertTrue(response.message.startswith('Failed '))
        self.assertEqual(mock_spi.transfer.call_count, 12)

    @patch('rotary_sensor_spi.as5048.SPI')
    def test_read_diagnostics_service(self, mock_spi):
        mock_spi.transfer.return_value = [0, 0]
        rospy.wait_for_service('windvane_read_diagnostics')
        read_diagnostics = rospy.ServiceProxy('windvane_read_diagnostics', Trigger)
        response = read_diagnostics()
        self.assertTrue(response.success)
        self.assertTrue(response.message.startswith('Diagnostics '))
        self.assertEqual(mock_spi.transfer.call_count, 4)

        mock_spi.reset_mock()
        mock_spi.transfer.return_value = [0xff, 0xff]
        rospy.wait_for_service('windvane_read_diagnostics')
        read_diagnostics = rospy.ServiceProxy('windvane_read_diagnostics', Trigger)
        response = read_diagnostics()
        self.assertFalse(response.success)
        self.assertTrue(response.message.startswith('Failed '))
        self.assertEqual(mock_spi.transfer.call_count, 4)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestCase, sys.argv)