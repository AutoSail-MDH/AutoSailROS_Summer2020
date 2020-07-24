#!/usr/bin/env python
from as5048a import AS5048A
from periphery import SPI
import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse
import threading


def handle_write_zero_position(req):
    """
    A handle for the service to be able to handle write_zero_position

    :param req: The requester object which contains the input from the user
    :type req: object
    """
    lock.acquire()
    response = as5048a.write_zero_position()
    if as5048a.is_error:
        errors = as5048a.clear_and_get_error()
        lock.release()
        message = """Failed to write zero position, the following errors were encountered:
        Parity error: {}
        Command invalid: {}
        Framing error: {}""".format(**errors)
        return TriggerResponse(False, message)
    if as5048a.is_parity_mismatch:
        lock.release()
        message = "Failed to write zero position, the received package has a mismatched parity"
        return TriggerResponse(False, message)
    lock.release()
    old = AS5048A.calc_angle(response['old_zero_position'])
    new = AS5048A.calc_angle(response['new_zero_position'])
    message = """Successfully wrote a new zero position
    Old Zero Position: {} rad
    New Zero Position: {} rad""".format(old, new)
    return TriggerResponse(True, message)


def handle_read_diagnostics(req):
    """
    A handle for the service to be able to handle read_diagnostics

    :param req: The requester object which contains the input from the user
    :type req: Object
    """
    lock.acquire()
    response = as5048a.read_diagnostics()
    if as5048a.is_error:
        errors = as5048a.clear_and_get_error()
        lock.release()
        message = """Failed to read diagnostics, the following errors were encountered:
        Parity error: {}
        Command invalid: {}
        Framing error: {}""".format(**errors)
        return TriggerResponse(False, message)
    if as5048a.is_parity_mismatch:
        lock.release()
        message = "Failed to read diagnostics, the received package has a mismatched parity"
        return TriggerResponse(False, message)
    lock.release()
    message = """Diagnostics and Automatic Gain Control (AGC)
    Comp High: {comp_hi}
    Comp Low: {comp_lo}
    CORDIC OverFlow (COF): {cof}
    Offset Compensation Finished (OCF): {ocf}
    Automatic Gain Control value (AGC value): {agc_val}""".format(**response)
    return TriggerResponse(True, message)


if __name__ == "__main__":
    lock = threading.Lock()

    spi = SPI("/dev/spidev0.0", 1, 1000000)
    as5048a = AS5048A(spi)
    # Setup ROS
    rospy.init_node('as5048a', anonymous=False)
    pub = rospy.Publisher('angle', Float32, queue_size=10)
    s1 = rospy.Service('windvane_write_zero_position', Trigger, handle_write_zero_position)
    s2 = rospy.Service('windvane_read_diagnostics', Trigger, handle_read_diagnostics)
    rate = rospy.Rate(10)  # 10Hz

    errors = as5048a.clear_error()
    if any(errors.values()):
        rospy.loginfo('Clear Error; Parity Error {parity_error}, Command Invalid {command_invalid}, Framing Error {framing_error}'.format(**errors))

    while not rospy.is_shutdown():
        lock.acquire()
        angle = as5048a.read_angle()
        pub.publish(angle)
        lock.release()
        rospy.loginfo('Angle (rad): {}'.format(angle))

        rate.sleep()
    del as5048a
