#!/usr/bin/env python
from __future__ import division # Fix python 2 division problem
from periphery import SPI
import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse
import math
import threading


class ParityMismatchException(Exception):
    pass


class ErrorFlagException(Exception):
    pass


class AS5048A:

    ADDR_NOP = 0x0000
    ADDR_ANGLE = 0x3FFF
    ADDR_CLEAR_ERROR_FLAG = 0x0001
    ADDR_ZERO_POSITION_HI = 0x0016
    ADDR_ZERO_POSITION_LO = 0x0017
    ADDR_DIAGNOSTICS = 0x3FFD

    def __init__(self):
        rospy.loginfo('Opening SPI connection for AS5048A')
        self.spi = SPI("/dev/spidev0.0", 1, 1000000)

    def __del__(self):
        rospy.loginfo('Closing SPI connection for AS5048A')
        self.spi.close()

    @staticmethod
    def check_parity_error(response):
        """
        Check the response message if the parity is wrong or not

        @param response: The 16bit address response
        @type response: 16bit number
        @return: True if parity is wrong, else false
        @rtype: boolean
        """
        given_parity = response >> 15
        real_parity = AS5048A.calc_even_parity(response & 0x7fff)
        return given_parity != real_parity

    @staticmethod
    def check_error_flag(response):
        """
        Returns a boolean depending on if the error flag is set

        @param response: The 16bit address response
        @type response: 16bit number
        @return: True if error flag is set, else false
        @rtype: boolean
        """
        return (response >> 14) & 0x1

    @staticmethod
    def calc_even_parity(addr):
        """
        Return 1 or 0 depending on if the given address is even or not.
        Used to calculate the even parity

        @param addr: Address to check
        @type addr: 16bit number
        @return: 1 if even, else 0
        @rtype: number boolean
        """
        for i in [8, 4, 2, 1]:
            addr ^= addr >> i
        return addr & 1

    @staticmethod
    def calc_angle(value):
        """
        Calculates the angle in radians with atan2 which makes the angle between [180, -180] and then returns it

        @param value: 14 bit angle value
        @type value: 16bit number
        @return: Angle in radians between [180, -180]
        @rtype: number
        """
        rad = value * (math.pi / 0x2000)
        return math.atan2(math.sin(rad), math.cos(rad)) * 180 / math.pi

    def transfer(self, addr):
        """
        Sends an address and received the response from previous transfer.
        Returns the response.

        @param addr: Address to send
        @type addr: 16bit number
        @return: The bits received
        @rtype: 16bit number
        """
        addr |= self.calc_even_parity(addr) << 15  # Set parity at the 16th bit
        to_send = [(addr >> 8) & 0xff, addr & 0xff]  # Split bytes into list
        rospy.logdebug('Sending [0x{:02x}, 0x{:02x}]'.format(*to_send))
        response = self.spi.transfer(to_send)
        rospy.logdebug('Received [0x{:02x}, 0x{:02x}]'.format(*response))
        r = (response[0] << 8) | response[1]  # Concatenate the bytes

        # Verify parity and check error flag, raise exception if an error is detected
        if self.check_error_flag(r):
            raise ErrorFlagException('Error flag set')
        elif self.check_parity_error(r):
            raise ParityMismatchException('Parity bit mismatch')
        return r

    def read(self, addr):
        """
        Prepares the given address by setting the read bit to 1.
        Transfers the prepared package, received the response and check it for error.

        @param addr: The 14bit address to send
        @type addr: 14bit number
        @return: The 14bit data value from the response
        @rtype: 14bit number
        """
        addr |= 0x4000  # Apply read bit to address
        # Transfer the read address and then a NOP to get the result of the previous read.
        try:
            self.transfer(addr)
            result = self.transfer(self.ADDR_NOP)
        except (ParityMismatchException, ErrorFlagException) as err:
            raise
        value = result & 0x3fff  # Extract data
        return value

    def write(self, addr, data):
        """
        Send address as a write command, skip the first response. Send the data and receive the old register,
        and finally send a NOP package to receive the new register.

        @param addr: The 14bit address to send
        @type addr: 14bit number
        @param data: Data bits to write
        @type data: number
        @return: The old and the new register as an dictionary
        @rtype: dictionary
        """
        try:
            self.transfer(addr)
        except (ParityMismatchException, ErrorFlagException) as err:
            raise
        # First transfer will return the old register, and the second the new register
        old_register = self.transfer(data)
        new_register = self.transfer(self.ADDR_NOP)
        return {'old_register': old_register, 'new_register': new_register}

    def read_angle(self):
        """
        Send the angle address as a read, return the response by calculating the angle.
        If the transfer fails, it will try to clear the error and resend

        @return: The angle in degrees
        @rtype: number
        """
        addr = self.ADDR_ANGLE
        # Tries to resend if an exception is caught
        for tries in range(0, 2):
            try:
                value = self.read(addr)
            except (ParityMismatchException, ErrorFlagException) as err:
                rospy.logerr(err)
                rospy.loginfo('Retrying...')
                errors = self.clear_error()  # Clear errors before retrying
                if any(errors.values()):
                    rospy.loginfo('Clear Error; Parity Error {parity_error}, Command Invalid {command_invalid}, Framing Error {framing_error}'.format(**errors))
                # Pass on the exception if the retry failed also, else continue
                if tries == 1:
                    raise
                else:
                    continue
            else:
                break
        return self.calc_angle(value)

    def clear_error(self):
        """
        Send a clear error signal and receive what kind of error was lit.

        @return: A dictionary containing the different type of errors
        @rtype: dictionary
        """
        addr = self.ADDR_CLEAR_ERROR_FLAG
        response = self.read(addr)
        # Extract error bits
        parity_error = (response >> 2) & 0x1  # Extract parity error bit
        command_invalid = (response >> 1) & 0x1  # Extract command invalid bit
        framing_error = response & 0x1  # Extract framing error bit
        return {'parity_error': parity_error, 'command_invalid': command_invalid, 'framing_error': framing_error}

    def write_zero_position(self):
        """
        Write a new zero position by sending both high and low bits. Received a response containing
        both old and new register.
        """
        response = self.read(self.ADDR_ANGLE)
        response_old_hi = self.read(self.ADDR_ZERO_POSITION_HI)
        response_old_lo = self.read(self.ADDR_ZERO_POSITION_LO)
        old = (response_old_hi << 6) | response_old_lo
        response = (response + old) % 0x4000

        # Extract zero position for high and low bits
        zero_position_hi = response >> 6
        zero_position_lo = response & 0b111111
        response_hi = self.write(self.ADDR_ZERO_POSITION_HI, zero_position_hi)
        response_lo = self.write(self.ADDR_ZERO_POSITION_LO, zero_position_lo)
        old = (response_hi['old_register'] << 6) | response_lo['old_register']
        new = (response_hi['new_register'] << 6) | response_lo['new_register']
        message = 'Wrote new zero position; old register 0x{:04x}, new register 0x{:04x}'.format(old, new)
        rospy.loginfo(message)
        return {'old_zero_position': old, 'new_zero_position': new}

    def read_diagnostics(self):
        """
        Sends a request to read the diagnostics of the sensor. Returns the diagnostics in a dictionary where
            - comp_high: Indicates a high magnetic field
            - comp_low: indiciates a weak magnetic field
            - cof: Bit is set if the CORDIC part indicated an out of range error,
                   which makes the angle and magnitude data invalid.
                   The last valid angular value will be used as the absolute output
            - ocf: Bit is set when the Offset Compensation Algorithm has finished. Always high after power up
            - agc_val: Represents the magnetic field value where 0 is high and 255 is low

        @return: The diagnostics
        @rtype: dictionary
        """
        addr = self.ADDR_DIAGNOSTICS
        response = self.read(addr)
        # Extract diagnostic variables
        comp_hi = (response >> 11) & 1
        comp_lo = (response >> 10) & 1
        cof = (response >> 9) & 1
        ocf = (response >> 8) & 1
        agc_val = response & 0xFF
        return {'comp_hi': comp_hi, 'comp_lo': comp_lo, 'cof': cof, 'ocf': ocf, 'agc_val': agc_val}


def handle_write_zero_position(req):
    """
    A handle for the service to be able to handle write_zero_position

    @param req: The requester object which contains the input from the user
    @type req: object
    """
    lock.acquire()
    response = as5048a.write_zero_position()
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

    @param req: The requester object which contains the input from the user
    @type req: object
    """
    lock.acquire()
    response = as5048a.read_diagnostics()
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

    as5048a = AS5048A()
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
