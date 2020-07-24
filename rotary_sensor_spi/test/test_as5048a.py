#!/usr/bin/env python
from unittest import TestCase, main
import math

from rotary_sensor_spi.scripts.as5048a import AS5048A


class TestAS5048A(TestCase):
    def test_check_parity_error(self):
        self.assertTrue(AS5048A.check_parity_error(0b1001001001010000))
        self.assertTrue(AS5048A.check_parity_error(0b0010110101100100))
        self.assertFalse(AS5048A.check_parity_error(0b1000010001010000))
        self.assertFalse(AS5048A.check_parity_error(0b0000110101100001))

    def test_check_error_flag(self):
        self.assertTrue(AS5048A.check_error_flag(0b0100100101100001))
        self.assertFalse(AS5048A.check_error_flag(0b0000100101100001))

    def test_calc_even_parity(self):
        self.assertEqual(AS5048A.calc_even_parity(0b100100101100001), 0)
        self.assertEqual(AS5048A.calc_even_parity(0b010110101000100), 0)
        self.assertEqual(AS5048A.calc_even_parity(0b000100101100001), 1)
        self.assertEqual(AS5048A.calc_even_parity(0b011111101000011), 1)

    def test_calc_angle(self):
        self.assertEqual(AS5048A.calc_angle(0x4000), 2*math.pi)
        self.assertEqual(AS5048A.calc_angle(0), 0)
        self.assertEqual(AS5048A.calc_angle(0x2000), math.pi)

    def test_transfer(self):
        self.assertEqual()

    def test_read(self):
        self.fail()

    def test_write(self):
        self.fail()

    def test_read_angle(self):
        self.fail()

    def test_clear_and_get_error(self):
        self.fail()

    def test_write_zero_position(self):
        self.fail()

    def test_read_diagnostics(self):
        self.fail()


if __name__ == "__main__":
    main()