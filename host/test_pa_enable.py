"""PA capability negotiation: legacy firmware, arming and rejected commands."""
import unittest
from unittest.mock import patch

from esp_link import Link


class SerialFirmware:
    def __init__(self, capability=True, reply=None):
        self.capability = capability
        self.reply = reply
        self.pending = b''
        self.writes = []

    def write(self, data):
        self.writes.append(data)
        if b'INFO' in data:
            self.pending += b'ESP32DATV 1' + (b' PA_GPIO 3' if self.capability else b'') + b'\r\n'
        elif data.startswith(b'PA '):
            self.pending += self.reply if self.reply is not None else b'OK ' + data.strip() + b' GPIO 3\r\n'

    def read(self, size):
        result, self.pending = self.pending[:size], self.pending[size:]
        return result

    def reset_input_buffer(self):
        self.pending = b''


class PaProtocolTests(unittest.TestCase):
    def link(self, firmware):
        with patch('esp_link.serial.Serial', return_value=firmware), patch('esp_link.time.sleep'):
            return Link('fake-port')

    def test_new_firmware_receives_explicit_on_and_off(self):
        firmware = SerialFirmware()
        link = self.link(firmware)
        self.assertEqual(link.pa_gpio, 3)
        link.configure_pa(True)
        link.configure_pa(False)
        self.assertEqual(firmware.writes[-2:], [b'PA 1\n', b'PA 0\n'])
        self.assertEqual(link.text, b'')

    def test_legacy_firmware_only_allows_pa_off(self):
        firmware = SerialFirmware(capability=False)
        link = self.link(firmware)
        link.configure_pa(False)
        with self.assertRaisesRegex(SystemExit, 'no PA enable support'):
            link.configure_pa(True)
        self.assertEqual(firmware.writes, [b'\nINFO\n'])

    def test_firmware_rejection_and_wrong_gpio_are_not_silently_accepted(self):
        link = self.link(SerialFirmware(reply=b'ERR PA\r\n'))
        with self.assertRaisesRegex(SystemExit, 'ESP: ERR PA'):
            link.configure_pa(True)
        link = self.link(SerialFirmware(reply=b'OK PA 1 GPIO 8\r\n'))
        with self.assertRaisesRegex(SystemExit, 'did not acknowledge'):
            link.configure_pa(True, timeout=.02)


if __name__ == '__main__':
    unittest.main()
