import ctypes as ct
import unittest

from a2c_sensor_tools.can_sensor_monitor import CAN_MSG_EXT
from a2c_sensor_tools.pcan_basic import (
    PCAN_API_VERSION,
    PCAN_BAUDRATES,
    PCAN_CHANNEL_AVAILABLE,
    PCAN_CHANNEL_CONDITION,
    PCAN_ERROR_OK,
    PCAN_ERROR_QRCVEMPTY,
    PCAN_MESSAGE_EXTENDED,
    PCAN_USBBUS_HANDLES,
    PcanBasic,
    TPCANMsg,
    TPCANTimestamp,
)


class _FakeFunction:
    def __init__(self, callback):
        self.callback = callback
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.callback(*args)


class _FakePcanDll:
    def __init__(self) -> None:
        self.initialized = []
        self.uninitialized = []
        self.writes = []
        self.read_items = []
        self.CAN_Initialize = _FakeFunction(self._initialize)
        self.CAN_Uninitialize = _FakeFunction(self._uninitialize)
        self.CAN_Read = _FakeFunction(self._read)
        self.CAN_Write = _FakeFunction(self._write)
        self.CAN_GetValue = _FakeFunction(self._get_value)
        self.CAN_GetErrorText = _FakeFunction(self._get_error_text)

    def _initialize(self, channel, bitrate, deprecated1, deprecated2, deprecated3):
        self.initialized.append((channel, bitrate, deprecated1, deprecated2, deprecated3))
        return PCAN_ERROR_OK

    def _uninitialize(self, channel):
        self.uninitialized.append(channel)
        return PCAN_ERROR_OK

    def _get_value(self, channel, parameter, buffer, length):
        if parameter == PCAN_API_VERSION:
            value = b"5.1.0.0\0"
            ct.memmove(buffer, value, min(length, len(value)))
            return PCAN_ERROR_OK
        if parameter == PCAN_CHANNEL_CONDITION:
            if channel != PCAN_USBBUS_HANDLES[0]:
                return 0x01400
            ct.cast(buffer, ct.POINTER(ct.c_uint32)).contents.value = PCAN_CHANNEL_AVAILABLE
            return PCAN_ERROR_OK
        return 0x04000

    def _get_error_text(self, status, language, buffer):
        del status, language
        ct.memmove(buffer, b"fake error\0", 11)
        return PCAN_ERROR_OK

    def _write(self, channel, message_pointer):
        message = ct.cast(message_pointer, ct.POINTER(TPCANMsg)).contents
        self.writes.append(
            (channel, message.ID, message.MSGTYPE, bytes(message.DATA[: message.LEN]))
        )
        return PCAN_ERROR_OK

    def _read(self, channel, message_pointer, timestamp_pointer):
        del channel
        if not self.read_items:
            return PCAN_ERROR_QRCVEMPTY
        can_id, message_type, data, millis = self.read_items.pop(0)
        message = ct.cast(message_pointer, ct.POINTER(TPCANMsg)).contents
        message.ID = can_id
        message.MSGTYPE = message_type
        message.LEN = len(data)
        for index, value in enumerate(data):
            message.DATA[index] = value
        timestamp = ct.cast(
            timestamp_pointer, ct.POINTER(TPCANTimestamp)
        ).contents
        timestamp.millis = millis
        return PCAN_ERROR_OK


class PcanBasicTests(unittest.TestCase):
    def setUp(self) -> None:
        self.dll = _FakePcanDll()
        self.api = PcanBasic(_dll=self.dll)

    def test_discovers_attached_usb_channel_and_api_version(self) -> None:
        self.assertEqual(self.api.api_version(), "5.1.0.0")
        channels = self.api.list_channels()
        self.assertEqual(len(channels), 1)
        self.assertEqual(channels[0].number, 0x51)
        self.assertEqual(channels[0].name, "PCAN_USBBUS1")

    def test_initializes_fixed_bitrate_writes_reads_and_releases(self) -> None:
        self.dll.read_items.append((0x18FF5012, PCAN_MESSAGE_EXTENDED, b"\x01\x02", 1234))
        with self.api.open_channel(0x51, 250_000) as channel:
            channel.write(0x3E8, b"\xEF\x04")
            frame = channel.read(0)

        self.assertEqual(self.dll.initialized, [(0x51, PCAN_BAUDRATES[250_000], 0, 0, 0)])
        self.assertEqual(self.dll.writes, [(0x51, 0x3E8, 0, b"\xEF\x04")])
        self.assertIsNotNone(frame)
        assert frame is not None
        self.assertEqual(frame.can_id, 0x18FF5012)
        self.assertEqual(frame.data, b"\x01\x02")
        self.assertTrue(frame.flags & CAN_MSG_EXT)
        self.assertEqual(frame.timestamp_ms, 1234)
        self.assertEqual(self.dll.uninitialized, [0x51])

    def test_empty_receive_queue_returns_none(self) -> None:
        with self.api.open_channel(0x51, 500_000) as channel:
            self.assertIsNone(channel.read(0))


if __name__ == "__main__":
    unittest.main()
