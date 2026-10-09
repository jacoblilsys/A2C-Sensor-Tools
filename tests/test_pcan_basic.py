import ctypes as ct
import unittest

from a2c_sensor_tools.can_sensor_monitor import CAN_MSG_ERROR_FRAME, CAN_MSG_EXT
from a2c_sensor_tools.pcan_basic import (
    PCAN_API_VERSION,
    PCAN_BAUDRATES,
    PCAN_BUSOFF_AUTORESET,
    PCAN_CHANNEL_AVAILABLE,
    PCAN_CHANNEL_CONDITION,
    PCAN_ERROR_BUSHEAVY,
    PCAN_ERROR_OK,
    PCAN_ERROR_QRCVEMPTY,
    PCAN_MESSAGE_EXTENDED,
    PCAN_MESSAGE_STATUS,
    PCAN_PARAMETER_ON,
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
        self.set_values = []
        self.writes = []
        self.read_items = []
        self.CAN_Initialize = _FakeFunction(self._initialize)
        self.CAN_Uninitialize = _FakeFunction(self._uninitialize)
        self.CAN_Read = _FakeFunction(self._read)
        self.CAN_Write = _FakeFunction(self._write)
        self.CAN_GetValue = _FakeFunction(self._get_value)
        self.CAN_SetValue = _FakeFunction(self._set_value)
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

    def _set_value(self, channel, parameter, buffer, length):
        value = ct.cast(buffer, ct.POINTER(ct.c_uint32)).contents.value
        self.set_values.append((channel, parameter, value, length))
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
        item = self.read_items.pop(0)
        if len(item) == 4:
            status = PCAN_ERROR_OK
            can_id, message_type, data, millis = item
        else:
            status, can_id, message_type, data, millis = item
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
        return status


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
        self.assertEqual(
            self.dll.set_values,
            [(0x51, PCAN_BUSOFF_AUTORESET, PCAN_PARAMETER_ON, 4)],
        )
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

    def test_bus_warning_returns_logged_error_frame_instead_of_raising(self) -> None:
        self.dll.read_items.append(
            (
                PCAN_ERROR_BUSHEAVY,
                0,
                PCAN_MESSAGE_STATUS,
                bytes((0, 0, 0, PCAN_ERROR_BUSHEAVY)),
                5678,
            )
        )
        with self.api.open_channel(0x51, 250_000) as channel:
            frame = channel.read(0)

        self.assertIsNotNone(frame)
        assert frame is not None
        self.assertTrue(frame.flags & CAN_MSG_ERROR_FRAME)
        self.assertEqual(frame.data, b"\x00\x00\x00\x08")
        self.assertEqual(frame.timestamp_ms, 5678)

    def test_read_continues_after_bus_warning(self) -> None:
        self.dll.read_items.extend(
            (
                (
                    PCAN_ERROR_BUSHEAVY,
                    0,
                    PCAN_MESSAGE_STATUS,
                    bytes((0, 0, 0, PCAN_ERROR_BUSHEAVY)),
                    5678,
                ),
                (0x123, 0, b"\xEF\x04\x00\x00\x01\x32", 5680),
            )
        )
        with self.api.open_channel(0x51, 250_000) as channel:
            warning = channel.read(0)
            response = channel.read(0)

        self.assertIsNotNone(warning)
        assert warning is not None
        self.assertTrue(warning.flags & CAN_MSG_ERROR_FRAME)
        self.assertIsNotNone(response)
        assert response is not None
        self.assertEqual(response.can_id, 0x123)
        self.assertEqual(response.data, b"\xEF\x04\x00\x00\x01\x32")

    def test_valid_message_buffer_is_kept_when_read_reports_bus_warning(self) -> None:
        self.dll.read_items.append(
            (
                PCAN_ERROR_BUSHEAVY,
                0x123,
                0,
                b"\xEF\x04\x00\x00\x01\x32",
                5680,
            )
        )
        with self.api.open_channel(0x51, 250_000) as channel:
            response = channel.read(0)

        self.assertIsNotNone(response)
        assert response is not None
        self.assertFalse(response.flags & CAN_MSG_ERROR_FRAME)
        self.assertEqual(response.can_id, 0x123)
        self.assertEqual(response.data, b"\xEF\x04\x00\x00\x01\x32")


if __name__ == "__main__":
    unittest.main()
