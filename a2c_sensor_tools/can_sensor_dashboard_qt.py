#!/usr/bin/env python3
# Copyright 2026 A2C
# SPDX-License-Identifier: Apache-2.0
"""Qt 6 configuration and live-data dashboard for the A2C IMU sensor."""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math
import queue
import struct
import sys
import threading
import time
from typing import Optional

from PySide6.QtCharts import QChart, QChartView, QLineSeries, QLogValueAxis, QValueAxis
from PySide6.QtCore import QPoint, QPointF, QSettings, QThread, QTimer, Qt, Signal
from PySide6.QtGui import QCloseEvent, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QTableWidget,
    QVBoxLayout,
    QWidget,
)

try:
    from .a2c_app_support import APP_VERSION, install_help_menu
    from .can_sensor_monitor import (
        ACCEL_FSR_BY_INDEX,
        CMD_GET_CALIBRATION_INFORMATION,
        CMD_GET_PERIODIC_TASK,
        CMD_GET_GYRO_CALIBRATION,
        CMD_GET_SAMPLING_TIME,
        CMD_GET_SENSOR_INFORMATION,
        CMD_GET_SYSTEM_MODE,
        CMD_GET_VIBRATION_CONFIGURATION,
        CMD_GET_YAW_REFERENCE,
        CMD_SEND_ACCELERATION,
        CMD_SEND_INCLINATION,
        CMD_SEND_RMS,
        CMD_SET_PERIODIC_TASK,
        CMD_SET_GYRO_CALIBRATION,
        CMD_SET_SYSTEM_MODE,
        CMD_CALIBRATE_USING_GRAVITY,
        GYRO_BIAS_MODE_NAMES,
        GYRO_GATE_STATE_NAMES,
        GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS,
        MODE_NAMES,
        RMS_VECTOR_NAME_BY_ID_OFFSET,
        CanFrame,
        KvaserCanlib,
        KvaserError,
        SAMPLE_POINT_SEGMENTS,
        SUPPORTED_BITRATES,
        SensorDecoder,
        decode_gyro_calibration_runtime,
        request_payload,
    )
    from .pcan_basic import PcanBasic, PcanError
except ImportError:  # Direct execution from the Tools directory.
    from a2c_app_support import APP_VERSION, install_help_menu  # type: ignore
    from can_sensor_monitor import (  # type: ignore
        ACCEL_FSR_BY_INDEX,
        CMD_GET_CALIBRATION_INFORMATION,
        CMD_GET_PERIODIC_TASK,
        CMD_GET_GYRO_CALIBRATION,
        CMD_GET_SAMPLING_TIME,
        CMD_GET_SENSOR_INFORMATION,
        CMD_GET_SYSTEM_MODE,
        CMD_GET_VIBRATION_CONFIGURATION,
        CMD_GET_YAW_REFERENCE,
        CMD_SEND_ACCELERATION,
        CMD_SEND_INCLINATION,
        CMD_SEND_RMS,
        CMD_SET_PERIODIC_TASK,
        CMD_SET_GYRO_CALIBRATION,
        CMD_SET_SYSTEM_MODE,
        CMD_CALIBRATE_USING_GRAVITY,
        GYRO_BIAS_MODE_NAMES,
        GYRO_GATE_STATE_NAMES,
        GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS,
        MODE_NAMES,
        RMS_VECTOR_NAME_BY_ID_OFFSET,
        CanFrame,
        KvaserCanlib,
        KvaserError,
        SAMPLE_POINT_SEGMENTS,
        SUPPORTED_BITRATES,
        SensorDecoder,
        decode_gyro_calibration_runtime,
        request_payload,
    )
    from pcan_basic import PcanBasic, PcanError  # type: ignore


APP_TITLE = "A2C IMU Dashboard"
ADAPTER_NAMES = {
    "kvaser": "Kvaser CANlib",
    "peak": "PEAK PCAN-Basic",
}

PLOT_RENDER_INTERVAL_MS = 40
LIVE_LABEL_INTERVAL_MS = 100
LOG_FLUSH_INTERVAL_MS = 100
LOG_QUEUE_CAPACITY = 10_000
LOG_FLUSH_BATCH_SIZE = 250
PLOT_BUFFER_CAPACITY = 5_000
PLOT_RENDER_POINT_LIMIT = 1_200
GYRO_CALIBRATION_TIMEOUT_MS = 15_000
GYRO_CALIBRATION_SAVE_TIMEOUT_MS = 5_000
GYRO_POLICY_MIN_FW = 0x131
GYRO_MANUAL_CALIBRATION_MIN_FW = 0x130

CMD_SET_SAVE_PARAMETERS = 0x50
CMD_SAVE_CALIBRATION = 0x21
SUBCMD_CALIBRATE_GYRO_STATIONARY = 0x12
SUBCMD_SAVE_CALIBRATION = 0xFF
CMD_SEND_COMBINED_AXIS = 0x09
CMD_SET_BANDWIDTH = 0x58
CMD_SET_FSR = 0x59
CMD_SET_VIBRATION_CONFIGURATION = 0x5A
CMD_SET_YAW_REFERENCE = 0x5B
CMD_SET_VIBRATION_STREAM = 0x5C
CMD_SET_AVERAGING = 0x60
CMD_GET_IMU_SETTINGS = 0x62
CMD_SET_BAUD_RATE = 0x67
CMD_SET_CAN_ID = 0x68
CMD_SET_FILTER_ID = 0x69
CMD_GET_BAUD_RATE = 0xE7
CMD_GET_CAN_ID = 0xE8
CMD_GET_FILTER_ID = 0xE9
CMD_GET_VIBRATION_STREAM = 0xC7
CMD_ACKNOWLEDGE = 0xAA
CAN_BAUD_CHANGE_CONFIRM_MARKER = 0xA5
CAN_BAUD_TRANSACTION_MIN_FW = 0x12C

VIBRATION_STREAM_SOURCE_FILTERED = 2
VIBRATION_STREAM_RATE_2KHZ = 2
VIBRATION_STREAM_RATE_4KHZ = 4
CAN_WORKER_BATCH_SIZE = 64
CAN_WORKER_BATCH_INTERVAL_S = 0.010

FILTER_STD_1_2 = 0x01
FILTER_STD_3_4 = 0x02
FILTER_EXT_1 = 0x03
FILTER_EXT_2 = 0x04

ACCEL_BANDWIDTHS = {
    0: "218 Hz (legacy readback)",
    1: "218 Hz",
    2: "99 Hz",
    3: "44 Hz",
    4: "21 Hz",
    5: "10 Hz",
    6: "5 Hz",
    7: "420 Hz",
}
ACCEL_BANDWIDTH_HZ = {
    0: 218.0,
    1: 218.0,
    2: 99.0,
    3: 44.0,
    4: 21.0,
    5: 10.0,
    6: 5.0,
    7: 420.0,
}
GYRO_BANDWIDTHS = {
    1: "176 Hz",
    2: "92 Hz",
    3: "41 Hz",
    4: "20 Hz",
    5: "10 Hz",
    6: "5 Hz",
}
GYRO_BANDWIDTH_HZ = {1: 176.0, 2: 92.0, 3: 41.0, 4: 20.0, 5: 10.0, 6: 5.0}
GYRO_FSRS = {0: "±250 dps", 1: "±500 dps", 2: "±1000 dps", 3: "±2000 dps"}
GYRO_COUNTS_PER_DPS = {0: 128.0, 1: 64.0, 2: 32.0, 3: 16.0}

GYRO_BIAS_MODE_LEGACY_MOBILE = 0
GYRO_BIAS_MODE_STATIONARY_AUTO = 1
GYRO_BIAS_MODE_FIXED = 2
DEFAULT_GYRO_THRESHOLD_MDPS = 250
DEFAULT_ACCEL_TOLERANCE_MG = 30
DEFAULT_GYRO_STATIONARY_DWELL_MS = 2000
CALIBRATION_RESULT_NAMES = {
    0: "calibration failed",
    1: "calibration completed",
    2: "sensor communication or setup failed",
    3: "sample collection failed or timed out",
    4: "sensor was moving during calibration",
    5: "sensor pose was invalid",
    6: "required offset was outside the hardware register range",
    7: "post-calibration verification failed",
    8: "the previous sensor mode could not be restored",
}

NORMAL_SAMPLE_RATE_HZ = 1000.0
VIBRATION_SAMPLE_RATE_HZ = 4000.0
VIBRATION_SENSOR_BANDWIDTH_HZ = 1046.0
FREQUENCY_RESPONSE_MIN_HZ = 0.5
FREQUENCY_RESPONSE_POINTS = 480
FREQUENCY_RESPONSE_FLOOR_DB = -80.0

PERIODIC_COMMANDS = {
    0x09: "Single-axis / J1939",
    0x0A: "Acceleration",
    0x0B: "Inclination / Euler",
    0x0C: "Acceleration min/max",
    0x0D: "Gyroscope",
    0x0F: "Set values to zero",
    0x10: "RMS acceleration",
    0xE4: "Measured sample rate",
}

PERIODIC_SUBCOMMANDS: dict[int, tuple[tuple[int, str], ...]] = {
    0x09: (
        (0x00, "X axis: linear accel, accel, gyro, Euler"),
        (0x01, "Y axis: linear accel, accel, gyro, Euler"),
        (0x02, "Z axis: linear accel, accel, gyro, Euler"),
    ),
    0x0A: (
        (0x00, "Acceleration XYZ"),
        (0x01, "Linear acceleration XYZ (fusion mode)"),
        (0x03, "Acceleration X: current/min/max"),
        (0x04, "Acceleration Y: current/min/max"),
        (0x05, "Acceleration Z: current/min/max"),
        (0x06, "Linear acceleration X: current/min/max"),
        (0x07, "Linear acceleration Y: current/min/max"),
        (0x08, "Linear acceleration Z: current/min/max"),
        (0x0A, "Linear acceleration Z: current/min/max (legacy duplicate)"),
    ),
    0x0B: (
        (0x00, "Euler XYZ: 1 degree/count"),
        (0x01, "Euler XYZ: 0.1 degree/count"),
        (0x02, "Euler XYZ: 0.01 degree/count"),
        (0x03, "Roll: current/min/max"),
        (0x04, "Pitch: current/min/max"),
        (0x05, "Yaw: current/min/max"),
    ),
    0x0C: (
        (0x00, "Acceleration X: current/min/max (legacy)"),
        (0x01, "Acceleration Y: current/min/max (legacy)"),
        (0x02, "Acceleration Z: current/min/max (legacy)"),
    ),
    0x0D: (
        (0x00, "Gyroscope XYZ"),
        (0x03, "Gyroscope X: current/min/max"),
        (0x04, "Gyroscope Y: current/min/max"),
        (0x05, "Gyroscope Z: current/min/max"),
    ),
    0x0F: (
        (0x01, "Reset acceleration X min/max"),
        (0x02, "Reset acceleration Y min/max"),
        (0x03, "Reset acceleration Z min/max"),
        (0x04, "Reset acceleration XYZ min/max"),
    ),
    0x10: ((0x00, "RMS output selected by vibration mode"),),
    0xE4: ((0x00, "Measured acquisition rate"),),
}


def periodic_subcommands(command: int) -> tuple[tuple[int, str], ...]:
    """Return the meaningful subcommands implemented for a periodic command."""
    return PERIODIC_SUBCOMMANDS.get(command, ())


def decode_rms_measurement(
    frame: CanFrame, counts_per_g: int, sensor_base_id: Optional[int]
) -> tuple[str, tuple[float, ...]]:
    """Decode either the XYZ or mode-selected vector RMS CAN payload."""
    if not frame.data or frame.data[0] != CMD_SEND_RMS:
        raise ValueError("not an RMS frame")
    if len(frame.data) == 7:
        counts = struct.unpack(">HHH", frame.data[1:7])
        return "XYZ", tuple(value / counts_per_g for value in counts)
    if len(frame.data) == 3:
        counts = struct.unpack(">H", frame.data[1:3])[0]
        vector_name = "Vector"
        if sensor_base_id is not None:
            vector_name = RMS_VECTOR_NAME_BY_ID_OFFSET.get(
                frame.can_id - sensor_base_id, vector_name
            )
        return vector_name, (counts / counts_per_g,)
    raise ValueError(f"unexpected RMS payload length {len(frame.data)}")


@dataclass(frozen=True)
class CombinedAxisMeasurement:
    axis: int
    linear_acceleration_g: float
    acceleration_g: float
    gyro_dps: float
    inclination_degrees: float


@dataclass(frozen=True)
class VibrationStreamSample:
    sequence: int
    acceleration_g: tuple[float, float, float]


@dataclass(frozen=True)
class GyroCalibrationConfiguration:
    mode: int
    gyro_threshold_mdps: int
    accel_tolerance_mg: int
    dwell_ms: int

    def __post_init__(self) -> None:
        if self.mode not in GYRO_BIAS_MODE_NAMES:
            raise ValueError(f"unknown gyro-bias mode {self.mode}")
        if not 50 <= self.gyro_threshold_mdps <= 3000:
            raise ValueError("gyro threshold must be between 50 and 3000 mdps")
        if not 5 <= self.accel_tolerance_mg <= 200:
            raise ValueError("acceleration tolerance must be between 5 and 200 mg")
        if not 500 <= self.dwell_ms <= 25_000 or self.dwell_ms % 100:
            raise ValueError(
                "stationary dwell must be between 500 and 25000 ms in 100 ms steps"
            )

    def payload(self) -> bytes:
        return bytes(
            (
                CMD_SET_GYRO_CALIBRATION,
                0,
                self.mode,
                self.dwell_ms // 100,
            )
        ) + struct.pack(
            ">HH",
            self.gyro_threshold_mdps,
            self.accel_tolerance_mg,
        )

    @classmethod
    def from_frame(cls, frame: CanFrame) -> "GyroCalibrationConfiguration":
        if len(frame.data) != 8 or frame.data[0] != CMD_GET_GYRO_CALIBRATION:
            raise ValueError("not a gyro-calibration configuration response")
        if frame.data[1] != 0:
            raise ValueError("unsupported gyro-calibration configuration subcommand")
        gyro_threshold_mdps, accel_tolerance_mg = struct.unpack(
            ">HH", frame.data[4:8]
        )
        return cls(
            mode=frame.data[2],
            gyro_threshold_mdps=gyro_threshold_mdps,
            accel_tolerance_mg=accel_tolerance_mg,
            dwell_ms=frame.data[3] * 100,
        )


def vibration_stream_payload(enable: bool, rate_khz: int, can_id: int) -> bytes:
    """Build the volatile high-rate waveform configuration command."""
    if enable and rate_khz not in (VIBRATION_STREAM_RATE_2KHZ, VIBRATION_STREAM_RATE_4KHZ):
        raise ValueError("vibration stream rate must be 2 or 4 kHz")
    if enable and not 0 < can_id <= 0x7FF:
        raise ValueError("vibration stream requires an 11-bit standard CAN ID")
    if not enable:
        return bytes((CMD_SET_VIBRATION_STREAM, 0, 0, 0, 0, 0, 0, 0))
    return bytes(
        (
            CMD_SET_VIBRATION_STREAM,
            1,
            rate_khz,
            VIBRATION_STREAM_SOURCE_FILTERED,
            (can_id >> 8) & 0xFF,
            can_id & 0xFF,
            0,
            0,
        )
    )


def decode_vibration_stream_sample(
    frame: CanFrame, counts_per_g: int
) -> VibrationStreamSample:
    """Decode [sequence, X, Y, Z], all signed axes and fields in big endian."""
    if frame.is_extended or len(frame.data) != 8:
        raise ValueError("not a standard eight-byte vibration stream frame")
    sequence, x, y, z = struct.unpack(">Hhhh", frame.data)
    return VibrationStreamSample(
        sequence=sequence,
        acceleration_g=(x / counts_per_g, y / counts_per_g, z / counts_per_g),
    )


def decode_combined_axis_measurement(
    frame: CanFrame,
    sensor_base_id: int,
    counts_per_g: int,
    gyro_counts_per_dps: float,
) -> CombinedAxisMeasurement:
    """Decode command 0x09, whose axis is carried in the CAN-ID offset."""
    axis = frame.can_id - sensor_base_id - 1
    if len(frame.data) != 8 or axis not in (0, 1, 2):
        raise ValueError("not a combined single-axis frame")
    linear_accel, accel, gyro, inclination = struct.unpack(">hhhh", frame.data)
    return CombinedAxisMeasurement(
        axis=axis,
        linear_acceleration_g=linear_accel / counts_per_g,
        acceleration_g=accel / counts_per_g,
        gyro_dps=gyro / gyro_counts_per_dps,
        inclination_degrees=inclination / 100.0,
    )

# Firmware baud enum, bitrate, sample point.
SENSOR_BAUD_CONFIGS = (
    (1, 1_000_000, "87.5"),
    (2, 500_000, "87.5"),
    (3, 250_000, "87.5"),
    (4, 125_000, "87.5"),
    (5, 100_000, "87.5"),
    (6, 50_000, "87.5"),
    (10, 1_000_000, "75"),
    (11, 500_000, "75"),
    (12, 250_000, "75"),
    (13, 125_000, "75"),
    (14, 100_000, "75"),
    (15, 50_000, "75"),
)


def format_bitrate(bitrate: int) -> str:
    return f"{bitrate / 1_000_000:g} Mbit/s" if bitrate >= 1_000_000 else f"{bitrate // 1000} kbit/s"


def parse_can_id(text: str, *, extended: bool = False) -> int:
    value_text = text.strip()
    if not value_text:
        raise ValueError("CAN ID is required")
    if value_text.lower().startswith("0x"):
        digits, base = value_text[2:], 16
    elif any(character in "abcdefABCDEF" for character in value_text):
        digits, base = value_text, 16
    else:
        digits, base = value_text, 10
    try:
        value = int(digits, base)
    except ValueError as exc:
        raise ValueError(f"invalid CAN ID: {text!r}") from exc
    # The firmware deliberately treats 0x1FFFFFFF as out of range.
    maximum = 0x1FFFFFFE if extended else 0x7FF
    if not 0 < value <= maximum:
        frame_type = "29-bit extended" if extended else "11-bit standard"
        raise ValueError(f"CAN ID 0x{value:X} is outside the {frame_type} range")
    return value


def padded_payload(command: int, subcommand: int = 0, data: bytes = b"") -> bytes:
    payload = bytes((command, subcommand)) + data
    if len(payload) > 8:
        raise ValueError("classic CAN payload cannot exceed eight bytes")
    return payload + bytes(8 - len(payload))


def baud_rate_payload(enum_value: int, auto_retransmit: bool, *, confirm: bool = False) -> bytes:
    """Build the staged/confirmed baud command introduced by firmware 0x12C."""
    marker = CAN_BAUD_CHANGE_CONFIRM_MARKER if confirm else 0
    return padded_payload(
        CMD_SET_BAUD_RATE,
        enum_value,
        bytes((int(auto_retransmit), marker)),
    )


def u32_setting_payload(command: int, subcommand: int, value: int) -> bytes:
    return padded_payload(command, subcommand, value.to_bytes(4, "big"))


@dataclass(frozen=True)
class PeriodicConfiguration:
    task: int
    enabled: bool
    command: int
    subcommand: int
    interval_ms: int

    def payload(self) -> bytes:
        return bytes(
            (
                CMD_SET_PERIODIC_TASK,
                self.task,
                int(self.enabled),
                self.command,
                self.subcommand,
                (self.interval_ms >> 8) & 0xFF,
                self.interval_ms & 0xFF,
                0,
            )
        )

    @classmethod
    def from_frame(cls, frame: CanFrame) -> "PeriodicConfiguration":
        if len(frame.data) < 7 or frame.data[0] != CMD_GET_PERIODIC_TASK:
            raise ValueError("not a periodic-task response")
        return cls(
            task=frame.data[1],
            enabled=bool(frame.data[2]),
            command=frame.data[3],
            subcommand=frame.data[4],
            interval_ms=int.from_bytes(frame.data[5:7], "big"),
        )


def can_api_for_adapter(adapter: str):
    if adapter == "peak":
        return PcanBasic()
    if adapter == "kvaser":
        return KvaserCanlib()
    raise ValueError(f"Unsupported CAN adapter: {adapter}")


class CanWorker(QThread):
    frames_received = Signal(object)
    frame_transmitted = Signal(int, bytes, bool)
    connection_changed = Signal(bool, str)
    worker_error = Signal(str)

    def __init__(
        self, adapter: str, channel_number: int, bitrate: int, sample_point: str
    ) -> None:
        super().__init__()
        self.adapter = adapter
        self.channel_number = channel_number
        self.bitrate = bitrate
        self.sample_point = sample_point
        self._stop_event = threading.Event()
        self._outgoing: queue.Queue[tuple[int, bytes, bool]] = queue.Queue()

    def enqueue(self, can_id: int, data: bytes, extended: bool = False) -> None:
        self._outgoing.put((can_id, data, extended))

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        pending_frames: list[CanFrame] = []
        try:
            api = can_api_for_adapter(self.adapter)
            channels = api.list_channels()
            selected = next((item for item in channels if item.number == self.channel_number), None)
            if selected is None:
                adapter_name = ADAPTER_NAMES[self.adapter]
                raise RuntimeError(
                    f"{adapter_name} channel 0x{self.channel_number:X} does not exist"
                )
            with api.open_channel(self.channel_number, self.bitrate, self.sample_point) as channel:
                self.connection_changed.emit(
                    True, f"{ADAPTER_NAMES[self.adapter]} — {selected.name}"
                )
                last_write = 0.0
                last_batch = time.monotonic()
                while not self._stop_event.is_set():
                    now = time.monotonic()
                    if now - last_write >= 0.005:
                        try:
                            can_id, data, extended = self._outgoing.get_nowait()
                        except queue.Empty:
                            pass
                        else:
                            channel.write(can_id, data, extended)
                            last_write = now
                            self.frame_transmitted.emit(can_id, data, extended)
                    frame = channel.read(5)
                    if frame is not None:
                        pending_frames.append(frame)
                    now = time.monotonic()
                    if pending_frames and (
                        len(pending_frames) >= CAN_WORKER_BATCH_SIZE
                        or now - last_batch >= CAN_WORKER_BATCH_INTERVAL_S
                        or frame is None
                    ):
                        self.frames_received.emit(tuple(pending_frames))
                        pending_frames.clear()
                        last_batch = now
        except Exception as exc:
            self.worker_error.emit(str(exc))
        finally:
            if pending_frames:
                self.frames_received.emit(tuple(pending_frames))
            self.connection_changed.emit(False, "")


def min_max_decimate(
    points: list[tuple[float, float]], max_points: int
) -> list[tuple[float, float]]:
    """Reduce plot points while retaining the first, last, and extrema in each bucket."""
    if max_points < 4:
        raise ValueError("max_points must be at least four")
    if len(points) <= max_points:
        return points

    interior = points[1:-1]
    bucket_count = max(1, (max_points - 2) // 2)
    result = [points[0]]
    for bucket in range(bucket_count):
        start = bucket * len(interior) // bucket_count
        end = (bucket + 1) * len(interior) // bucket_count
        values = interior[start:end]
        if not values:
            continue
        minimum = min(values, key=lambda item: item[1])
        maximum = max(values, key=lambda item: item[1])
        if minimum == maximum:
            result.append(minimum)
        elif minimum[0] < maximum[0]:
            result.extend((minimum, maximum))
        else:
            result.extend((maximum, minimum))
    result.append(points[-1])
    return result


def sensor_bandwidth_magnitude(frequency_hz: float, bandwidth_hz: float) -> float:
    """First-order model anchored to the sensor's quoted -3 dB bandwidth."""
    return 1.0 / math.sqrt(1.0 + (frequency_hz / bandwidth_hz) ** 2)


def block_average_magnitude(frequency_hz: float, sample_rate_hz: float, samples: int) -> float:
    if samples <= 1:
        return 1.0
    omega = 2.0 * math.pi * frequency_hz / sample_rate_hz
    denominator = samples * math.sin(omega * 0.5)
    if abs(denominator) < 1.0e-12:
        return 1.0
    return abs(math.sin(samples * omega * 0.5) / denominator)


def software_biquad_magnitude(
    frequency_hz: float,
    sample_rate_hz: float,
    cutoff_hz: float,
    q: float,
    highpass: bool,
) -> float:
    omega_cutoff = 2.0 * math.pi * cutoff_hz / sample_rate_hz
    cosine = math.cos(omega_cutoff)
    alpha = math.sin(omega_cutoff) / (2.0 * q)
    a0_inverse = 1.0 / (1.0 + alpha)
    if highpass:
        b0 = ((1.0 + cosine) * 0.5) * a0_inverse
        b1 = (-(1.0 + cosine)) * a0_inverse
    else:
        b0 = ((1.0 - cosine) * 0.5) * a0_inverse
        b1 = (1.0 - cosine) * a0_inverse
    b2 = b0
    a1 = (-2.0 * cosine) * a0_inverse
    a2 = (1.0 - alpha) * a0_inverse

    omega = 2.0 * math.pi * frequency_hz / sample_rate_hz
    z_inverse = complex(math.cos(omega), -math.sin(omega))
    numerator = b0 + b1 * z_inverse + b2 * z_inverse * z_inverse
    denominator = 1.0 + a1 * z_inverse + a2 * z_inverse * z_inverse
    return abs(numerator / denominator)


def software_lowpass_magnitude(frequency_hz: float, cutoff_hz: float) -> float:
    magnitude = 1.0
    for q in (0.5411961001, 1.3065629649):
        magnitude *= software_biquad_magnitude(
            frequency_hz, VIBRATION_SAMPLE_RATE_HZ, cutoff_hz, q, False
        )
    return magnitude


def software_highpass_magnitude(frequency_hz: float, cutoff_hz: float) -> float:
    return software_biquad_magnitude(
        frequency_hz, VIBRATION_SAMPLE_RATE_HZ, cutoff_hz, 0.7071067812, True
    )


def magnitude_db(magnitude: float) -> float:
    return max(FREQUENCY_RESPONSE_FLOOR_DB, 20.0 * math.log10(max(magnitude, 1.0e-12)))


def interpolate_response_db(
    values: list[tuple[float, float]], frequency_hz: float
) -> float:
    """Interpolate a logarithmic-frequency response curve in dB."""
    if not values:
        return math.nan
    frequencies = [item[0] for item in values]
    index = bisect_left(frequencies, frequency_hz)
    if index <= 0:
        return values[0][1]
    if index >= len(values):
        return values[-1][1]
    frequency_a, amplitude_a = values[index - 1]
    frequency_b, amplitude_b = values[index]
    log_a = math.log(frequency_a)
    log_b = math.log(frequency_b)
    fraction = (math.log(frequency_hz) - log_a) / (log_b - log_a)
    return amplitude_a + fraction * (amplitude_b - amplitude_a)


def response_level_crossings(
    values: list[tuple[float, float]], level_db: float = -3.0
) -> list[float]:
    """Return logarithmically interpolated frequencies crossing a dB level."""
    crossings: list[float] = []
    for (frequency_a, amplitude_a), (frequency_b, amplitude_b) in zip(
        values, values[1:]
    ):
        difference_a = amplitude_a - level_db
        difference_b = amplitude_b - level_db
        if difference_a == 0.0:
            crossing = frequency_a
        elif difference_a * difference_b > 0.0 or amplitude_a == amplitude_b:
            continue
        else:
            fraction = (level_db - amplitude_a) / (amplitude_b - amplitude_a)
            crossing = math.exp(
                math.log(frequency_a)
                + fraction * (math.log(frequency_b) - math.log(frequency_a))
            )
        if not crossings or not math.isclose(crossing, crossings[-1], rel_tol=1.0e-6):
            crossings.append(crossing)
    if values and values[-1][1] == level_db:
        crossings.append(values[-1][0])
    return crossings


def logarithmic_frequencies(maximum_hz: float) -> list[float]:
    ratio = (maximum_hz / FREQUENCY_RESPONSE_MIN_HZ) ** (
        1.0 / (FREQUENCY_RESPONSE_POINTS - 1)
    )
    return [FREQUENCY_RESPONSE_MIN_HZ * ratio**index for index in range(FREQUENCY_RESPONSE_POINTS)]


def calculate_frequency_response(
    output_kind: str,
    accel_bandwidth_hz: float,
    gyro_bandwidth_hz: float,
    average_samples: int,
    lowpass_enabled: bool,
    lowpass_hz: float,
    highpass_enabled: bool,
    highpass_hz: float,
    vibration_window_ms: int,
) -> tuple[float, dict[str, list[tuple[float, float]]]]:
    """Calculate modeled stage and combined amplitude responses for an actual output path."""
    if output_kind in ("normal_accel", "normal_gyro"):
        sample_rate = NORMAL_SAMPLE_RATE_HZ
        sensor_bandwidth = (
            accel_bandwidth_hz if output_kind == "normal_accel" else gyro_bandwidth_hz
        )
        average_count = average_samples
        combined_name = "Combined acceleration" if output_kind == "normal_accel" else "Combined gyro"
        include_lowpass = False
        include_highpass = False
    elif output_kind in ("vibration_rms", "vibration_filtered", "vibration_stream_2k"):
        sample_rate = VIBRATION_SAMPLE_RATE_HZ
        sensor_bandwidth = VIBRATION_SENSOR_BANDWIDTH_HZ
        average_count = 1
        combined_name = {
            "vibration_rms": "Combined RMS input",
            "vibration_filtered": "Combined filtered sample",
            "vibration_stream_2k": "Combined 2 kHz stream",
        }[output_kind]
        include_lowpass = lowpass_enabled
        include_highpass = highpass_enabled
    else:
        return NORMAL_SAMPLE_RATE_HZ / 2.0, {}

    maximum_hz = (
        1000.0 if output_kind == "vibration_stream_2k" else sample_rate / 2.0
    )
    frequencies = logarithmic_frequencies(maximum_hz)
    sensor_values = [sensor_bandwidth_magnitude(value, sensor_bandwidth) for value in frequencies]
    stages: list[tuple[str, list[float]]] = [
        (f"Sensor BW model ({sensor_bandwidth:g} Hz)", sensor_values)
    ]
    combined = list(sensor_values)

    if average_count > 1 or output_kind.startswith("normal_"):
        average_values = [
            block_average_magnitude(value, sample_rate, average_count) for value in frequencies
        ]
        average_name = f"Rolling average (N={average_count})"
        stages.append((average_name, average_values))
        combined = [left * right for left, right in zip(combined, average_values)]

    if include_lowpass:
        lowpass_values = [software_lowpass_magnitude(value, lowpass_hz) for value in frequencies]
        stages.append((f"Software LP4 ({lowpass_hz:g} Hz)", lowpass_values))
        combined = [left * right for left, right in zip(combined, lowpass_values)]

    if include_highpass:
        highpass_values = [software_highpass_magnitude(value, highpass_hz) for value in frequencies]
        stages.append((f"Software HP2 ({highpass_hz:g} Hz)", highpass_values))
        combined = [left * right for left, right in zip(combined, highpass_values)]

    if output_kind == "vibration_stream_2k":
        pair_average_values = [
            block_average_magnitude(value, VIBRATION_SAMPLE_RATE_HZ, 2)
            for value in frequencies
        ]
        stages.append(("Adjacent-pair average (4 kHz to 2 kHz)", pair_average_values))
        combined = [left * right for left, right in zip(combined, pair_average_values)]

    stages.append((combined_name, combined))
    curves = {
        name: [(frequency, magnitude_db(value)) for frequency, value in zip(frequencies, values)]
        for name, values in stages
    }
    return maximum_hz, curves


class TimeSeriesChart(QChartView):
    def __init__(
        self,
        title: str,
        names: tuple[str, ...],
        y_title: str,
        fixed_y: Optional[tuple[float, float]] = None,
        window_seconds: float = 10.0,
        positive_y: bool = False,
        buffer_capacity: int = PLOT_BUFFER_CAPACITY,
    ) -> None:
        chart = QChart()
        super().__init__(chart)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setMinimumHeight(250)
        self._window_seconds = window_seconds
        self._fixed_y = fixed_y
        self._positive_y = positive_y
        self._start_time: Optional[float] = None
        self._points: deque[tuple[float, tuple[float, ...]]] = deque(
            maxlen=buffer_capacity
        )
        self._series: list[QLineSeries] = []
        self._dirty = False
        self._dropped_samples = 0

        chart.setTitle(title)
        chart.legend().setVisible(True)
        chart.legend().setAlignment(Qt.AlignmentFlag.AlignBottom)
        self._axis_x = QValueAxis()
        self._axis_x.setTitleText("Time (s)")
        self._axis_x.setRange(0.0, window_seconds)
        self._axis_x.setLabelFormat("%.1f")
        self._axis_y = QValueAxis()
        self._axis_y.setTitleText(y_title)
        self._axis_y.setLabelFormat("%.2f")
        if fixed_y is not None:
            self._axis_y.setRange(*fixed_y)
        elif positive_y:
            self._axis_y.setRange(0.0, 1.25)
        else:
            self._axis_y.setRange(-1.25, 1.25)
        chart.addAxis(self._axis_x, Qt.AlignmentFlag.AlignBottom)
        chart.addAxis(self._axis_y, Qt.AlignmentFlag.AlignLeft)

        for name in names:
            series = QLineSeries()
            series.setName(name)
            chart.addSeries(series)
            series.attachAxis(self._axis_x)
            series.attachAxis(self._axis_y)
            self._series.append(series)

    def configure_series(self, names: tuple[str, ...]) -> None:
        if not 1 <= len(names) <= len(self._series):
            raise ValueError("chart must have between one and three visible series")
        for index, series in enumerate(self._series):
            visible = index < len(names)
            series.setVisible(visible)
            if visible:
                series.setName(names[index])
            else:
                series.clear()
        self._dirty = True

    def ingest_sample(self, values: tuple[float, ...], timestamp: Optional[float] = None) -> None:
        if len(values) != len(self._series):
            raise ValueError(f"expected {len(self._series)} chart values, got {len(values)}")
        now = time.monotonic() if timestamp is None else timestamp
        if self._start_time is None:
            self._start_time = now
        elapsed = now - self._start_time
        if len(self._points) == self._points.maxlen:
            self._dropped_samples += 1
        self._points.append((elapsed, values))
        start = max(0.0, elapsed - self._window_seconds)
        while self._points and self._points[0][0] < start - 0.5:
            self._points.popleft()
        self._dirty = True

    # Compatibility alias for callers outside this module. Samples are now rendered by a timer.
    add_sample = ingest_sample

    def render_pending(self, force: bool = False) -> bool:
        if not self._dirty and not force:
            return False
        if not self._points:
            self._dirty = False
            return False

        points = list(self._points)
        elapsed = points[-1][0]
        start = max(0.0, elapsed - self._window_seconds)
        for axis, series in enumerate(self._series):
            if not series.isVisible():
                continue
            axis_points = [
                (item[0], item[1][axis])
                for item in points
                if math.isfinite(item[1][axis])
            ]
            rendered = min_max_decimate(axis_points, PLOT_RENDER_POINT_LIMIT)
            series.replace([QPointF(x, y) for x, y in rendered])
        self._axis_x.setRange(start, max(self._window_seconds, elapsed))
        if self._fixed_y is None:
            finite_values = [
                abs(value)
                for _, values_at_time in points
                for axis, value in enumerate(values_at_time)
                if self._series[axis].isVisible() and math.isfinite(value)
            ]
            if finite_values:
                peak = max(finite_values)
                limit = max(1.25, peak * 1.15)
                if self._positive_y:
                    self._axis_y.setRange(0.0, limit)
                else:
                    self._axis_y.setRange(-limit, limit)
        self._dirty = False
        return True

    @property
    def buffered_sample_count(self) -> int:
        return len(self._points)

    @property
    def dropped_sample_count(self) -> int:
        return self._dropped_samples

    def clear_samples(self) -> None:
        self._points.clear()
        self._start_time = None
        self._dirty = False
        self._dropped_samples = 0
        for series in self._series:
            series.clear()
        self._axis_x.setRange(0.0, self._window_seconds)


class FrequencyResponseChart(QChartView):
    def __init__(self) -> None:
        chart = QChart()
        super().__init__(chart)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setMouseTracking(True)
        self.viewport().setMouseTracking(True)
        self.setMinimumHeight(300)
        self._chart = chart
        self._series: list[QLineSeries] = []
        self._curve_values: dict[str, list[tuple[float, float]]] = {}

        chart.setTitle("Modeled amplitude response")
        chart.legend().setVisible(True)
        chart.legend().setAlignment(Qt.AlignmentFlag.AlignBottom)
        self._axis_x = QLogValueAxis()
        self._axis_x.setTitleText("Frequency (Hz)")
        self._axis_x.setBase(10.0)
        self._axis_x.setLabelFormat("%.3g")
        self._axis_x.setRange(FREQUENCY_RESPONSE_MIN_HZ, NORMAL_SAMPLE_RATE_HZ / 2.0)
        self._axis_y = QValueAxis()
        self._axis_y.setTitleText("Amplitude (dB)")
        self._axis_y.setLabelFormat("%.0f")
        self._axis_y.setRange(FREQUENCY_RESPONSE_FLOOR_DB, 3.0)
        chart.addAxis(self._axis_x, Qt.AlignmentFlag.AlignBottom)
        chart.addAxis(self._axis_y, Qt.AlignmentFlag.AlignLeft)

        self._minus_three_series = QLineSeries()
        self._minus_three_series.setName("−3 dB reference")
        reference_pen = QPen(Qt.GlobalColor.red)
        reference_pen.setStyle(Qt.PenStyle.DashLine)
        reference_pen.setWidth(1)
        self._minus_three_series.setPen(reference_pen)
        chart.addSeries(self._minus_three_series)
        self._minus_three_series.attachAxis(self._axis_x)
        self._minus_three_series.attachAxis(self._axis_y)

        self._cursor_series = QLineSeries()
        cursor_pen = QPen(Qt.GlobalColor.darkGray)
        cursor_pen.setStyle(Qt.PenStyle.DotLine)
        cursor_pen.setWidth(1)
        self._cursor_series.setPen(cursor_pen)
        chart.addSeries(self._cursor_series)
        self._cursor_series.attachAxis(self._axis_x)
        self._cursor_series.attachAxis(self._axis_y)
        self._cursor_series.setVisible(False)
        for marker in chart.legend().markers(self._cursor_series):
            marker.setVisible(False)

        self._cursor_label = QLabel(self.viewport())
        self._cursor_label.setTextFormat(Qt.TextFormat.PlainText)
        self._cursor_label.setAttribute(
            Qt.WidgetAttribute.WA_TransparentForMouseEvents, True
        )
        self._cursor_label.setStyleSheet(
            "QLabel { background: palette(base); color: palette(text); "
            "border: 1px solid palette(mid); padding: 4px; }"
        )
        self._cursor_label.hide()

    def set_response(
        self, maximum_hz: float, curves: dict[str, list[tuple[float, float]]]
    ) -> list[float]:
        for series in self._series:
            self._chart.removeSeries(series)
            series.deleteLater()
        self._series.clear()
        self._curve_values = curves

        for name, values in curves.items():
            series = QLineSeries()
            series.setName(name)
            series.replace([QPointF(frequency, amplitude) for frequency, amplitude in values])
            self._chart.addSeries(series)
            series.attachAxis(self._axis_x)
            series.attachAxis(self._axis_y)
            self._series.append(series)
        self._axis_x.setRange(FREQUENCY_RESPONSE_MIN_HZ, maximum_hz)
        self._minus_three_series.replace(
            [
                QPointF(FREQUENCY_RESPONSE_MIN_HZ, -3.0),
                QPointF(maximum_hz, -3.0),
            ]
        )
        self._cursor_series.setVisible(False)
        self._cursor_label.hide()
        combined_values = next(reversed(curves.values()), []) if curves else []
        return response_level_crossings(combined_values)

    def values_at_frequency(self, frequency_hz: float) -> dict[str, float]:
        return {
            name: interpolate_response_db(values, frequency_hz)
            for name, values in self._curve_values.items()
        }

    def mouseMoveEvent(self, event) -> None:
        position = event.position()
        if not self._chart.plotArea().contains(position) or not self._curve_values:
            self._cursor_series.setVisible(False)
            self._cursor_label.hide()
            super().mouseMoveEvent(event)
            return

        mapped = self._chart.mapToValue(position, self._minus_three_series)
        frequency_hz = min(
            max(mapped.x(), self._axis_x.min()), self._axis_x.max()
        )
        values = self.values_at_frequency(frequency_hz)
        self._cursor_series.replace(
            [
                QPointF(frequency_hz, self._axis_y.min()),
                QPointF(frequency_hz, self._axis_y.max()),
            ]
        )
        self._cursor_series.setVisible(True)
        lines = [f"{frequency_hz:.4g} Hz"]
        lines.extend(f"{name}: {amplitude:+.2f} dB" for name, amplitude in values.items())
        self._cursor_label.setText("\n".join(lines))
        self._cursor_label.adjustSize()
        desired = position.toPoint() + QPoint(12, 12)
        maximum_x = max(4, self.viewport().width() - self._cursor_label.width() - 4)
        maximum_y = max(4, self.viewport().height() - self._cursor_label.height() - 4)
        self._cursor_label.move(
            min(max(4, desired.x()), maximum_x),
            min(max(4, desired.y()), maximum_y),
        )
        self._cursor_label.show()
        self._cursor_label.raise_()
        super().mouseMoveEvent(event)

    def leaveEvent(self, event) -> None:
        self._cursor_series.setVisible(False)
        self._cursor_label.hide()
        super().leaveEvent(event)


@dataclass
class PeriodicRow:
    enabled: QCheckBox
    command: QComboBox
    subcommand: QComboBox
    interval: QSpinBox
    status: QLabel


@dataclass
class PendingBaudChange:
    enum_value: int
    bitrate: int
    sample_point: str
    auto_retransmit: bool
    previous_bitrate: int
    previous_sample_point: str
    phase: str = "staged"
    confirmation_attempts: int = 0


class SensorDashboard(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.setMinimumSize(1180, 760)
        self.resize(1380, 880)

        self._settings = QSettings("A2C", "IMUSensorDashboard")
        self._worker: Optional[CanWorker] = None
        self._connected = False
        self._channel_by_adapter: dict[str, int] = {}
        self._decoder = SensorDecoder()
        self._frames_received = 0
        self._last_rate_count = 0
        self._current_mode: Optional[int] = None
        self._current_sensor_response: Optional[tuple[int, bool]] = None
        self._firmware_version: Optional[int] = None
        self._gyro_policy_supported: Optional[bool] = None
        self._pending_baud_change: Optional[PendingBaudChange] = None
        self._poll_counter = 0
        self._log_paused = False
        self._pending_log_lines: deque[str] = deque(maxlen=LOG_QUEUE_CAPACITY)
        self._pending_log_dropped = 0
        self._pending_label_text: dict[QLabel, str] = {}
        self._can_timestamp_origin_ms: Optional[int] = None
        self._can_timestamp_origin_host = 0.0
        self._can_timestamp_last_raw: Optional[int] = None
        self._can_timestamp_wrap_ms = 0
        self._periodic_rows: list[PeriodicRow] = []
        self._periodic_configurations: dict[int, PeriodicConfiguration] = {}
        self._stream_counts = {
            CMD_SEND_COMBINED_AXIS: 0,
            CMD_SEND_ACCELERATION: 0,
            CMD_SEND_RMS: 0,
            CMD_SEND_INCLINATION: 0,
        }
        self._stream_last_rate_counts = dict.fromkeys(self._stream_counts, 0)
        self._stream_rates = dict.fromkeys(self._stream_counts, 0)
        self._stream_last_seen: dict[int, Optional[float]] = {
            CMD_SEND_COMBINED_AXIS: None,
            CMD_SEND_ACCELERATION: None,
            CMD_SEND_RMS: None,
            CMD_SEND_INCLINATION: None,
        }
        self._rms_series_mode: Optional[str] = None
        self._vibration_stream_enabled = False
        self._vibration_stream_rate_khz = VIBRATION_STREAM_RATE_4KHZ
        self._vibration_stream_can_id = 0x148
        self._vibration_stream_frames = 0
        self._vibration_stream_last_rate_count = 0
        self._vibration_stream_rate = 0
        self._vibration_stream_last_seen: Optional[float] = None
        self._vibration_stream_last_sequence: Optional[int] = None
        self._vibration_stream_timestamp: Optional[float] = None
        self._vibration_stream_sequence_gaps = 0
        self._vibration_stream_firmware_stats = (0, 0, 0)
        self._vibration_stream_status_flags = 0
        self._frequency_response_dialog: Optional[QDialog] = None
        self._gyro_calibration_in_progress = False
        self._gyro_calibration_save_pending = False
        self._gyro_calibration_operation = 0
        self._gyro_calibration_response: Optional[tuple[int, bool]] = None
        self._gyro_calibration_request: Optional[tuple[int, bool]] = None
        self._gyro_runtime_diagnostics: dict[int, dict[str, object]] = {}
        self._combined_latest = {
            "acceleration": [math.nan, math.nan, math.nan],
            "linear_acceleration": [math.nan, math.nan, math.nan],
            "gyro": [math.nan, math.nan, math.nan],
            "inclination": [math.nan, math.nan, math.nan],
        }

        self._release_checker = install_help_menu(self, APP_TITLE)
        self._build_ui()
        self._restore_settings()
        self.request_id_edit.textChanged.connect(self._request_target_changed)
        self.request_extended_checkbox.toggled.connect(
            self._request_target_changed
        )
        self.refresh_channels(show_error=False)

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(100)
        self._poll_timer.timeout.connect(self._poll_live_data)
        self._poll_timer.start()
        self._rate_timer = QTimer(self)
        self._rate_timer.setInterval(1000)
        self._rate_timer.timeout.connect(self._update_message_rate)
        self._rate_timer.start()
        self._render_timer = QTimer(self)
        self._render_timer.setInterval(PLOT_RENDER_INTERVAL_MS)
        self._render_timer.timeout.connect(self._render_visible_chart)
        self._render_timer.start()
        self._label_timer = QTimer(self)
        self._label_timer.setInterval(LIVE_LABEL_INTERVAL_MS)
        self._label_timer.timeout.connect(self._flush_live_labels)
        self._label_timer.start()
        self._log_timer = QTimer(self)
        self._log_timer.setInterval(LOG_FLUSH_INTERVAL_MS)
        self._log_timer.timeout.connect(self._flush_log)
        self._log_timer.start()

    def _build_ui(self) -> None:
        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(8)

        connection_group = QGroupBox("CAN connection")
        connection_layout = QHBoxLayout(connection_group)
        self.adapter_combo = QComboBox()
        for adapter, display_name in ADAPTER_NAMES.items():
            self.adapter_combo.addItem(display_name, adapter)
        self.adapter_combo.currentIndexChanged.connect(self._adapter_changed)
        self.channel_combo = QComboBox()
        self.channel_combo.setMinimumWidth(290)
        self.channel_combo.currentIndexChanged.connect(self._channel_changed)
        self.refresh_channels_button = QPushButton("Refresh")
        self.refresh_channels_button.clicked.connect(self.refresh_channels)
        self.host_baud_combo = QComboBox()
        for bitrate in SUPPORTED_BITRATES:
            self.host_baud_combo.addItem(format_bitrate(bitrate), bitrate)
        self.host_sample_combo = QComboBox()
        for sample in SAMPLE_POINT_SEGMENTS:
            self.host_sample_combo.addItem(f"{sample}%", sample)
        self.request_id_edit = QLineEdit("0x3E8")
        self.request_id_edit.setMaximumWidth(100)
        self.request_extended_checkbox = QCheckBox("Extended request")
        self.connect_button = QPushButton("Connect")
        self.connect_button.setMinimumWidth(100)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.connection_status = QLabel("Disconnected")
        self.connection_status.setStyleSheet("color: #a02020; font-weight: bold;")

        for label, widget in (
            ("Adapter", self.adapter_combo),
            ("Channel", self.channel_combo),
            ("Bus baud", self.host_baud_combo),
            ("Sample", self.host_sample_combo),
            ("Request ID", self.request_id_edit),
        ):
            connection_layout.addWidget(QLabel(label))
            connection_layout.addWidget(widget)
        connection_layout.addWidget(self.request_extended_checkbox)
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.connection_status)
        connection_layout.addStretch(1)
        outer.addWidget(connection_group)

        utility_row = QHBoxLayout()
        self.refresh_all_button = QPushButton("Refresh all sensor settings")
        self.refresh_all_button.clicked.connect(self.refresh_all_settings)
        self.save_flash_button = QPushButton("Save current settings to sensor flash…")
        self.save_flash_button.clicked.connect(self.save_settings_to_flash)
        self.frequency_response_button = QPushButton("Frequency response plot")
        self.frequency_response_button.setMinimumHeight(36)
        self.frequency_response_button.setStyleSheet("font-weight: 700;")
        self.frequency_response_button.clicked.connect(
            self.show_frequency_response_dialog
        )
        self.response_id_label = QLabel("Response: —")
        self.message_rate_label = QLabel("0 frame/s")
        utility_row.addWidget(self.refresh_all_button)
        utility_row.addWidget(self.save_flash_button)
        utility_row.addStretch(1)
        utility_row.addWidget(self.response_id_label)
        utility_row.addWidget(self.message_rate_label)
        outer.addLayout(utility_row)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_dashboard_tab(), "Dashboard")
        self.tabs.addTab(self._build_periodic_tab(), "Periodic messages")
        self._log_tab_index = self.tabs.addTab(self._build_log_tab(), "CAN log")
        self.tabs.currentChanged.connect(self._render_visible_chart)
        outer.addWidget(self.tabs, 1)

        self.setCentralWidget(central)
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Select a CAN adapter and channel, then connect")
        self._connection_required = [
            self.refresh_all_button,
            self.save_flash_button,
            self.apply_can_button,
            self.change_sensor_baud_button,
            self.apply_imu_button,
            self.apply_gyro_calibration_button,
            self.refresh_gyro_calibration_button,
            self.refresh_gyro_status_button,
            self.calibrate_gyro_button,
            self.zero_yaw_button,
            self.set_yaw_target_button,
            self.clear_yaw_button,
            self.vibration_stream_start_button,
            self.vibration_stream_stop_button,
            self.vibration_stream_refresh_button,
            self.refresh_periodic_button,
            self.apply_periodic_button,
            self.disable_periodic_button,
        ]
        self._set_connection_controls(False)

    def _build_dashboard_tab(self) -> QWidget:
        tab = QWidget()
        tab_layout = QHBoxLayout(tab)
        splitter = QSplitter(Qt.Orientation.Horizontal)

        settings_container = QWidget()
        settings_layout = QVBoxLayout(settings_container)
        settings_layout.setContentsMargins(2, 2, 6, 2)
        settings_layout.addWidget(self._build_firmware_group())
        settings_layout.addWidget(self._build_sensor_can_group())
        settings_layout.addWidget(self._build_imu_group())
        settings_layout.addWidget(self._build_gyro_calibration_group())
        settings_layout.addWidget(self._build_vibration_stream_group())
        settings_layout.addWidget(self.frequency_response_button)
        settings_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(settings_container)
        scroll.setMinimumWidth(450)

        live_container = QWidget()
        live_layout = QVBoxLayout(live_container)
        live_layout.setContentsMargins(6, 2, 2, 2)
        live_header = QHBoxLayout()
        self.live_poll_checkbox = QCheckBox("Poll sensor at 10 Hz")
        self.live_poll_checkbox.setChecked(False)
        self.live_poll_checkbox.toggled.connect(self._live_polling_changed)
        self.live_input_label = QLabel()
        self.live_input_label.setStyleSheet("color: #315d87; font-weight: bold;")
        self.sample_rate_label = QLabel("Measured acquisition: —")
        self.yaw_reference_label = QLabel("Yaw reference: —")
        self.clear_charts_button = QPushButton("Clear charts")
        self.clear_charts_button.clicked.connect(self._clear_charts)
        live_header.addWidget(self.live_poll_checkbox)
        live_header.addWidget(self.sample_rate_label)
        live_header.addWidget(self.yaw_reference_label)
        live_header.addStretch(1)
        live_header.addWidget(self.clear_charts_button)
        live_layout.addLayout(live_header)
        live_layout.addWidget(self.live_input_label)

        self.accel_values_label = QLabel("X —     Y —     Z —")
        self.accel_values_label.setStyleSheet("font: 600 15px 'Segoe UI';")
        self.accel_stream_label = QLabel("Acceleration: waiting for frames")
        self.accel_stream_label.setStyleSheet("color: #666;")
        self.accel_chart = TimeSeriesChart("Acceleration", ("X", "Y", "Z"), "g")

        self.rms_values_label = QLabel("RMS: —")
        self.rms_values_label.setStyleSheet("font: 600 15px 'Segoe UI';")
        self.rms_stream_label = QLabel("Vibration RMS: waiting for frames")
        self.rms_stream_label.setStyleSheet("color: #666;")
        self.rms_chart = TimeSeriesChart(
            "Vibration RMS", ("X", "Y", "Z"), "g RMS", positive_y=True
        )

        self.fusion_values_label = QLabel("Roll —     Pitch —     Yaw —")
        self.fusion_values_label.setStyleSheet("font: 600 15px 'Segoe UI';")
        self.fusion_stream_label = QLabel("Fusion: waiting for frames")
        self.fusion_stream_label.setStyleSheet("color: #666;")
        self.fusion_chart = TimeSeriesChart(
            "Sensor fusion Euler angles", ("Roll", "Pitch", "Yaw"), "degrees", (-180.0, 180.0)
        )

        self.combined_axis_value_labels = [
            QLabel(f"{axis}: —") for axis in ("X", "Y", "Z")
        ]
        for label in self.combined_axis_value_labels:
            label.setStyleSheet("font: 600 14px 'Consolas';")
        self.combined_stream_label = QLabel("Combined 0x09: waiting for frames")
        self.combined_stream_label.setStyleSheet("color: #666;")
        self.combined_accel_chart = TimeSeriesChart(
            "Acceleration from 0x09", ("X", "Y", "Z"), "g"
        )
        self.combined_linear_accel_chart = TimeSeriesChart(
            "Linear acceleration from 0x09", ("X", "Y", "Z"), "g"
        )
        self.combined_gyro_chart = TimeSeriesChart(
            "Gyroscope from 0x09", ("X", "Y", "Z"), "degrees/s"
        )
        self.combined_inclination_chart = TimeSeriesChart(
            "Inclination from 0x09", ("X", "Y", "Z"), "degrees", (-180.0, 180.0)
        )
        combined_page = QWidget()
        combined_layout = QVBoxLayout(combined_page)
        combined_layout.setContentsMargins(4, 6, 4, 4)
        for label in self.combined_axis_value_labels:
            combined_layout.addWidget(label)
        combined_layout.addWidget(self.combined_stream_label)
        self.combined_measurement_tabs = QTabWidget()
        self.combined_measurement_tabs.setDocumentMode(True)
        self.combined_measurement_tabs.addTab(self.combined_accel_chart, "Acceleration")
        self.combined_measurement_tabs.addTab(
            self.combined_linear_accel_chart, "Linear acceleration"
        )
        self.combined_measurement_tabs.addTab(self.combined_gyro_chart, "Gyroscope")
        self.combined_measurement_tabs.addTab(
            self.combined_inclination_chart, "Inclination"
        )
        combined_layout.addWidget(self.combined_measurement_tabs, 1)

        self.waveform_values_label = QLabel("X —     Y —     Z —")
        self.waveform_values_label.setStyleSheet("font: 600 15px 'Segoe UI';")
        self.waveform_stream_label = QLabel("Filtered waveform: disabled")
        self.waveform_stream_label.setStyleSheet("color: #666;")
        self.waveform_chart = TimeSeriesChart(
            "Filtered vibration waveform",
            ("X", "Y", "Z"),
            "g",
            window_seconds=2.0,
            buffer_capacity=12_000,
        )
        waveform_page = self._build_plot_page(
            self.waveform_values_label, self.waveform_stream_label, self.waveform_chart
        )

        frequency_page = QWidget()
        self.frequency_response_page = frequency_page
        frequency_layout = QVBoxLayout(frequency_page)
        frequency_layout.setContentsMargins(4, 6, 4, 4)
        frequency_header = QHBoxLayout()
        frequency_header.addWidget(QLabel("Output path"))
        self.frequency_response_output_combo = QComboBox()
        self.frequency_response_output_combo.setMinimumWidth(250)
        frequency_header.addWidget(self.frequency_response_output_combo)
        self.frequency_response_summary_label = QLabel()
        self.frequency_response_summary_label.setWordWrap(True)
        self.frequency_response_summary_label.setStyleSheet("color: #315d87; font-weight: bold;")
        frequency_header.addWidget(self.frequency_response_summary_label, 1)
        frequency_layout.addLayout(frequency_header)
        self.frequency_response_crossing_label = QLabel("Combined −3 dB crossings: —")
        self.frequency_response_crossing_label.setStyleSheet(
            "color: #a02020; font-weight: bold;"
        )
        frequency_layout.addWidget(self.frequency_response_crossing_label)
        self.frequency_response_chart = FrequencyResponseChart()
        frequency_layout.addWidget(self.frequency_response_chart, 1)
        frequency_note = QLabel(
            "The selectable sensor BW curve is an approximation anchored to the datasheet "
            "−3 dB bandwidth; the IMU does not expose its internal coefficients. Software "
            "LP/HP, rolling-average, and 2 kHz pair-average curves use the exact firmware equations. RMS is nonlinear, "
            "so its window length is reported but is not represented as a linear Bode stage. "
            "Move the pointer across the plot to read frequency and all curve values."
        )
        frequency_note.setWordWrap(True)
        frequency_note.setStyleSheet("color: #666;")
        frequency_layout.addWidget(frequency_note)

        self.live_plot_tabs = QTabWidget()
        self.live_plot_tabs.setDocumentMode(True)
        self.live_plot_tabs.addTab(
            self._build_plot_page(self.accel_values_label, self.accel_stream_label, self.accel_chart),
            "Acceleration",
        )
        self.live_plot_tabs.addTab(
            self._build_plot_page(self.rms_values_label, self.rms_stream_label, self.rms_chart),
            "Vibration RMS",
        )
        self.live_plot_tabs.addTab(
            self._build_plot_page(self.fusion_values_label, self.fusion_stream_label, self.fusion_chart),
            "Fusion",
        )
        self.live_plot_tabs.addTab(combined_page, "Combined 0x09")
        self._waveform_tab_index = self.live_plot_tabs.addTab(
            waveform_page, "Filtered waveform"
        )
        self.live_plot_tabs.currentChanged.connect(self._render_visible_chart)
        self.combined_measurement_tabs.currentChanged.connect(self._render_visible_chart)
        self._stream_tab_index = {
            CMD_SEND_ACCELERATION: (0, "Acceleration"),
            CMD_SEND_RMS: (1, "Vibration RMS"),
            CMD_SEND_INCLINATION: (2, "Fusion"),
            CMD_SEND_COMBINED_AXIS: (3, "Combined 0x09"),
        }
        live_layout.addWidget(self.live_plot_tabs, 1)
        self.mode_combo.currentIndexChanged.connect(self._imu_mode_controls_changed)
        self.frequency_response_output_combo.currentIndexChanged.connect(
            self._update_frequency_response
        )
        for combo in (self.accel_bw_combo, self.gyro_bw_combo):
            combo.currentIndexChanged.connect(self._update_frequency_response)
        for spin in (
            self.standard_average_spin,
            self.lowpass_hz_spin,
            self.highpass_hz_spin,
            self.vibration_window_spin,
        ):
            spin.valueChanged.connect(self._update_frequency_response)
        self.lowpass_hz_spin.valueChanged.connect(self._update_highpass_limit)
        self.lowpass_checkbox.toggled.connect(self._imu_filter_controls_changed)
        self.highpass_checkbox.toggled.connect(self._imu_filter_controls_changed)
        self._imu_mode_controls_changed()
        self._live_polling_changed(False)

        splitter.addWidget(scroll)
        splitter.addWidget(live_container)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([480, 860])
        tab_layout.addWidget(splitter)
        return tab

    @staticmethod
    def _build_plot_page(values: QLabel, status: QLabel, chart: TimeSeriesChart) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(4, 6, 4, 4)
        layout.addWidget(values)
        layout.addWidget(status)
        layout.addWidget(chart, 1)
        return page

    def _build_firmware_group(self) -> QGroupBox:
        group = QGroupBox("Sensor information")
        form = QFormLayout(group)
        self.firmware_label = QLabel("—")
        self.hardware_label = QLabel("—")
        self.sensor_type_label = QLabel("—")
        self.serial_label = QLabel("—")
        for label in (self.firmware_label, self.hardware_label, self.sensor_type_label, self.serial_label):
            label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        form.addRow("Firmware", self.firmware_label)
        form.addRow("Hardware", self.hardware_label)
        form.addRow("Sensor type", self.sensor_type_label)
        form.addRow("Serial number", self.serial_label)
        return group

    def _build_sensor_can_group(self) -> QGroupBox:
        group = QGroupBox("Sensor CAN settings")
        grid = QGridLayout(group)

        self.sensor_baud_combo = QComboBox()
        for enum_value, bitrate, sample in SENSOR_BAUD_CONFIGS:
            self.sensor_baud_combo.addItem(
                f"{format_bitrate(bitrate)} @ {sample}%", (enum_value, bitrate, sample)
            )
        self.auto_retransmit_checkbox = QCheckBox("Auto retransmission")
        self.change_sensor_baud_button = QPushButton("Change baud and reset…")
        self.change_sensor_baud_button.clicked.connect(self.change_sensor_baud)
        grid.addWidget(QLabel("Stored baud"), 0, 0)
        grid.addWidget(self.sensor_baud_combo, 0, 1, 1, 2)
        grid.addWidget(self.auto_retransmit_checkbox, 0, 3)
        grid.addWidget(self.change_sensor_baud_button, 0, 4)

        self.sensor_tx_id_edit = QLineEdit("0x125")
        self.sensor_tx_format_combo = QComboBox()
        self.sensor_tx_format_combo.addItem("Standard 11-bit", False)
        self.sensor_tx_format_combo.addItem("Extended 29-bit", True)
        grid.addWidget(QLabel("Sensor TX ID"), 1, 0)
        grid.addWidget(self.sensor_tx_id_edit, 1, 1)
        grid.addWidget(self.sensor_tx_format_combo, 1, 2, 1, 2)

        self.std_filter_edits = [QLineEdit() for _ in range(4)]
        for index, edit in enumerate(self.std_filter_edits):
            edit.setPlaceholderText(f"Filter {index + 1}")
            edit.setMaximumWidth(90)
        grid.addWidget(QLabel("Standard filters"), 2, 0)
        for index, edit in enumerate(self.std_filter_edits):
            grid.addWidget(edit, 2, index + 1)

        self.ext_filter_edits = [QLineEdit(), QLineEdit()]
        for index, edit in enumerate(self.ext_filter_edits):
            edit.setPlaceholderText(f"Extended {index + 1}")
        grid.addWidget(QLabel("Extended filters"), 3, 0)
        grid.addWidget(self.ext_filter_edits[0], 3, 1, 1, 2)
        grid.addWidget(self.ext_filter_edits[1], 3, 3, 1, 2)

        self.apply_can_button = QPushButton("Apply CAN IDs (volatile)")
        self.apply_can_button.clicked.connect(self.apply_sensor_can_settings)
        note = QLabel("Filter changes take effect in hardware after save and restart.")
        note.setWordWrap(True)
        note.setStyleSheet("color: #666;")
        grid.addWidget(self.apply_can_button, 4, 0, 1, 2)
        grid.addWidget(note, 4, 2, 1, 3)
        return group

    @staticmethod
    def _fill_combo(combo: QComboBox, values: dict[int, str]) -> None:
        for value, name in values.items():
            combo.addItem(name, value)

    def _build_imu_group(self) -> QGroupBox:
        group = QGroupBox("IMU settings")
        grid = QGridLayout(group)

        self.mode_combo = QComboBox()
        for mode, name in MODE_NAMES.items():
            self.mode_combo.addItem(f"{mode}: {name}", mode)
        self.accel_fsr_combo = QComboBox()
        for index, full_scale in ACCEL_FSR_BY_INDEX.items():
            self.accel_fsr_combo.addItem(f"±{full_scale} g", index)
        self.gyro_fsr_combo = QComboBox()
        self._fill_combo(self.gyro_fsr_combo, GYRO_FSRS)
        self.accel_bw_combo = QComboBox()
        self._fill_combo(self.accel_bw_combo, ACCEL_BANDWIDTHS)
        self.gyro_bw_combo = QComboBox()
        self._fill_combo(self.gyro_bw_combo, GYRO_BANDWIDTHS)
        self.standard_average_spin = QSpinBox()
        self.standard_average_spin.setRange(1, 16)
        self.standard_average_spin.setToolTip(
            "Sliding N-sample mean at 1 kHz; the result updates for every new sample."
        )
        self.rms_average_spin = QSpinBox()
        self.rms_average_spin.setRange(1, 16)

        self.mode_label = QLabel("Mode")
        self.accel_fsr_label = QLabel("Accel FSR")
        self.gyro_fsr_label = QLabel("Gyro FSR")
        self.accel_bw_label = QLabel("Accel BW")
        self.gyro_bw_label = QLabel("Gyro BW")
        self.standard_average_label = QLabel("Rolling average samples")
        self.rms_average_label = QLabel("RMS samples (legacy)")
        self.vibration_window_label = QLabel("Vibration/RMS window")

        grid.addWidget(self.mode_label, 0, 0)
        grid.addWidget(self.mode_combo, 0, 1, 1, 3)
        grid.addWidget(self.accel_fsr_label, 1, 0)
        grid.addWidget(self.accel_fsr_combo, 1, 1)
        grid.addWidget(self.gyro_fsr_label, 1, 2)
        grid.addWidget(self.gyro_fsr_combo, 1, 3)
        grid.addWidget(self.accel_bw_label, 2, 0)
        grid.addWidget(self.accel_bw_combo, 2, 1)
        grid.addWidget(self.gyro_bw_label, 2, 2)
        grid.addWidget(self.gyro_bw_combo, 2, 3)
        grid.addWidget(self.standard_average_label, 3, 0)
        grid.addWidget(self.standard_average_spin, 3, 1)
        grid.addWidget(self.rms_average_label, 3, 2)
        grid.addWidget(self.rms_average_spin, 3, 3)

        self.lowpass_checkbox = QCheckBox("Low-pass")
        self.lowpass_hz_spin = QSpinBox()
        self.lowpass_hz_spin.setRange(50, 800)
        self.lowpass_hz_spin.setSuffix(" Hz")
        self.highpass_checkbox = QCheckBox("High-pass")
        self.highpass_hz_spin = QSpinBox()
        self.highpass_hz_spin.setRange(1, 200)
        self.highpass_hz_spin.setSuffix(" Hz")
        self.vibration_window_spin = QSpinBox()
        self.vibration_window_spin.setRange(10, 1000)
        self.vibration_window_spin.setSuffix(" ms")
        grid.addWidget(self.lowpass_checkbox, 4, 0)
        grid.addWidget(self.lowpass_hz_spin, 4, 1)
        grid.addWidget(self.highpass_checkbox, 4, 2)
        grid.addWidget(self.highpass_hz_spin, 4, 3)
        grid.addWidget(self.vibration_window_label, 5, 0)
        grid.addWidget(self.vibration_window_spin, 5, 1)

        self.apply_imu_button = QPushButton("Apply IMU settings (volatile)")
        self.apply_imu_button.clicked.connect(self.apply_imu_settings)
        self.zero_yaw_button = QPushButton("Zero yaw")
        self.zero_yaw_button.clicked.connect(lambda: self._set_yaw_reference(0x00))
        self.yaw_target_spin = QDoubleSpinBox()
        self.yaw_target_spin.setRange(-180.0, 180.0)
        self.yaw_target_spin.setDecimals(2)
        self.yaw_target_spin.setSingleStep(5.0)
        self.yaw_target_spin.setSuffix("°")
        self.set_yaw_target_button = QPushButton("Set yaw target")
        self.set_yaw_target_button.clicked.connect(self._set_yaw_target)
        self.clear_yaw_button = QPushButton("Clear yaw reference")
        self.clear_yaw_button.clicked.connect(lambda: self._set_yaw_reference(0x02))
        grid.addWidget(self.apply_imu_button, 6, 0, 1, 2)
        grid.addWidget(self.zero_yaw_button, 6, 2)
        grid.addWidget(self.clear_yaw_button, 6, 3)
        self.yaw_target_label = QLabel("Yaw target")
        grid.addWidget(self.yaw_target_label, 7, 0)
        grid.addWidget(self.yaw_target_spin, 7, 1)
        grid.addWidget(self.set_yaw_target_button, 7, 2, 1, 2)
        self.imu_mode_note_label = QLabel()
        self.imu_mode_note_label.setWordWrap(True)
        self.imu_mode_note_label.setStyleSheet("color: #666;")
        grid.addWidget(self.imu_mode_note_label, 8, 0, 1, 4)

        self._shared_operating_widgets = [self.accel_fsr_label, self.accel_fsr_combo]
        self._normal_mode_widgets = [
            self.gyro_fsr_label,
            self.gyro_fsr_combo,
            self.accel_bw_label,
            self.accel_bw_combo,
            self.gyro_bw_label,
            self.gyro_bw_combo,
            self.standard_average_label,
            self.standard_average_spin,
        ]
        self._vibration_mode_widgets = [
            self.lowpass_checkbox,
            self.lowpass_hz_spin,
            self.highpass_checkbox,
            self.highpass_hz_spin,
            self.vibration_window_label,
            self.vibration_window_spin,
        ]
        self._fusion_mode_widgets = [
            self.zero_yaw_button,
            self.clear_yaw_button,
            self.yaw_target_label,
            self.yaw_target_spin,
            self.set_yaw_target_button,
        ]
        legacy_tooltip = (
            "Legacy stored setting: the normal path does not publish RMS and the 4 kHz "
            "vibration path uses the vibration window instead."
        )
        self.rms_average_label.setToolTip(legacy_tooltip)
        self.rms_average_spin.setToolTip(legacy_tooltip)
        self.rms_average_label.setEnabled(False)
        self.rms_average_spin.setEnabled(False)
        return group

    def _build_gyro_calibration_group(self) -> QGroupBox:
        group = QGroupBox("Gyro bias and stationary calibration")
        self.gyro_calibration_group = group
        grid = QGridLayout(group)

        self.gyro_bias_policy_combo = QComboBox()
        self.gyro_bias_policy_combo.addItem(
            "Stationary automatic (recommended)", GYRO_BIAS_MODE_STATIONARY_AUTO
        )
        self.gyro_bias_policy_combo.addItem(
            "Fixed hardware offset (no learning)", GYRO_BIAS_MODE_FIXED
        )
        self.gyro_bias_policy_combo.addItem(
            "Legacy continuous MOBILE (not motion-safe)",
            GYRO_BIAS_MODE_LEGACY_MOBILE,
        )
        self.gyro_bias_policy_combo.currentIndexChanged.connect(
            self._update_gyro_calibration_controls
        )

        self.gyro_stationary_dwell_spin = QSpinBox()
        self.gyro_stationary_dwell_spin.setRange(500, 25_000)
        self.gyro_stationary_dwell_spin.setSingleStep(100)
        self.gyro_stationary_dwell_spin.setSuffix(" ms")
        self.gyro_stationary_dwell_spin.setValue(
            DEFAULT_GYRO_STATIONARY_DWELL_MS
        )

        self.gyro_threshold_spin = QDoubleSpinBox()
        self.gyro_threshold_spin.setRange(0.050, 3.000)
        self.gyro_threshold_spin.setDecimals(3)
        self.gyro_threshold_spin.setSingleStep(0.050)
        self.gyro_threshold_spin.setSuffix(" dps")
        self.gyro_threshold_spin.setValue(DEFAULT_GYRO_THRESHOLD_MDPS / 1000.0)

        self.accel_norm_tolerance_spin = QSpinBox()
        self.accel_norm_tolerance_spin.setRange(5, 200)
        self.accel_norm_tolerance_spin.setSuffix(" mg")
        self.accel_norm_tolerance_spin.setValue(DEFAULT_ACCEL_TOLERANCE_MG)

        grid.addWidget(QLabel("Runtime bias policy"), 0, 0)
        grid.addWidget(self.gyro_bias_policy_combo, 0, 1, 1, 3)
        grid.addWidget(QLabel("Stationary dwell"), 1, 0)
        grid.addWidget(self.gyro_stationary_dwell_spin, 1, 1)
        grid.addWidget(QLabel("Corrected gyro limit"), 1, 2)
        grid.addWidget(self.gyro_threshold_spin, 1, 3)
        grid.addWidget(QLabel("|accel|-1 g tolerance"), 2, 0)
        grid.addWidget(self.accel_norm_tolerance_spin, 2, 1)

        self.apply_gyro_calibration_button = QPushButton(
            "Apply bias policy (volatile)"
        )
        self.apply_gyro_calibration_button.clicked.connect(
            self.apply_gyro_calibration_configuration
        )
        self.refresh_gyro_calibration_button = QPushButton("Refresh")
        self.refresh_gyro_calibration_button.clicked.connect(
            self.refresh_gyro_calibration_configuration
        )
        self.refresh_gyro_status_button = QPushButton("Refresh status")
        self.refresh_gyro_status_button.clicked.connect(
            self.refresh_gyro_calibration_status
        )
        self.calibrate_gyro_button = QPushButton(
            "Calibrate gyro now (stationary, any angle)..."
        )
        self.calibrate_gyro_button.clicked.connect(
            self.calibrate_gyro_stationary
        )
        grid.addWidget(self.apply_gyro_calibration_button, 3, 0, 1, 2)
        grid.addWidget(self.refresh_gyro_calibration_button, 3, 2)
        grid.addWidget(self.refresh_gyro_status_button, 3, 3)
        grid.addWidget(self.calibrate_gyro_button, 4, 0, 1, 4)

        self.gyro_calibration_status_label = QLabel(
            "Waiting for gyro-bias configuration"
        )
        self.gyro_calibration_status_label.setWordWrap(True)
        self.gyro_calibration_status_label.setStyleSheet("color: #666;")
        self.gyro_runtime_status_label = QLabel(
            "Runtime gate diagnostics have not been read"
        )
        self.gyro_runtime_status_label.setWordWrap(True)
        self.gyro_runtime_status_label.setStyleSheet("color: #666;")
        note = QLabel(
            "Gyro-only calibration works at any fixed pitch/roll. Keep the machine "
            "parked and vibration-free; it does not zero inclination or alter accel offsets."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #666;")
        grid.addWidget(self.gyro_calibration_status_label, 5, 0, 1, 4)
        grid.addWidget(self.gyro_runtime_status_label, 6, 0, 1, 4)
        grid.addWidget(note, 7, 0, 1, 4)

        self._gyro_stationary_setting_widgets = [
            self.gyro_stationary_dwell_spin,
            self.gyro_threshold_spin,
            self.accel_norm_tolerance_spin,
        ]
        self._update_gyro_calibration_controls()
        return group

    def _build_vibration_stream_group(self) -> QGroupBox:
        group = QGroupBox("High-rate filtered waveform (volatile)")
        grid = QGridLayout(group)
        self.vibration_stream_rate_combo = QComboBox()
        self.vibration_stream_rate_combo.addItem("2 kHz (adjacent-pair average)", 2)
        self.vibration_stream_rate_combo.addItem("4 kHz (every filtered sample)", 4)
        self.vibration_stream_rate_combo.setCurrentIndex(1)
        self.vibration_stream_rate_combo.currentIndexChanged.connect(
            self._update_vibration_stream_controls
        )
        self.vibration_stream_id_edit = QLineEdit("0x148")
        self.vibration_stream_id_edit.setMaximumWidth(100)
        self.vibration_stream_start_button = QPushButton("Start")
        self.vibration_stream_start_button.clicked.connect(self.start_vibration_stream)
        self.vibration_stream_stop_button = QPushButton("Stop")
        self.vibration_stream_stop_button.clicked.connect(self.stop_vibration_stream)
        self.vibration_stream_refresh_button = QPushButton("Refresh")
        self.vibration_stream_refresh_button.clicked.connect(
            self.refresh_vibration_stream_status
        )
        self.vibration_stream_log_checkbox = QCheckBox("Log every waveform frame")
        self.vibration_stream_status_label = QLabel("Disabled")
        self.vibration_stream_status_label.setWordWrap(True)
        self.vibration_stream_status_label.setStyleSheet("color: #666;")
        note = QLabel(
            "Requires vibration mode and 1 Mbit/s. The 2 kHz stream also requires the "
            "software low-pass enabled at 400 Hz or below. Its CAN ID must not overlap "
            "the sensor response-ID range."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #666;")

        grid.addWidget(QLabel("Transmit rate"), 0, 0)
        grid.addWidget(self.vibration_stream_rate_combo, 0, 1, 1, 3)
        grid.addWidget(QLabel("Standard CAN ID"), 1, 0)
        grid.addWidget(self.vibration_stream_id_edit, 1, 1)
        grid.addWidget(self.vibration_stream_start_button, 1, 2)
        grid.addWidget(self.vibration_stream_stop_button, 1, 3)
        grid.addWidget(self.vibration_stream_refresh_button, 1, 4)
        grid.addWidget(self.vibration_stream_log_checkbox, 2, 0, 1, 3)
        grid.addWidget(self.vibration_stream_status_label, 3, 0, 1, 5)
        grid.addWidget(note, 4, 0, 1, 5)
        return group

    def _build_periodic_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        toolbar = QHBoxLayout()
        self.refresh_periodic_button = QPushButton("Refresh slots")
        self.refresh_periodic_button.clicked.connect(self.refresh_periodic_settings)
        self.apply_periodic_button = QPushButton("Apply all slots (volatile)")
        self.apply_periodic_button.clicked.connect(self.apply_periodic_settings)
        self.disable_periodic_button = QPushButton("Disable all slots")
        self.disable_periodic_button.clicked.connect(self.disable_all_periodic)
        toolbar.addWidget(self.refresh_periodic_button)
        toolbar.addWidget(self.apply_periodic_button)
        toolbar.addWidget(self.disable_periodic_button)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)

        self.periodic_table = QTableWidget(8, 5)
        self.periodic_table.setHorizontalHeaderLabels(
            ("Enabled", "Command", "Subcommand", "Interval (ms)", "Sensor readback")
        )
        self.periodic_table.verticalHeader().setVisible(True)
        for task in range(1, 9):
            self.periodic_table.setVerticalHeaderItem(task - 1, self._table_item(f"Task {task}"))
            enabled = QCheckBox()
            enabled_container = QWidget()
            enabled_layout = QHBoxLayout(enabled_container)
            enabled_layout.setContentsMargins(0, 0, 0, 0)
            enabled_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
            enabled_layout.addWidget(enabled)
            command = QComboBox()
            for value, name in PERIODIC_COMMANDS.items():
                command.addItem(f"0x{value:02X} — {name}", value)
            subcommand = QComboBox()
            subcommand.setMinimumWidth(300)
            command.currentIndexChanged.connect(
                lambda _index, command_box=command, subcommand_box=subcommand:
                    self._populate_periodic_subcommands(command_box, subcommand_box)
            )
            self._populate_periodic_subcommands(command, subcommand)
            interval = QSpinBox()
            interval.setRange(1, 65_535)
            interval.setValue(100)
            interval.setSuffix(" ms")
            status = QLabel("Not read")
            self.periodic_table.setCellWidget(task - 1, 0, enabled_container)
            self.periodic_table.setCellWidget(task - 1, 1, command)
            self.periodic_table.setCellWidget(task - 1, 2, subcommand)
            self.periodic_table.setCellWidget(task - 1, 3, interval)
            self.periodic_table.setCellWidget(task - 1, 4, status)
            self._periodic_rows.append(PeriodicRow(enabled, command, subcommand, interval, status))
        header = self.periodic_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        layout.addWidget(self.periodic_table)
        note = QLabel(
            "Slots are changed in RAM first. Disabling a slot stops transmission but retains its command and "
            "interval for reuse. Save to sensor flash only after verifying the configuration."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #666;")
        layout.addWidget(note)
        return tab

    @staticmethod
    def _populate_periodic_subcommands(command: QComboBox, subcommand: QComboBox) -> None:
        subcommand.clear()
        for value, name in periodic_subcommands(int(command.currentData())):
            subcommand.addItem(f"0x{value:02X} — {name}", value)

    @staticmethod
    def _table_item(text: str):
        from PySide6.QtWidgets import QTableWidgetItem

        return QTableWidgetItem(text)

    def _build_log_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        toolbar = QHBoxLayout()
        self.pause_log_checkbox = QCheckBox("Pause display")
        self.pause_log_checkbox.toggled.connect(self._set_log_paused)
        clear_button = QPushButton("Clear log")
        clear_button.clicked.connect(self._clear_log)
        toolbar.addWidget(self.pause_log_checkbox)
        toolbar.addWidget(clear_button)
        toolbar.addStretch(1)
        layout.addLayout(toolbar)
        self.can_log_edit = QPlainTextEdit()
        self.can_log_edit.setReadOnly(True)
        self.can_log_edit.document().setMaximumBlockCount(5000)
        font = QFont("Consolas")
        font.setStyleHint(QFont.StyleHint.Monospace)
        self.can_log_edit.setFont(font)
        layout.addWidget(self.can_log_edit)
        return tab

    def _restore_settings(self) -> None:
        adapter = str(self._settings.value("adapter", "kvaser"))
        index = self.adapter_combo.findData(adapter)
        if index >= 0:
            self.adapter_combo.setCurrentIndex(index)
        bitrate = int(self._settings.value("bitrate", 250_000))
        index = self.host_baud_combo.findData(bitrate)
        if index >= 0:
            self.host_baud_combo.setCurrentIndex(index)
        sample = str(self._settings.value("sample_point", "87.5"))
        index = self.host_sample_combo.findData(sample)
        if index >= 0:
            self.host_sample_combo.setCurrentIndex(index)
        self.request_id_edit.setText(str(self._settings.value("request_id", "0x3E8")))
        self.vibration_stream_id_edit.setText(
            str(self._settings.value("vibration_stream_id", "0x148"))
        )
        stream_rate = int(self._settings.value("vibration_stream_rate_khz", 4))
        self._set_combo_data(self.vibration_stream_rate_combo, stream_rate)
        geometry = self._settings.value("geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)

    def _save_settings(self) -> None:
        adapter = str(self.adapter_combo.currentData())
        self._settings.setValue("adapter", adapter)
        channel = self.channel_combo.currentData()
        if channel is not None:
            self._channel_by_adapter[adapter] = int(channel)
        for saved_adapter, saved_channel in self._channel_by_adapter.items():
            self._settings.setValue(f"channel_{saved_adapter}", saved_channel)
        self._settings.setValue("bitrate", self.host_baud_combo.currentData())
        self._settings.setValue("sample_point", self.host_sample_combo.currentData())
        self._settings.setValue("request_id", self.request_id_edit.text().strip())
        self._settings.setValue(
            "vibration_stream_id", self.vibration_stream_id_edit.text().strip()
        )
        self._settings.setValue(
            "vibration_stream_rate_khz", self.vibration_stream_rate_combo.currentData()
        )
        self._settings.setValue("geometry", self.saveGeometry())

    def _apply_adapter_ui(self) -> None:
        is_kvaser = self.adapter_combo.currentData() == "kvaser"
        self.host_sample_combo.setEnabled(not self._connected and is_kvaser)
        self.host_sample_combo.setToolTip(
            "Kvaser bit timing sample point"
            if is_kvaser
            else "PEAK classic-CAN bitrate presets define the bit timing and sample point."
        )

    def _adapter_changed(self, _index: int) -> None:
        self._apply_adapter_ui()
        self.refresh_channels(show_error=False)

    def _channel_changed(self, _index: int) -> None:
        adapter = self.adapter_combo.currentData()
        channel = self.channel_combo.currentData()
        if adapter is not None and channel is not None:
            self._channel_by_adapter[str(adapter)] = int(channel)

    def refresh_channels(self, show_error: bool = True) -> None:
        adapter = str(self.adapter_combo.currentData())
        adapter_name = ADAPTER_NAMES[adapter]
        previous = self._channel_by_adapter.get(adapter)
        if previous is None:
            default_channel = 0 if adapter == "kvaser" else 0x51
            saved = self._settings.value(f"channel_{adapter}")
            if saved is None and adapter == "kvaser":
                saved = self._settings.value("channel", default_channel)
            previous = int(saved if saved is not None else default_channel)
        self.channel_combo.clear()
        try:
            channels = can_api_for_adapter(adapter).list_channels()
        except (KvaserError, PcanError, OSError, ValueError) as exc:
            self.channel_combo.addItem(f"{adapter_name} unavailable", None)
            if show_error:
                QMessageBox.warning(self, APP_TITLE, str(exc))
            return
        for channel in channels:
            description = f" — {channel.description}" if channel.description else ""
            self.channel_combo.addItem(
                f"{channel.name}{description}", channel.number
            )
        if not channels:
            self.channel_combo.addItem(f"No {adapter_name} channels found", None)
        index = self.channel_combo.findData(previous)
        self.channel_combo.setCurrentIndex(index if index >= 0 else 0)

    def toggle_connection(self) -> None:
        if self._connected or (self._worker is not None and self._worker.isRunning()):
            self.disconnect_can()
        else:
            self.connect_can()

    def connect_can(self) -> None:
        channel = self.channel_combo.currentData()
        if channel is None:
            QMessageBox.warning(self, APP_TITLE, "Select an available CAN channel")
            return
        try:
            parse_can_id(self.request_id_edit.text(), extended=self.request_extended_checkbox.isChecked())
        except ValueError as exc:
            QMessageBox.warning(self, APP_TITLE, str(exc))
            return
        self._save_settings()
        self.connect_button.setEnabled(False)
        self.connection_status.setText("Connecting…")
        self._worker = CanWorker(
            str(self.adapter_combo.currentData()),
            int(channel),
            int(self.host_baud_combo.currentData()),
            str(self.host_sample_combo.currentData()),
        )
        self._worker.frames_received.connect(self._handle_frames)
        self._worker.frame_transmitted.connect(self._handle_transmitted)
        self._worker.connection_changed.connect(self._connection_changed)
        self._worker.worker_error.connect(self._worker_error)
        self._worker.start()

    def disconnect_can(self) -> None:
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(2000)
            self._worker = None
        self._connection_changed(False, "")

    def _connection_changed(self, connected: bool, channel_name: str) -> None:
        self._connected = connected
        if connected:
            self._reset_can_timestamp_clock()
        else:
            if self._gyro_calibration_in_progress:
                self._finish_gyro_calibration_operation(
                    "Calibration interrupted by CAN disconnect"
                )
            self._current_mode = None
            self._current_sensor_response = None
            self._decoder.sensor_can_id = None
            self._decoder.system_mode = None
            self._firmware_version = None
            self._gyro_policy_supported = None
            self._clear_gyro_runtime_diagnostics(
                "Runtime gate diagnostics have not been read"
            )
        self.connect_button.setEnabled(True)
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.connection_status.setText(f"Connected: {channel_name}" if connected else "Disconnected")
        self.connection_status.setStyleSheet(
            "color: #197032; font-weight: bold;" if connected else "color: #a02020; font-weight: bold;"
        )
        for widget in (
            self.adapter_combo,
            self.channel_combo,
            self.refresh_channels_button,
            self.host_baud_combo,
        ):
            widget.setEnabled(not connected)
        self._apply_adapter_ui()
        self._set_connection_controls(connected)
        self._imu_mode_controls_changed()
        if connected:
            if (
                self._pending_baud_change is not None
                and self._pending_baud_change.phase == "reconnecting"
            ):
                self._pending_baud_change.phase = "confirming"
                self.statusBar().showMessage(
                    "Connected at the trial baud; confirming it with the sensorâ€¦"
                )
                QTimer.singleShot(100, self._send_baud_confirmation)
                QTimer.singleShot(6000, self._baud_confirmation_timeout)
            else:
                self.statusBar().showMessage("Connected in normal/active CAN mode; no host filters")
                QTimer.singleShot(80, self.refresh_all_settings)
        else:
            self.statusBar().showMessage("Disconnected")
        self._refresh_stream_status()
        self._refresh_vibration_stream_display()

    def _reset_can_timestamp_clock(self) -> None:
        self._can_timestamp_origin_ms = None
        self._can_timestamp_origin_host = 0.0
        self._can_timestamp_last_raw = None
        self._can_timestamp_wrap_ms = 0

    def _frame_time(self, frame: CanFrame) -> float:
        """Map the Kvaser millisecond clock to monotonic seconds, including wraparound."""
        host_time = time.monotonic()
        raw = int(frame.timestamp_ms) & 0xFFFFFFFF
        if raw == 0 and self._can_timestamp_origin_ms is None:
            return host_time
        if self._can_timestamp_origin_ms is None:
            self._can_timestamp_origin_ms = raw
            self._can_timestamp_origin_host = host_time
            self._can_timestamp_last_raw = raw
        elif (
            self._can_timestamp_last_raw is not None
            and raw < self._can_timestamp_last_raw
            and self._can_timestamp_last_raw - raw > 0x80000000
        ):
            self._can_timestamp_wrap_ms += 1 << 32
        self._can_timestamp_last_raw = raw
        extended_ms = self._can_timestamp_wrap_ms + raw
        return self._can_timestamp_origin_host + (
            extended_ms - self._can_timestamp_origin_ms
        ) / 1000.0

    def _set_connection_controls(self, enabled: bool) -> None:
        enabled = enabled and not self._gyro_calibration_in_progress
        for widget in self._connection_required:
            widget.setEnabled(enabled)
        self.live_poll_checkbox.setEnabled(enabled)
        request_editable = not self._gyro_calibration_in_progress
        self.request_id_edit.setEnabled(request_editable)
        self.request_extended_checkbox.setEnabled(request_editable)

    def _worker_error(self, message: str) -> None:
        self._append_log(f"ERROR {message}")
        QMessageBox.critical(self, APP_TITLE, message)

    def _request_id(self) -> tuple[int, bool]:
        extended = self.request_extended_checkbox.isChecked()
        return parse_can_id(self.request_id_edit.text(), extended=extended), extended

    def _request_target_changed(self, _value=None) -> None:
        if self._gyro_calibration_in_progress:
            return
        self._current_mode = None
        self._current_sensor_response = None
        self._firmware_version = None
        self._gyro_policy_supported = None
        self._decoder.sensor_can_id = None
        self._decoder.system_mode = None
        for label in (
            self.firmware_label,
            self.hardware_label,
            self.sensor_type_label,
            self.serial_label,
        ):
            label.setText("stale - refresh target")
        self._clear_gyro_runtime_diagnostics(
            "Request target changed; refresh this sensor before applying settings"
        )
        self.gyro_calibration_status_label.setText(
            "Waiting for firmware and gyro-bias configuration from the new target"
        )
        self._imu_mode_controls_changed()

    def _send(self, data: bytes) -> None:
        if not self._connected or self._worker is None:
            return
        calibration_target: Optional[tuple[int, bool]] = None
        if self._gyro_calibration_in_progress:
            command_pair = tuple(data[:2]) if len(data) >= 2 else ()
            if command_pair not in (
                (CMD_CALIBRATE_USING_GRAVITY, SUBCMD_CALIBRATE_GYRO_STATIONARY),
                (CMD_SAVE_CALIBRATION, SUBCMD_SAVE_CALIBRATION),
            ):
                return
            calibration_target = self._gyro_calibration_request
            if calibration_target is None:
                return
        if calibration_target is None:
            try:
                can_id, extended = self._request_id()
            except ValueError as exc:
                self.statusBar().showMessage(str(exc))
                return
        else:
            can_id, extended = calibration_target
        self._worker.enqueue(can_id, data, extended)

    def _schedule_payloads(self, payloads: list[bytes], spacing_ms: int = 15) -> None:
        for index, payload in enumerate(payloads):
            QTimer.singleShot(index * spacing_ms, lambda value=payload: self._send(value))

    def refresh_all_settings(self) -> None:
        if not self._connected:
            return
        self._clear_gyro_runtime_diagnostics(
            "Reading a fresh runtime gate diagnostic snapshot..."
        )
        payloads = [
            request_payload(CMD_GET_SYSTEM_MODE),
            request_payload(CMD_GET_IMU_SETTINGS),
            request_payload(CMD_GET_SENSOR_INFORMATION, 0x04),
            request_payload(CMD_GET_SENSOR_INFORMATION, 0x05),
            request_payload(CMD_GET_SENSOR_INFORMATION, 0x06),
            request_payload(CMD_GET_SENSOR_INFORMATION, 0x14),
            request_payload(CMD_GET_BAUD_RATE),
            request_payload(CMD_GET_CAN_ID),
            request_payload(CMD_GET_FILTER_ID, FILTER_STD_1_2),
            request_payload(CMD_GET_FILTER_ID, FILTER_STD_3_4),
            request_payload(CMD_GET_FILTER_ID, FILTER_EXT_1),
            request_payload(CMD_GET_FILTER_ID, FILTER_EXT_2),
            request_payload(CMD_GET_VIBRATION_CONFIGURATION),
            request_payload(CMD_GET_SAMPLING_TIME),
            request_payload(CMD_GET_YAW_REFERENCE),
            request_payload(CMD_GET_GYRO_CALIBRATION),
            request_payload(CMD_GET_VIBRATION_STREAM, 0),
            request_payload(CMD_GET_VIBRATION_STREAM, 1),
        ]
        payloads.extend(
            request_payload(CMD_GET_CALIBRATION_INFORMATION, subcommand)
            for subcommand in GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS
        )
        payloads.extend(request_payload(CMD_GET_PERIODIC_TASK, task) for task in range(1, 9))
        self._schedule_payloads(payloads)
        self.statusBar().showMessage("Refreshing sensor settings…")

    def refresh_periodic_settings(self) -> None:
        self._schedule_payloads(
            [request_payload(CMD_GET_PERIODIC_TASK, task) for task in range(1, 9)]
        )

    def refresh_vibration_stream_status(self) -> None:
        self._schedule_payloads(
            [
                request_payload(CMD_GET_VIBRATION_STREAM, 0),
                request_payload(CMD_GET_VIBRATION_STREAM, 1),
            ]
        )

    def _configured_sensor_base_id(self) -> Optional[int]:
        try:
            if bool(self.sensor_tx_format_combo.currentData()):
                return None
            return parse_can_id(self.sensor_tx_id_edit.text())
        except ValueError:
            return self._decoder.sensor_can_id

    def start_vibration_stream(self) -> None:
        try:
            stream_id = parse_can_id(self.vibration_stream_id_edit.text())
        except ValueError as exc:
            QMessageBox.warning(self, APP_TITLE, str(exc))
            return
        rate_khz = int(self.vibration_stream_rate_combo.currentData())
        if int(self.host_baud_combo.currentData()) != 1_000_000:
            QMessageBox.warning(self, APP_TITLE, "The high-rate stream requires 1 Mbit/s CAN")
            return
        if self._current_mode is None or not 2 <= self._current_mode <= 6:
            QMessageBox.warning(self, APP_TITLE, "Select and apply a vibration mode first")
            return
        if rate_khz == VIBRATION_STREAM_RATE_2KHZ and (
            not self.lowpass_checkbox.isChecked() or self.lowpass_hz_spin.value() > 400
        ):
            QMessageBox.warning(
                self,
                APP_TITLE,
                "The 2 kHz stream requires the low-pass enabled at 400 Hz or below",
            )
            return
        base_id = self._configured_sensor_base_id()
        if base_id is not None and base_id <= stream_id <= base_id + 34:
            QMessageBox.warning(
                self,
                APP_TITLE,
                f"Stream ID 0x{stream_id:X} overlaps sensor response IDs "
                f"0x{base_id:X}–0x{base_id + 34:X}",
            )
            return

        self._vibration_stream_can_id = stream_id
        self._vibration_stream_rate_khz = rate_khz
        self._vibration_stream_last_sequence = None
        self._vibration_stream_timestamp = None
        self._vibration_stream_sequence_gaps = 0
        self._vibration_stream_enabled = True
        self.waveform_chart.clear_samples()
        self._send(vibration_stream_payload(True, rate_khz, stream_id))
        self._update_vibration_stream_controls()
        self._refresh_vibration_stream_display()
        QTimer.singleShot(100, self.refresh_vibration_stream_status)
        self.statusBar().showMessage(
            f"Requested {rate_khz} kHz filtered waveform on standard ID 0x{stream_id:X}"
        )

    def stop_vibration_stream(self) -> None:
        self._send(vibration_stream_payload(False, 0, 0))
        self._vibration_stream_enabled = False
        self._update_vibration_stream_controls()
        self._refresh_vibration_stream_display()
        QTimer.singleShot(100, self.refresh_vibration_stream_status)
        self.statusBar().showMessage("Requested high-rate waveform stop")

    def _poll_live_data(self) -> None:
        if (
            not self._connected
            or self._gyro_calibration_in_progress
            or not self.live_poll_checkbox.isChecked()
        ):
            return
        self._send(request_payload(CMD_SEND_ACCELERATION, 0))
        if self._current_mode == 1:
            self._send(request_payload(CMD_SEND_INCLINATION, 2))
        elif self._current_mode is not None and self._current_mode >= 2:
            self._send(request_payload(CMD_SEND_RMS, 0))
        self._poll_counter += 1
        if self._poll_counter >= 10:
            self._poll_counter = 0
            self._send(request_payload(CMD_GET_SAMPLING_TIME))
            if self._current_mode == 1:
                self._send(request_payload(CMD_GET_YAW_REFERENCE))

    def _handle_transmitted(self, can_id: int, data: bytes, extended: bool) -> None:
        self._append_log(
            f"TX {'EXT' if extended else 'STD'} 0x{can_id:X} [{data.hex(' ').upper()}]"
        )

    def _handle_frames(self, frames: tuple[CanFrame, ...]) -> None:
        """Consume one queued Qt event containing many CAN frames."""
        for frame in frames:
            self._handle_frame(frame)

    def _combined_axis_for_frame(self, frame: CanFrame) -> Optional[int]:
        base_id = self._decoder.sensor_can_id
        if base_id is None or len(frame.data) != 8:
            return None
        axis = frame.can_id - base_id - 1
        if axis not in (0, 1, 2):
            return None
        return axis if axis in self._configured_combined_axes() else None

    def _configured_combined_axes(self) -> set[int]:
        return {
            configuration.subcommand
            for configuration in self._periodic_configurations.values()
            if (
                configuration.enabled
                and configuration.command == CMD_SEND_COMBINED_AXIS
                and configuration.subcommand in (0, 1, 2)
            )
        }

    def _clear_unconfigured_combined_axes(self) -> None:
        configured_axes = self._configured_combined_axes()
        for axis, axis_name in enumerate(("X", "Y", "Z")):
            if axis in configured_axes:
                continue
            self._queue_label_text(self.combined_axis_value_labels[axis], f"{axis_name}: —")
            for values in self._combined_latest.values():
                values[axis] = math.nan

    def _handle_combined_axis_measurement(
        self, measurement: CombinedAxisMeasurement, timestamp: float
    ) -> None:
        self._record_stream_frame(CMD_SEND_COMBINED_AXIS)
        axis_name = ("X", "Y", "Z")[measurement.axis]
        self._queue_label_text(
            self.combined_axis_value_labels[measurement.axis],
            f"{axis_name}:  Accel {measurement.acceleration_g:+.4f} g   "
            f"Linear {measurement.linear_acceleration_g:+.4f} g   "
            f"Gyro {measurement.gyro_dps:+.3f} °/s   "
            f"Inclination {measurement.inclination_degrees:+.2f}°",
        )
        self._combined_latest["acceleration"][measurement.axis] = measurement.acceleration_g
        self._combined_latest["linear_acceleration"][measurement.axis] = (
            measurement.linear_acceleration_g
        )
        self._combined_latest["gyro"][measurement.axis] = measurement.gyro_dps
        self._combined_latest["inclination"][measurement.axis] = measurement.inclination_degrees
        self.combined_accel_chart.ingest_sample(
            tuple(self._combined_latest["acceleration"]), timestamp
        )
        self.combined_linear_accel_chart.ingest_sample(
            tuple(self._combined_latest["linear_acceleration"]), timestamp
        )
        self.combined_gyro_chart.ingest_sample(tuple(self._combined_latest["gyro"]), timestamp)
        self.combined_inclination_chart.ingest_sample(
            tuple(self._combined_latest["inclination"]), timestamp
        )

    def _handle_vibration_stream_frame(self, frame: CanFrame) -> None:
        sample = decode_vibration_stream_sample(frame, self._decoder.counts_per_g)
        output_rate_hz = self._vibration_stream_rate_khz * 1000
        now = time.monotonic()
        if self._vibration_stream_last_sequence is None:
            self._vibration_stream_timestamp = now
        else:
            delta = (sample.sequence - self._vibration_stream_last_sequence) & 0xFFFF
            if delta == 0:
                return
            if delta > 0x8000:
                # A newly enabled sensor stream restarts at sequence zero.
                self._vibration_stream_timestamp = now
            else:
                self._vibration_stream_sequence_gaps += max(0, delta - 1)
                assert self._vibration_stream_timestamp is not None
                self._vibration_stream_timestamp += delta / output_rate_hz
        self._vibration_stream_last_sequence = sample.sequence
        assert self._vibration_stream_timestamp is not None
        self._vibration_stream_frames += 1
        self._vibration_stream_last_seen = now
        self.waveform_chart.ingest_sample(
            sample.acceleration_g, self._vibration_stream_timestamp
        )
        values = sample.acceleration_g
        self._queue_label_text(
            self.waveform_values_label,
            f"Sequence {sample.sequence:5d}     X {values[0]:+8.4f} g     "
            f"Y {values[1]:+8.4f} g     Z {values[2]:+8.4f} g",
        )
        if self.vibration_stream_log_checkbox.isChecked():
            self._append_log(
                f"RX STREAM STD 0x{frame.can_id:X} [{frame.data.hex(' ').upper()}] "
                f"seq={sample.sequence} X={values[0]:+.4f} Y={values[1]:+.4f} Z={values[2]:+.4f} g"
            )

    def _handle_frame(self, frame: CanFrame) -> None:
        self._frames_received += 1
        if frame.is_error:
            self._append_log(f"RX CAN ERROR flags=0x{frame.flags:X}")
            return
        if (
            not frame.is_extended
            and frame.can_id == self._vibration_stream_can_id
            and len(frame.data) == 8
        ):
            self._handle_vibration_stream_frame(frame)
            return
        timestamp = self._frame_time(frame)
        combined_axis = self._combined_axis_for_frame(frame)
        combined_measurement: Optional[CombinedAxisMeasurement] = None
        if combined_axis is not None:
            gyro_index = int(self.gyro_fsr_combo.currentData())
            combined_measurement = decode_combined_axis_measurement(
                frame,
                int(self._decoder.sensor_can_id),
                self._decoder.counts_per_g,
                GYRO_COUNTS_PER_DPS[gyro_index],
            )
            axis_name = ("X", "Y", "Z")[combined_measurement.axis]
            decoded = (
                f"COMBINED 0x09 {axis_name}: lin={combined_measurement.linear_acceleration_g:+.4f} g, "
                f"accel={combined_measurement.acceleration_g:+.4f} g, "
                f"gyro={combined_measurement.gyro_dps:+.3f} dps, "
                f"inclination={combined_measurement.inclination_degrees:+.2f} deg"
            )
        else:
            decoded = self._decoder.decode(frame)
        self._append_log(
            f"RX {'EXT' if frame.is_extended else 'STD'} 0x{frame.can_id:X} "
            f"[{frame.data.hex(' ').upper()}]" + (f"  {decoded}" if decoded else "")
        )
        if not frame.data:
            return
        self._queue_label_text(
            self.response_id_label,
            f"Response: 0x{frame.can_id:X}{' EXT' if frame.is_extended else ''}",
        )
        if combined_measurement is not None:
            self._handle_combined_axis_measurement(combined_measurement, timestamp)
            return
        data = frame.data
        command = data[0]

        if command == CMD_SEND_ACCELERATION and len(data) >= 8 and data[1] in (0x00, 0x01):
            self._record_stream_frame(CMD_SEND_ACCELERATION)
            counts = struct.unpack(">hhh", data[2:8])
            values = tuple(value / self._decoder.counts_per_g for value in counts)
            measurement = "Linear acceleration" if data[1] == 0x01 else "Acceleration"
            self._queue_label_text(
                self.accel_values_label,
                f"{measurement}: X {values[0]:+8.4f} g     Y {values[1]:+8.4f} g     "
                f"Z {values[2]:+8.4f} g",
            )
            self.accel_chart.ingest_sample(values, timestamp)
        elif command == CMD_SEND_RMS and len(data) in (3, 7):
            self._record_stream_frame(CMD_SEND_RMS)
            rms_name, rms_values = decode_rms_measurement(
                frame, self._decoder.counts_per_g, self._decoder.sensor_can_id
            )
            if len(rms_values) == 3:
                series_mode = "XYZ"
                series_names = ("X", "Y", "Z")
                chart_values = rms_values
                self._queue_label_text(
                    self.rms_values_label,
                    f"RMS XYZ: X {rms_values[0]:8.4f} g     Y {rms_values[1]:8.4f} g     "
                    f"Z {rms_values[2]:8.4f} g",
                )
            else:
                series_mode = rms_name
                series_names = (f"{rms_name} vector",)
                chart_values = (rms_values[0], 0.0, 0.0)
                self._queue_label_text(
                    self.rms_values_label,
                    f"RMS {rms_name} vector: {rms_values[0]:8.4f} g",
                )
            if series_mode != self._rms_series_mode:
                self.rms_chart.clear_samples()
                self.rms_chart.configure_series(series_names)
                self._rms_series_mode = series_mode
            self.rms_chart.ingest_sample(chart_values, timestamp)
        elif command == CMD_SEND_INCLINATION and len(data) >= 8 and data[1] <= 2:
            self._record_stream_frame(CMD_SEND_INCLINATION)
            scale = (1.0, 10.0, 100.0)[data[1]]
            values = tuple(value / scale for value in struct.unpack(">hhh", data[2:8]))
            self._queue_label_text(
                self.fusion_values_label,
                f"Roll {values[0]:+8.2f}°     Pitch {values[1]:+8.2f}°     Yaw {values[2]:+8.2f}°",
            )
            self.fusion_chart.ingest_sample(values, timestamp)
        elif command == CMD_GET_SYSTEM_MODE and len(data) >= 2:
            mode = data[2] if len(data) >= 4 and data[1] == 0 else data[1]
            response_identity = (frame.can_id, frame.is_extended)
            if self._current_sensor_response != response_identity:
                self._firmware_version = None
                self._gyro_policy_supported = None
                self._clear_gyro_runtime_diagnostics(
                    "Waiting for this sensor's firmware and gyro-policy readback"
                )
            self._current_mode = mode
            self._current_sensor_response = response_identity
            self._set_combo_data(self.mode_combo, mode)
            self._imu_mode_controls_changed()
        elif command == CMD_SET_FSR and len(data) >= 3:
            if data[1] == 1:
                self._set_combo_data(self.accel_fsr_combo, data[2])
            elif data[1] == 2:
                self._set_combo_data(self.gyro_fsr_combo, data[2])
        elif command == CMD_SET_BANDWIDTH and len(data) >= 3:
            if data[1] == 1:
                self._set_combo_data(self.accel_bw_combo, data[2])
            elif data[1] == 2:
                self._set_combo_data(self.gyro_bw_combo, data[2])
        elif command == CMD_SET_AVERAGING and len(data) >= 4:
            self.standard_average_spin.setValue(data[2])
            self.rms_average_spin.setValue(data[3])
        elif command == CMD_GET_VIBRATION_CONFIGURATION and len(data) >= 8:
            self.lowpass_hz_spin.setValue(int.from_bytes(data[4:6], "big"))
            self.lowpass_checkbox.setChecked(bool(data[1] & 0x01))
            self._update_highpass_limit()
            self.highpass_hz_spin.setValue(int.from_bytes(data[2:4], "big"))
            self.highpass_checkbox.setChecked(bool(data[1] & 0x02))
            self.vibration_window_spin.setValue(int.from_bytes(data[6:8], "big"))
        elif command == CMD_GET_SAMPLING_TIME and len(data) >= 8:
            rate = int.from_bytes(data[2:6], "big") / 1000.0
            window = int.from_bytes(data[6:8], "big")
            self.sample_rate_label.setText(f"Measured acquisition: {rate:.3f} Hz / {window} ms")
        elif command == CMD_GET_YAW_REFERENCE and len(data) >= 8:
            raw_yaw, output_yaw, offset = struct.unpack(">hhh", data[2:8])
            self.yaw_reference_label.setText(
                f"Yaw reference: {'active' if data[1] else 'clear'} · "
                f"raw {raw_yaw / 100.0:+.2f}° · output {output_yaw / 100.0:+.2f}° · "
                f"offset {offset / 100.0:+.2f}°"
            )
        elif (
            command == CMD_GET_CALIBRATION_INFORMATION
            and len(data) == 8
            and data[1] == 0x00
        ):
            if (
                self._gyro_calibration_in_progress
                or (
                    self._current_sensor_response is not None
                    and self._current_sensor_response
                    != (frame.can_id, frame.is_extended)
                )
            ):
                return
            result = data[2]
            description = CALIBRATION_RESULT_NAMES.get(
                result, f"unknown result 0x{result:02X}"
            )
            self.gyro_calibration_status_label.setText(
                f"Last calibration diagnostic: {description}; "
                f"pre-calibration measurements "
                f"{'available' if data[3] else 'not available'}, "
                f"post-calibration measurements "
                f"{'available' if data[4] else 'not available'}"
            )
        elif (
            command == CMD_GET_CALIBRATION_INFORMATION
            and len(data) == 8
            and data[1] in GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS
        ):
            if (
                self._current_sensor_response is not None
                and self._current_sensor_response
                != (frame.can_id, frame.is_extended)
            ):
                return
            try:
                subcommand, values = decode_gyro_calibration_runtime(frame)
            except ValueError as exc:
                self.gyro_runtime_status_label.setText(
                    f"Invalid gyro runtime diagnostic response: {exc}"
                )
            else:
                self._gyro_runtime_diagnostics[subcommand] = values
                self._refresh_gyro_runtime_display()
        elif command == CMD_GET_GYRO_CALIBRATION and len(data) == 8:
            if (
                self._current_sensor_response is not None
                and self._current_sensor_response
                != (frame.can_id, frame.is_extended)
            ):
                return
            try:
                configuration = GyroCalibrationConfiguration.from_frame(frame)
            except ValueError as exc:
                self.gyro_calibration_status_label.setText(
                    f"Invalid gyro-bias configuration response: {exc}"
                )
            else:
                self._gyro_policy_supported = True
                self._clear_gyro_runtime_diagnostics(
                    "Runtime diagnostics need a fresh status read"
                )
                self._set_combo_data(
                    self.gyro_bias_policy_combo, configuration.mode
                )
                self.gyro_stationary_dwell_spin.setValue(
                    configuration.dwell_ms
                )
                self.gyro_threshold_spin.setValue(
                    configuration.gyro_threshold_mdps / 1000.0
                )
                self.accel_norm_tolerance_spin.setValue(
                    configuration.accel_tolerance_mg
                )
                self.gyro_calibration_status_label.setText(
                    f"{GYRO_BIAS_MODE_NAMES[configuration.mode]}; "
                    f"dwell {configuration.dwell_ms} ms, "
                    f"gyro <= {configuration.gyro_threshold_mdps / 1000.0:.3f} dps, "
                    f"|accel|-1 g <= {configuration.accel_tolerance_mg} mg"
                )
                self._update_gyro_calibration_controls()
        elif command == CMD_GET_VIBRATION_STREAM and len(data) >= 8:
            if data[1] == 0:
                previous_id = self._vibration_stream_can_id
                previous_rate = self._vibration_stream_rate_khz
                self._vibration_stream_enabled = bool(data[2])
                if data[3] in (2, 4):
                    self._vibration_stream_rate_khz = data[3]
                    self._set_combo_data(self.vibration_stream_rate_combo, data[3])
                stream_id = int.from_bytes(data[5:7], "big")
                if 0 < stream_id <= 0x7FF:
                    self._vibration_stream_can_id = stream_id
                    self.vibration_stream_id_edit.setText(f"0x{stream_id:X}")
                if (
                    previous_id != self._vibration_stream_can_id
                    or previous_rate != self._vibration_stream_rate_khz
                ):
                    self._vibration_stream_last_sequence = None
                    self._vibration_stream_timestamp = None
                self._vibration_stream_status_flags = data[7]
                self._update_vibration_stream_controls()
            elif data[1] == 1:
                self._vibration_stream_firmware_stats = (
                    int.from_bytes(data[2:4], "big"),
                    int.from_bytes(data[4:6], "big"),
                    int.from_bytes(data[6:8], "big"),
                )
            self._refresh_vibration_stream_display()
        elif command == CMD_GET_SENSOR_INFORMATION and len(data) >= 6:
            value = int.from_bytes(data[2:6], "big")
            if data[1] == 0x04:
                if (
                    self._current_sensor_response is None
                    or self._current_sensor_response
                    == (frame.can_id, frame.is_extended)
                ):
                    self._firmware_version = value
                    if value < GYRO_POLICY_MIN_FW:
                        self._gyro_policy_supported = False
                        self.gyro_calibration_status_label.setText(
                            f"Firmware 0x{value:X} uses the legacy gyro-bias policy; "
                            "configurable stationary-auto requires firmware 0x131 or newer"
                        )
                        self._clear_gyro_runtime_diagnostics(
                            "Runtime stationary-gate diagnostics are unsupported"
                        )
                    self._update_gyro_calibration_controls()
            labels = {
                0x04: (self.firmware_label, f"0x{value:X}"),
                0x05: (self.hardware_label, f"0x{value:08X}"),
                0x06: (self.sensor_type_label, f"0x{value:08X}"),
                0x14: (self.serial_label, f"0x{value:08X}"),
            }
            if data[1] in labels:
                label, text = labels[data[1]]
                label.setText(text)
        elif command == CMD_ACKNOWLEDGE and len(data) >= 5:
            acknowledged_command = data[1]
            acknowledged_subcommand = data[2]
            code = int.from_bytes(data[3:5], "big")
            if (
                acknowledged_command == CMD_CALIBRATE_USING_GRAVITY
                and acknowledged_subcommand == SUBCMD_CALIBRATE_GYRO_STATIONARY
                and self._gyro_calibration_in_progress
                and not self._gyro_calibration_save_pending
                and self._gyro_calibration_frame_matches(frame)
            ):
                if code == 1:
                    self._gyro_calibration_save_pending = True
                    self.gyro_calibration_status_label.setText(
                        "Gyro calibration completed; saving calibration offsets..."
                    )
                    QTimer.singleShot(
                        0,
                        lambda: self._send(
                            request_payload(
                                CMD_SAVE_CALIBRATION, SUBCMD_SAVE_CALIBRATION
                            )
                        ),
                    )
                    operation = self._gyro_calibration_operation
                    QTimer.singleShot(
                        GYRO_CALIBRATION_SAVE_TIMEOUT_MS,
                        lambda value=operation: self._gyro_calibration_timeout(
                            value, True
                        ),
                    )
                else:
                    description = CALIBRATION_RESULT_NAMES.get(
                        code, f"unknown result 0x{code:04X}"
                    )
                    self._finish_gyro_calibration_operation(
                        f"Gyro calibration failed: {description}; reading diagnostics"
                    )
                    QTimer.singleShot(
                        0,
                        lambda: self._send(
                            request_payload(CMD_GET_CALIBRATION_INFORMATION, 0x00)
                        ),
                    )
            elif (
                acknowledged_command == CMD_SAVE_CALIBRATION
                and acknowledged_subcommand == SUBCMD_SAVE_CALIBRATION
                and self._gyro_calibration_in_progress
                and self._gyro_calibration_save_pending
                and self._gyro_calibration_frame_matches(frame)
            ):
                if code == 1:
                    self._finish_gyro_calibration_operation(
                        "Gyro calibration completed and saved to sensor flash"
                    )
                else:
                    self._finish_gyro_calibration_operation(
                        "Gyro calibration succeeded, but saving failed "
                        f"(code 0x{code:04X})"
                    )
            elif acknowledged_command == CMD_SET_BAUD_RATE:
                if code == 0x0001 and self._pending_baud_change is not None:
                    self.statusBar().showMessage(
                        "Sensor accepted the trial baud; reconnecting to confirm itâ€¦"
                    )
                elif code == 0x0002 and self._pending_baud_change is not None:
                    self._complete_baud_change("Sensor baud confirmed and saved")
        elif command == CMD_GET_BAUD_RATE and len(data) >= 4:
            self._set_sensor_baud_enum(data[1])
            self.auto_retransmit_checkbox.setChecked(bool(data[2]))
        elif command == CMD_GET_CAN_ID and len(data) >= 6:
            extended = data[1] == 2
            self._set_combo_data(self.sensor_tx_format_combo, extended)
            self.sensor_tx_id_edit.setText(f"0x{int.from_bytes(data[2:6], 'big'):X}")
        elif command == CMD_GET_FILTER_ID and len(data) >= 6:
            if data[1] in (FILTER_STD_1_2, FILTER_STD_3_4):
                offset = 0 if data[1] == FILTER_STD_1_2 else 2
                self.std_filter_edits[offset].setText(f"0x{int.from_bytes(data[2:4], 'big'):X}")
                self.std_filter_edits[offset + 1].setText(f"0x{int.from_bytes(data[4:6], 'big'):X}")
            elif data[1] in (FILTER_EXT_1, FILTER_EXT_2):
                offset = 0 if data[1] == FILTER_EXT_1 else 1
                self.ext_filter_edits[offset].setText(f"0x{int.from_bytes(data[2:6], 'big'):X}")
        elif command == CMD_GET_PERIODIC_TASK and len(data) >= 7:
            self._update_periodic_row(PeriodicConfiguration.from_frame(frame))
        elif command == 0xFE and len(data) >= 5:
            error = int.from_bytes(data[3:5], "big")
            calibration_nack = (
                data[1] == CMD_CALIBRATE_USING_GRAVITY
                and data[2] == SUBCMD_CALIBRATE_GYRO_STATIONARY
                and self._gyro_calibration_in_progress
                and not self._gyro_calibration_save_pending
                and self._gyro_calibration_frame_matches(frame)
            )
            save_nack = (
                data[1] == CMD_SAVE_CALIBRATION
                and data[2] == SUBCMD_SAVE_CALIBRATION
                and self._gyro_calibration_in_progress
                and self._gyro_calibration_save_pending
                and self._gyro_calibration_frame_matches(frame)
            )
            if calibration_nack or save_nack:
                stage = "calibration save" if save_nack else "gyro calibration"
                self._finish_gyro_calibration_operation(
                    f"Sensor rejected {stage} with error 0x{error:04X}"
                )
            if (
                data[1] == CMD_SET_BAUD_RATE
                and error == 0x003D
                and self._pending_baud_change is not None
                and self._pending_baud_change.phase == "confirming"
            ):
                # A retry can arrive after the first confirmation was already
                # committed but its ACK was lost. Receiving this response at
                # the new baud proves that the first confirmation succeeded.
                self._complete_baud_change(
                    "Sensor baud is saved (confirmation retry was already committed)"
                )
                return
            if (
                data[1] in (CMD_SET_GYRO_CALIBRATION, CMD_GET_GYRO_CALIBRATION)
                and (
                    self._current_sensor_response is None
                    or self._current_sensor_response
                    == (frame.can_id, frame.is_extended)
                )
            ):
                self._gyro_policy_supported = False
                self.gyro_calibration_status_label.setText(
                    "Gyro-bias configuration is unsupported or was rejected "
                    f"(error 0x{error:04X})"
                )
                self._update_gyro_calibration_controls()
            self.statusBar().showMessage(
                f"Sensor NACK: command 0x{data[1]:02X}, subcommand 0x{data[2]:02X}, error 0x{error:04X}"
            )

    @staticmethod
    def _set_combo_data(combo: QComboBox, value) -> None:
        index = combo.findData(value)
        if index >= 0:
            combo.setCurrentIndex(index)

    def _set_sensor_baud_enum(self, enum_value: int) -> None:
        for index in range(self.sensor_baud_combo.count()):
            item = self.sensor_baud_combo.itemData(index)
            if item and item[0] == enum_value:
                self.sensor_baud_combo.setCurrentIndex(index)
                return

    def _append_log(self, text: str) -> None:
        if self._log_paused:
            return
        if len(self._pending_log_lines) == self._pending_log_lines.maxlen:
            self._pending_log_lines.popleft()
            self._pending_log_dropped += 1
        self._pending_log_lines.append(f"{time.strftime('%H:%M:%S')} {text}")

    def _set_log_paused(self, paused: bool) -> None:
        self._log_paused = paused
        if paused:
            self._pending_log_lines.clear()
            self._pending_log_dropped = 0

    def _clear_log(self) -> None:
        self._pending_log_lines.clear()
        self._pending_log_dropped = 0
        self.can_log_edit.clear()

    def _flush_log(self) -> None:
        if self._log_paused or (not self._pending_log_lines and not self._pending_log_dropped):
            return
        bar = self.can_log_edit.verticalScrollBar()
        at_bottom = bar.value() >= bar.maximum() - 2
        lines: list[str] = []
        if self._pending_log_dropped:
            lines.append(
                f"{time.strftime('%H:%M:%S')} DISPLAY WARNING: "
                f"{self._pending_log_dropped} queued log lines discarded"
            )
            self._pending_log_dropped = 0
        for _ in range(min(LOG_FLUSH_BATCH_SIZE, len(self._pending_log_lines))):
            lines.append(self._pending_log_lines.popleft())
        if not lines:
            return
        self.can_log_edit.appendPlainText("\n".join(lines))
        if self.tabs.currentIndex() == self._log_tab_index and at_bottom:
            bar.setValue(bar.maximum())

    def _queue_label_text(self, label: QLabel, text: str) -> None:
        self._pending_label_text[label] = text

    def _flush_live_labels(self) -> None:
        pending = self._pending_label_text
        self._pending_label_text = {}
        for label, text in pending.items():
            label.setText(text)

    def _render_visible_chart(self, _index: Optional[int] = None) -> None:
        if not hasattr(self, "tabs") or self.tabs.currentIndex() != 0:
            return
        index = self.live_plot_tabs.currentIndex()
        chart: Optional[TimeSeriesChart]
        if index == 0:
            chart = self.accel_chart
        elif index == 1:
            chart = self.rms_chart
        elif index == 2:
            chart = self.fusion_chart
        elif index == 3:
            widget = self.combined_measurement_tabs.currentWidget()
            chart = widget if isinstance(widget, TimeSeriesChart) else None
        elif index == self._waveform_tab_index:
            chart = self.waveform_chart
        else:
            chart = None
        if chart is not None:
            chart.render_pending()

    def _update_message_rate(self) -> None:
        rate = self._frames_received - self._last_rate_count
        self._last_rate_count = self._frames_received
        charts = (
            self.accel_chart,
            self.rms_chart,
            self.fusion_chart,
            self.combined_accel_chart,
            self.combined_linear_accel_chart,
            self.combined_gyro_chart,
            self.combined_inclination_chart,
            self.waveform_chart,
        )
        plot_drops = sum(chart.dropped_sample_count for chart in charts)
        drop_text = f" · {plot_drops} plot-buffer drops" if plot_drops else ""
        self.message_rate_label.setText(
            f"{rate} frame/s · {self._frames_received} total{drop_text}"
        )
        for command in self._stream_counts:
            count = self._stream_counts[command]
            self._stream_rates[command] = count - self._stream_last_rate_counts[command]
            self._stream_last_rate_counts[command] = count
        self._vibration_stream_rate = (
            self._vibration_stream_frames - self._vibration_stream_last_rate_count
        )
        self._vibration_stream_last_rate_count = self._vibration_stream_frames
        self._refresh_stream_status()
        self._refresh_vibration_stream_display()
        if self._connected and self._vibration_stream_enabled:
            self._send(request_payload(CMD_GET_VIBRATION_STREAM, 1))

    def _live_polling_changed(self, enabled: bool) -> None:
        if enabled:
            self.live_input_label.setText(
                "Live input: dashboard polling enabled; periodic frames are also displayed"
            )
        else:
            self.live_input_label.setText(
                "Live input: periodic/passive CAN only — the dashboard sends no live-data requests"
            )
        self._refresh_stream_status()

    def _record_stream_frame(self, command: int) -> None:
        self._stream_counts[command] += 1
        self._stream_last_seen[command] = time.monotonic()

    def _stream_source(self, command: int) -> str:
        tasks = [
            configuration.task
            for configuration in self._periodic_configurations.values()
            if configuration.enabled and configuration.command == command
        ]
        periodic = (
            "periodic " + ", ".join(f"Task {task}" for task in sorted(tasks))
            if tasks
            else "no matching periodic task read back"
        )
        if self.live_poll_checkbox.isChecked():
            return f"polling + {periodic}"
        return periodic if tasks else "passive CAN; source not identified as a periodic task"

    def _refresh_stream_status(self) -> None:
        if not hasattr(self, "accel_stream_label"):
            return
        now = time.monotonic()
        for command, name, label in (
            (CMD_SEND_COMBINED_AXIS, "Combined 0x09", self.combined_stream_label),
            (CMD_SEND_ACCELERATION, "Acceleration", self.accel_stream_label),
            (CMD_SEND_RMS, "Vibration RMS", self.rms_stream_label),
            (CMD_SEND_INCLINATION, "Fusion", self.fusion_stream_label),
        ):
            seen = self._stream_last_seen[command]
            if not self._connected:
                activity = "disconnected"
            elif seen is None:
                activity = "waiting for frames"
            else:
                age = now - seen
                activity = "live" if age < 2.0 else f"stale ({age:.1f} s ago)"
            label.setText(
                f"{name}: {activity} · {self._stream_rates[command]} msg/s · "
                f"{self._stream_source(command)}"
            )
            tab_index, tab_name = self._stream_tab_index[command]
            tab_rate = self._stream_rates[command]
            self.live_plot_tabs.setTabText(
                tab_index, f"{tab_name} ({tab_rate}/s)" if tab_rate else tab_name
            )

    def _refresh_vibration_stream_display(self) -> None:
        if not hasattr(self, "waveform_stream_label"):
            return
        now = time.monotonic()
        if not self._connected:
            activity = "disconnected"
        elif self._vibration_stream_last_seen is None:
            activity = "waiting for frames" if self._vibration_stream_enabled else "disabled"
        else:
            age = now - self._vibration_stream_last_seen
            if not self._vibration_stream_enabled:
                activity = "disabled"
            else:
                activity = "live" if age < 2.0 else f"stale ({age:.1f} s ago)"

        produced, transmitted, dropped = self._vibration_stream_firmware_stats
        status = (
            f"Filtered waveform: {activity} · {self._vibration_stream_rate} msg/s · "
            f"{self._vibration_stream_rate_khz} kHz on 0x{self._vibration_stream_can_id:X} · "
            f"sequence gaps {self._vibration_stream_sequence_gaps}"
        )
        self.waveform_stream_label.setText(status)
        prerequisites = []
        if not self._vibration_stream_status_flags & 0x01:
            prerequisites.append("sensor not in vibration mode")
        if not self._vibration_stream_status_flags & 0x02:
            prerequisites.append("sensor CAN not at 1 Mbit/s")
        if self._vibration_stream_rate_khz == 2 and not self._vibration_stream_status_flags & 0x04:
            prerequisites.append("2 kHz LP requirement not met")
        details = (
            f"{'Enabled' if self._vibration_stream_enabled else 'Disabled'} · "
            f"firmware produced/sent/discarded (low 16 bits): "
            f"{produced}/{transmitted}/{dropped}"
        )
        if prerequisites:
            details += " · " + "; ".join(prerequisites)
        self.vibration_stream_status_label.setText(details)
        self.live_plot_tabs.setTabText(
            self._waveform_tab_index,
            f"Filtered waveform ({self._vibration_stream_rate}/s)"
            if self._vibration_stream_rate
            else "Filtered waveform",
        )

    def apply_sensor_can_settings(self) -> None:
        try:
            tx_extended = bool(self.sensor_tx_format_combo.currentData())
            tx_id = parse_can_id(self.sensor_tx_id_edit.text(), extended=tx_extended)
            std = [parse_can_id(edit.text()) for edit in self.std_filter_edits]
            ext = [parse_can_id(edit.text(), extended=True) for edit in self.ext_filter_edits]
        except ValueError as exc:
            QMessageBox.warning(self, APP_TITLE, str(exc))
            return
        payloads = [
            u32_setting_payload(CMD_SET_CAN_ID, 2 if tx_extended else 1, tx_id),
            padded_payload(CMD_SET_FILTER_ID, FILTER_STD_1_2, std[0].to_bytes(2, "big") + std[1].to_bytes(2, "big")),
            padded_payload(CMD_SET_FILTER_ID, FILTER_STD_3_4, std[2].to_bytes(2, "big") + std[3].to_bytes(2, "big")),
            u32_setting_payload(CMD_SET_FILTER_ID, FILTER_EXT_1, ext[0]),
            u32_setting_payload(CMD_SET_FILTER_ID, FILTER_EXT_2, ext[1]),
        ]
        self._schedule_payloads(payloads)
        QTimer.singleShot(300, self._refresh_can_settings)
        self.statusBar().showMessage("Applied volatile CAN ID/filter settings")

    def _refresh_can_settings(self) -> None:
        self._schedule_payloads(
            [
                request_payload(CMD_GET_BAUD_RATE),
                request_payload(CMD_GET_CAN_ID),
                request_payload(CMD_GET_FILTER_ID, FILTER_STD_1_2),
                request_payload(CMD_GET_FILTER_ID, FILTER_STD_3_4),
                request_payload(CMD_GET_FILTER_ID, FILTER_EXT_1),
                request_payload(CMD_GET_FILTER_ID, FILTER_EXT_2),
            ]
        )

    def change_sensor_baud(self) -> None:
        enum_value, bitrate, sample = self.sensor_baud_combo.currentData()
        transactional = (
            self._firmware_version is None
            or self._firmware_version >= CAN_BAUD_TRANSACTION_MIN_FW
        )
        behavior = (
            "The dashboard will reconnect at the trial baud and confirm it. "
            "If confirmation fails, firmware 0x12C+ automatically returns to the old baud."
            if transactional
            else
            "This older firmware saves immediately and resets; the dashboard will reconnect automatically."
        )
        answer = QMessageBox.warning(
            self,
            "Change sensor baud rate?",
            f"Change the sensor baud to {format_bitrate(bitrate)} at {sample}%?\n\n{behavior}",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        auto_retransmit = self.auto_retransmit_checkbox.isChecked()
        previous_bitrate = int(self.host_baud_combo.currentData())
        previous_sample = str(self.host_sample_combo.currentData())
        self._send(baud_rate_payload(enum_value, auto_retransmit))
        self.statusBar().showMessage("Baud change staged; waiting for the old-rate acknowledgementâ€¦")

        if transactional:
            self._pending_baud_change = PendingBaudChange(
                enum_value=enum_value,
                bitrate=bitrate,
                sample_point=sample,
                auto_retransmit=auto_retransmit,
                previous_bitrate=previous_bitrate,
                previous_sample_point=previous_sample,
            )
        else:
            self._pending_baud_change = None
        QTimer.singleShot(300, lambda: self._begin_baud_reconnect(bitrate, sample))

    def _begin_baud_reconnect(self, bitrate: int, sample_point: str) -> None:
        if self._pending_baud_change is not None:
            self._pending_baud_change.phase = "reconnecting"
        self.disconnect_can()
        self._set_combo_data(self.host_baud_combo, bitrate)
        self._set_combo_data(self.host_sample_combo, sample_point)
        self.statusBar().showMessage("Reconnecting at the selected baudâ€¦")
        QTimer.singleShot(400, self.connect_can)

    def _send_baud_confirmation(self) -> None:
        pending = self._pending_baud_change
        if pending is None or pending.phase != "confirming" or not self._connected:
            return
        pending.confirmation_attempts += 1
        self._send(
            baud_rate_payload(
                pending.enum_value,
                pending.auto_retransmit,
                confirm=True,
            )
        )
        self.statusBar().showMessage(
            f"Confirming trial baud (attempt {pending.confirmation_attempts}/3)â€¦"
        )
        if pending.confirmation_attempts < 3:
            QTimer.singleShot(900, self._send_baud_confirmation)

    def _complete_baud_change(self, detail: str) -> None:
        self._pending_baud_change = None
        self.statusBar().showMessage(detail)
        QTimer.singleShot(150, self.refresh_all_settings)

    def _baud_confirmation_timeout(self) -> None:
        pending = self._pending_baud_change
        if pending is None or pending.phase != "confirming":
            return
        pending.phase = "awaiting_rollback"
        self.statusBar().showMessage(
            "No baud confirmation received; waiting for the sensor's automatic rollbackâ€¦"
        )
        # Firmware rolls back 15 seconds after switching. This timer starts
        # roughly six seconds into that interval, so ten more seconds leaves a
        # little margin before reconnecting at the previous host baud.
        QTimer.singleShot(10000, self._reconnect_after_baud_rollback)

    def _reconnect_after_baud_rollback(self) -> None:
        pending = self._pending_baud_change
        if pending is None or pending.phase != "awaiting_rollback":
            return
        previous_bitrate = pending.previous_bitrate
        previous_sample = pending.previous_sample_point
        self._pending_baud_change = None
        self.disconnect_can()
        self._set_combo_data(self.host_baud_combo, previous_bitrate)
        self._set_combo_data(self.host_sample_combo, previous_sample)
        self.statusBar().showMessage("Reconnecting at the previous baud after rollbackâ€¦")
        QTimer.singleShot(400, self.connect_can)

    def _imu_mode_controls_changed(self, _index: Optional[int] = None) -> None:
        mode = int(self.mode_combo.currentData())
        normal_mode = mode == 1
        vibration_mode = 2 <= mode <= 6
        operating_mode = normal_mode or vibration_mode

        for widget in self._shared_operating_widgets:
            widget.setEnabled(operating_mode)
        for widget in self._normal_mode_widgets:
            widget.setEnabled(normal_mode)
        for widget in self._vibration_mode_widgets:
            widget.setEnabled(vibration_mode)
        for widget in self._fusion_mode_widgets:
            widget.setEnabled(
                normal_mode
                and self._connected
                and not self._gyro_calibration_in_progress
            )

        self.lowpass_hz_spin.setEnabled(vibration_mode and self.lowpass_checkbox.isChecked())
        self.highpass_hz_spin.setEnabled(vibration_mode and self.highpass_checkbox.isChecked())
        self.rms_average_label.setEnabled(False)
        self.rms_average_spin.setEnabled(False)

        if normal_mode:
            self.imu_mode_note_label.setText(
                "Normal/fusion: 1 kHz accel + gyro; sensor BW and standard rolling average are active. "
                "Software vibration filters/window are inactive."
            )
        elif vibration_mode:
            self.imu_mode_note_label.setText(
                "Vibration: 4 kHz accelerometer only; accel FSR and software LP/HP/window are active. "
                "Selectable sensor BW, gyro settings, and legacy sample averages are inactive."
            )
        else:
            self.imu_mode_note_label.setText(
                "Standby: no measurement output path is active; select a running mode to edit its settings."
            )
        self._populate_frequency_response_outputs(mode)
        self._update_gyro_calibration_controls()
        self._update_vibration_stream_controls()

    def _clear_gyro_runtime_diagnostics(self, message: str) -> None:
        self._gyro_runtime_diagnostics.clear()
        self.gyro_runtime_status_label.setText(message)

    def _finish_gyro_calibration_operation(self, message: str) -> None:
        self._gyro_calibration_operation += 1
        self._gyro_calibration_in_progress = False
        self._gyro_calibration_save_pending = False
        self._gyro_calibration_response = None
        self._gyro_calibration_request = None
        self.gyro_calibration_status_label.setText(message)
        self._set_connection_controls(self._connected)
        self._imu_mode_controls_changed()
        if self._connected and self._gyro_policy_supported is True:
            QTimer.singleShot(100, self.refresh_gyro_calibration_status)

    def _gyro_calibration_frame_matches(self, frame: CanFrame) -> bool:
        return self._gyro_calibration_response == (
            frame.can_id,
            frame.is_extended,
        )

    def _gyro_calibration_timeout(
        self, operation: int, waiting_for_save: bool
    ) -> None:
        if (
            operation != self._gyro_calibration_operation
            or not self._gyro_calibration_in_progress
            or self._gyro_calibration_save_pending != waiting_for_save
        ):
            return
        stage = "saving calibration" if waiting_for_save else "calibration"
        self._finish_gyro_calibration_operation(
            f"Timed out while {stage}; outcome unknown because the reply may have "
            "been lost. Controls have been re-enabled; read diagnostics before retrying"
        )
        if not waiting_for_save and self._connected:
            self._send(
                request_payload(CMD_GET_CALIBRATION_INFORMATION, 0x00)
            )

    def _update_gyro_calibration_controls(
        self, _index: Optional[int] = None
    ) -> None:
        if not hasattr(self, "gyro_calibration_group"):
            return
        normal_mode = int(self.mode_combo.currentData()) == 1
        confirmed_normal_mode = self._current_mode == 1
        policy_supported = self._gyro_policy_supported is True
        manual_supported = (
            self._firmware_version is not None
            and self._firmware_version >= GYRO_MANUAL_CALIBRATION_MIN_FW
        )
        stationary_auto = (
            int(self.gyro_bias_policy_combo.currentData())
            == GYRO_BIAS_MODE_STATIONARY_AUTO
        )
        self.gyro_calibration_group.setEnabled(normal_mode)
        self.gyro_bias_policy_combo.setEnabled(normal_mode and policy_supported)
        for widget in self._gyro_stationary_setting_widgets:
            widget.setEnabled(
                normal_mode and policy_supported and stationary_auto
            )
        can_act = (
            normal_mode
            and confirmed_normal_mode
            and self._connected
            and not self._gyro_calibration_in_progress
        )
        self.apply_gyro_calibration_button.setEnabled(
            can_act and policy_supported
        )
        self.refresh_gyro_calibration_button.setEnabled(
            can_act and policy_supported
        )
        self.refresh_gyro_status_button.setEnabled(
            can_act and policy_supported
        )
        self.calibrate_gyro_button.setEnabled(
            can_act and manual_supported
        )

    def _refresh_gyro_runtime_display(self) -> None:
        diagnostics = self._gyro_runtime_diagnostics
        lines: list[str] = []
        summary = diagnostics.get(0x0B)
        qualified = diagnostics.get(0x0C)
        if summary is not None:
            mode = int(summary["mode"])
            gate_state = int(summary["gate_state"])
            reasons = summary["rejection_reasons"]
            reason_text = ", ".join(reasons) if reasons else "none"
            qualified_text = ""
            if qualified is not None:
                qualified_text = f"; stationary {qualified['qualified_ms']} ms"
            lines.append(
                f"Runtime: {GYRO_BIAS_MODE_NAMES.get(mode, 'unknown mode')}; "
                f"{GYRO_GATE_STATE_NAMES.get(gate_state, 'unknown state')}"
                f"{qualified_text}; vendor accuracy {summary['vendor_accuracy']}; "
                f"rejection reasons: {reason_text}"
            )

        accel = diagnostics.get(0x10)
        gyro = diagnostics.get(0x11)
        metric_parts: list[str] = []
        if accel is not None:
            gravity_drift = int(accel["gravity_drift_mdeg"])
            gravity_drift_text = (
                "invalid/opposite vector"
                if gravity_drift == 0xFFFF
                else f"{gravity_drift / 1000.0:.3f} deg"
            )
            metric_parts.append(
                f"accel |a| {int(accel['accel_norm_mg']) / 1000.0:.3f} g, "
                f"noise {accel['accel_noise_mg']} mg, "
                f"drift {gravity_drift_text}"
            )
        if gyro is not None:
            metric_parts.append(
                f"gyro norm {int(gyro['gyro_norm_mdps']) / 1000.0:.3f} dps, "
                f"noise {int(gyro['gyro_noise_mdps']) / 1000.0:.3f} dps, "
                f"temp span {int(gyro['temperature_span_centi_c']) / 100.0:.2f} C"
            )
        if metric_parts:
            lines.append("Gate metrics: " + " | ".join(metric_parts))

        accepted_bias = diagnostics.get(0x0D)
        last_step = diagnostics.get(0x0E)
        counters = diagnostics.get(0x0F)
        detail_parts: list[str] = []
        if accepted_bias is not None:
            bias = accepted_bias["accepted_bias_mdps"]
            detail_parts.append(
                "bias X/Y/Z "
                f"{bias[0] / 1000.0:+.3f}/{bias[1] / 1000.0:+.3f}/{bias[2] / 1000.0:+.3f} dps"
            )
        if last_step is not None:
            step = last_step["last_bias_step_mdps"]
            detail_parts.append(
                "last step "
                f"{step[0] / 1000.0:+.3f}/{step[1] / 1000.0:+.3f}/{step[2] / 1000.0:+.3f} dps"
            )
        if counters is not None:
            detail_parts.append(
                f"accepted/rejected/rearmed {counters['accepted_count']}/"
                f"{counters['rejected_count']}/{counters['rearm_count']}"
            )
        if detail_parts:
            lines.append("Bias history: " + " | ".join(detail_parts))

        if lines:
            self.gyro_runtime_status_label.setText("\n".join(lines))

    def _imu_filter_controls_changed(self, _checked: bool) -> None:
        mode = int(self.mode_combo.currentData())
        vibration_mode = 2 <= mode <= 6
        self._update_highpass_limit()
        self.lowpass_hz_spin.setEnabled(vibration_mode and self.lowpass_checkbox.isChecked())
        self.highpass_hz_spin.setEnabled(vibration_mode and self.highpass_checkbox.isChecked())
        self._update_frequency_response()
        self._update_vibration_stream_controls()

    def _update_vibration_stream_controls(self, _value: Optional[int] = None) -> None:
        if not hasattr(self, "vibration_stream_start_button"):
            return
        vibration_mode = self._current_mode is not None and 2 <= self._current_mode <= 6
        one_mbit = int(self.host_baud_combo.currentData()) == 1_000_000
        can_start = (
            self._connected
            and vibration_mode
            and one_mbit
            and not self._gyro_calibration_in_progress
        )
        self.vibration_stream_start_button.setEnabled(
            can_start and not self._vibration_stream_enabled
        )
        self.vibration_stream_stop_button.setEnabled(
            self._connected
            and self._vibration_stream_enabled
            and not self._gyro_calibration_in_progress
        )
        self.vibration_stream_refresh_button.setEnabled(
            self._connected and not self._gyro_calibration_in_progress
        )

    def _update_highpass_limit(self, _value: Optional[int] = None) -> None:
        maximum = (
            min(200, self.lowpass_hz_spin.value() - 1)
            if self.lowpass_checkbox.isChecked()
            else 200
        )
        self.highpass_hz_spin.setMaximum(maximum)

    def _populate_frequency_response_outputs(self, mode: int) -> None:
        previous = self.frequency_response_output_combo.currentData()
        if mode == 1:
            options = (
                ("Acceleration rolling-average output", "normal_accel"),
                ("Gyroscope rolling-average output", "normal_gyro"),
            )
        elif 2 <= mode <= 6:
            options = (
                ("RMS vibration input path", "vibration_rms"),
                ("Filtered acceleration (0x0A / 4 kHz stream)", "vibration_filtered"),
                ("Filtered 2 kHz pair-average stream", "vibration_stream_2k"),
            )
        else:
            options = (("No active sampled output", "standby"),)

        self.frequency_response_output_combo.blockSignals(True)
        self.frequency_response_output_combo.clear()
        for label, value in options:
            self.frequency_response_output_combo.addItem(label, value)
        previous_index = self.frequency_response_output_combo.findData(previous)
        self.frequency_response_output_combo.setCurrentIndex(
            previous_index if previous_index >= 0 else 0
        )
        self.frequency_response_output_combo.blockSignals(False)
        self._update_frequency_response()

    def show_frequency_response_dialog(self) -> None:
        """Open a modeless response view so setting changes remain interactive."""
        if self._frequency_response_dialog is None:
            dialog = QDialog(self)
            dialog.setWindowTitle("IMU frequency response")
            dialog.setModal(False)
            dialog.setSizeGripEnabled(True)
            dialog.resize(920, 640)
            layout = QVBoxLayout(dialog)
            layout.setContentsMargins(10, 10, 10, 10)
            layout.addWidget(self.frequency_response_page)
            self._frequency_response_dialog = dialog

        self._update_frequency_response()
        self._frequency_response_dialog.show()
        self._frequency_response_dialog.raise_()
        self._frequency_response_dialog.activateWindow()

    def _update_frequency_response(self, _value: Optional[float] = None) -> None:
        if not hasattr(self, "frequency_response_chart"):
            return
        output_kind = str(self.frequency_response_output_combo.currentData())
        accel_bandwidth = ACCEL_BANDWIDTH_HZ.get(
            int(self.accel_bw_combo.currentData()), 218.0
        )
        gyro_bandwidth = GYRO_BANDWIDTH_HZ.get(
            int(self.gyro_bw_combo.currentData()), 176.0
        )
        window_ms = self.vibration_window_spin.value()
        maximum_hz, curves = calculate_frequency_response(
            output_kind,
            accel_bandwidth,
            gyro_bandwidth,
            self.standard_average_spin.value(),
            self.lowpass_checkbox.isChecked(),
            float(self.lowpass_hz_spin.value()),
            self.highpass_checkbox.isChecked(),
            float(self.highpass_hz_spin.value()),
            window_ms,
        )
        minus_three_crossings = self.frequency_response_chart.set_response(
            maximum_hz, curves
        )
        if minus_three_crossings:
            crossing_text = ", ".join(
                f"{frequency:.4g} Hz" for frequency in minus_three_crossings
            )
            self.frequency_response_crossing_label.setText(
                f"Combined −3 dB crossing{'s' if len(minus_three_crossings) != 1 else ''}: "
                f"{crossing_text}"
            )
        else:
            self.frequency_response_crossing_label.setText(
                "Combined −3 dB crossings: none within the plotted range"
            )

        if output_kind == "normal_accel":
            average_count = self.standard_average_spin.value()
            summary = (
                f"1 kHz · accel sensor BW {accel_bandwidth:g} Hz · "
                f"{average_count}-sample rolling average · updated at 1000 output/s"
            )
        elif output_kind == "normal_gyro":
            average_count = self.standard_average_spin.value()
            summary = (
                f"1 kHz · gyro sensor BW {gyro_bandwidth:g} Hz · "
                f"{average_count}-sample rolling average · updated at 1000 output/s"
            )
        elif output_kind == "vibration_rms":
            lowpass = f"LP4 {self.lowpass_hz_spin.value()} Hz" if self.lowpass_checkbox.isChecked() else "LP off"
            highpass = f"HP2 {self.highpass_hz_spin.value()} Hz" if self.highpass_checkbox.isChecked() else "HP off"
            summary = (
                f"4 kHz · fixed sensor BW ≈1046 Hz · {lowpass} · {highpass} · "
                f"RMS window {window_ms} ms ({window_ms * 4} samples; nonlinear)"
            )
        elif output_kind == "vibration_filtered":
            lowpass = f"LP4 {self.lowpass_hz_spin.value()} Hz" if self.lowpass_checkbox.isChecked() else "LP off"
            highpass = f"HP2 {self.highpass_hz_spin.value()} Hz" if self.highpass_checkbox.isChecked() else "HP off"
            summary = (
                f"4 kHz · fixed sensor BW ≈1046 Hz · {lowpass} · {highpass} · "
                "every filtered sample updates 0x0A; periodic interval limits its CAN rate"
            )
        elif output_kind == "vibration_stream_2k":
            lowpass = f"LP4 {self.lowpass_hz_spin.value()} Hz" if self.lowpass_checkbox.isChecked() else "LP off"
            highpass = f"HP2 {self.highpass_hz_spin.value()} Hz" if self.highpass_checkbox.isChecked() else "HP off"
            summary = (
                f"4 kHz acquisition · fixed sensor BW ≈1046 Hz · {lowpass} · {highpass} · "
                "adjacent-pair average and decimation to 2 kHz; LP must be ≤400 Hz"
            )
        else:
            summary = "Standby has no active sampled output or frequency response."
        self.frequency_response_summary_label.setText(summary)

    def apply_imu_settings(self) -> None:
        mode = int(self.mode_combo.currentData())
        payloads: list[bytes] = []
        if mode != 0:
            payloads.append(
                padded_payload(
                    CMD_SET_FSR, 1, bytes((int(self.accel_fsr_combo.currentData()),))
                )
            )
        if mode == 1:
            accel_bw = int(self.accel_bw_combo.currentData())
            payloads.append(
                padded_payload(CMD_SET_FSR, 2, bytes((int(self.gyro_fsr_combo.currentData()),)))
            )
            # Index zero is a legacy 218 Hz readback value that the setter does not accept.
            if accel_bw != 0:
                payloads.append(padded_payload(CMD_SET_BANDWIDTH, 1, bytes((accel_bw,))))
            payloads.extend(
                (
                    padded_payload(
                        CMD_SET_BANDWIDTH,
                        2,
                        bytes((int(self.gyro_bw_combo.currentData()),)),
                    ),
                    padded_payload(
                        CMD_SET_AVERAGING,
                        0,
                        bytes(
                            (
                                self.standard_average_spin.value(),
                                self.rms_average_spin.value(),
                            )
                        ),
                    ),
                )
            )
        elif 2 <= mode <= 6:
            flags = int(self.lowpass_checkbox.isChecked()) | (
                int(self.highpass_checkbox.isChecked()) << 1
            )
            payloads.append(
                bytes(
                    (
                        CMD_SET_VIBRATION_CONFIGURATION,
                        flags,
                        (self.highpass_hz_spin.value() >> 8) & 0xFF,
                        self.highpass_hz_spin.value() & 0xFF,
                        (self.lowpass_hz_spin.value() >> 8) & 0xFF,
                        self.lowpass_hz_spin.value() & 0xFF,
                        (self.vibration_window_spin.value() >> 8) & 0xFF,
                        self.vibration_window_spin.value() & 0xFF,
                    )
                )
            )
        payloads.append(request_payload(CMD_SET_SYSTEM_MODE, mode))
        self._schedule_payloads(payloads, spacing_ms=20)
        self.statusBar().showMessage("Applied volatile IMU settings; waiting for mode transition")
        QTimer.singleShot(500, self._refresh_imu_settings)

    def _refresh_imu_settings(self) -> None:
        self._clear_gyro_runtime_diagnostics(
            "Reading a fresh runtime gate diagnostic snapshot..."
        )
        payloads = [
            request_payload(CMD_GET_IMU_SETTINGS),
            request_payload(CMD_GET_SYSTEM_MODE),
            request_payload(CMD_GET_VIBRATION_CONFIGURATION),
            request_payload(CMD_GET_SAMPLING_TIME),
            request_payload(CMD_GET_GYRO_CALIBRATION),
            request_payload(CMD_GET_VIBRATION_STREAM, 0),
        ]
        payloads.extend(
            request_payload(CMD_GET_CALIBRATION_INFORMATION, subcommand)
            for subcommand in GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS
        )
        self._schedule_payloads(payloads)

    def _gyro_calibration_configuration(self) -> GyroCalibrationConfiguration:
        return GyroCalibrationConfiguration(
            mode=int(self.gyro_bias_policy_combo.currentData()),
            gyro_threshold_mdps=round(self.gyro_threshold_spin.value() * 1000.0),
            accel_tolerance_mg=self.accel_norm_tolerance_spin.value(),
            dwell_ms=self.gyro_stationary_dwell_spin.value(),
        )

    def apply_gyro_calibration_configuration(self) -> None:
        if self._gyro_policy_supported is not True:
            QMessageBox.warning(
                self,
                APP_TITLE,
                "This sensor has not confirmed configurable gyro-bias support.",
            )
            return
        try:
            configuration = self._gyro_calibration_configuration()
        except ValueError as exc:
            QMessageBox.warning(self, APP_TITLE, str(exc))
            return
        self._clear_gyro_runtime_diagnostics(
            "Bias policy changed; runtime diagnostics need a fresh status read"
        )
        self._send(configuration.payload())
        self.gyro_calibration_status_label.setText(
            "Applied volatile gyro-bias policy; waiting for readback"
        )
        QTimer.singleShot(100, self.refresh_gyro_calibration_configuration)

    def refresh_gyro_calibration_configuration(self) -> None:
        if (
            self._firmware_version is not None
            and self._firmware_version < GYRO_POLICY_MIN_FW
        ):
            return
        self._send(request_payload(CMD_GET_GYRO_CALIBRATION))

    def refresh_gyro_calibration_status(self) -> None:
        if self._gyro_policy_supported is not True:
            return
        self._clear_gyro_runtime_diagnostics(
            "Reading a fresh runtime gate diagnostic snapshot..."
        )
        self._schedule_payloads(
            [
                request_payload(CMD_GET_CALIBRATION_INFORMATION, subcommand)
                for subcommand in GYRO_RUNTIME_DIAGNOSTIC_SUBCOMMANDS
            ]
        )

    def calibrate_gyro_stationary(self) -> None:
        if self._current_sensor_response is None:
            QMessageBox.warning(
                self,
                APP_TITLE,
                "Refresh the sensor mode before starting gyro calibration.",
            )
            return
        if (
            self._firmware_version is None
            or self._firmware_version < GYRO_MANUAL_CALIBRATION_MIN_FW
        ):
            QMessageBox.warning(
                self,
                APP_TITLE,
                "Gyro-only calibration requires firmware 0x130 or newer.",
            )
            return
        try:
            request_id, request_extended = self._request_id()
        except ValueError as exc:
            QMessageBox.warning(self, APP_TITLE, str(exc))
            return
        request_description = (
            f"{'extended' if request_extended else 'standard'} CAN ID "
            f"0x{request_id:X}"
        )
        answer = QMessageBox.question(
            self,
            "Calibrate gyro only",
            "Park the machine and keep it completely stationary and free of vibration.\n\n"
            "The machine may be at any fixed pitch or roll. This changes only gyro "
            "offsets; it does not zero inclination or modify accelerometer offsets.\n\n"
            f"The command is sent on {request_description}. Every A2C sensor accepting "
            "that request ID may calibrate and save. Connect only the sensor being "
            "calibrated unless you have verified a unique request/filter ID.\n\n"
            "A successful result will be saved to calibration flash. Continue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._gyro_calibration_operation += 1
        operation = self._gyro_calibration_operation
        self._gyro_calibration_in_progress = True
        self._gyro_calibration_save_pending = False
        self._gyro_calibration_response = self._current_sensor_response
        self._gyro_calibration_request = (request_id, request_extended)
        self._clear_gyro_runtime_diagnostics(
            "Runtime diagnostics are paused while gyro calibration is running"
        )
        self.gyro_calibration_status_label.setText(
            "Gyro calibration running; keep the machine stationary..."
        )
        self._set_connection_controls(self._connected)
        self._update_gyro_calibration_controls()
        self._send(
            request_payload(
                CMD_CALIBRATE_USING_GRAVITY,
                SUBCMD_CALIBRATE_GYRO_STATIONARY,
            )
        )
        QTimer.singleShot(
            GYRO_CALIBRATION_TIMEOUT_MS,
            lambda value=operation: self._gyro_calibration_timeout(value, False),
        )

    def _set_yaw_reference(self, action: int) -> None:
        self._send(padded_payload(CMD_SET_YAW_REFERENCE, action))
        QTimer.singleShot(100, lambda: self._send(request_payload(CMD_GET_YAW_REFERENCE)))

    def _set_yaw_target(self) -> None:
        centidegrees = round(self.yaw_target_spin.value() * 100.0)
        self._send(
            padded_payload(
                CMD_SET_YAW_REFERENCE,
                0x01,
                int(centidegrees).to_bytes(2, "big", signed=True),
            )
        )
        QTimer.singleShot(100, lambda: self._send(request_payload(CMD_GET_YAW_REFERENCE)))

    def _periodic_configuration(self, task: int) -> PeriodicConfiguration:
        row = self._periodic_rows[task - 1]
        return PeriodicConfiguration(
            task=task,
            enabled=row.enabled.isChecked(),
            command=int(row.command.currentData()),
            subcommand=int(row.subcommand.currentData()),
            interval_ms=row.interval.value(),
        )

    def apply_periodic_settings(self) -> None:
        payloads = [self._periodic_configuration(task).payload() for task in range(1, 9)]
        self._schedule_payloads(payloads, spacing_ms=20)
        QTimer.singleShot(400, self.refresh_periodic_settings)
        self.statusBar().showMessage("Applied volatile periodic-message settings")

    def disable_all_periodic(self) -> None:
        answer = QMessageBox.question(
            self,
            "Disable periodic messages?",
            "Disable all eight periodic slots in RAM?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        payloads = []
        for task, row in enumerate(self._periodic_rows, 1):
            row.enabled.setChecked(False)
            payloads.append(self._periodic_configuration(task).payload())
        self._schedule_payloads(payloads, spacing_ms=20)
        QTimer.singleShot(400, self.refresh_periodic_settings)

    def _update_periodic_row(self, configuration: PeriodicConfiguration) -> None:
        if not 1 <= configuration.task <= 8:
            return
        self._periodic_configurations[configuration.task] = configuration
        self._clear_unconfigured_combined_axes()
        row = self._periodic_rows[configuration.task - 1]
        row.enabled.setChecked(configuration.enabled)
        index = row.command.findData(configuration.command)
        if index >= 0:
            row.command.setCurrentIndex(index)
        subcommand_index = row.subcommand.findData(configuration.subcommand)
        valid_subcommand = subcommand_index >= 0 and index >= 0
        unconfigured = (
            not configuration.enabled
            and configuration.command == 0
            and configuration.subcommand == 0
            and configuration.interval_ms == 0
        )
        if valid_subcommand:
            row.subcommand.setCurrentIndex(subcommand_index)
        if configuration.interval_ms:
            row.interval.setValue(configuration.interval_ms)
        if unconfigured:
            row.status.setText("Off · unconfigured")
        else:
            row.status.setText(
                f"{'On' if configuration.enabled else 'Off'} · 0x{configuration.command:02X}/"
                f"0x{configuration.subcommand:02X} · {configuration.interval_ms} ms"
                + (" · unsupported readback" if not valid_subcommand else "")
            )
        row.status.setStyleSheet(
            "color: #a02020;"
            if not valid_subcommand and not unconfigured
            else ("color: #197032;" if configuration.enabled else "color: #666;")
        )
        self._refresh_stream_status()

    def save_settings_to_flash(self) -> None:
        answer = QMessageBox.warning(
            self,
            "Save settings to sensor flash?",
            "This writes the current CAN, IMU, vibration, and periodic settings to nonvolatile flash. Continue?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer == QMessageBox.StandardButton.Yes:
            self._send(request_payload(CMD_SET_SAVE_PARAMETERS, 0xFF))
            self.statusBar().showMessage("Save-to-flash command sent")

    def _clear_charts(self) -> None:
        self.accel_chart.clear_samples()
        self.rms_chart.clear_samples()
        self.fusion_chart.clear_samples()
        self.combined_accel_chart.clear_samples()
        self.combined_linear_accel_chart.clear_samples()
        self.combined_gyro_chart.clear_samples()
        self.combined_inclination_chart.clear_samples()
        self.waveform_chart.clear_samples()
        self._vibration_stream_last_sequence = None
        self._vibration_stream_timestamp = None
        self._vibration_stream_sequence_gaps = 0
        for values in self._combined_latest.values():
            values[:] = [math.nan, math.nan, math.nan]

    def closeEvent(self, event: QCloseEvent) -> None:
        self.disconnect_can()
        self._save_settings()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName(APP_TITLE)
    app.setApplicationVersion(APP_VERSION)
    app.setOrganizationName("A2C")
    window = SensorDashboard()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
