import math
import os
import struct
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from a2c_sensor_tools.can_sensor_dashboard_qt import (
        CMD_CALIBRATE_USING_GRAVITY,
        CMD_GET_CALIBRATION_INFORMATION,
        CMD_GET_GYRO_CALIBRATION,
        CMD_SAVE_CALIBRATION,
        CMD_SET_FILTER_ID,
        CMD_SET_AVERAGING,
        CMD_SET_BANDWIDTH,
        CMD_SET_GYRO_CALIBRATION,
        CMD_SET_VIBRATION_CONFIGURATION,
        CMD_SET_VIBRATION_STREAM,
        CMD_SEND_ACCELERATION,
        CMD_SEND_COMBINED_AXIS,
        CanWorker,
        CombinedAxisMeasurement,
        FILTER_STD_1_2,
        GYRO_CALIBRATION_TIMEOUT_MS,
        PLOT_BUFFER_CAPACITY,
        PLOT_RENDER_POINT_LIMIT,
        PendingBaudChange,
        PeriodicConfiguration,
        GyroCalibrationConfiguration,
        SensorDashboard,
        SUBCMD_CALIBRATE_GYRO_STATIONARY,
        SUBCMD_SAVE_CALIBRATION,
        TimeSeriesChart,
        baud_rate_payload,
        block_average_magnitude,
        calculate_frequency_response,
        decode_combined_axis_measurement,
        decode_rms_measurement,
        decode_vibration_stream_sample,
        interpolate_response_db,
        min_max_decimate,
        padded_payload,
        parse_can_id,
        periodic_subcommands,
        response_level_crossings,
        request_payload,
        software_highpass_magnitude,
        software_lowpass_magnitude,
        u32_setting_payload,
        vibration_stream_payload,
    )
    import a2c_sensor_tools.can_sensor_dashboard_qt as dashboard_module
    from a2c_sensor_tools.can_sensor_monitor import CanFrame, ChannelInfo
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication, QMessageBox
except ImportError as exc:
    QT_IMPORT_ERROR = exc
else:
    QT_IMPORT_ERROR = None


@unittest.skipIf(QT_IMPORT_ERROR is not None, f"Qt 6 unavailable: {QT_IMPORT_ERROR}")
class SensorDashboardProtocolTests(unittest.TestCase):
    def test_can_id_parsing_supports_decimal_and_hex(self) -> None:
        self.assertEqual(parse_can_id("1000"), 0x3E8)
        self.assertEqual(parse_can_id("0x3E8"), 0x3E8)
        self.assertEqual(parse_can_id("18DAF110", extended=True), 0x18DAF110)

    def test_standard_id_rejects_extended_value(self) -> None:
        with self.assertRaisesRegex(ValueError, "11-bit"):
            parse_can_id("0x800")

    def test_extended_id_matches_firmware_strict_upper_bound(self) -> None:
        self.assertEqual(parse_can_id("0x1FFFFFFE", extended=True), 0x1FFFFFFE)
        with self.assertRaisesRegex(ValueError, "29-bit"):
            parse_can_id("0x1FFFFFFF", extended=True)

    def test_filter_payload_is_padded_to_classic_can_dlc(self) -> None:
        payload = padded_payload(
            CMD_SET_FILTER_ID,
            FILTER_STD_1_2,
            (1000).to_bytes(2, "big") + (1001).to_bytes(2, "big"),
        )
        self.assertEqual(payload, bytes.fromhex("69 01 03 E8 03 E9 00 00"))

    def test_u32_setting_payload_is_big_endian(self) -> None:
        self.assertEqual(
            u32_setting_payload(0x68, 0x01, 0x125),
            bytes.fromhex("68 01 00 00 01 25 00 00"),
        )

    def test_baud_change_payload_has_explicit_confirmation_marker(self) -> None:
        self.assertEqual(
            baud_rate_payload(1, True),
            bytes.fromhex("67 01 01 00 00 00 00 00"),
        )
        self.assertEqual(
            baud_rate_payload(1, True, confirm=True),
            bytes.fromhex("67 01 01 A5 00 00 00 00"),
        )

    def test_periodic_configuration_round_trip(self) -> None:
        configuration = PeriodicConfiguration(3, True, 0x0A, 0, 250)
        self.assertEqual(configuration.payload(), bytes.fromhex("52 03 01 0A 00 00 FA 00"))
        response = CanFrame(0x125, bytes.fromhex("C1 03 01 0A 00 00 FA"))
        self.assertEqual(PeriodicConfiguration.from_frame(response), configuration)

    def test_gyro_calibration_configuration_round_trip(self) -> None:
        configuration = GyroCalibrationConfiguration(
            mode=1,
            gyro_threshold_mdps=250,
            accel_tolerance_mg=30,
            dwell_ms=2000,
        )
        self.assertEqual(
            configuration.payload(),
            bytes.fromhex("5D 00 01 14 00 FA 00 1E"),
        )
        response = CanFrame(0x125, bytes.fromhex("C8 00 01 14 00 FA 00 1E"))
        self.assertEqual(GyroCalibrationConfiguration.from_frame(response), configuration)

    def test_gyro_calibration_configuration_rejects_out_of_range_values(self) -> None:
        valid = {
            "mode": 1,
            "gyro_threshold_mdps": 250,
            "accel_tolerance_mg": 30,
            "dwell_ms": 2000,
        }
        for field, value in (
            ("mode", 3),
            ("gyro_threshold_mdps", 49),
            ("gyro_threshold_mdps", 3001),
            ("accel_tolerance_mg", 4),
            ("accel_tolerance_mg", 201),
            ("dwell_ms", 499),
            ("dwell_ms", 25001),
            ("dwell_ms", 2050),
        ):
            case = dict(valid)
            case[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                GyroCalibrationConfiguration(**case)

    def test_periodic_subcommands_match_firmware_handlers(self) -> None:
        self.assertEqual(
            [value for value, _name in periodic_subcommands(0x0B)],
            [0x00, 0x01, 0x02, 0x03, 0x04, 0x05],
        )
        self.assertEqual(
            [value for value, _name in periodic_subcommands(0x0D)],
            [0x00, 0x03, 0x04, 0x05],
        )
        self.assertEqual(periodic_subcommands(0x10)[0][0], 0x00)
        self.assertEqual(periodic_subcommands(0xFE), ())

    def test_rms_measurement_decodes_xyz_and_vector_payloads(self) -> None:
        xyz = CanFrame(0x143, bytes.fromhex("10 03 E8 07 D0 0B B8"))
        self.assertEqual(decode_rms_measurement(xyz, 1000, 0x125), ("XYZ", (1.0, 2.0, 3.0)))

        xz_vector = CanFrame(0x146, bytes.fromhex("10 03 E8"))
        self.assertEqual(decode_rms_measurement(xz_vector, 1000, 0x125), ("XZ", (1.0,)))

    def test_combined_axis_measurement_uses_can_id_for_axis(self) -> None:
        frame = CanFrame(0x127, struct.pack(">hhhh", 100, 1000, 320, -4500))
        self.assertEqual(
            decode_combined_axis_measurement(frame, 0x125, 1000, 32.0),
            CombinedAxisMeasurement(
                axis=1,
                linear_acceleration_g=0.1,
                acceleration_g=1.0,
                gyro_dps=10.0,
                inclination_degrees=-45.0,
            ),
        )

    def test_vibration_stream_payload_and_sample_are_big_endian(self) -> None:
        self.assertEqual(
            vibration_stream_payload(True, 4, 0x148),
            bytes.fromhex("5C 01 04 02 01 48 00 00"),
        )
        self.assertEqual(
            vibration_stream_payload(False, 0, 0),
            bytes.fromhex("5C 00 00 00 00 00 00 00"),
        )
        frame = CanFrame(0x148, struct.pack(">Hhhh", 0x1234, 1000, -2000, 3000))
        sample = decode_vibration_stream_sample(frame, 1000)
        self.assertEqual(sample.sequence, 0x1234)
        self.assertEqual(sample.acceleration_g, (1.0, -2.0, 3.0))


@unittest.skipIf(QT_IMPORT_ERROR is not None, f"Qt 6 unavailable: {QT_IMPORT_ERROR}")
class SensorDashboardPlotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_peak_selection_disables_kvaser_sample_point(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        peak_index = dashboard.adapter_combo.findData("peak")
        kvaser_index = dashboard.adapter_combo.findData("kvaser")

        dashboard.adapter_combo.setCurrentIndex(peak_index)
        self.assertFalse(dashboard.host_sample_combo.isEnabled())
        self.assertIn("PEAK", dashboard.host_sample_combo.toolTip())

        dashboard.adapter_combo.setCurrentIndex(kvaser_index)
        self.assertTrue(dashboard.host_sample_combo.isEnabled())
        dashboard.deleteLater()

    def test_peak_channel_refresh_uses_pcan_backend(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        dashboard.adapter_combo.setCurrentIndex(
            dashboard.adapter_combo.findData("peak")
        )
        peak_api = unittest.mock.Mock()
        peak_api.list_channels.return_value = [
            ChannelInfo(0x51, "PCAN_USBBUS1", "PEAK PCAN-USB, available")
        ]
        with patch.object(
            dashboard_module, "can_api_for_adapter", return_value=peak_api
        ) as factory:
            dashboard.refresh_channels(show_error=False)

        factory.assert_called_once_with("peak")
        self.assertEqual(dashboard.channel_combo.currentData(), 0x51)
        self.assertIn("PCAN_USBBUS1", dashboard.channel_combo.currentText())
        dashboard.deleteLater()

    def test_can_worker_opens_selected_peak_channel(self) -> None:
        worker = CanWorker("peak", 0x51, 250_000, "87.5")

        class FakeChannel:
            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc, traceback) -> None:
                pass

            def read(self, _timeout_ms: int):
                worker.stop()
                return None

        class FakeApi:
            def __init__(self) -> None:
                self.opened = None

            def list_channels(self):
                return [ChannelInfo(0x51, "PCAN_USBBUS1", "available")]

            def open_channel(self, channel: int, bitrate: int, sample_point: str):
                self.opened = (channel, bitrate, sample_point)
                return FakeChannel()

        api = FakeApi()
        connection_events = []
        worker.connection_changed.connect(
            lambda connected, name: connection_events.append((connected, name))
        )
        with patch.object(
            dashboard_module, "can_api_for_adapter", return_value=api
        ) as factory:
            worker.run()

        factory.assert_called_once_with("peak")
        self.assertEqual(api.opened, (0x51, 250_000, "87.5"))
        self.assertTrue(connection_events[0][0])
        self.assertIn("PEAK PCAN-Basic", connection_events[0][1])
        self.assertEqual(connection_events[-1], (False, ""))
        worker.deleteLater()

    def test_min_max_decimation_preserves_extrema_and_bounds_output(self) -> None:
        points = [(float(index), 0.0) for index in range(5_000)]
        points[1_234] = (1_234.0, 87.0)
        points[3_456] = (3_456.0, -91.0)

        rendered = min_max_decimate(points, PLOT_RENDER_POINT_LIMIT)

        self.assertLessEqual(len(rendered), PLOT_RENDER_POINT_LIMIT)
        self.assertEqual(rendered[0], points[0])
        self.assertEqual(rendered[-1], points[-1])
        self.assertIn(points[1_234], rendered)
        self.assertIn(points[3_456], rendered)

    def test_ingestion_does_not_render_until_requested(self) -> None:
        chart = TimeSeriesChart("Test", ("X", "Y", "Z"), "g")
        for index in range(3_000):
            chart.ingest_sample(
                (float(index), float(-index), float(index % 17)), index / 300.0
            )

        self.assertEqual(chart._series[0].count(), 0)
        self.assertEqual(chart.buffered_sample_count, 3_000)
        self.assertTrue(chart.render_pending())
        self.assertLessEqual(chart._series[0].count(), PLOT_RENDER_POINT_LIMIT)
        self.assertFalse(chart.render_pending())
        chart.deleteLater()

    def test_software_filter_models_are_minus_three_db_at_cutoff(self) -> None:
        expected = 1.0 / (2.0**0.5)
        self.assertAlmostEqual(software_highpass_magnitude(10.0, 10.0), expected, places=5)
        self.assertAlmostEqual(software_lowpass_magnitude(400.0, 400.0), expected, places=5)

    def test_block_average_model_has_expected_first_null(self) -> None:
        self.assertAlmostEqual(block_average_magnitude(250.0, 1000.0, 4), 0.0, places=12)
        self.assertEqual(block_average_magnitude(250.0, 1000.0, 1), 1.0)

    def test_response_cursor_interpolation_and_minus_three_crossing_are_logarithmic(self) -> None:
        values = [(1.0, 0.0), (10.0, -6.0)]
        expected_frequency = math.sqrt(10.0)
        self.assertAlmostEqual(
            interpolate_response_db(values, expected_frequency), -3.0, places=10
        )
        self.assertEqual(len(response_level_crossings(values)), 1)
        self.assertAlmostEqual(
            response_level_crossings(values)[0], expected_frequency, places=10
        )

    def test_response_model_uses_only_stages_in_selected_firmware_path(self) -> None:
        normal_maximum, normal = calculate_frequency_response(
            "normal_accel", 99.0, 92.0, 4, True, 400.0, True, 10.0, 100
        )
        self.assertEqual(normal_maximum, 500.0)
        self.assertTrue(any(name.startswith("Rolling average") for name in normal))
        self.assertFalse(any(name.startswith("Software") for name in normal))

        vibration_maximum, vibration = calculate_frequency_response(
            "vibration_rms", 99.0, 92.0, 4, True, 400.0, True, 10.0, 100
        )
        self.assertEqual(vibration_maximum, 2000.0)
        self.assertTrue(any(name.startswith("Software LP4") for name in vibration))
        self.assertTrue(any(name.startswith("Software HP2") for name in vibration))
        self.assertFalse(any(name.startswith("Rolling average") for name in vibration))

        filtered_maximum, filtered = calculate_frequency_response(
            "vibration_filtered", 99.0, 92.0, 4, True, 400.0, True, 10.0, 100
        )
        self.assertEqual(filtered_maximum, 2000.0)
        self.assertTrue(any(name.startswith("Software HP2") for name in filtered))
        self.assertFalse(any("average" in name.lower() for name in filtered))

        stream_maximum, stream = calculate_frequency_response(
            "vibration_stream_2k", 99.0, 92.0, 4, True, 400.0, True, 10.0, 100
        )
        self.assertEqual(stream_maximum, 1000.0)
        self.assertTrue(any(name.startswith("Adjacent-pair average") for name in stream))

    def test_plot_buffer_is_bounded_and_reports_overflow(self) -> None:
        chart = TimeSeriesChart("Test", ("X", "Y", "Z"), "g")
        for index in range(PLOT_BUFFER_CAPACITY + 7):
            chart.ingest_sample((float(index), 0.0, 0.0), 1.0)

        self.assertEqual(chart.buffered_sample_count, PLOT_BUFFER_CAPACITY)
        self.assertEqual(chart.dropped_sample_count, 7)
        chart.deleteLater()

    def test_dashboard_coalesces_high_rate_frames_and_renders_only_visible_chart(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        for index in range(150):
            counts = struct.pack(">hhh", index, -index, 1_000)
            dashboard._handle_frame(
                CanFrame(
                    0x125,
                    bytes((CMD_SEND_ACCELERATION, 0)) + counts,
                    timestamp_ms=10_000 + index * 20,
                )
            )

        self.assertEqual(dashboard.accel_chart.buffered_sample_count, 150)
        self.assertAlmostEqual(dashboard.accel_chart._points[-1][0], 2.98, places=6)
        self.assertEqual(dashboard.accel_chart._series[0].count(), 0)
        self.assertEqual(len(dashboard._pending_log_lines), 150)
        self.assertEqual(len(dashboard._pending_label_text), 2)

        dashboard.rms_chart.ingest_sample((1.0, 2.0, 3.0), 1.0)
        dashboard._render_visible_chart()
        self.assertGreater(dashboard.accel_chart._series[0].count(), 0)
        self.assertEqual(dashboard.rms_chart._series[0].count(), 0)

        dashboard.live_plot_tabs.setCurrentIndex(1)
        self.assertEqual(dashboard.rms_chart._series[0].count(), 1)
        dashboard._flush_live_labels()
        self.assertIn("+0.1490", dashboard.accel_values_label.text())
        dashboard._flush_log()
        self.assertFalse(dashboard._pending_log_lines)

        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_combined_measurements_have_persistent_axis_rows(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        dashboard._periodic_configurations = {
            1: PeriodicConfiguration(1, True, CMD_SEND_COMBINED_AXIS, 0, 20),
            2: PeriodicConfiguration(2, False, CMD_SEND_COMBINED_AXIS, 1, 20),
            3: PeriodicConfiguration(3, True, CMD_SEND_COMBINED_AXIS, 2, 20),
        }
        dashboard._clear_unconfigured_combined_axes()
        dashboard._handle_combined_axis_measurement(
            CombinedAxisMeasurement(0, 0.1, 1.0, 2.0, 3.0), 1.0
        )
        dashboard._handle_combined_axis_measurement(
            CombinedAxisMeasurement(2, -0.1, -1.0, -2.0, -3.0), 1.02
        )
        dashboard._flush_live_labels()

        rows = [label.text() for label in dashboard.combined_axis_value_labels]
        self.assertEqual(len(rows), 3)
        self.assertTrue(rows[0].startswith("X:  Accel +1.0000 g"))
        self.assertEqual(rows[1], "Y: —")
        self.assertTrue(rows[2].startswith("Z:  Accel -1.0000 g"))

        dashboard._update_periodic_row(
            PeriodicConfiguration(3, False, CMD_SEND_COMBINED_AXIS, 2, 20)
        )
        dashboard._flush_live_labels()
        self.assertEqual(dashboard.combined_axis_value_labels[2].text(), "Z: —")

        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_imu_controls_and_apply_payloads_follow_selected_mode(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        captured: list[bytes] = []
        dashboard._schedule_payloads = lambda payloads, spacing_ms=15: captured.extend(payloads)

        dashboard._set_combo_data(dashboard.mode_combo, 1)
        self.assertTrue(dashboard.accel_bw_combo.isEnabled())
        self.assertTrue(dashboard.gyro_fsr_combo.isEnabled())
        self.assertFalse(dashboard.highpass_checkbox.isEnabled())
        self.assertFalse(dashboard.rms_average_spin.isEnabled())
        dashboard.apply_imu_settings()
        normal_commands = [payload[0] for payload in captured]
        self.assertIn(CMD_SET_BANDWIDTH, normal_commands)
        self.assertIn(CMD_SET_AVERAGING, normal_commands)
        self.assertNotIn(CMD_SET_VIBRATION_CONFIGURATION, normal_commands)

        captured.clear()
        dashboard._set_combo_data(dashboard.mode_combo, 2)
        dashboard.highpass_checkbox.setChecked(True)
        self.assertFalse(dashboard.accel_bw_combo.isEnabled())
        self.assertFalse(dashboard.gyro_fsr_combo.isEnabled())
        self.assertTrue(dashboard.highpass_checkbox.isEnabled())
        self.assertTrue(dashboard.highpass_hz_spin.isEnabled())
        self.assertEqual(dashboard.frequency_response_output_combo.currentData(), "vibration_rms")
        self.assertGreater(len(dashboard.frequency_response_chart._series), 0)
        dashboard.apply_imu_settings()
        vibration_commands = [payload[0] for payload in captured]
        self.assertIn(CMD_SET_VIBRATION_CONFIGURATION, vibration_commands)
        self.assertNotIn(CMD_SET_BANDWIDTH, vibration_commands)
        self.assertNotIn(CMD_SET_AVERAGING, vibration_commands)

        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_gyro_bias_controls_and_readback_follow_selected_policy(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        dashboard._connected = True
        dashboard._set_combo_data(dashboard.mode_combo, 1)
        dashboard._update_gyro_calibration_controls()

        self.assertTrue(dashboard.gyro_calibration_group.isEnabled())
        self.assertFalse(dashboard.apply_gyro_calibration_button.isEnabled())
        dashboard._current_mode = 1
        dashboard._firmware_version = 0x130
        dashboard._gyro_policy_supported = False
        dashboard._update_gyro_calibration_controls()
        self.assertFalse(dashboard.apply_gyro_calibration_button.isEnabled())
        self.assertTrue(dashboard.calibrate_gyro_button.isEnabled())

        dashboard._firmware_version = 0x131
        dashboard._gyro_policy_supported = True
        dashboard._update_gyro_calibration_controls()
        self.assertTrue(dashboard.apply_gyro_calibration_button.isEnabled())
        self.assertTrue(dashboard.gyro_stationary_dwell_spin.isEnabled())
        self.assertTrue(dashboard.gyro_threshold_spin.isEnabled())
        self.assertTrue(dashboard.accel_norm_tolerance_spin.isEnabled())

        dashboard._set_combo_data(dashboard.gyro_bias_policy_combo, 2)
        self.assertFalse(dashboard.gyro_stationary_dwell_spin.isEnabled())
        self.assertFalse(dashboard.gyro_threshold_spin.isEnabled())
        self.assertFalse(dashboard.accel_norm_tolerance_spin.isEnabled())
        self.assertTrue(dashboard.apply_gyro_calibration_button.isEnabled())

        dashboard._handle_frame(
            CanFrame(0x125, bytes.fromhex("C8 00 01 14 00 FA 00 1E"))
        )
        self.assertEqual(dashboard.gyro_bias_policy_combo.currentData(), 1)
        self.assertEqual(dashboard.gyro_stationary_dwell_spin.value(), 2000)
        self.assertAlmostEqual(dashboard.gyro_threshold_spin.value(), 0.250)
        self.assertEqual(dashboard.accel_norm_tolerance_spin.value(), 30)
        self.assertIn("stationary automatic", dashboard.gyro_calibration_status_label.text())

        dashboard._handle_frame(
            CanFrame(0x125, bytes.fromhex("18 0B 01 01 02 03 00 11"))
        )
        dashboard._handle_frame(
            CanFrame(0x125, bytes.fromhex("18 0C 00 00 07 D0 00 00"))
        )
        dashboard._handle_frame(
            CanFrame(0x125, bytes.fromhex("18 10 03 E8 00 05 00 64"))
        )
        dashboard._handle_frame(
            CanFrame(0x125, bytes.fromhex("18 11 00 FA 00 32 00 19"))
        )
        runtime_text = dashboard.gyro_runtime_status_label.text()
        self.assertIn("qualifying stationary", runtime_text)
        self.assertIn("stationary 2000 ms", runtime_text)
        self.assertIn("accel norm, gyro noise", runtime_text)
        self.assertIn("accel |a| 1.000 g", runtime_text)
        self.assertIn("gyro norm 0.250 dps", runtime_text)

        diagnostics_requests: list[bytes] = []
        dashboard._schedule_payloads = (
            lambda payloads, spacing_ms=15: diagnostics_requests.extend(payloads)
        )
        dashboard.refresh_gyro_calibration_status()
        self.assertEqual(
            [payload[1] for payload in diagnostics_requests],
            list(range(0x0B, 0x12)),
        )

        dashboard._set_combo_data(dashboard.mode_combo, 2)
        self.assertFalse(dashboard.gyro_calibration_group.isEnabled())

        dashboard.request_id_edit.setText("0x3E9")
        self.assertIsNone(dashboard._current_mode)
        self.assertIsNone(dashboard._current_sensor_response)
        self.assertIsNone(dashboard._gyro_policy_supported)
        self.assertFalse(dashboard.calibrate_gyro_button.isEnabled())
        self.assertEqual(dashboard.firmware_label.text(), "stale - refresh target")
        self.assertEqual(dashboard.hardware_label.text(), "stale - refresh target")
        self.assertEqual(dashboard.sensor_type_label.text(), "stale - refresh target")
        self.assertEqual(dashboard.serial_label.text(), "stale - refresh target")

        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_apply_and_manual_gyro_calibration_use_expected_can_commands(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        dashboard._connected = True
        dashboard._current_mode = 1
        dashboard._current_sensor_response = (0x125, False)
        dashboard._firmware_version = 0x131
        dashboard._gyro_policy_supported = True
        dashboard._set_combo_data(dashboard.mode_combo, 1)
        captured: list[bytes] = []
        dashboard._send = captured.append

        with patch(
            "a2c_sensor_tools.can_sensor_dashboard_qt.QTimer.singleShot",
            side_effect=lambda _delay, callback: callback(),
        ):
            dashboard.apply_gyro_calibration_configuration()
        self.assertEqual(captured[0], bytes.fromhex("5D 00 01 14 00 FA 00 1E"))
        self.assertEqual(captured[1][0], CMD_GET_GYRO_CALIBRATION)

        captured.clear()
        with patch(
            "a2c_sensor_tools.can_sensor_dashboard_qt.QMessageBox.question",
            return_value=QMessageBox.StandardButton.Yes,
        ), patch(
            "a2c_sensor_tools.can_sensor_dashboard_qt.QTimer.singleShot",
            side_effect=lambda delay, callback: callback() if delay < 1000 else None,
        ):
            dashboard.calibrate_gyro_stationary()
            self.assertEqual(
                captured,
                [
                    bytes(
                        (
                            CMD_CALIBRATE_USING_GRAVITY,
                            SUBCMD_CALIBRATE_GYRO_STATIONARY,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0,
                        )
                    )
                ],
            )
            dashboard._handle_frame(
                CanFrame(
                    0x126,
                    bytes((0xAA, CMD_CALIBRATE_USING_GRAVITY, SUBCMD_CALIBRATE_GYRO_STATIONARY, 0, 1)),
                )
            )
            self.assertEqual(len(captured), 1)
            dashboard._handle_frame(
                CanFrame(
                    0x125,
                    bytes((0xAA, CMD_CALIBRATE_USING_GRAVITY, SUBCMD_CALIBRATE_GYRO_STATIONARY, 0, 1)),
                )
            )
            self.assertEqual(captured[-1][0], CMD_SAVE_CALIBRATION)
            self.assertEqual(captured[-1][1], SUBCMD_SAVE_CALIBRATION)
            dashboard._handle_frame(
                CanFrame(
                    0x125,
                    bytes((0xAA, CMD_SAVE_CALIBRATION, SUBCMD_SAVE_CALIBRATION, 0, 1)),
                )
            )
            self.assertIn(
                "saved to sensor flash",
                dashboard.gyro_calibration_status_label.text(),
            )
            completed_count = len(captured)
            dashboard._handle_frame(
                CanFrame(
                    0x125,
                    bytes((0xAA, CMD_CALIBRATE_USING_GRAVITY, SUBCMD_CALIBRATE_GYRO_STATIONARY, 0, 1)),
                )
            )
            self.assertEqual(len(captured), completed_count)

            captured.clear()
            dashboard._gyro_calibration_in_progress = True
            dashboard._gyro_calibration_response = (0x125, False)
            dashboard._gyro_calibration_request = (0x3E8, False)
            dashboard._handle_frame(
                CanFrame(
                    0x125,
                    bytes((0xAA, CMD_CALIBRATE_USING_GRAVITY, SUBCMD_CALIBRATE_GYRO_STATIONARY, 0, 0)),
                )
            )
            self.assertEqual(captured[-1][:2], bytes.fromhex("18 00"))
            failure_text = dashboard.gyro_calibration_status_label.text()
            dashboard._handle_frame(
                CanFrame(0x126, bytes.fromhex("18 00 04 01 00 FF FF 00"))
            )
            self.assertEqual(
                dashboard.gyro_calibration_status_label.text(),
                failure_text,
            )
            dashboard._handle_frame(
                CanFrame(0x125, bytes.fromhex("18 00 04 01 00 FF FF 00"))
            )
            self.assertIn(
                "sensor was moving during calibration",
                dashboard.gyro_calibration_status_label.text(),
            )

        self.assertFalse(dashboard._gyro_calibration_in_progress)
        self.assertFalse(dashboard._gyro_calibration_save_pending)
        self.assertIn(
            "sensor was moving during calibration",
            dashboard.gyro_calibration_status_label.text(),
        )

        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_gyro_calibration_recovers_from_timeout_nack_and_disconnect(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        dashboard._connected = True
        dashboard._current_mode = 1
        dashboard._current_sensor_response = (0x125, False)
        dashboard._firmware_version = 0x131
        dashboard._gyro_policy_supported = True
        dashboard._set_combo_data(dashboard.mode_combo, 1)
        dashboard._update_gyro_calibration_controls()
        sent: list[bytes] = []
        dashboard._send = sent.append
        scheduled: list[tuple[int, object]] = []

        with patch(
            "a2c_sensor_tools.can_sensor_dashboard_qt.QMessageBox.question",
            return_value=QMessageBox.StandardButton.Yes,
        ), patch(
            "a2c_sensor_tools.can_sensor_dashboard_qt.QTimer.singleShot",
            side_effect=lambda delay, callback: scheduled.append((delay, callback)),
        ):
            dashboard.calibrate_gyro_stationary()

        self.assertTrue(dashboard._gyro_calibration_in_progress)
        self.assertFalse(dashboard.calibrate_gyro_button.isEnabled())
        self.assertEqual(scheduled[-1][0], GYRO_CALIBRATION_TIMEOUT_MS)
        scheduled[-1][1]()
        self.assertFalse(dashboard._gyro_calibration_in_progress)
        self.assertTrue(dashboard.calibrate_gyro_button.isEnabled())
        self.assertIn("Timed out", dashboard.gyro_calibration_status_label.text())
        self.assertIn("outcome unknown", dashboard.gyro_calibration_status_label.text())
        self.assertEqual(
            sent[-1],
            request_payload(CMD_GET_CALIBRATION_INFORMATION, 0x00),
        )

        dashboard._gyro_calibration_operation += 1
        dashboard._gyro_calibration_in_progress = True
        dashboard._gyro_calibration_response = (0x125, False)
        dashboard._gyro_calibration_request = (0x3E8, False)
        dashboard._handle_frame(
            CanFrame(
                0x125,
                bytes((0xFE, CMD_CALIBRATE_USING_GRAVITY, SUBCMD_CALIBRATE_GYRO_STATIONARY, 0x00, 0x3F)),
            )
        )
        self.assertFalse(dashboard._gyro_calibration_in_progress)
        self.assertIn("error 0x003F", dashboard.gyro_calibration_status_label.text())

        dashboard._handle_frame(
            CanFrame(0x125, bytes.fromhex("18 0B 01 01 02 03 00 00"))
        )
        self.assertTrue(dashboard._gyro_runtime_diagnostics)
        dashboard._schedule_payloads = lambda _payloads, spacing_ms=15: None
        dashboard.refresh_gyro_calibration_status()
        self.assertFalse(dashboard._gyro_runtime_diagnostics)

        dashboard._gyro_calibration_operation += 1
        dashboard._gyro_calibration_in_progress = True
        dashboard._gyro_calibration_response = (0x125, False)
        dashboard._gyro_calibration_request = (0x3E8, False)
        dashboard._gyro_runtime_diagnostics[0x0B] = {"stale": True}
        dashboard._connection_changed(False, "")
        self.assertFalse(dashboard._gyro_calibration_in_progress)
        self.assertFalse(dashboard._gyro_runtime_diagnostics)
        self.assertIsNone(dashboard._current_mode)
        self.assertIsNone(dashboard._current_sensor_response)
        self.assertIn(
            "interrupted by CAN disconnect",
            dashboard.gyro_calibration_status_label.text(),
        )

        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_gyro_calibration_busy_guard_and_request_target_are_atomic(self) -> None:
        class Worker:
            def __init__(self) -> None:
                self.frames: list[tuple[int, bytes, bool]] = []

            def enqueue(self, can_id: int, data: bytes, extended: bool) -> None:
                self.frames.append((can_id, data, extended))

        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        worker = Worker()
        dashboard._worker = worker
        dashboard._connected = True
        dashboard._current_mode = 1
        dashboard._current_sensor_response = (0x125, False)
        dashboard._gyro_calibration_in_progress = True
        dashboard._gyro_calibration_request = (0x3E8, False)
        dashboard.request_id_edit.setText("0x456")
        dashboard.live_poll_checkbox.setChecked(True)
        dashboard._set_connection_controls(True)

        self.assertFalse(dashboard.request_id_edit.isEnabled())
        self.assertFalse(dashboard.live_poll_checkbox.isEnabled())
        dashboard._poll_live_data()
        dashboard._send(request_payload(CMD_GET_GYRO_CALIBRATION))
        self.assertEqual(worker.frames, [])

        dashboard._send(
            request_payload(
                CMD_CALIBRATE_USING_GRAVITY,
                SUBCMD_CALIBRATE_GYRO_STATIONARY,
            )
        )
        dashboard._send(
            request_payload(CMD_SAVE_CALIBRATION, SUBCMD_SAVE_CALIBRATION)
        )
        self.assertEqual(
            worker.frames,
            [
                (
                    0x3E8,
                    request_payload(
                        CMD_CALIBRATE_USING_GRAVITY,
                        SUBCMD_CALIBRATE_GYRO_STATIONARY,
                    ),
                    False,
                ),
                (
                    0x3E8,
                    request_payload(
                        CMD_SAVE_CALIBRATION,
                        SUBCMD_SAVE_CALIBRATION,
                    ),
                    False,
                ),
            ],
        )

        dashboard._gyro_policy_supported = False
        dashboard._finish_gyro_calibration_operation("done")
        self.assertTrue(dashboard.request_id_edit.isEnabled())

        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_dashboard_ingests_stream_sequence_and_reports_gaps(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        dashboard._vibration_stream_can_id = 0x148
        dashboard._vibration_stream_rate_khz = 2
        dashboard._vibration_stream_enabled = True
        dashboard._handle_frame(
            CanFrame(0x148, struct.pack(">Hhhh", 10, 1000, -2000, 3000))
        )
        dashboard._handle_frame(
            CanFrame(0x148, struct.pack(">Hhhh", 12, 1100, -2100, 3100))
        )

        self.assertEqual(dashboard.waveform_chart.buffered_sample_count, 2)
        self.assertEqual(dashboard._vibration_stream_sequence_gaps, 1)
        self.assertAlmostEqual(
            dashboard.waveform_chart._points[-1][0], 2 / 2000.0, places=7
        )
        self.assertEqual(len(dashboard._pending_log_lines), 0)
        dashboard._flush_live_labels()
        self.assertIn("Sequence    12", dashboard.waveform_values_label.text())

        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_dashboard_starts_4khz_stream_with_selected_standard_id(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        sent: list[bytes] = []
        dashboard._send = sent.append
        dashboard._connected = True
        dashboard._current_mode = 2
        dashboard._set_combo_data(dashboard.host_baud_combo, 1_000_000)
        dashboard.vibration_stream_id_edit.setText("0x148")
        dashboard._set_combo_data(dashboard.vibration_stream_rate_combo, 4)

        dashboard.start_vibration_stream()

        self.assertEqual(sent, [bytes.fromhex("5C 01 04 02 01 48 00 00")])
        self.assertTrue(dashboard._vibration_stream_enabled)

        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_dashboard_recognizes_committed_baud_ack(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        dashboard._pending_baud_change = PendingBaudChange(
            enum_value=1,
            bitrate=1_000_000,
            sample_point="87.5",
            auto_retransmit=True,
            previous_bitrate=250_000,
            previous_sample_point="87.5",
            phase="confirming",
        )

        dashboard._handle_frame(CanFrame(0x125, bytes.fromhex("AA 67 01 00 02")))

        self.assertIsNone(dashboard._pending_baud_change)
        self.assertIn("confirmed and saved", dashboard.statusBar().currentMessage())
        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()

    def test_frequency_response_button_opens_live_modeless_view(self) -> None:
        with patch.object(SensorDashboard, "refresh_channels"):
            dashboard = SensorDashboard()
        tab_names = [
            dashboard.live_plot_tabs.tabText(index)
            for index in range(dashboard.live_plot_tabs.count())
        ]
        self.assertNotIn("Frequency response", tab_names)
        self.assertEqual(dashboard.frequency_response_button.text(), "Frequency response plot")
        self.assertIn("font-weight: 700", dashboard.frequency_response_button.styleSheet())

        dashboard.frequency_response_button.click()
        self.assertIsNotNone(dashboard._frequency_response_dialog)
        self.assertTrue(dashboard._frequency_response_dialog.isVisible())
        self.assertFalse(dashboard._frequency_response_dialog.isModal())

        dashboard._set_combo_data(dashboard.mode_combo, 2)
        dashboard.lowpass_checkbox.setChecked(True)
        dashboard.lowpass_hz_spin.setValue(300)
        self.assertIn("LP4 300 Hz", dashboard.frequency_response_summary_label.text())
        self.assertIn("−3 dB crossing", dashboard.frequency_response_crossing_label.text())
        cursor_values = dashboard.frequency_response_chart.values_at_frequency(100.0)
        self.assertTrue(cursor_values)
        self.assertTrue(all(math.isfinite(value) for value in cursor_values.values()))
        reference_points = dashboard.frequency_response_chart._minus_three_series.points()
        self.assertEqual(len(reference_points), 2)
        self.assertTrue(all(point.y() == -3.0 for point in reference_points))
        QApplication.processEvents()
        plot_center = dashboard.frequency_response_chart.chart().plotArea().center().toPoint()
        QTest.mouseMove(dashboard.frequency_response_chart.viewport(), plot_center)
        QApplication.processEvents()
        self.assertTrue(dashboard.frequency_response_chart._cursor_label.isVisible())
        self.assertIn("Hz", dashboard.frequency_response_chart._cursor_label.text())
        self.assertTrue(dashboard.frequency_response_chart._cursor_series.isVisible())

        dashboard._frequency_response_dialog.close()
        for timer in (
            dashboard._poll_timer,
            dashboard._rate_timer,
            dashboard._render_timer,
            dashboard._label_timer,
            dashboard._log_timer,
        ):
            timer.stop()
        dashboard.deleteLater()


if __name__ == "__main__":
    unittest.main()
