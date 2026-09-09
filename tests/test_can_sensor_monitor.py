import unittest

from a2c_sensor_tools.can_sensor_monitor import (
    CAN_MSG_STD,
    CanFrame,
    MessageStatistics,
    SensorDecoder,
    default_startup_queries,
    decode_gyro_calibration_runtime,
    gyro_rejection_reasons,
    message_stream,
    parse_integer,
    request_payload,
)


class SensorDecoderTests(unittest.TestCase):
    def test_average_is_signed_big_endian_and_scaled(self):
        decoder = SensorDecoder(accel_fsr_g=16)
        frame = CanFrame(
            0x125,
            bytes((0x0A, 0x00, 0x03, 0xE8, 0xFC, 0x18, 0x00, 0x7D)),
            CAN_MSG_STD,
        )

        result = decoder.decode(frame)

        self.assertIn("X=  +1.0000 g", result)
        self.assertIn("Y=  -1.0000 g", result)
        self.assertIn("Z=  +0.1250 g", result)
        self.assertNotIn("counts=", result)
        self.assertEqual(decoder.sensor_can_id, 0x125)

    def test_accel_fsr_reply_changes_scaling(self):
        decoder = SensorDecoder(accel_fsr_g=16)

        result = decoder.decode(CanFrame(0x321, bytes((0x59, 0x01, 0x00, 0x00))))

        self.assertIn("+/-2 g", result)
        self.assertEqual(decoder.counts_per_g, 10_000)
        self.assertEqual(decoder.sensor_can_id, 0x321)

    def test_xyz_rms_is_unsigned_and_scaled(self):
        decoder = SensorDecoder(accel_fsr_g=2)
        frame = CanFrame(0x143, bytes((0x10, 0x00, 0x64, 0x01, 0xF4, 0x27, 0x10)))

        result = decoder.decode(frame)

        self.assertIn("X=  +0.0100 g", result)
        self.assertIn("Y=  +0.0500 g", result)
        self.assertIn("Z=  +1.0000 g", result)
        self.assertNotIn("counts=", result)

    def test_vector_rms_uses_unsigned_value_and_id_offset(self):
        decoder = SensorDecoder(accel_fsr_g=16, sensor_can_id=0x125)
        frame = CanFrame(0x125 + 34, bytes((0x10, 0x9C, 0x40)))

        result = decoder.decode(frame)

        self.assertIn("RMS XYZ", result)
        self.assertIn("40.0000 g", result)
        self.assertNotIn("counts=", result)

    def test_vibration_configuration(self):
        decoder = SensorDecoder()
        frame = CanFrame(0x125, bytes((0xC5, 0x03, 0x00, 0x0A, 0x01, 0x90, 0x00, 0x64)))

        result = decoder.decode(frame)

        self.assertIn("LP=on (400 Hz)", result)
        self.assertIn("HP=on (10 Hz)", result)
        self.assertIn("window=100 ms", result)

    def test_sample_rate_diagnostics(self):
        decoder = SensorDecoder()
        frame = CanFrame(
            0x125,
            bytes((0xE4, 0x02, 0x00, 0x3D, 0x09, 0x00, 0x03, 0xE8)),
        )

        result = decoder.decode(frame)

        self.assertIn("measured=4000.000 Hz", result)
        self.assertIn("over 1000 ms", result)

    def test_yaw_reference_diagnostics_are_signed_centidegrees(self):
        decoder = SensorDecoder()
        frame = CanFrame(
            0x125,
            bytes((0xC6, 0x01, 0xFB, 0x2E, 0x11, 0xD7, 0x16, 0xA9)),
        )

        result = decoder.decode(frame)

        self.assertIn("raw=-12.34 deg", result)
        self.assertIn("output=+45.67 deg", result)
        self.assertIn("offset=+58.01 deg", result)

    def test_gyro_calibration_configuration_uses_physical_units(self):
        decoder = SensorDecoder()
        frame = CanFrame(
            0x125,
            bytes.fromhex("C8 00 01 14 00 FA 00 1E"),
        )

        result = decoder.decode(frame)

        self.assertIn("stationary automatic", result)
        self.assertIn("0.250 dps", result)
        self.assertIn("+/-30 mg", result)
        self.assertIn("dwell=2000 ms", result)
        self.assertEqual(decoder.sensor_can_id, 0x125)

    def test_gyro_runtime_summary_decodes_state_and_rejection_bits(self):
        frame = CanFrame(0x125, bytes.fromhex("18 0B 01 01 02 03 00 11"))

        subcommand, values = decode_gyro_calibration_runtime(frame)
        result = SensorDecoder().decode(frame)

        self.assertEqual(subcommand, 0x0B)
        self.assertEqual(values["version"], 1)
        self.assertEqual(values["mode"], 1)
        self.assertEqual(values["gate_state"], 2)
        self.assertEqual(values["vendor_accuracy"], 3)
        self.assertEqual(values["rejection_mask"], 0x0011)
        self.assertEqual(values["rejection_reasons"], ("accel norm", "gyro noise"))
        self.assertIn("qualifying stationary", result)
        self.assertIn("accel norm, gyro noise", result)

    def test_gyro_runtime_bias_counters_and_metrics_use_documented_units(self):
        cases = (
            (
                "18 0C 00 00 07 D0 00 00",
                {"qualified_ms": 2000},
                "qualified=2000 ms",
            ),
            (
                "18 0D FF 06 00 FA FE 0C",
                {"accepted_bias_mdps": (-250, 250, -500)},
                "X=-0.250, Y=+0.250, Z=-0.500 dps",
            ),
            (
                "18 0E 00 0A FF EC 00 1E",
                {"last_bias_step_mdps": (10, -20, 30)},
                "X=+0.010, Y=-0.020, Z=+0.030 dps",
            ),
            (
                "18 0F 00 02 00 03 00 04",
                {"accepted_count": 2, "rejected_count": 3, "rearm_count": 4},
                "accepted=2, rejected=3, rearmed=4",
            ),
            (
                "18 10 03 E8 00 05 00 64",
                {"accel_norm_mg": 1000, "accel_noise_mg": 5, "gravity_drift_mdeg": 100},
                "accel norm=1.000 g, noise=5 mg, gravity drift=0.100 deg",
            ),
            (
                "18 11 00 FA 00 32 00 19",
                {"gyro_norm_mdps": 250, "gyro_noise_mdps": 50, "temperature_span_centi_c": 25},
                "gyro norm=0.250 dps, noise=0.050 dps, temperature span=0.25 C",
            ),
        )
        decoder = SensorDecoder()
        for payload, expected_values, expected_text in cases:
            with self.subTest(payload=payload):
                frame = CanFrame(0x125, bytes.fromhex(payload))
                _subcommand, values = decode_gyro_calibration_runtime(frame)
                self.assertEqual(values, expected_values)
                self.assertIn(expected_text, decoder.decode(frame))

        saturated_drift = decoder.decode(
            CanFrame(0x125, bytes.fromhex("18 10 03 E8 00 05 FF FF"))
        )
        self.assertIn("gravity drift=invalid/opposite vector", saturated_drift)

    def test_gyro_runtime_decoder_rejects_wrong_frames_and_reports_unknown_bits(self):
        with self.assertRaises(ValueError):
            decode_gyro_calibration_runtime(
                CanFrame(0x125, bytes.fromhex("18 0A 00 00 00 00 00 00"))
            )
        self.assertEqual(
            gyro_rejection_reasons(0x0100),
            ("FIFO/SPI data quality",),
        )
        self.assertEqual(gyro_rejection_reasons(0x0200), ("unknown 0x0200",))

    def test_inclination_centidegrees(self):
        decoder = SensorDecoder()
        frame = CanFrame(
            0x125,
            bytes((0x0B, 0x02, 0x04, 0xD2, 0xFB, 0x2E, 0x11, 0xD7)),
        )

        result = decoder.decode(frame)

        self.assertIn("roll=  +12.34 deg", result)
        self.assertIn("pitch=  -12.34 deg", result)
        self.assertIn("yaw=  +45.67 deg", result)

    def test_aggregate_settings_mode_uses_third_byte(self):
        decoder = SensorDecoder()

        result = decoder.decode(CanFrame(0x125, bytes((0xC0, 0x00, 0x02, 0x00))))

        self.assertIn("MODE    2", result)
        self.assertEqual(decoder.system_mode, 2)

    def test_periodic_task_configuration(self):
        decoder = SensorDecoder()

        result = decoder.decode(
            CanFrame(0x125, bytes((0xC1, 0x03, 0x01, 0x0A, 0x00, 0x00, 0xFA)))
        )

        self.assertEqual(
            result,
            "PERIODIC task=3, on, command=0x0A, subcommand=0x00, interval=250 ms",
        )

    def test_sensor_information(self):
        decoder = SensorDecoder()

        result = decoder.decode(
            CanFrame(0x125, bytes((0xEF, 0x04, 0x00, 0x00, 0x01, 0x22)))
        )

        self.assertEqual(result, "INFO    firmware version=0x00000122 (290)")

    def test_information_and_error_commands_have_distinct_labels(self):
        _info_key, info_label = message_stream(
            CanFrame(0x125, bytes((0xEF, 0x04, 0, 0, 1, 0x22)))
        )
        _error_key, error_label = message_stream(
            CanFrame(0x125, bytes((0xFE, 0x62, 0, 0, 1)))
        )

        self.assertEqual(info_label, "INFO FIRMWARE VERSION (0xEF)")
        self.assertEqual(error_label, "NACK/ERROR FOR 0x62 (0xFE)")

    def test_gyro_diagnostics_have_distinct_statistics_rows(self):
        summary_key, summary_label = message_stream(
            CanFrame(0x125, bytes.fromhex("18 0B 01 01 02 03 00 00"))
        )
        bias_key, bias_label = message_stream(
            CanFrame(0x125, bytes.fromhex("18 0D 00 00 00 00 00 00"))
        )
        _config_key, config_label = message_stream(
            CanFrame(0x125, bytes.fromhex("C8 00 01 14 00 FA 00 1E"))
        )

        self.assertNotEqual(summary_key, bias_key)
        self.assertEqual(summary_label, "GYRO CAL SUMMARY (0x18)")
        self.assertEqual(bias_label, "GYRO CAL ACCEPTED BIAS (0x18)")
        self.assertEqual(config_label, "GYRO CAL CONFIG (0xC8)")

        _legacy_key, legacy_label = message_stream(
            CanFrame(0x125, bytes.fromhex("18 00 01 01 01 00 00 00"))
        )
        self.assertEqual(legacy_label, "CALIBRATION ITEM 0X00 (0x18)")

    def test_message_rate_uses_rolling_one_second_window(self):
        statistics = MessageStatistics("test")
        statistics.record(0.1, "first")
        statistics.record(0.5, "second")
        statistics.record(1.2, "third")

        self.assertEqual(statistics.count, 3)
        self.assertAlmostEqual(statistics.rate(1.2), 1.0 / 0.7)
        self.assertEqual(statistics.latest, "third")

    def test_request_payload_is_eight_bytes(self):
        self.assertEqual(request_payload(0x40, 2), bytes((0x40, 2, 0, 0, 0, 0, 0, 0)))

    def test_default_startup_queries_include_gyro_configuration_and_diagnostics(self):
        queries = default_startup_queries()

        self.assertIn((0xC8, 0x00), queries)
        self.assertEqual(
            [subcommand for command, subcommand in queries if command == 0x18],
            list(range(0x0B, 0x12)),
        )

    def test_hex_integer_parser(self):
        self.assertEqual(parse_integer("0x3e8"), 1000)


if __name__ == "__main__":
    unittest.main()
