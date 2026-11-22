import socket
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from companion import CisternInputCompanion, INPUT_KEYS


class CompanionTests(unittest.TestCase):
    def setUp(self):
        self.client = Mock()
        self.udp = Mock()
        self.address = ("127.0.0.1", 46005)
        self.companion = CisternInputCompanion(
            self.client, self.udp, self.address, "garden-test", "cistern-test"
        )
        self.prefix = "garden/garden-test/modules/cistern-test/input/"

    def message(self, key, payload):
        return SimpleNamespace(topic=self.prefix + key, payload=payload)

    def test_subscribes_again_after_reconnect(self):
        for _ in range(2):
            self.client.on_connect(self.client, None, None, SimpleNamespace(value=0))
        self.assertEqual(self.client.subscribe.call_count, 2)
        self.client.subscribe.assert_called_with(self.prefix + "+", qos=1)

    def test_failed_connection_does_not_subscribe(self):
        self.client.on_connect(self.client, None, None, 1)
        self.client.subscribe.assert_not_called()

    def test_preserves_wire_format_for_all_inputs(self):
        for key in INPUT_KEYS:
            self.udp.reset_mock()
            self.client.on_message(None, None, self.message(key, b"18.25"))
            packets = [call.args[0] for call in self.udp.sendto.call_args_list]
            if key in {"level_mm", "ultrasonic_height", "water_temperature",
                       "air_temperature"}:
                self.assertIn(f"{key}=18.250000".encode(), packets)
            else:
                self.assertNotIn(f"{key}=18.250000".encode(), packets)
            if key in {
                "level_mm", "pressure_sensor_height", "pressure_measurement_range_mm",
                "pressure_zero_voltage", "pressure_span_voltage",
            }:
                self.assertTrue(any(packet.startswith(b"pressure_adc_raw=") for packet in packets))

    def test_rejects_malformed_and_nonfinite_values(self):
        for payload in [b"", b"invalid", b"\xff", b"NaN", b"inf", b"-inf", b"1e999"]:
            self.client.on_message(None, None, self.message("level_mm", payload))
        self.udp.sendto.assert_not_called()

    def test_forwards_temperature_rom_ids_as_exact_hex_strings(self):
        self.client.on_message(
            None, None,
            self.message("water_temperature_rom_id", b"2801000000000029"),
        )
        self.assertEqual(
            self.udp.sendto.call_args.args[0],
            b"water_temperature_rom_id=2801000000000029",
        )

    def test_rejects_invalid_temperature_rom_ids(self):
        for payload in (b"28", b"zz01000000000029", b"\xff" * 16):
            self.client.on_message(
                None, None, self.message("air_temperature_rom_id", payload)
            )
        self.udp.sendto.assert_not_called()

    def test_rejects_other_modules_unknown_keys_and_nested_topics(self):
        for topic in [
            "garden/other/modules/cistern-test/input/level_mm",
            "garden/garden-test/modules/other/input/level_mm",
            self.prefix + "unsupported", self.prefix + "nested/level_mm",
        ]:
            self.client.on_message(None, None, SimpleNamespace(topic=topic, payload=b"800"))
        self.udp.sendto.assert_not_called()

    def test_clamps_environment_for_adc_model_but_forwards_original_level(self):
        self.client.on_message(None, None, self.message("level_mm", b"-10"))
        packets = [call.args[0] for call in self.udp.sendto.call_args_list]
        self.assertEqual(packets, [b"level_mm=-10.000000", b"pressure_adc_raw=0"])

    def test_span_calibrates_adc_without_forwarding_span_to_firmware(self):
        self.client.on_message(None, None, self.message("pressure_span_voltage", b"3.54"))
        packets = [call.args[0] for call in self.udp.sendto.call_args_list]
        self.assertEqual(packets, [b"pressure_adc_raw=1142"])

    def test_5000_mm_range_and_3_54v_span_at_2430mm_head(self):
        self.client.on_message(None, None, self.message("level_mm", b"2450"))
        self.client.on_message(None, None, self.message("pressure_sensor_height", b"20"))
        self.client.on_message(None, None, self.message("pressure_measurement_range_mm", b"5000"))
        self.client.on_message(None, None, self.message("pressure_span_voltage", b"3.54"))
        self.assertEqual(self.companion.pressure_adc_raw(), 2135)
        packets = [call.args[0] for call in self.udp.sendto.call_args_list]
        self.assertFalse(any(packet.startswith(b"pressure_measurement_range_mm=")
                            for packet in packets))
        self.assertFalse(any(packet.startswith(b"pressure_span_voltage=")
                            for packet in packets))

    def test_overrange_span_clips_companion_adc_at_4095(self):
        self.client.on_message(None, None, self.message("level_mm", b"2450"))
        self.client.on_message(None, None, self.message("pressure_sensor_height", b"20"))
        self.client.on_message(None, None, self.message("pressure_measurement_range_mm", b"5000"))
        self.client.on_message(None, None, self.message("pressure_span_voltage", b"6.79"))
        self.assertEqual(self.companion.pressure_adc_raw(), 4095)

    def test_rounded_3300mv_dashboard_value_can_be_below_adc_full_scale(self):
        self.client.on_message(None, None, self.message("level_mm", b"2443"))
        self.client.on_message(None, None, self.message("pressure_sensor_height", b"20"))
        self.client.on_message(None, None, self.message("pressure_measurement_range_mm", b"3000"))
        self.client.on_message(None, None, self.message("pressure_span_voltage", b"4.08"))
        self.assertEqual(self.companion.pressure_adc_raw(), 4089)

    def test_udp_roundtrip(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(1)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                companion = CisternInputCompanion(self.client, sender, receiver.getsockname())
                companion.on_message(
                    None, None,
                    SimpleNamespace(
                        topic="garden/garden-1/modules/cistern-1/input/level_mm", payload=b"825"
                    ),
                )
                self.assertEqual(receiver.recvfrom(96)[0], b"level_mm=825.000000")


if __name__ == "__main__":
    unittest.main()
