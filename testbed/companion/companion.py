"""Forward MQTT environment inputs to the cistern's native-sim UDP endpoint."""

import logging
import math
import os
import socket
from typing import Any

LOG = logging.getLogger("cistern-test-companion")
INPUT_KEYS = frozenset({
    "level_mm", "pressure_sensor_height", "pressure_measurement_range_mm",
    "pressure_zero_voltage",
    "pressure_span_voltage", "ultrasonic_height", "water_temperature",
    "air_temperature", "water_temperature_rom_id", "air_temperature_rom_id",
})
ADC_INPUT_KEYS = frozenset({
    "level_mm", "pressure_sensor_height", "pressure_measurement_range_mm",
    "pressure_zero_voltage", "pressure_span_voltage",
})
DEFAULTS = {
    "level_mm": 800.0,
    "pressure_sensor_height": 20.0,
    "pressure_measurement_range_mm": 3000.0,
    "pressure_zero_voltage": 0.0,
    "pressure_span_voltage": 3.3,
}
ADC_REFERENCE_MV = 3300.0
ADC_MAX_RAW = 4095


class CisternInputCompanion:
    """Convert dashboard pressure calibration into native-sim ADC counts."""

    def __init__(
        self,
        client: Any,
        udp_socket: socket.socket,
        simulator_address: tuple[str, int],
        garden_id: str = "garden-1",
        module_id: str = "cistern-1",
    ) -> None:
        self.client = client
        self.udp_socket = udp_socket
        self.simulator_address = simulator_address
        self.input_prefix = f"garden/{garden_id}/modules/{module_id}/input/"
        self.inputs = dict(DEFAULTS)
        client.on_connect = self.on_connect
        client.on_message = self.on_message

    def on_connect(self, client, _userdata, _flags, reason_code, _properties=None):
        if int(getattr(reason_code, "value", reason_code)) == 0:
            # Re-subscription also replays retained environment values.
            client.subscribe(f"{self.input_prefix}+", qos=1)
            LOG.info("Subscribed to cistern inputs: %s+", self.input_prefix)

    def on_message(self, _client, _userdata, message):
        if not message.topic.startswith(self.input_prefix):
            return
        key = message.topic[len(self.input_prefix):]
        if key not in INPUT_KEYS:
            return
        if key in {"water_temperature_rom_id", "air_temperature_rom_id"}:
            try:
                rom_id = message.payload.decode("ascii").strip()
            except (AttributeError, UnicodeDecodeError):
                return
            if len(rom_id) != 16 or any(char not in "0123456789abcdefABCDEF"
                                        for char in rom_id):
                return
            try:
                self._send(f"{key}={rom_id.lower()}")
            except OSError:
                LOG.exception("Could not forward cistern ROM ID %s", key)
            return
        try:
            value = float(message.payload)
        except (ValueError, TypeError, OverflowError):
            return
        if not math.isfinite(value):
            return
        if key in self.inputs:
            self.inputs[key] = value
        try:
            # Only environment values and the emulated ADC sample enter the
            # module. Pressure calibration remains private to the companion;
            # firmware scaling comes exclusively from its own CO_OD.
            if key in {"level_mm", "ultrasonic_height", "water_temperature",
                       "air_temperature"}:
                self._send(f"{key}={value:.6f}")
            if key in ADC_INPUT_KEYS:
                self._send(f"pressure_adc_raw={self.pressure_adc_raw()}")
        except OSError:
            LOG.exception("Could not forward cistern input %s", key)

    def pressure_adc_raw(self) -> int:
        level = max(0.0, min(2500.0, self.inputs["level_mm"]))
        mount = max(0.0, min(2500.0, self.inputs["pressure_sensor_height"]))
        range_mm = max(1.0, min(5000.0, self.inputs["pressure_measurement_range_mm"]))
        zero = max(0.0, min(3.3, self.inputs["pressure_zero_voltage"]))
        span = max(0.0, min(10.0, self.inputs["pressure_span_voltage"]))
        fraction = max(0.0, min(1.0, (level - mount) / range_mm))
        voltage = max(0.0, min(ADC_REFERENCE_MV / 1000.0,
                               zero + fraction * (span - zero)))
        return int(voltage * ADC_MAX_RAW / (ADC_REFERENCE_MV / 1000.0) + 0.5)

    def _send(self, packet: str) -> None:
        self.udp_socket.sendto(packet.encode("ascii"), self.simulator_address)


def run_from_environment() -> None:
    import paho.mqtt.client as mqtt

    garden = os.getenv("GARDEN_ID", "garden-1")
    module = os.getenv("MODULE_ID", "cistern-1")
    host = os.getenv("MQTT_HOST", "mqtt")
    port = int(os.getenv("MQTT_PORT", "1883"))
    simulator = os.getenv("SIMULATOR_HOST", os.getenv("UDP_HOST", "127.0.0.1"))
    simulator_port = int(os.getenv("SIMULATOR_IPC_PORT", os.getenv("UDP_PORT", "46005")))
    client_id = os.getenv("MQTT_CLIENT_ID", "virtual-garden-cistern-input")
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, clean_session=True
    )
    client.reconnect_delay_set(min_delay=1, max_delay=30)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        CisternInputCompanion(
            client, udp_socket, (simulator, simulator_port), garden, module
        )
        try:
            client.connect_async(host, port, 60)
            client.loop_forever(retry_first_connection=True)
        finally:
            client.disconnect()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    run_from_environment()
