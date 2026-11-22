# Cistern native-sim companion

The cistern repository owns the environment-input bridge in
`testbed/companion/companion.py`, its dependency file, Dockerfile, and tests.
Importing the companion does not connect to MQTT or start a process.

## Firmware source selection

`src/native_sim_ds18b20.c` implements the virtual 1-Wire temperature sensors.
CMake includes it only for `CONFIG_BOARD_NATIVE_SIM`, so hardware-target builds
do not compile simulator-only code. The file's non-simulator fallback stubs
also include the errno and Zephyr utility definitions they need, so they remain
valid if compiled directly in a non-simulator configuration.

## Protocol and responsibilities

The companion subscribes to
`garden/{GARDEN_ID}/modules/{MODULE_ID}/input/+` at QoS 1, including retained
values after reconnect. Supported keys:

- `level_mm`, `pressure_sensor_height`
- `pressure_zero_voltage`, `pressure_span_voltage`
- `ultrasonic_height`
- `water_temperature`, `air_temperature`

Finite numeric payloads are forwarded as ASCII `key=value` UDP datagrams with
six decimal places, preserving the firmware protocol on port 46005.
Invalid/nonfinite values and unsupported topics are ignored. Sensor transfer
models, clamping, calibration, and CANopen measurements remain in firmware.
This is an input-only companion; measurements go through CANopen, not MQTT IPC.

## Run directly

```sh
python3 -m pip install -r testbed/companion/requirements.txt
GARDEN_ID=garden-1 MODULE_ID=cistern-1 MQTT_HOST=127.0.0.1 \
  python3 testbed/companion/companion.py
```

| Variable | Default |
| --- | --- |
| `GARDEN_ID` | `garden-1` |
| `MODULE_ID` | `cistern-1` |
| `MQTT_HOST` | `mqtt` |
| `MQTT_PORT` | `1883` |
| `MQTT_CLIENT_ID` | `virtual-garden-cistern-input` |
| `SIMULATOR_HOST` | `127.0.0.1` |
| `SIMULATOR_IPC_PORT` | `46005` |

`UDP_HOST` and `UDP_PORT` remain fallback aliases for existing deployments.
The companion retries initial MQTT connection and reconnects automatically.

## Container and tests

From the repository root:

```sh
docker build -f testbed/companion/Dockerfile -t cistern-companion:local .
python3 -m unittest discover -s testbed/companion -v
```

To also verify retained MQTT input replay and reconnect against a broker:

```sh
RUN_MQTT_INTEGRATION=1 MQTT_HOST=127.0.0.1 MQTT_PORT=1883 \
  python3 -m unittest discover -s testbed/companion -p test_mqtt_broker.py -v
```

This uses unique test topics and removes retained test messages afterwards.

Use an explicit simulator hostname/address when running the companion and
firmware in different network namespaces. In Virtual Garden both share the
pool companion's network namespace, so loopback works. Virtual Garden builds
the companion image from this repository, with no local script or bind mount.
