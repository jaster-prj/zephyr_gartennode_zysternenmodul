"""Optional broker-backed retained-input/reconnect test on isolated test topics."""

import os
import socket
import time
import unittest
import uuid

from companion import CisternInputCompanion


@unittest.skipUnless(os.getenv("RUN_MQTT_INTEGRATION") == "1", "requires MQTT broker")
class BrokerTests(unittest.TestCase):
    def test_retained_input_and_reconnect(self):
        import paho.mqtt.client as mqtt

        unique = uuid.uuid4().hex
        topic = f"garden/companion-test-{unique}/modules/cistern-1/input/level_mm"
        host = os.getenv("MQTT_HOST", "127.0.0.1")
        port = int(os.getenv("MQTT_PORT", "1883"))
        publisher = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"pub-{unique}")
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"sub-{unique}")
        with (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver,
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender,
        ):
            receiver.bind(("127.0.0.1", 0))
            receiver.setblocking(False)
            CisternInputCompanion(
                client, sender, receiver.getsockname(), f"companion-test-{unique}", "cistern-1"
            )
            publisher.connect(host, port)
            publisher.loop_start()

            def receive(timeout=5):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    client.loop(timeout=0.05)
                    try:
                        return receiver.recvfrom(96)[0]
                    except BlockingIOError:
                        pass
                raise TimeoutError("No forwarded input received")

            try:
                publisher.publish(topic, b"825.25", qos=1, retain=True).wait_for_publish(5)
                client.connect(host, port)
                self.assertEqual(receive(), b"level_mm=825.250000")

                client.disconnect()
                client.loop(timeout=0.05)
                client.reconnect()
                self.assertEqual(receive(), b"level_mm=825.250000")

                publisher.publish(topic, b"nan", qos=1).wait_for_publish(5)
                with self.assertRaises(TimeoutError):
                    receive(timeout=0.3)
            finally:
                client.disconnect()
                client.loop(timeout=0.05)
                publisher.publish(topic, b"", qos=1, retain=True).wait_for_publish(5)
                publisher.disconnect()
                publisher.loop_stop()


if __name__ == "__main__":
    unittest.main()
