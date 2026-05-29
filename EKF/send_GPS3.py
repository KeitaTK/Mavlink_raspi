#!/usr/bin/env python3
"""
Motive -> ArduPilot bridge with:
- UDP Motive receive (typically 50Hz)
- GPS_INPUT periodic send to ArduPilot (15Hz)
- SYSTEM_TIME periodic send from Raspberry Pi to Pixhawk (1Hz)
"""

import socket
import pickle
import time
import threading
from datetime import datetime
from pymavlink import mavutil


class ArduPilotConnector:
    def __init__(self):
        """ArduPilot connection wrapper."""
        self.master = None
        self.target_system = None
        self.target_component = None
        self.connect_to_autopilot()

    def connect_to_autopilot(self):
        """Connect to ArduPilot via serial and validate with heartbeat/version."""
        try:
            print("Starting ArduPilot connection...")
            print("Connecting telem1...")
            self.master = mavutil.mavlink_connection('/dev/ttyAMA0', 1000000, rtscts=True)

            print("Waiting heartbeat...")
            self.master.wait_heartbeat()

            self.target_system = self.master.target_system
            self.target_component = self.master.target_component

            print(f"Heartbeat received from system {self.target_system}, component {self.target_component}")

            self.master.mav.autopilot_version_request_send(
                self.target_system,
                self.target_component,
            )

            msg = self.master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
            if msg:
                print("ArduPilot connection established")
                print(f"Flight SW Version: {msg.flight_sw_version}")
                return True

            print("AUTOPILOT_VERSION receive timeout")
            return False

        except Exception as e:
            print(f"ArduPilot connection failed: {e}")
            self.master = None
            return False

    def send_gps_input(self, lat, lon, alt, yaw_deg):
        """Send MAVLink GPS_INPUT."""
        if not self.master:
            return False

        try:
            yaw_cdeg = int(yaw_deg * 100) if yaw_deg is not None else 0

            self.master.mav.gps_input_send(
                int(time.time() * 1e6),  # time_usec
                0,                       # gps_id
                0,                       # ignore_flags
                0,                       # time_week_ms
                0,                       # time_week
                3,                       # fix_type: 3D Fix
                int(lat * 1e7),          # lat (degE7)
                int(lon * 1e7),          # lon (degE7)
                alt,                     # alt (m)
                0.8,                     # hdop
                1.0,                     # vdop
                0.0, 0.0, 0.0,           # vn, ve, vd (m/s)
                0.1,                     # speed_accuracy
                0.01,                    # horiz_accuracy
                0.01,                    # vert_accuracy
                12,                      # satellites_visible
                yaw_cdeg,                # yaw (cdeg)
            )
            return True

        except Exception as e:
            print(f"GPS_INPUT send error: {e}")
            return False

    def send_system_time(self):
        """Send MAVLink SYSTEM_TIME with host UNIX time."""
        if not self.master:
            return False

        try:
            time_unix_usec = int(time.time() * 1e6)
            self.master.mav.system_time_send(
                time_unix_usec,
                0,  # time_boot_ms unknown on host side
            )
            return True

        except Exception as e:
            print(f"SYSTEM_TIME send error: {e}")
            return False


class UDPReceiver:
    def __init__(self, host='0.0.0.0', port=15769):
        """UDP receiver for Motive packet stream."""
        self.host = host
        self.port = port
        self.socket = None
        self.setup_udp()

    def setup_udp(self):
        """Configure UDP socket."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.settimeout(0.1)
            print(f"UDP receive started: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"UDP setup error: {e}")
            return False

    def receive_data(self):
        """Receive one UDP packet and unpickle payload."""
        try:
            data, addr = self.socket.recvfrom(2048)
            return pickle.loads(data), addr
        except socket.timeout:
            return None, None
        except Exception:
            return None, None

    def close(self):
        """Close UDP socket."""
        if self.socket:
            self.socket.close()


class PeriodicSender:
    def __init__(self, ardupilot_connector, rate_hz=15):
        """Periodic GPS_INPUT sender with high-precision timing."""
        self.ardupilot = ardupilot_connector
        self.rate_hz = rate_hz
        self.interval = 1.0 / rate_hz
        self.latest_data = None
        self.data_lock = threading.Lock()
        self.running = False
        self.send_thread = None

        # Stats
        self.send_count = 0
        self.success_count = 0
        self.timing_errors = []

    def update_data(self, lat, lon, alt, yaw_deg):
        """Update latest GPS source data."""
        with self.data_lock:
            self.latest_data = {
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'yaw': yaw_deg,
                'timestamp': time.time(),
            }

    def start(self):
        """Start periodic GPS send thread."""
        self.running = True
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()
        print(f"GPS periodic send started: {self.rate_hz}Hz ({self.interval * 1000:.1f}ms interval)")

    def stop(self):
        """Stop periodic GPS send thread."""
        self.running = False
        if self.send_thread:
            self.send_thread.join()

    def _send_loop(self):
        """Send loop with coarse sleep + short busy-wait for timing accuracy."""
        next_send_time = time.time()

        while self.running:
            current_time = time.time()
            next_send_time += self.interval

            sleep_time = next_send_time - current_time
            if sleep_time > 0:
                if sleep_time > 0.001:
                    time.sleep(sleep_time - 0.001)

                while time.time() < next_send_time:
                    pass

            actual_time = time.time()
            timing_error = (actual_time - next_send_time) * 1000
            self.timing_errors.append(timing_error)
            if len(self.timing_errors) > 100:
                self.timing_errors.pop(0)

            with self.data_lock:
                if self.latest_data:
                    success = self.ardupilot.send_gps_input(
                        self.latest_data['lat'],
                        self.latest_data['lon'],
                        self.latest_data['alt'],
                        self.latest_data['yaw'],
                    )

                    self.send_count += 1
                    if success:
                        self.success_count += 1

                    if self.send_count % (self.rate_hz * 5) == 0:
                        avg_error = sum(self.timing_errors) / len(self.timing_errors)
                        max_error = max(self.timing_errors)
                        min_error = min(self.timing_errors)
                        success_rate = self.success_count / self.send_count * 100

                        print(f"[GPS 15Hz] sent: {self.send_count}, success: {success_rate:.1f}%")
                        print(
                            f"  timing error: avg {avg_error:.3f}ms, "
                            f"max {max_error:.3f}ms, min {min_error:.3f}ms"
                        )

            if abs(timing_error) > 10:
                next_send_time = time.time()


class SystemTimeSender:
    def __init__(self, ardupilot_connector, rate_hz=1.0):
        """Periodic SYSTEM_TIME sender for host UNIX time sync logging."""
        self.ardupilot = ardupilot_connector
        self.rate_hz = rate_hz
        self.interval = 1.0 / rate_hz
        self.running = False
        self.send_thread = None

        # Stats
        self.send_count = 0
        self.success_count = 0

    def start(self):
        """Start SYSTEM_TIME periodic thread."""
        self.running = True
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()
        print(f"SYSTEM_TIME periodic send started: {self.rate_hz:.1f}Hz")

    def stop(self):
        """Stop SYSTEM_TIME periodic thread."""
        self.running = False
        if self.send_thread:
            self.send_thread.join()

    def _send_loop(self):
        """Simple periodic loop for SYSTEM_TIME send."""
        next_send_time = time.time()

        while self.running:
            current_time = time.time()
            if current_time < next_send_time:
                time.sleep(min(next_send_time - current_time, 0.1))
                continue

            success = self.ardupilot.send_system_time()
            self.send_count += 1
            if success:
                self.success_count += 1

            if self.send_count % 30 == 0:
                success_rate = self.success_count / self.send_count * 100
                unix_sec = int(time.time())
                print(
                    f"[SYSTEM_TIME 1Hz] sent: {self.send_count}, "
                    f"success: {success_rate:.1f}%, unix: {unix_sec}"
                )

            next_send_time += self.interval
            if (current_time - next_send_time) > 1.0:
                next_send_time = current_time + self.interval


def motive_to_ardupilot_bridge_15hz_with_system_time():
    """Main bridge: Motive UDP -> GPS_INPUT 15Hz + SYSTEM_TIME 1Hz."""

    print("=" * 60)
    print("Motive to ArduPilot Bridge (GPS 15Hz + SYSTEM_TIME 1Hz)")
    print("=" * 60)

    ardupilot = ArduPilotConnector()
    if not ardupilot.master:
        print("ERROR: ArduPilot connection failed")
        return

    udp_receiver = UDPReceiver('0.0.0.0', 15769)
    if not udp_receiver.socket:
        print("ERROR: UDP receiver setup failed")
        return

    periodic_sender = PeriodicSender(ardupilot, rate_hz=15)
    periodic_sender.start()

    system_time_sender = SystemTimeSender(ardupilot, rate_hz=1.0)
    system_time_sender.start()

    print("Bridge started: receiving Motive UDP packets")
    print("Sending GPS_INPUT at 15Hz and SYSTEM_TIME at 1Hz")
    print("Press Ctrl+C to stop")

    packet_count = 0
    last_stats_time = time.time()

    try:
        while True:
            received_data, _addr = udp_receiver.receive_data()
            if received_data is None:
                continue

            packet_count += 1

            status = received_data.get('status', 'UNKNOWN')
            if status == 'SUCCESS':
                lat = received_data.get('latitude', 0.0)
                lon = received_data.get('longitude', 0.0)
                alt = received_data.get('altitude', 0.0)
                yaw = received_data.get('yaw_degrees', 0.0)

                periodic_sender.update_data(lat, lon, alt, yaw)

                if packet_count % 100 == 0:
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    print(f"[{timestamp}] #{packet_count}: source data updated")
                    print(f"  position: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.3f}m")
                    print(f"  attitude: yaw={yaw:.2f}deg")

            current_time = time.time()
            if current_time - last_stats_time >= 10.0:
                dt = current_time - last_stats_time
                packet_rate = packet_count / dt if dt > 0 else 0.0
                print(f"UDP receive rate: {packet_rate:.1f} pkt/s")
                packet_count = 0
                last_stats_time = current_time

    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("Bridge stopped")
        print("=" * 60)

        print(f"GPS_INPUT sends: {periodic_sender.send_count}")
        print(f"GPS_INPUT success: {periodic_sender.success_count}")
        if periodic_sender.send_count > 0:
            success_percentage = periodic_sender.success_count / periodic_sender.send_count * 100
            print(f"GPS_INPUT success rate: {success_percentage:.2f}%")

        if periodic_sender.timing_errors:
            avg_error = sum(periodic_sender.timing_errors) / len(periodic_sender.timing_errors)
            print(f"GPS_INPUT avg timing error: {avg_error:.3f}ms")

        print(f"SYSTEM_TIME sends: {system_time_sender.send_count}")
        print(f"SYSTEM_TIME success: {system_time_sender.success_count}")
        if system_time_sender.send_count > 0:
            st_success = system_time_sender.success_count / system_time_sender.send_count * 100
            print(f"SYSTEM_TIME success rate: {st_success:.2f}%")

        print("=" * 60)

    except Exception as e:
        print(f"Unexpected error: {e}")

    finally:
        periodic_sender.stop()
        system_time_sender.stop()
        udp_receiver.close()
        print("Resources cleaned up")


if __name__ == "__main__":
    motive_to_ardupilot_bridge_15hz_with_system_time()
