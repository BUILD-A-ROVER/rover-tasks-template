"""
Rover Client API for Workshop Attendees.
Provides a unified interface for teleoperation, obstacle avoidance, and visual tracking.
Seamlessly toggles between physical rover hardware (mock=False) and simulated environment (mock=True).
"""

import json
import time
import socket
import urllib.request
import logging
from typing import List, Optional, Any

from api.mock_rover import MockRover

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None
    np = None
    OPENCV_AVAILABLE = False

logger = logging.getLogger("rover_client")


class RoverClient:
    """
    High-level Python API client to control the Raspberry Pi 4B rover.
    """

    MAX_LINEAR_VELOCITY = 0.6    # m/s
    MAX_ANGULAR_VELOCITY = 1.2   # rad/s

    def __init__(self, mock: bool = True, host: str = "localhost", port: int = 5555, camera_port: int = 8080):
        """
        Initialize RoverClient.

        :param mock: If True, uses in-memory MockRover. If False, connects to rover-core HAL.
        :param host: IP address or hostname of rover-core server.
        :param port: TCP socket IPC port (default: 5555).
        :param camera_port: HTTP MJPEG stream port (default: 8080).
        """
        self.mock = mock
        self.host = host
        self.port = port
        self.camera_port = camera_port

        self._mock_rover: Optional[MockRover] = None
        self._sock: Optional[socket.socket] = None

        if self.mock:
            self._mock_rover = MockRover()
            logger.info("RoverClient initialized in MOCK SIMULATION mode.")
        else:
            self._connect_ipc()
            logger.info(f"RoverClient connected to physical hardware on {self.host}:{self.port}")

    def _connect_ipc(self) -> None:
        """Establish persistent TCP connection to rover-core IPC server."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(2.0)
            self._sock.connect((self.host, self.port))
        except Exception as e:
            logger.warning(f"Unable to connect to physical hardware at {self.host}:{self.port} ({e}). Falling back to MOCK mode.")
            self.mock = True
            self._mock_rover = MockRover()

    def _send_ipc_cmd(self, command_dict: dict) -> dict:
        """Helper to send JSON request and await response from rover-core."""
        if self.mock or self._sock is None:
            return {"status": "error", "message": "Client running in mock mode"}

        try:
            payload = json.dumps(command_dict) + "\n"
            self._sock.sendall(payload.encode("utf-8"))

            buffer = ""
            start_time = time.time()
            while time.time() - start_time < 2.0:
                chunk = self._sock.recv(4096).decode("utf-8")
                if not chunk:
                    break
                buffer += chunk
                if "\n" in buffer:
                    line, _ = buffer.split("\n", 1)
                    return json.loads(line)
        except Exception as e:
            logger.error(f"IPC socket transmission error: {e}")
            self._connect_ipc()  # Attempt reconnect

        return {"status": "error", "message": "IPC transmission failed"}

    def drive(self, linear_velocity: float, angular_velocity: float) -> bool:
        """
        Command rover linear (m/s) and angular (rad/s) velocities.
        Automatically clamps inputs to safety threshold bounds.

        :param linear_velocity: Target linear speed (-0.6 to 0.6 m/s).
        :param angular_velocity: Target angular turn speed (-1.2 to 1.2 rad/s).
        :return: True if command executed successfully.
        """
        clamped_linear = max(-self.MAX_LINEAR_VELOCITY, min(self.MAX_LINEAR_VELOCITY, float(linear_velocity)))
        clamped_angular = max(-self.MAX_ANGULAR_VELOCITY, min(self.MAX_ANGULAR_VELOCITY, float(angular_velocity)))

        if self.mock and self._mock_rover:
            self._mock_rover.set_drive_cmd(clamped_linear, clamped_angular)
            return True

        res = self._send_ipc_cmd({
            "action": "drive",
            "linear": clamped_linear,
            "angular": clamped_angular
        })
        return res.get("status") == "ok"

    def stop(self) -> bool:
        """Immediately stop all rover movement."""
        return self.drive(0.0, 0.0)

    def get_lidar_scan(self) -> List[float]:
        """
        Fetch 360-degree LiDAR distance array in meters.
        Angles: 0 deg = front, 90 deg = right, 180 deg = rear, 270 deg = left.

        :return: List of 360 float values representing ranges in meters.
        """
        if self.mock and self._mock_rover:
            return self._mock_rover.get_lidar_scan()

        res = self._send_ipc_cmd({"action": "get_lidar"})
        if res.get("status") == "ok":
            return res.get("ranges", [10.0] * 360)

        return [10.0] * 360

    def get_camera_frame(self) -> Optional[Any]:
        """
        Fetch current OpenCV BGR camera frame.

        :return: BGR numpy image array (640x480) or None if capture fails.
        """
        if self.mock and self._mock_rover:
            return self._mock_rover.get_camera_frame()

        if not OPENCV_AVAILABLE or np is None:
            return None

        # Fetch HTTP MJPEG single frame snapshot from camera streamer
        try:
            url = f"http://{self.host}:{self.camera_port}/stream.mjpeg"
            stream = urllib.request.urlopen(url, timeout=1.5)
            bytes_data = b""
            for _ in range(20):
                bytes_data += stream.read(1024)
                a = bytes_data.find(b"\xff\xd8")  # JPEG start boundary
                b = bytes_data.find(b"\xff\xd9")  # JPEG end boundary
                if a != -1 and b != -1:
                    jpg = bytes_data[a : b + 2]
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    stream.close()
                    return frame
            stream.close()
        except Exception as e:
            logger.debug(f"HTTP camera stream snapshot exception: {e}")

        return None

    def close(self) -> None:
        """Close connection resources."""
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
