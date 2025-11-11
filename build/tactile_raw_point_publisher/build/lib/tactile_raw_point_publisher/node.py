from typing import List, Sequence, Optional, Any
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

# --- Try both common filenames for your driver module ---
PaxiniTac: Optional[Any] = None
try:
    from Paxini_Tac import PaxiniTac as _PaxiniTac
    PaxiniTac = _PaxiniTac
except Exception:
    try:
        from Paxini_Tac import PaxiniTac as _PaxiniTac
        PaxiniTac = _PaxiniTac
    except Exception:
        PaxiniTac = None


class PaxiniReader:
    """
    Thin adapter over your PaxiniTac API.
    - Reuses one serial handle (tac.ser)
    - Switches sensor ports via get_ser_response(..., "choose_portX")
    - Reads raw data via get_raw_sensor_data(..., "get_data", 348, 6)
    - Normalizes outputs to [[x, y, z], ...] floats
    """

    def __init__(self, logger: Node.get_logger):
        self._logger = logger
        if PaxiniTac is None:
            raise ImportError(
                "Cannot import PaxiniTac. Place Paxini_Tac.py or paxini_tac.py inside the package "
                "or add it to PYTHONPATH."
            )
        self.tac = PaxiniTac()

        # Basic serial sanity check
        try:
            if hasattr(self.tac, "ser") and getattr(self.tac.ser, "is_open", False):
                self._logger.info("PaxiniTac serial port is open.")
            else:
                # Some drivers may auto-open in methods; we’ll just warn here.
                self._logger.warn("PaxiniTac serial appears closed; driver may open it on first use.")
        except Exception as e:
            self._logger.warn(f"Unable to check serial status: {e}")

        # Map ROS "sensor_id" -> your port selector command
        self._port_cmd = {
            1: "choose_port1",
            3: "choose_port3",
        }

    def read_points_xyz(self, sensor_id: int) -> List[Sequence[float]]:
        """
        Returns [[x,y,z], ...] for the given sensor_id.
        If upstream field is empty ('响应数据域为空'), returns [].
        """
        if sensor_id not in self._port_cmd:
            self._logger.error(f"Unsupported sensor_id={sensor_id}. Use 1 or 3.")
            return []

        # 1) Select sensor port (e.g., choose_port1 / choose_port3)
        try:
            self.tac.get_ser_response(self.tac.ser, self._port_cmd[sensor_id])
        except Exception as e:
            self._logger.error(f"选择传感器端口失败 (sensor {sensor_id}): {e}")
            return []

        # 2) Read raw data (expects up to 348 items; your print code slices [:348])
        try:
            raw = self.tac.get_raw_sensor_data(self.tac.ser, "get_data", 348, 6)
        except Exception as e:
            self._logger.error(f"读取传感器数据异常 (sensor {sensor_id}): {e}")
            return []

        # 3) Normalize to [[x,y,z], ...]
        points: List[Sequence[float]] = self._normalize_to_xyz(raw)
        return points

    def _normalize_to_xyz(self, raw) -> List[Sequence[float]]:
        """
        Accepts either:
          - List[ [a,b,c], ... ]
          - Flat List[ a, b, c, a, b, c, ... ]
        Takes at most 348 entries if nested, or 348 values if flat (== 116 triplets).
        """
        if not raw:
            return []

        # Case A: nested like [[-12, 9, 28], ...]
        if isinstance(raw, list) and isinstance(raw[0], (list, tuple)):
            nested = raw[:348]  # keep first 348 triplets at most (as per your print)
            out: List[Sequence[float]] = []
            for t in nested:
                if not t:
                    continue
                # take first 3 values; coerce to float
                x = float(t[0]) if len(t) >= 1 else 0.0
                y = float(t[1]) if len(t) >= 2 else 0.0
                z = float(t[2]) if len(t) >= 3 else 0.0
                out.append([x, y, z])
            return out

        # Case B: flat list of numbers -> chunk into triplets
        if isinstance(raw, list) and isinstance(raw[0], (int, float)):
            flat = raw[:348]  # 348 values => 116 triplets
            out: List[Sequence[float]] = []
            for i in range(0, len(flat) - (len(flat) % 3), 3):
                x = float(flat[i])
                y = float(flat[i + 1])
                z = float(flat[i + 2])
                out.append([x, y, z])
            return out

        # Unknown shape
        self._logger.warn("原始数据格式非预期，忽略该帧。")
        return []


class TactileRawPointPublisher(Node):
    def __init__(self):
        super().__init__('tactile_raw_point_publisher')

        # Parameters
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('sensor_1_frame', 'sensor_1_link')
        self.declare_parameter('sensor_3_frame', 'sensor_3_link')
        self.declare_parameter('sensor_1_id', 1)
        self.declare_parameter('sensor_3_id', 3)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.sensor_1_frame = str(self.get_parameter('sensor_1_frame').value)
        self.sensor_3_frame = str(self.get_parameter('sensor_3_frame').value)
        self.sensor_1_id = int(self.get_parameter('sensor_1_id').value)
        self.sensor_3_id = int(self.get_parameter('sensor_3_id').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub_1 = self.create_publisher(PointCloud2, '/tactile/sensor_1/raw_point_data', qos)
        self.pub_3 = self.create_publisher(PointCloud2, '/tactile/sensor_3/raw_point_data', qos)

        self._last_empty_warn_1 = 0.0
        self._last_empty_warn_3 = 0.0

        # Wrap your driver
        self.reader = PaxiniReader(self.get_logger())

        # Timer
        period = 1.0 / max(1e-3, rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Publishing PointCloud2 at {rate_hz:.1f} Hz:\n'
            f'  /tactile/sensor_1/raw_point_data (frame={self.sensor_1_frame}, id={self.sensor_1_id})\n'
            f'  /tactile/sensor_3/raw_point_data (frame={self.sensor_3_frame}, id={self.sensor_3_id})'
        )

    # ---- helpers ----
    def _make_header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def _make_pcl2(self, points_xyz: List[Sequence[float]], frame_id: str) -> PointCloud2:
        gen = ((float(p[0]), float(p[1]), float(p[2])) for p in points_xyz)
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        header = self._make_header(frame_id)
        cloud = point_cloud2.create_cloud(header, fields, gen)
        return cloud

    def _publish_one(self, sensor_id: int, frame_id: str, publisher, warn_attr: str):
        try:
            points = self.reader.read_points_xyz(sensor_id) or []
        except Exception as e:
            if time.time() - getattr(self, warn_attr) > 1.0:
                self.get_logger().error(f'传感器 {sensor_id} 读取失败: {e}')
                setattr(self, warn_attr, time.time())
            points = []

        if not points:
            if time.time() - getattr(self, warn_attr) > 1.0:
                self.get_logger().warn(f'传感器 {sensor_id} 响应数据域为空，发布空 PointCloud2。')
                setattr(self, warn_attr, time.time())
            publisher.publish(self._make_pcl2([], frame_id))
            return

        publisher.publish(self._make_pcl2(points, frame_id))

    # ---- timer ----
    def _on_timer(self):
        self._publish_one(self.sensor_1_id, self.sensor_1_frame, self.pub_1, '_last_empty_warn_1')
        self._publish_one(self.sensor_3_id, self.sensor_3_frame, self.pub_3, '_last_empty_warn_3')


def main():
    rclpy.init()
    node = TactileRawPointPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
