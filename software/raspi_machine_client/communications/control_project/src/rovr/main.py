import asyncio
import json
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def _load_dotenv(env_path: Path) -> None:
    """Populate os.environ from a simple KEY=VALUE file (no export of already-set keys)."""
    if not env_path.is_file():
        return
    for raw_line in env_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip()
        # Strip surrounding quotes if present
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
            value = value[1:-1]
        if key and key not in os.environ:
            os.environ[key] = value


_load_dotenv(Path(__file__).resolve().parents[2] / ".env")


def _require_env(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise RuntimeError(
            f"Environment variable {name} is not set. "
            f"Define it in control_project/.env (see .env.example) or export it manually."
        )
    return value


from aiortc.contrib.media import MediaPlayer
from aiortc.mediastreams import MediaStreamError, MediaStreamTrack
from av.frame import Frame
from av.packet import Packet
from rcm import WebConnector
from rcm.data.Communication import WebCredentials
from rcm.data.RtcConnection import RtcConnection
from rcm.service.Logger import LoggerStore
from rcm.service.RtcWrapper import RtcWrapper

# --- WebRTC reconnection safety patch ----------------------------------------
# The rcm library creates one RTCPeerConnection for the entire process lifetime.
# If the signaling socket drops and reconnects, the library tries to reuse the
# already-closed peer connection and raises InvalidStateError.  The exception is
# swallowed silently as an "unretreived task exception", so WebRTC stays dead
# with no recovery path.
#
# Fix: wrap _consume_messages so any unhandled exception causes a clean process
# exit.  The service_loop in run_full_stack.sh restarts the comm service from
# scratch, giving a fresh RTCPeerConnection for the new lobby session.
_orig_consume_messages = WebConnector._consume_messages


async def _safe_consume_messages(self: WebConnector) -> None:
    try:
        await _orig_consume_messages(self)
    except Exception as exc:
        print(
            f"[WebRTC] Fatal error in signaling loop — {type(exc).__name__}: {exc}. "
            "Exiting for clean restart via service_loop.",
            flush=True,
        )
        os._exit(1)


WebConnector._consume_messages = _safe_consume_messages
# -----------------------------------------------------------------------------

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float32
except ImportError:  # pragma: no cover - allows running without ROS2 deps
    rclpy = None
    Twist = None
    ExternalShutdownException = None
    SingleThreadedExecutor = None
    Node = None
    Float32 = None

try:
    from rovr_interfaces.msg import ArmCommand as RosArmCommandMsg
    from rovr_interfaces.msg import CameraCommand as RosCameraCommandMsg
except ImportError:  # pragma: no cover - arm/camera remain disabled if interfaces are unavailable
    RosArmCommandMsg = None
    RosCameraCommandMsg = None

# Raspberry Pi 5 profile
CAMERA_OPTIONS = {
    "video_size": "1280x720",
    "framerate": "20",
    "input_format": "mjpeg",
    "fflags": "nobuffer",
    "flags": "low_delay",
    "analyzeduration": "0",
    "probesize": "32",
    "max_delay": "0",
    "avioflags": "direct",
    "use_wallclock_as_timestamps": "1",
    "thread_queue_size": "2",
}

WEBRTC_DEBUG_ENABLED = os.environ.get("ROVR_WEBRTC_DEBUG", "0").strip().lower() not in {"0", "false", "no"}


def _debug_webrtc(label: str, value: Any) -> None:
    if not WEBRTC_DEBUG_ENABLED:
        return
    try:
        if isinstance(value, (dict, list)):
            text = json.dumps(value, ensure_ascii=False)
        else:
            text = str(value)
    except Exception:
        text = repr(value)
    print(f"[WebRTC DEBUG] {label}: {text}", flush=True)


class LatestFrameVideoTrack(MediaStreamTrack):
    """Wraps a source track and drops stale queued frames to minimize latency."""

    kind = "video"

    def __init__(self, source: MediaStreamTrack):
        super().__init__()
        self._source = source

    async def recv(self) -> Frame | Packet:
        frame_or_packet: Frame | Packet = await self._source.recv()

        source_queue = getattr(self._source, "_queue", None)
        if source_queue is not None:
            while True:
                try:
                    queued = source_queue.get_nowait()
                except asyncio.QueueEmpty:
                    break

                if queued is None:
                    self.stop()
                    raise MediaStreamError
                frame_or_packet = queued

        return frame_or_packet

    def stop(self) -> None:
        super().stop()
        self._source.stop()


def install_low_latency_video_patch() -> None:
    if getattr(RtcWrapper, "_low_latency_patch_installed", False):
        return

    def register_video_track_low_latency(self, source: str, format: str, options: dict[str, Any]) -> bool:
        if self._RtcWrapper__initiated_connection:
            LoggerStore.get_logger().warning("Cannot add video track since connection was already initiated!")
            return False

        try:
            player = MediaPlayer(file=source, format=format, options=options)
            if player.video is None:
                LoggerStore.get_logger().warning(f"No video stream found in source {source} in RTC connection")
                return False

            wrapped_track = LatestFrameVideoTrack(player.video)
            self._RtcWrapper__peer.addTrack(wrapped_track)
            LoggerStore.get_logger().info(
                f"Added low-latency video track from source {source} in RTC connection"
            )
            self._RtcWrapper__players.append(player)

            extra_tracks = getattr(self, "_low_latency_wrapped_tracks", [])
            extra_tracks.append(wrapped_track)
            setattr(self, "_low_latency_wrapped_tracks", extra_tracks)
            return True
        except Exception as e:
            LoggerStore.get_logger().warning(f"Failed to add track from source {source} in RTC connection: {e}")
            return False

    RtcWrapper.register_video_track = register_video_track_low_latency
    RtcWrapper._low_latency_patch_installed = True


def parse_data_channel_message(raw_message: Any) -> dict[str, Any] | None:
    _debug_webrtc("raw_message_type", type(raw_message).__name__)
    _debug_webrtc("raw_message", raw_message)

    if isinstance(raw_message, dict):
        _debug_webrtc("parsed_message", raw_message)
        return raw_message

    if isinstance(raw_message, bytes):
        raw_message = raw_message.decode("utf-8", errors="replace")
        _debug_webrtc("decoded_message", raw_message)

    if not isinstance(raw_message, str):
        LoggerStore.get_logger().warning(
            f"Unsupported message type in data channel: {type(raw_message).__name__}"
        )
        return None

    try:
        parsed = json.loads(raw_message)
    except json.JSONDecodeError as exc:
        LoggerStore.get_logger().warning(f"Invalid JSON in data channel message: {exc}")
        return None

    if not isinstance(parsed, dict):
        LoggerStore.get_logger().warning("Data channel message must be a JSON object")
        return None

    _debug_webrtc("parsed_message", parsed)
    return parsed


def clamp_power(power: float) -> float:
    """Clamp power to the valid hardware range [0.1, 0.9]."""
    return max(0.1, min(0.9, power))


@dataclass(frozen=True)
class DriveCommand:
    linear: int
    angular: float
    power: float


@dataclass(frozen=True)
class CameraServoCommand:
    servos: tuple[int, int]


@dataclass(frozen=True)
class ArmServoCommand:
    timestamp: float
    deadman: bool
    servos: tuple[int, int, int, int, int, int]


ARM_JOINT_DEG_LIMITS: tuple[tuple[float, float], ...] = (
    (-180.0, 180.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
)
DEFAULT_CAMERA_YAW_DEG_LIMITS: tuple[float, float] = (-140.0, 145.0)
DEFAULT_CAMERA_PITCH_DEG_LIMITS: tuple[float, float] = (-70.0, 27.0)
DEFAULT_ROBOT_CONFIG_DIR = Path(__file__).resolve().parents[4] / "config"
DEFAULT_ARM_LIMITS_PATH = str(DEFAULT_ROBOT_CONFIG_DIR / "arm_limits.json")
DEFAULT_CAMERA_LIMITS_PATH = str(DEFAULT_ROBOT_CONFIG_DIR / "camera_limits.json")
DEFAULT_CAMERA_SAFETY_MARGIN_TICKS = 80
DEFAULT_CAMERA_WRAP_GUARD_TICKS = 8


def _load_degree_limits_from_env(env_prefix: str, fallback: tuple[float, float]) -> tuple[float, float]:
    min_key = f"{env_prefix}_MIN"
    max_key = f"{env_prefix}_MAX"
    raw_min = os.environ.get(min_key)
    raw_max = os.environ.get(max_key)
    if raw_min is None or raw_max is None:
        return fallback
    try:
        low = float(raw_min)
        high = float(raw_max)
    except ValueError:
        return fallback
    if not high > low:
        return fallback
    return low, high


CAMERA_YAW_DEG_LIMITS = _load_degree_limits_from_env(
    "ROVR_CAMERA_YAW_DEG",
    DEFAULT_CAMERA_YAW_DEG_LIMITS,
)
CAMERA_PITCH_DEG_LIMITS = _load_degree_limits_from_env(
    "ROVR_CAMERA_PITCH_DEG",
    DEFAULT_CAMERA_PITCH_DEG_LIMITS,
)


def _load_camera_axis_calibration(
    axis_id: str,
    *,
    fallback_home: int,
    fallback_low: int = 0,
    fallback_high: int = 4095,
) -> tuple[int, int, int]:
    """Returns (minimum, home, maximum) for one camera axis."""
    path = os.environ.get("ROVR_CAMERA_LIMITS_FILE", DEFAULT_CAMERA_LIMITS_PATH)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            raw = json.load(handle)
    except Exception:
        return fallback_low, fallback_home, fallback_high

    if not isinstance(raw, dict):
        return fallback_low, fallback_home, fallback_high
    item = raw.get(axis_id)
    if not isinstance(item, dict):
        return fallback_low, fallback_home, fallback_high

    try:
        minimum = int(item["min"])
        maximum = int(item["max"])
        home = int(item.get("home", fallback_home))
    except (TypeError, ValueError, KeyError):
        return fallback_low, fallback_home, fallback_high

    if minimum > maximum:
        return fallback_low, fallback_home, fallback_high
    margin_raw = os.environ.get("ROVR_CAMERA_SAFETY_MARGIN_TICKS")
    try:
        margin_ticks = int(margin_raw) if margin_raw is not None else int(DEFAULT_CAMERA_SAFETY_MARGIN_TICKS)
    except ValueError:
        margin_ticks = int(DEFAULT_CAMERA_SAFETY_MARGIN_TICKS)
    margin_ticks = max(0, margin_ticks)
    if (maximum - minimum) > (2 * margin_ticks):
        minimum += margin_ticks
        maximum -= margin_ticks

    center_raw = item.get("center_raw")
    center_virtual = item.get("center_virtual", 2048)
    if center_raw is not None:
        try:
            center_raw_i = int(center_raw)
            center_virtual_i = int(center_virtual)
            wrap_guard_raw = os.environ.get("ROVR_CAMERA_WRAP_GUARD_TICKS")
            wrap_guard = int(wrap_guard_raw) if wrap_guard_raw is not None else int(DEFAULT_CAMERA_WRAP_GUARD_TICKS)
            wrap_guard = max(0, wrap_guard)
            high_wrap_threshold = center_virtual_i + (4096 - center_raw_i)
            safe_high = high_wrap_threshold - 1 - wrap_guard
            low_wrap_threshold = center_virtual_i - center_raw_i
            safe_low = low_wrap_threshold + wrap_guard
            if safe_high >= minimum and maximum > safe_high:
                maximum = safe_high
            if safe_low <= maximum and minimum < safe_low:
                minimum = safe_low
        except (TypeError, ValueError):
            pass
    if home < minimum:
        home = minimum
    if home > maximum:
        home = maximum
    return minimum, home, maximum


CAMERA_PAN_CALIBRATION = _load_camera_axis_calibration("1", fallback_home=2048)
CAMERA_TILT_CALIBRATION = _load_camera_axis_calibration("2", fallback_home=2048)


def _load_gripper_presets(
    servo_id: str = "6",
    *,
    fallback_open: int = 2048,
    fallback_closed: int = 3357,
) -> tuple[int, int]:
    """Load gripper open/closed tick values in virtual command-space."""
    path = os.environ.get("ROVR_ARM_LIMITS_FILE", DEFAULT_ARM_LIMITS_PATH)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            raw = json.load(handle)
    except Exception:
        return fallback_open, fallback_closed

    item = raw.get(servo_id)
    if not isinstance(item, dict):
        return fallback_open, fallback_closed

    center_raw = item.get("center_raw")
    center_virtual = item.get("center_virtual", 2048)

    def raw_to_virtual(raw_val: int) -> int:
        if center_raw is not None:
            return int(raw_val) - int(center_raw) + int(center_virtual)
        return int(raw_val)

    # Prefer explicit command-space values, fall back to raw conversion
    open_cmd = item.get("gripper_open_cmd")
    closed_cmd = item.get("gripper_closed_cmd")

    if open_cmd is None:
        open_raw = item.get("gripper_open_raw")
        if open_raw is not None:
            open_cmd = raw_to_virtual(int(open_raw))
    if closed_cmd is None:
        closed_raw = item.get("gripper_closed_raw")
        if closed_raw is not None:
            closed_cmd = raw_to_virtual(int(closed_raw))

    if open_cmd is None or closed_cmd is None:
        return fallback_open, fallback_closed

    # Clamp to servo limits
    lo = int(item.get("min", 0))
    hi = int(item.get("max", 4095))
    return max(lo, min(hi, int(open_cmd))), max(lo, min(hi, int(closed_cmd)))


GRIPPER_OPEN_TICK, GRIPPER_CLOSED_TICK = _load_gripper_presets()


class RosCommandBridge:
    """Publishes drive/camera/arm commands from WebRTC into ROS2 topics."""

    def __init__(self, hold_last_drive_sec: float = 0.4, publish_rate_hz: float = 20.0):
        if rclpy is None or Twist is None or Float32 is None or Node is None or SingleThreadedExecutor is None:
            raise RuntimeError("ROS2 python dependencies are not available")

        self._publish_rate_hz = max(1.0, float(publish_rate_hz))
        self._hold_last_drive_sec = max(0.0, float(hold_last_drive_sec))

        self._node: Node | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        self._lock = threading.Lock()
        self._latest_drive: DriveCommand | None = None
        self._latest_drive_update: float = 0.0
        self._stop_published = False
        self._pending_camera: CameraServoCommand | None = None
        self._pending_arm: ArmServoCommand | None = None
        self._owns_rclpy_context = False

    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True

        self._node = Node("control_project_command_bridge")
        self._cmd_vel_pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
        self._cmd_power_pub = self._node.create_publisher(Float32, "/cmd_power", 10)
        self._camera_pub = (
            self._node.create_publisher(RosCameraCommandMsg, "/camera/target_cmd", 10)
            if RosCameraCommandMsg is not None
            else None
        )
        self._arm_pub = (
            self._node.create_publisher(RosArmCommandMsg, "/arm/target_cmd", 10)
            if RosArmCommandMsg is not None
            else None
        )
        self._node.create_timer(1.0 / self._publish_rate_hz, self._on_publish_tick)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin_worker, name="ros-command-bridge", daemon=True)
        self._spin_thread.start()

        self._node.get_logger().info("ROS command bridge started: /cmd_vel, /cmd_power")
        if self._camera_pub is None:
            self._node.get_logger().warning(
                "Camera ROS interface is unavailable (rovr_interfaces not found); camera publishing is disabled"
            )
        if self._arm_pub is None:
            self._node.get_logger().warning(
                "Arm ROS interface is unavailable (rovr_interfaces not found); arm publishing is disabled"
            )

    def stop(self) -> None:
        self._stop_event.set()
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)

        if self._executor is not None and self._node is not None:
            try:
                self._executor.remove_node(self._node)
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass

        if self._owns_rclpy_context and rclpy is not None and rclpy.ok():
            rclpy.shutdown()

    def submit_drive(self, command: DriveCommand) -> None:
        with self._lock:
            self._latest_drive = command
            self._latest_drive_update = time.monotonic()
            self._stop_published = False

    def submit_camera(self, command: CameraServoCommand) -> None:
        with self._lock:
            self._pending_camera = command

    def submit_arm(self, command: ArmServoCommand) -> None:
        with self._lock:
            self._pending_arm = command

    def _spin_worker(self) -> None:
        while not self._stop_event.is_set() and self._executor is not None and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception as exc:
                if ExternalShutdownException is not None and isinstance(exc, ExternalShutdownException):
                    break
                if self._stop_event.is_set():
                    break
                raise

    def _on_publish_tick(self) -> None:
        camera: CameraServoCommand | None
        arm: ArmServoCommand | None
        now = time.monotonic()
        drive: DriveCommand | None
        with self._lock:
            camera = self._pending_camera
            arm = self._pending_arm
            self._pending_camera = None
            self._pending_arm = None
            drive = self._latest_drive
            last_update = self._latest_drive_update
            stop_already_published = self._stop_published

        if camera is not None:
            self._publish_camera(camera)
        if arm is not None:
            self._publish_arm(arm)

        if drive is None:
            return

        if (now - last_update) <= self._hold_last_drive_sec:
            self._publish_drive(drive)
            return

        if not stop_already_published:
            self._publish_drive(DriveCommand(linear=0, angular=0.0, power=0.1))
            with self._lock:
                self._stop_published = True

    def _publish_drive(self, command: DriveCommand) -> None:
        twist = Twist()
        twist.linear.x = float(command.linear)
        twist.angular.z = float(command.angular)
        power = Float32()
        power.data = float(command.power)
        self._cmd_vel_pub.publish(twist)
        self._cmd_power_pub.publish(power)

    def _publish_camera(self, command: CameraServoCommand) -> None:
        if self._camera_pub is None or RosCameraCommandMsg is None:
            return
        msg = RosCameraCommandMsg()
        msg.servos = list(command.servos)
        self._camera_pub.publish(msg)

    def _publish_arm(self, command: ArmServoCommand) -> None:
        if self._arm_pub is None or RosArmCommandMsg is None:
            return
        msg = RosArmCommandMsg()
        msg.timestamp = float(command.timestamp)
        msg.deadman_switch = bool(command.deadman)
        msg.servos = list(command.servos)
        self._arm_pub.publish(msg)


ROS_COMMAND_BRIDGE: RosCommandBridge | None = None


def apply_drive_command(command: DriveCommand) -> None:
    direction_label = {-1: "reverse", 0: "stop", 1: "forward"}[command.linear]
    LoggerStore.get_logger().info(
        "drive command accepted: "
        f"power={command.power:.1f}, linear={direction_label}({command.linear}), angular={command.angular:.3f}"
    )
    if ROS_COMMAND_BRIDGE is not None:
        ROS_COMMAND_BRIDGE.submit_drive(command)


def apply_camera_command(command: CameraServoCommand) -> None:
    LoggerStore.get_logger().info(
        f"camera command accepted: servos={list(command.servos)}"
    )
    if ROS_COMMAND_BRIDGE is not None:
        ROS_COMMAND_BRIDGE.submit_camera(command)


def apply_arm_command(command: ArmServoCommand) -> None:
    LoggerStore.get_logger().info(
        "arm command accepted: "
        f"deadman={command.deadman}, servos={list(command.servos)}"
    )
    if ROS_COMMAND_BRIDGE is not None:
        ROS_COMMAND_BRIDGE.submit_arm(command)


def try_start_ros_command_bridge() -> None:
    global ROS_COMMAND_BRIDGE
    if rclpy is None:
        LoggerStore.get_logger().warning(
            "ROS2 libraries are not available. Commands will not be published to ROS topics."
        )
        return

    try:
        hold_sec = float(os.environ.get("ROVR_HOLD_DRIVE_SEC", "1.5"))
        bridge = RosCommandBridge(hold_last_drive_sec=hold_sec)
        bridge.start()
        ROS_COMMAND_BRIDGE = bridge
    except Exception as exc:
        LoggerStore.get_logger().warning(f"Failed to start ROS command bridge: {exc}")


def parse_drive_block(raw_drive: Any) -> DriveCommand | None:
    if not isinstance(raw_drive, dict):
        LoggerStore.get_logger().warning("drive block must be a JSON object")
        return

    linear_raw = raw_drive.get("linear")
    angular_raw = raw_drive.get("angular")
    power_raw = raw_drive.get("power")
    if not isinstance(linear_raw, (int, float)):
        LoggerStore.get_logger().warning("drive.linear must be a number")
        return
    if not isinstance(angular_raw, (int, float)):
        LoggerStore.get_logger().warning("drive.angular must be a number")
        return
    if not isinstance(power_raw, (int, float)):
        LoggerStore.get_logger().warning("drive.power must be a number")
        return

    linear = _axis_to_linear_direction(float(linear_raw))
    angular = max(-1.0, min(1.0, float(angular_raw)))
    power = clamp_power(float(power_raw))

    return DriveCommand(
        linear=linear,
        angular=angular,
        power=power,
    )


def _axis_to_linear_direction(axis_value: float) -> int:
    # Keep drive semantics compatible with ROS side: -1 (reverse), 0 (stop), 1 (forward).
    if axis_value > 0.0:
        return 1
    if axis_value < 0.0:
        return -1
    return 0


def parse_drive_content_block(raw_content: Any) -> DriveCommand | None:
    """Parses VR drive payload shape:
    {
      "content": {
        "direction": {"x": <angular>, "y": <linear-axis>},
        "power": <0.1..0.9>
      }
    }
    """
    if not isinstance(raw_content, dict):
        LoggerStore.get_logger().warning("drive.content must be a JSON object")
        return None

    direction_raw = raw_content.get("direction")
    power_raw = raw_content.get("power")
    if not isinstance(direction_raw, dict):
        LoggerStore.get_logger().warning("drive.content.direction must be an object")
        return None

    angular_raw = direction_raw.get("x")
    linear_axis_raw = direction_raw.get("y")
    if not isinstance(angular_raw, (int, float)):
        LoggerStore.get_logger().warning("drive.content.direction.x must be a number")
        return None
    if not isinstance(linear_axis_raw, (int, float)):
        LoggerStore.get_logger().warning("drive.content.direction.y must be a number")
        return None
    if not isinstance(power_raw, (int, float)):
        LoggerStore.get_logger().warning("drive.content.power must be a number")
        return None

    angular = max(-1.0, min(1.0, float(angular_raw)))
    linear_axis = max(-1.0, min(1.0, float(linear_axis_raw)))

    return parse_drive_block(
        {
            "linear": _axis_to_linear_direction(linear_axis),
            "angular": angular,
            "power": float(power_raw),
        }
    )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def _angle_to_tick(angle_deg: float, degree_limits: tuple[float, float]) -> int:
    low, high = degree_limits
    if not high > low:
        raise ValueError("Invalid degree limits")
    bounded = _clamp(angle_deg, low, high)
    ratio = (bounded - low) / (high - low)
    return int(round(ratio * 4095.0))


def _angle_to_tick_with_calibration(
    angle_deg: float,
    degree_limits: tuple[float, float],
    axis_calibration: tuple[int, int, int],
) -> int:
    low_deg, high_deg = float(degree_limits[0]), float(degree_limits[1])
    if not high_deg > low_deg:
        raise ValueError("Invalid degree limits")

    minimum, home, maximum = axis_calibration
    bounded = _clamp(float(angle_deg), low_deg, high_deg)
    if bounded >= 0.0:
        positive_span = max(1e-9, high_deg)
        ratio = bounded / positive_span
        raw_tick = int(round(float(home) + ratio * float(maximum - home)))
    else:
        negative_span = max(1e-9, abs(low_deg))
        ratio = abs(bounded) / negative_span
        raw_tick = int(round(float(home) - ratio * float(home - minimum)))

    return int(_clamp(float(raw_tick), float(minimum), float(maximum)))


def _parse_timestamp(payload: dict[str, Any]) -> float:
    ts_raw = payload.get("ts")
    if isinstance(ts_raw, (int, float)):
        return float(ts_raw)
    return float(time.time())


def _parse_gripper_tick(raw_gripper: Any) -> int | None:
    if isinstance(raw_gripper, str):
        normalized = raw_gripper.strip().lower()
        if normalized in {"open", "opened"}:
            return int(GRIPPER_OPEN_TICK)
        if normalized in {"close", "closed"}:
            return int(GRIPPER_CLOSED_TICK)
        LoggerStore.get_logger().warning("arm.gripper must be 'open', 'closed', or an integer tick")
        return None

    if isinstance(raw_gripper, (int, float)):
        return int(_clamp(float(raw_gripper), 0.0, 4095.0))

    LoggerStore.get_logger().warning("arm.gripper is required")
    return None


def parse_camera_block(raw_camera: Any) -> CameraServoCommand | None:
    if not isinstance(raw_camera, dict):
        LoggerStore.get_logger().warning("camera block must be a JSON object")
        return None

    servos_raw = raw_camera.get("servos")
    if isinstance(servos_raw, list):
        if len(servos_raw) != 2:
            LoggerStore.get_logger().warning("camera.servos must have length 2")
            return None
        try:
            pan = int(_clamp(float(servos_raw[0]), 0.0, 4095.0))
            tilt = int(_clamp(float(servos_raw[1]), 0.0, 4095.0))
        except (TypeError, ValueError):
            LoggerStore.get_logger().warning("camera.servos must contain numeric values")
            return None
        return CameraServoCommand(servos=(pan, tilt))

    yaw_raw = raw_camera.get("yaw_deg")
    pitch_raw = raw_camera.get("pitch_deg")
    if isinstance(yaw_raw, (int, float)) and isinstance(pitch_raw, (int, float)):
        pan = _angle_to_tick_with_calibration(
            float(yaw_raw),
            CAMERA_YAW_DEG_LIMITS,
            CAMERA_PAN_CALIBRATION,
        )
        tilt = _angle_to_tick_with_calibration(
            float(pitch_raw),
            CAMERA_PITCH_DEG_LIMITS,
            CAMERA_TILT_CALIBRATION,
        )
        return CameraServoCommand(servos=(pan, tilt))

    LoggerStore.get_logger().warning(
        "camera must contain either servos[2] or yaw_deg/pitch_deg"
    )
    return None


def parse_arm_block(raw_arm: Any, timestamp: float) -> ArmServoCommand | None:
    if not isinstance(raw_arm, dict):
        LoggerStore.get_logger().warning("arm block must be a JSON object")
        return None

    deadman = bool(raw_arm.get("deadman", False))
    servos_raw = raw_arm.get("servos")
    if isinstance(servos_raw, list):
        if len(servos_raw) == 6:
            try:
                values = [int(_clamp(float(value), 0.0, 4095.0)) for value in servos_raw]
            except (TypeError, ValueError):
                LoggerStore.get_logger().warning("arm.servos must contain numeric values")
                return None
            return ArmServoCommand(
                timestamp=float(timestamp),
                deadman=deadman,
                servos=(values[0], values[1], values[2], values[3], values[4], values[5]),
            )
        if len(servos_raw) == 5:
            try:
                base = [int(_clamp(float(value), 0.0, 4095.0)) for value in servos_raw]
            except (TypeError, ValueError):
                LoggerStore.get_logger().warning("arm.servos must contain numeric values")
                return None
            gripper = _parse_gripper_tick(raw_arm.get("gripper"))
            if gripper is None:
                return None
            return ArmServoCommand(
                timestamp=float(timestamp),
                deadman=deadman,
                servos=(base[0], base[1], base[2], base[3], base[4], int(gripper)),
            )
        LoggerStore.get_logger().warning("arm.servos must have length 5 or 6")
        return None

    joints_raw = raw_arm.get("joints_deg")
    if isinstance(joints_raw, list):
        if len(joints_raw) != 5:
            LoggerStore.get_logger().warning("arm.joints_deg must have length 5")
            return None
        try:
            base_ticks = [
                _angle_to_tick(float(angle), ARM_JOINT_DEG_LIMITS[index])
                for index, angle in enumerate(joints_raw)
            ]
        except (TypeError, ValueError):
            LoggerStore.get_logger().warning("arm.joints_deg must contain numeric values")
            return None
        gripper = _parse_gripper_tick(raw_arm.get("gripper"))
        if gripper is None:
            return None
        return ArmServoCommand(
            timestamp=float(timestamp),
            deadman=deadman,
            servos=(
                int(base_ticks[0]),
                int(base_ticks[1]),
                int(base_ticks[2]),
                int(base_ticks[3]),
                int(base_ticks[4]),
                int(gripper),
            ),
        )

    LoggerStore.get_logger().warning(
        "arm must contain either servos[5|6] or joints_deg[5] + gripper"
    )
    return None


def handle_control_blocks(payload: dict[str, Any], required_block: str | None = None) -> None:
    if required_block is not None and payload.get(required_block) is None:
        LoggerStore.get_logger().warning(f"Message type requires '{required_block}' block")
        return

    timestamp = _parse_timestamp(payload)
    has_any_block = False

    drive_raw = payload.get("drive")
    if drive_raw is not None:
        has_any_block = True
        drive = parse_drive_block(drive_raw)
        if drive is not None:
            apply_drive_command(drive)

    camera_raw = payload.get("camera")
    if camera_raw is not None:
        has_any_block = True
        camera = parse_camera_block(camera_raw)
        if camera is not None:
            apply_camera_command(camera)

    arm_raw = payload.get("arm")
    if arm_raw is not None:
        has_any_block = True
        arm = parse_arm_block(arm_raw, timestamp=timestamp)
        if arm is not None:
            apply_arm_command(arm)

    if not has_any_block:
        LoggerStore.get_logger().warning("Control message does not contain drive/camera/arm blocks")


def handle_locomotion_payload(content: dict[str, Any]) -> None:
    """Legacy compatibility path: locomotion -> drive mapping."""
    if not isinstance(content, dict):
        LoggerStore.get_logger().warning("locomotion.content must be a JSON object")
        return

    power_raw = content.get("power")
    direction_raw = content.get("direction")
    if not isinstance(direction_raw, dict):
        LoggerStore.get_logger().warning("locomotion.content.direction must be an object")
        return

    drive = parse_drive_block({
        "linear": direction_raw.get("x"),
        "angular": direction_raw.get("y"),
        "power": power_raw,
    })
    if drive is None:
        return
    apply_drive_command(drive)


def handle_drive_payload(payload: dict[str, Any]) -> None:
    # 1) Preferred legacy/hybrid shape: {"type":"drive","drive":{...}}
    drive_raw = payload.get("drive")
    if drive_raw is not None:
        drive = parse_drive_block(drive_raw)
        if drive is not None:
            apply_drive_command(drive)
        return

    # 2) New VR shape: {"type":"drive","content":{"direction":{"x","y"},"power":...}}
    content_raw = payload.get("content")
    if content_raw is not None:
        drive = parse_drive_content_block(content_raw)
        if drive is not None:
            apply_drive_command(drive)
        return

    LoggerStore.get_logger().warning(
        "drive message must contain either 'drive' block or 'content' block"
    )


def handle_camera_payload(payload: dict[str, Any]) -> None:
    # 1) Legacy/hybrid shape: {"type":"camera","camera":{...}}
    camera_raw = payload.get("camera")
    if camera_raw is not None:
        camera = parse_camera_block(camera_raw)
        if camera is not None:
            apply_camera_command(camera)
        return

    # 2) New VR shape: {"type":"camera","content":{"yaw_deg":..., "pitch_deg":...}}
    content_raw = payload.get("content")
    if content_raw is not None:
        if isinstance(content_raw, dict) and content_raw.get("camera") is not None:
            camera = parse_camera_block(content_raw.get("camera"))
        else:
            camera = parse_camera_block(content_raw)
        if camera is not None:
            apply_camera_command(camera)
        return

    LoggerStore.get_logger().warning(
        "camera message must contain either 'camera' block or 'content' block"
    )


def handle_data_channel_message(raw_message: Any) -> None:
    payload = parse_data_channel_message(raw_message)
    if payload is None:
        return

    message_type = payload.get("type")
    _debug_webrtc("message_type", message_type)
    if not isinstance(message_type, str):
        LoggerStore.get_logger().warning("Data message must contain string field 'type'")
        return

    if message_type == "control_frame":
        handle_control_blocks(payload)
        return

    if message_type == "drive":
        handle_drive_payload(payload)
        return

    if message_type == "camera":
        handle_camera_payload(payload)
        return

    if message_type == "arm":
        # Support VR shape: {"type":"arm","content":{...}} alongside
        # legacy shape: {"type":"arm","arm":{...}}
        if payload.get("arm") is None and isinstance(payload.get("content"), dict):
            payload["arm"] = payload["content"]
        # VR controller has no physical deadman switch — sending an arm command
        # implies active control.  Homing is handled by the heartbeat watchdog
        # when commands stop arriving.
        arm_block = payload.get("arm")
        if isinstance(arm_block, dict) and "deadman" not in arm_block:
            arm_block["deadman"] = True
        handle_control_blocks(payload, required_block="arm")
        return

    if message_type == "locomotion":
        handle_locomotion_payload(payload.get("content"))
        return

    LoggerStore.get_logger().warning(f"Unsupported data message type: {message_type}")


async def app():
    install_low_latency_video_patch()
    try_start_ros_command_bridge()

    websocket_url = _require_env("ROVR_WS_URL")
    credentials = WebCredentials(
        _require_env("ROVR_DEVICE_ID"),
        _require_env("ROVR_DEVICE_SECRET"),
        {
            "urls": _require_env("ROVR_TURN_URL"),
            "user": _require_env("ROVR_TURN_USER"),
            "pass": _require_env("ROVR_TURN_PASS"),
        },
    )
    connector = WebConnector(
        credentials,
        on_lobby=lambda lobby_id: print(f"Joined lobby with id: {lobby_id}", flush=True),
        on_connected=lambda connection: on_connected(connection),
        on_error=lambda err: print(f"Error occurred: {err}", flush=True),
        on_closed=lambda: print("Websocket connection closed", flush=True),
    )

    print(f"Connecting WebSocket: {websocket_url}", flush=True)
    await connector.init_connection(websocket_url)
    if connector.rtcWrapper is None:
        raise RuntimeError("RTC wrapper was not initialized after init_connection()")
    connector.rtcWrapper.register_channel("data")

    disable_video = os.environ.get("ROVR_DISABLE_VIDEO", "").strip().lower() in {"1", "true", "yes"}
    primary_video_device = os.environ.get("ROVR_VIDEO_DEVICE", "/dev/video0")
    fallback_video_device = os.environ.get("ROVR_VIDEO_FALLBACK_DEVICE", "/dev/video1")
    if disable_video:
        print("Skipping video track registration because ROVR_DISABLE_VIDEO is enabled", flush=True)
    else:
        video_candidates: list[str] = []
        for candidate in (primary_video_device, fallback_video_device):
            value = str(candidate).strip()
            if value and value not in video_candidates:
                video_candidates.append(value)

        video_registered = False
        for device_path in video_candidates:
            if not os.path.exists(device_path):
                continue
            if connector.rtcWrapper.register_video_track(device_path, "v4l2", CAMERA_OPTIONS):
                video_registered = True
                break

        if not video_registered:
            print(
                "Skipping video track registration because no available device could be opened: "
                f"{video_candidates}",
                flush=True,
            )

    try:
        while True:
            await asyncio.sleep(1)
    finally:
        if ROS_COMMAND_BRIDGE is not None:
            ROS_COMMAND_BRIDGE.stop()


def on_connected(connection: RtcConnection):
    connection.register_channel_listener("data", handle_data_channel_message)
    connection.on_channel_open("data", lambda: LoggerStore.get_logger().info("Data channel is open"))


def main():
    asyncio.run(app())


if __name__ == "__main__":
    main()
