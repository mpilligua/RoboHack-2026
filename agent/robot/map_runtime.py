from __future__ import annotations

import hashlib
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

try:
    import roslibpy  # type: ignore
except ImportError:  # pragma: no cover - local unit tests run without ROS deps
    roslibpy = None


TF_STALE_S = 0.5
SCAN_STALE_S = 1.0
LOCAL_COSTMAP_STALE_S = 2.0
GLOBAL_COSTMAP_STALE_S = 8.0
DEFAULT_MAP_WARN_S = 30.0
UNKNOWN = -1
FREE = 0
OCCUPIED = 100
OCCUPIED_THRESHOLD = 50
MAX_GRID_SIZE_M = 8.0
MIN_CELL_SIZE_M = 0.1
MAX_CELL_SIZE_M = 1.0


@dataclass
class _CacheEntry:
    data: Optional[dict] = None
    receipt_ts: Optional[float] = None

    def age_s(self) -> Optional[float]:
        if self.receipt_ts is None:
            return None
        return max(0.0, time.time() - self.receipt_ts)


@dataclass
class _Transform2D:
    x: float
    y: float
    yaw: float
    stamp: Optional[float]
    receipt_ts: float
    is_static: bool = False

    def age_s(self) -> float:
        return max(0.0, time.time() - self.receipt_ts)


def _yaw_from_quat(q: dict) -> float:
    siny_cosp = 2.0 * (float(q.get("w", 1.0)) * float(q.get("z", 0.0)) + float(q.get("x", 0.0)) * float(q.get("y", 0.0)))
    cosy_cosp = 1.0 - 2.0 * (float(q.get("y", 0.0)) ** 2 + float(q.get("z", 0.0)) ** 2)
    return math.atan2(siny_cosp, cosy_cosp)


def _norm_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


def _compose(a: _Transform2D, b: _Transform2D) -> _Transform2D:
    ca = math.cos(a.yaw)
    sa = math.sin(a.yaw)
    x = a.x + ca * b.x - sa * b.y
    y = a.y + sa * b.x + ca * b.y
    return _Transform2D(
        x=x,
        y=y,
        yaw=_norm_angle(a.yaw + b.yaw),
        stamp=b.stamp or a.stamp,
        receipt_ts=min(a.receipt_ts, b.receipt_ts),
        is_static=a.is_static and b.is_static,
    )


def _invert(t: _Transform2D) -> _Transform2D:
    c = math.cos(t.yaw)
    s = math.sin(t.yaw)
    x = -(c * t.x + s * t.y)
    y = -(-s * t.x + c * t.y)
    return _Transform2D(
        x=x,
        y=y,
        yaw=_norm_angle(-t.yaw),
        stamp=t.stamp,
        receipt_ts=t.receipt_ts,
        is_static=t.is_static,
    )


def _distance(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


class Nav2PathClient:
    """Best-effort Nav2 planning adapter with optional actionlib support."""

    def __init__(
        self,
        ros_client: Any,
        *,
        action_name: str = "/compute_path_to_pose",
        action_type: str = "nav2_msgs/action/ComputePathToPose",
        injected_compute: Optional[Callable[[dict, dict, float], dict]] = None,
    ) -> None:
        self._ros_client = ros_client
        self._action_name = action_name
        self._action_type = action_type
        self._injected_compute = injected_compute
        self._action_client = None
        self._action_failed = False

    def compute_path(self, start_pose: dict, goal_pose: dict, timeout_s: float = 3.0) -> dict:
        if self._injected_compute is not None:
            return self._injected_compute(start_pose, goal_pose, timeout_s)
        if self._action_failed or roslibpy is None or not hasattr(roslibpy, "actionlib"):
            raise RuntimeError("nav2 action client unavailable over rosbridge")
        if self._action_client is None:
            try:
                self._action_client = roslibpy.actionlib.ActionClient(
                    self._ros_client,
                    self._action_name,
                    self._action_type,
                )
            except Exception as exc:
                self._action_failed = True
                raise RuntimeError(f"could not initialize nav2 action client: {exc}") from exc

        done = threading.Event()
        outcome: dict[str, Any] = {}
        goal_msg = {
            "goal": _pose_stamped(goal_pose),
            "start": _pose_stamped(start_pose),
            "planner_id": "",
            "use_start": True,
        }

        def _on_result(result: dict) -> None:
            outcome["result"] = result
            done.set()

        try:
            goal = roslibpy.actionlib.Goal(self._action_client, roslibpy.Message(goal_msg))
            goal.send(result_callback=_on_result)
        except Exception as exc:
            self._action_failed = True
            raise RuntimeError(f"nav2 compute_path_to_pose send failed: {exc}") from exc

        if not done.wait(timeout_s):
            try:
                goal.cancel()
            except Exception:
                pass
            raise RuntimeError("nav2 compute_path_to_pose timed out")
        return outcome.get("result") or {}


class Nav2NavigateClient:
    """Best-effort Nav2 navigate_to_pose adapter with lightweight status caching."""

    def __init__(
        self,
        ros_client: Any,
        *,
        action_name: str = "/navigate_to_pose",
        action_type: str = "nav2_msgs/action/NavigateToPose",
        injected_send: Optional[Callable[[dict, float], dict]] = None,
        injected_cancel: Optional[Callable[[str], dict]] = None,
    ) -> None:
        self._ros_client = ros_client
        self._action_name = action_name
        self._action_type = action_type
        self._injected_send = injected_send
        self._injected_cancel = injected_cancel
        self._action_client = None
        self._action_failed = False
        self._lock = threading.Lock()
        self._goal = None
        self._status = "idle"
        self._status_ts: Optional[float] = None
        self._last_goal: Optional[dict] = None
        self._last_result: Optional[dict] = None
        self._last_error: Optional[str] = None

    def navigate_to_pose(self, goal_pose: dict, timeout_s: float = 3.0) -> dict:
        if self._injected_send is not None:
            result = self._injected_send(goal_pose, timeout_s)
            with self._lock:
                self._status = str(result.get("status", "accepted"))
                self._status_ts = time.time()
                self._last_goal = dict(goal_pose)
                self._last_result = dict(result)
                self._last_error = None
            return result
        if self._action_failed or roslibpy is None or not hasattr(roslibpy, "actionlib"):
            raise RuntimeError("nav2 navigate_to_pose action client unavailable over rosbridge")
        if self._action_client is None:
            try:
                self._action_client = roslibpy.actionlib.ActionClient(
                    self._ros_client,
                    self._action_name,
                    self._action_type,
                )
            except Exception as exc:
                self._action_failed = True
                raise RuntimeError(f"could not initialize nav2 navigate client: {exc}") from exc

        result_ready = threading.Event()
        goal_msg = {
            "pose": _pose_stamped(goal_pose),
            "behavior_tree": "",
        }

        def _on_result(result: dict) -> None:
            with self._lock:
                self._last_result = result
                self._status = self._result_status(result)
                self._status_ts = time.time()
                self._last_error = None if self._status == "succeeded" else self._result_error(result)
            result_ready.set()

        try:
            goal = roslibpy.actionlib.Goal(self._action_client, roslibpy.Message(goal_msg))
            with self._lock:
                self._goal = goal
                self._status = "sent"
                self._status_ts = time.time()
                self._last_goal = dict(goal_pose)
                self._last_error = None
            goal.send(result_callback=_on_result)
        except Exception as exc:
            self._action_failed = True
            raise RuntimeError(f"nav2 navigate_to_pose send failed: {exc}") from exc

        # Wait briefly for an immediate transport-side failure, but do not block for completion.
        result_ready.wait(timeout=min(max(timeout_s, 0.0), 0.25))
        return self.get_status()

    def cancel(self, reason: str = "stop") -> dict:
        if self._injected_cancel is not None:
            result = self._injected_cancel(reason)
            with self._lock:
                self._status = str(result.get("status", "cancelled"))
                self._status_ts = time.time()
                self._last_error = None
            return result
        with self._lock:
            goal = self._goal
        if goal is None:
            return {"status": "idle", "reason": "no active nav2 goal"}
        try:
            goal.cancel()
        except Exception as exc:
            raise RuntimeError(f"nav2 cancel failed: {exc}") from exc
        with self._lock:
            self._status = "cancel_requested"
            self._status_ts = time.time()
            self._last_error = None
        return self.get_status(reason=reason)

    def get_status(self, *, reason: Optional[str] = None) -> dict:
        with self._lock:
            stamp = self._status_ts
            status = self._status
            goal = None if self._last_goal is None else dict(self._last_goal)
            result = None if self._last_result is None else dict(self._last_result)
            error = self._last_error
        age_s = None if stamp is None else max(0.0, time.time() - stamp)
        payload = {
            "status": status,
            "updated_at": stamp,
            "age_s": age_s,
            "goal_pose": goal,
            "result": result,
        }
        if error:
            payload["error"] = error
        if reason is not None:
            payload["reason"] = reason
        return payload

    def _result_status(self, result: dict) -> str:
        result_blob = result.get("result", result)
        code = result_blob.get("error_code")
        if code in (0, None):
            return "succeeded"
        return "failed"

    def _result_error(self, result: dict) -> Optional[str]:
        result_blob = result.get("result", result)
        code = result_blob.get("error_code")
        if code in (0, None):
            return None
        msg = result_blob.get("error_msg") or result_blob.get("message") or f"error_code={code}"
        return str(msg)


def _pose_stamped(pose: dict) -> dict:
    yaw = float(pose.get("yaw_rad", pose.get("yaw", 0.0)))
    half = yaw / 2.0
    return {
        "header": {"frame_id": "map"},
        "pose": {
            "position": {
                "x": float(pose["x_m"]),
                "y": float(pose["y_m"]),
                "z": 0.0,
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(half),
                "w": math.cos(half),
            },
        },
    }


class MapRuntime:
    def __init__(
        self,
        *,
        ros_client: Any = None,
        base_frame: str = "rslidar",
        map_frame: str = "map",
        odom_frame: str = "odom",
        map_warn_s: float = DEFAULT_MAP_WARN_S,
        nav2_client: Optional[Nav2PathClient] = None,
        nav2_navigate_client: Optional[Nav2NavigateClient] = None,
    ) -> None:
        self.base_frame = base_frame
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        self.map_warn_s = float(map_warn_s)
        self._lock = threading.RLock()
        self._ros_client = ros_client
        self._subs: list[Any] = []
        self._topics = {
            "map": _CacheEntry(),
            "map_metadata": _CacheEntry(),
            "scan": _CacheEntry(),
            "local_costmap": _CacheEntry(),
            "global_costmap": _CacheEntry(),
        }
        self._transforms: dict[tuple[str, str], _Transform2D] = {}
        self._nav2_client = nav2_client or Nav2PathClient(ros_client) if ros_client is not None else nav2_client
        self._nav2_navigate_client = (
            nav2_navigate_client or Nav2NavigateClient(ros_client)
            if ros_client is not None else nav2_navigate_client
        )
        if ros_client is not None:
            self._subscribe_all()

    def _subscribe_all(self) -> None:
        topics = [
            ("map", "/map", "nav_msgs/OccupancyGrid", self._on_map),
            ("map_metadata", "/map_metadata", "nav_msgs/MapMetaData", self._on_map_metadata),
            ("scan", "/scan", "sensor_msgs/LaserScan", self._on_scan),
            ("tf", "/tf", "tf2_msgs/TFMessage", self._on_tf),
            ("tf_static", "/tf_static", "tf2_msgs/TFMessage", self._on_tf_static),
            ("local_costmap", "/local_costmap/costmap", "nav_msgs/OccupancyGrid", self._on_local_costmap),
            ("global_costmap", "/global_costmap/costmap", "nav_msgs/OccupancyGrid", self._on_global_costmap),
        ]
        if roslibpy is None:
            return
        for _name, topic_name, msg_type, callback in topics:
            topic = roslibpy.Topic(self._ros_client, topic_name, msg_type)
            topic.subscribe(callback)
            self._subs.append(topic)

    def close(self) -> None:
        for sub in self._subs:
            try:
                sub.unsubscribe()
            except Exception:
                pass
        self._subs = []

    # --- test-friendly ingestion helpers ---

    def ingest_map(self, msg: dict) -> None:
        self._on_map(msg)

    def ingest_map_metadata(self, msg: dict) -> None:
        self._on_map_metadata(msg)

    def ingest_scan(self, msg: dict) -> None:
        self._on_scan(msg)

    def ingest_tf(self, msg: dict) -> None:
        self._on_tf(msg)

    def ingest_tf_static(self, msg: dict) -> None:
        self._on_tf_static(msg)

    def ingest_local_costmap(self, msg: dict) -> None:
        self._on_local_costmap(msg)

    def ingest_global_costmap(self, msg: dict) -> None:
        self._on_global_costmap(msg)

    # --- topic callbacks ---

    def _on_map(self, msg: dict) -> None:
        self._set_topic("map", msg)

    def _on_map_metadata(self, msg: dict) -> None:
        self._set_topic("map_metadata", msg)

    def _on_scan(self, msg: dict) -> None:
        self._set_topic("scan", msg)

    def _on_local_costmap(self, msg: dict) -> None:
        self._set_topic("local_costmap", msg)

    def _on_global_costmap(self, msg: dict) -> None:
        self._set_topic("global_costmap", msg)

    def _on_tf(self, msg: dict) -> None:
        self._ingest_tf_message(msg, is_static=False)

    def _on_tf_static(self, msg: dict) -> None:
        self._ingest_tf_message(msg, is_static=True)

    def _set_topic(self, key: str, msg: dict) -> None:
        with self._lock:
            self._topics[key] = _CacheEntry(data=msg, receipt_ts=time.time())

    def _ingest_tf_message(self, msg: dict, *, is_static: bool) -> None:
        now = time.time()
        transforms = msg.get("transforms") or []
        with self._lock:
            for t in transforms:
                header = t.get("header") or {}
                parent = str(header.get("frame_id", "")).strip("/")
                child = str(t.get("child_frame_id", "")).strip("/")
                if not parent or not child:
                    continue
                translation = t.get("transform", {}).get("translation", {})
                rotation = t.get("transform", {}).get("rotation", {})
                stamp = header.get("stamp")
                ts = None
                if isinstance(stamp, dict):
                    ts = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) / 1e9
                self._transforms[(parent, child)] = _Transform2D(
                    x=float(translation.get("x", 0.0)),
                    y=float(translation.get("y", 0.0)),
                    yaw=_yaw_from_quat(rotation),
                    stamp=ts,
                    receipt_ts=now,
                    is_static=is_static,
                )

    # --- cache helpers ---

    def freshness(self) -> dict:
        with self._lock:
            data = {k: v.age_s() for k, v in self._topics.items()}
        tf_age = None
        try:
            tf_age = self._lookup_transform(self.map_frame, self.base_frame).age_s()
        except Exception:
            pass
        return {
            "tf_age_s": tf_age,
            "scan_age_s": data["scan"],
            "map_age_s": data["map"],
            "map_metadata_age_s": data["map_metadata"],
            "local_costmap_age_s": data["local_costmap"],
            "global_costmap_age_s": data["global_costmap"],
        }

    def get_map_signature(self) -> str:
        grid = self._get_topic_data("map")
        metadata = self._effective_map_metadata()
        if metadata is None:
            raise RuntimeError("map metadata unavailable")
        info = metadata.get("info", metadata)
        digest = hashlib.sha1()
        digest.update(str(info.get("width", 0)).encode())
        digest.update(str(info.get("height", 0)).encode())
        digest.update(str(info.get("resolution", 0.0)).encode())
        origin = info.get("origin", {}).get("position", {})
        digest.update(str(origin.get("x", 0.0)).encode())
        digest.update(str(origin.get("y", 0.0)).encode())
        if grid is not None:
            cells = list(grid.get("data") or [])
            if cells:
                step = max(1, len(cells) // 128)
                sampled = cells[::step][:128]
                digest.update(",".join(str(int(v)) for v in sampled).encode())
        return digest.hexdigest()[:16]

    def get_robot_pose_in_map(self) -> dict:
        transform = self._lookup_transform(self.map_frame, self.base_frame)
        if not transform.is_static and transform.age_s() > TF_STALE_S:
            raise RuntimeError(f"tf is stale ({transform.age_s():.2f}s > {TF_STALE_S:.2f}s)")
        return {
            "pose": {"x_m": transform.x, "y_m": transform.y, "yaw_rad": transform.yaw},
            "frame_id": self.map_frame,
            "base_frame": self.base_frame,
            "stamp": transform.stamp,
            "age_s": transform.age_s(),
            "source": "tf",
        }

    def get_map_summary(self) -> dict:
        warnings: list[str] = []
        metadata = self._effective_map_metadata()
        if metadata is None:
            raise RuntimeError("map metadata unavailable")
        grid = self._get_topic_data("map")
        info = metadata.get("info", metadata)
        counts = None
        coverage_ratio = None
        if grid is None or grid.get("data") is None:
            warnings.append("full map grid unavailable; using metadata only")
        else:
            cells = list(grid.get("data") or [])
            free = sum(1 for v in cells if int(v) == 0)
            occupied = sum(1 for v in cells if int(v) > 0)
            unknown = sum(1 for v in cells if int(v) < 0)
            counts = {"free": free, "occupied": occupied, "unknown": unknown}
            known = free + occupied
            coverage_ratio = (known / len(cells)) if cells else 0.0
        pose_available = True
        try:
            self.get_robot_pose_in_map()
        except Exception as exc:
            pose_available = False
            warnings.append(f"robot pose unavailable: {exc}")
        ages = self.freshness()
        if ages["map_age_s"] is not None and ages["map_age_s"] > self.map_warn_s:
            warnings.append(f"map is stale ({ages['map_age_s']:.1f}s old)")
        return {
            "warnings": warnings,
            "data": {
                "map": {
                    "width": int(info.get("width", 0)),
                    "height": int(info.get("height", 0)),
                    "resolution_m": float(info.get("resolution", 0.0)),
                    "origin": self._origin_dict(info.get("origin", {})),
                },
                "cell_counts": counts,
                "coverage_ratio": coverage_ratio,
                "freshness": ages,
                "robot_pose_available": pose_available,
                "map_signature": self.get_map_signature(),
            },
        }

    def get_local_map_context(self, radius_m: float = 3.0) -> dict:
        radius_m = max(0.5, min(float(radius_m), 8.0))
        pose = self.get_robot_pose_in_map()
        scan = self._require_topic_fresh("scan", SCAN_STALE_S)
        sectors = {
            "front": (-math.pi / 6, math.pi / 6),
            "front_left": (math.pi / 6, math.pi / 2),
            "left": (math.pi / 2, 5 * math.pi / 6),
            "rear": (5 * math.pi / 6, math.pi),
            "right": (-5 * math.pi / 6, -math.pi / 2),
            "front_right": (-math.pi / 2, -math.pi / 6),
        }
        hits = {name: [] for name in sectors}
        for point in self._scan_points_base(scan):
            if point["range_m"] > radius_m:
                continue
            sector = self._sector_for_angle(point["angle_rad"], sectors)
            hits[sector].append(point["range_m"])

        local_costmap = self._get_topic_data("local_costmap")
        local_age = self._topic_age("local_costmap")
        if local_costmap is not None and local_age is not None and local_age <= LOCAL_COSTMAP_STALE_S:
            local_pose = self._lookup_transform(self.odom_frame, self.base_frame)
            for name, mid_angle in {
                "front": 0.0,
                "front_left": math.pi / 3,
                "left": 2.0 * math.pi / 3,
                "rear": math.pi,
                "right": -2.0 * math.pi / 3,
                "front_right": -math.pi / 3,
            }.items():
                obstacle = self._ray_costmap_distance(local_costmap, local_pose, mid_angle, radius_m)
                if obstacle is not None:
                    hits[name].append(obstacle)

        sector_info = {}
        blocked = []
        free_dirs = []
        unknown = []
        nearest = None
        for name, distances in hits.items():
            if distances:
                d = min(distances)
                nearest = d if nearest is None else min(nearest, d)
                status = "occupied" if d <= min(1.0, radius_m * 0.5) else "free"
            else:
                d = None
                status = "unknown"
            sector_info[name] = {"status": status, "nearest_obstacle_m": d}
            if status == "occupied":
                blocked.append(name)
            elif status == "free":
                free_dirs.append(name)
            else:
                unknown.append(name)

        notes = []
        if local_costmap is None:
            notes.append("local costmap unavailable; using scan-only reasoning")
        return {
            "warnings": [],
            "data": {
                "radius_m": radius_m,
                "pose": pose["pose"],
                "nearest_obstacle_m": nearest,
                "sectors": sector_info,
                "blocked_directions": blocked,
                "free_directions": free_dirs,
                "unknown_directions": unknown,
                "notes": notes,
                "freshness": self.freshness(),
            },
        }

    def get_local_occupancy_grid(self, size_m: float = 5.0, cell_size_m: float = 0.25) -> dict:
        size_m = max(1.0, min(float(size_m), MAX_GRID_SIZE_M))
        cell_size_m = max(MIN_CELL_SIZE_M, min(float(cell_size_m), MAX_CELL_SIZE_M))
        pose = self.get_robot_pose_in_map()["pose"]
        self._require_topic_fresh("scan", SCAN_STALE_S)
        width = int(math.ceil(size_m / cell_size_m))
        if width % 2 == 0:
            width += 1
        height = width
        center = width // 2
        grid = [[UNKNOWN for _ in range(width)] for _ in range(height)]

        half = size_m / 2.0
        static_map = self._get_topic_data("map")
        local_costmap = self._get_topic_data("local_costmap")
        for row in range(height):
            for col in range(width):
                dx = (col - center) * cell_size_m
                dy = (center - row) * cell_size_m
                wx = pose["x_m"] + dx
                wy = pose["y_m"] + dy
                value = UNKNOWN
                if static_map is not None:
                    value = self._sample_occupancy(static_map, wx, wy, source_frame=self.map_frame)
                if local_costmap is not None:
                    local_value = self._sample_occupancy(local_costmap, wx, wy, source_frame=self.odom_frame, map_frame=self.map_frame)
                    if local_value != UNKNOWN:
                        value = local_value
                grid[row][col] = value

        scan = self._get_topic_data("scan")
        if scan is not None:
            for point in self._scan_points_base(scan):
                if point["range_m"] > half * math.sqrt(2.0):
                    continue
                local_x = point["range_m"] * math.cos(point["angle_rad"])
                local_y = point["range_m"] * math.sin(point["angle_rad"])
                map_x, map_y = self._base_to_map(local_x, local_y)
                hit_col = center + int(round((map_x - pose["x_m"]) / cell_size_m))
                hit_row = center - int(round((map_y - pose["y_m"]) / cell_size_m))
                for r, c in self._bresenham(center, center, hit_row, hit_col):
                    if 0 <= r < height and 0 <= c < width and grid[r][c] == UNKNOWN:
                        grid[r][c] = FREE
                if 0 <= hit_row < height and 0 <= hit_col < width:
                    grid[hit_row][hit_col] = OCCUPIED

        grid[center][center] = FREE
        return {
            "warnings": [],
            "data": {
                "center_pose": pose,
                "size_m": size_m,
                "cell_size_m": cell_size_m,
                "width": width,
                "height": height,
                "robot_cell": {"row": center, "col": center},
                "grid": grid,
                "legend": {"free": FREE, "occupied": OCCUPIED, "unknown": UNKNOWN},
                "freshness": self.freshness(),
            },
        }

    def compare_map_vs_live_scan(self) -> dict:
        self.get_robot_pose_in_map()
        scan = self._require_topic_fresh("scan", SCAN_STALE_S)
        static_map = self._get_topic_data("map")
        if static_map is None:
            raise RuntimeError("static map unavailable")
        local_costmap = self._get_topic_data("local_costmap")
        global_costmap = self._get_topic_data("global_costmap")
        total = 0
        agree = 0
        mismatch = 0
        unknown = 0
        mismatch_sectors: dict[str, int] = {}
        close_mismatches = 0
        for point in self._scan_points_base(scan):
            if point["range_m"] < float(scan.get("range_min", 0.0)) or point["range_m"] > min(float(scan.get("range_max", 0.0)), 8.0):
                continue
            total += 1
            map_x, map_y = self._base_to_map(
                point["range_m"] * math.cos(point["angle_rad"]),
                point["range_m"] * math.sin(point["angle_rad"]),
            )
            vals = []
            vals.append(self._sample_occupancy(static_map, map_x, map_y, source_frame=self.map_frame))
            if global_costmap is not None:
                vals.append(self._sample_occupancy(global_costmap, map_x, map_y, source_frame=self.map_frame))
            if local_costmap is not None:
                vals.append(self._sample_occupancy(local_costmap, map_x, map_y, source_frame=self.odom_frame, map_frame=self.map_frame))
            known_vals = [v for v in vals if v != UNKNOWN]
            if not known_vals:
                unknown += 1
                continue
            if any(v >= OCCUPIED_THRESHOLD for v in known_vals):
                agree += 1
            else:
                mismatch += 1
                sector = self._sector_for_angle(point["angle_rad"], {
                    "front": (-math.pi / 4, math.pi / 4),
                    "left": (math.pi / 4, 3 * math.pi / 4),
                    "rear": (3 * math.pi / 4, math.pi),
                    "right": (-3 * math.pi / 4, -math.pi / 4),
                })
                mismatch_sectors[sector] = mismatch_sectors.get(sector, 0) + 1
                if point["range_m"] <= 2.5:
                    close_mismatches += 1
        denom = max(total, 1)
        agreement_ratio = agree / denom
        mismatch_ratio = mismatch / denom
        unknown_ratio = unknown / denom
        return {
            "warnings": [],
            "data": {
                "agreement_ratio": agreement_ratio,
                "mismatch_ratio": mismatch_ratio,
                "unknown_ratio": unknown_ratio,
                "sectors_with_mismatch": sorted(mismatch_sectors, key=mismatch_sectors.get, reverse=True),
                "possible_dynamic_obstacle": close_mismatches >= 4 and mismatch_ratio >= 0.15,
                "possible_localization_issue": mismatch_ratio >= 0.45 and agreement_ratio <= 0.35,
                "freshness": self.freshness(),
            },
        }

    def compute_path_to_waypoint(self, waypoint_pose: dict, timeout_s: float = 3.0) -> dict:
        start = self.get_robot_pose_in_map()["pose"]
        warnings: list[str] = []
        result: Optional[dict] = None
        method = "heuristic"
        nav2_error = None
        if self._nav2_client is not None:
            try:
                result = self._nav2_client.compute_path(start, waypoint_pose, timeout_s=timeout_s)
                method = "nav2_plan"
            except Exception as exc:
                nav2_error = str(exc)
                warnings.append(f"nav2 planning unavailable: {exc}")
        if result is not None:
            path_points = self._extract_path_points(result)
            if path_points:
                return {
                    "method": method,
                    "path_points": path_points,
                    "path_length_m": self._path_length(path_points),
                    "warnings": warnings,
                }
        heuristic = self._heuristic_path_check(start, waypoint_pose)
        heuristic["warnings"] = warnings + heuristic.get("warnings", [])
        if nav2_error is not None:
            heuristic["nav2_error"] = nav2_error
        return heuristic

    def navigate_to_pose(self, goal_pose: dict, timeout_s: float = 3.0) -> dict:
        if self._nav2_navigate_client is None:
            raise RuntimeError("nav2 navigate_to_pose unavailable")
        return self._nav2_navigate_client.navigate_to_pose(goal_pose, timeout_s=timeout_s)

    def cancel_navigation(self, reason: str = "stop") -> dict:
        if self._nav2_navigate_client is None:
            raise RuntimeError("nav2 navigation unavailable")
        return self._nav2_navigate_client.cancel(reason=reason)

    def get_navigation_status(self) -> dict:
        if self._nav2_navigate_client is None:
            raise RuntimeError("nav2 navigation unavailable")
        return self._nav2_navigate_client.get_status()

    # --- planning helpers ---

    def _heuristic_path_check(self, start: dict, goal: dict) -> dict:
        local_costmap = self._get_topic_data("local_costmap")
        global_costmap = self._get_topic_data("global_costmap")
        distance = _distance(start["x_m"], start["y_m"], goal["x_m"], goal["y_m"])
        blocked_reason = None
        confidence = "low"
        if local_costmap is not None:
            local_pose = self._lookup_transform(self.odom_frame, self.base_frame)
            angle_local = math.atan2(goal["y_m"] - start["y_m"], goal["x_m"] - start["x_m"]) - self._lookup_transform(self.map_frame, self.base_frame).yaw
            obstacle = self._ray_costmap_distance(local_costmap, local_pose, angle_local, min(distance, 4.0))
            if obstacle is not None and obstacle < min(distance, 4.0):
                blocked_reason = f"local costmap shows obstacle about {obstacle:.1f}m ahead"
        if blocked_reason is None and global_costmap is not None:
            steps = max(3, int(distance / max(0.25, float(global_costmap.get("info", {}).get("resolution", 0.2)))))
            for i in range(1, steps + 1):
                frac = i / steps
                x = start["x_m"] + frac * (goal["x_m"] - start["x_m"])
                y = start["y_m"] + frac * (goal["y_m"] - start["y_m"])
                value = self._sample_occupancy(global_costmap, x, y, source_frame=self.map_frame)
                if value >= OCCUPIED_THRESHOLD:
                    blocked_reason = "global costmap indicates the route crosses occupied space"
                    break
            if blocked_reason is None:
                confidence = "medium"
        path_points = [start, goal]
        return {
            "method": "costmap_heuristic",
            "path_points": path_points,
            "path_length_m": distance,
            "reachable": blocked_reason is None,
            "blocking_reason": blocked_reason,
            "confidence": confidence,
            "warnings": [],
        }

    def _extract_path_points(self, result: dict) -> list[dict]:
        path = result.get("path") or result.get("result", {}).get("path") or result.get("result", {}).get("result", {}).get("path")
        if not isinstance(path, dict):
            return []
        poses = path.get("poses") or []
        points = []
        for pose in poses:
            pp = pose.get("pose", {}).get("position", {})
            qq = pose.get("pose", {}).get("orientation", {})
            points.append({
                "x_m": float(pp.get("x", 0.0)),
                "y_m": float(pp.get("y", 0.0)),
                "yaw_rad": _yaw_from_quat(qq),
            })
        return points

    def _path_length(self, points: list[dict]) -> float:
        if len(points) < 2:
            return 0.0
        total = 0.0
        for prev, cur in zip(points, points[1:]):
            total += _distance(prev["x_m"], prev["y_m"], cur["x_m"], cur["y_m"])
        return total

    # --- transform helpers ---

    def _lookup_transform(self, parent: str, child: str) -> _Transform2D:
        parent = parent.strip("/")
        child = child.strip("/")
        with self._lock:
            direct = self._transforms.get((parent, child))
            if direct is not None:
                return direct
            inverse = self._transforms.get((child, parent))
            if inverse is not None:
                return _invert(inverse)
            if parent == self.map_frame and child == self.base_frame:
                map_to_odom = self._transforms.get((self.map_frame, self.odom_frame))
                odom_to_base = self._transforms.get((self.odom_frame, self.base_frame))
                if map_to_odom is not None and odom_to_base is not None:
                    return _compose(map_to_odom, odom_to_base)
            if parent == self.odom_frame and child == self.base_frame:
                odom_to_base = self._transforms.get((self.odom_frame, self.base_frame))
                if odom_to_base is not None:
                    return odom_to_base
            if parent == self.map_frame and child == self.odom_frame:
                map_to_odom = self._transforms.get((self.map_frame, self.odom_frame))
                if map_to_odom is not None:
                    return map_to_odom
        raise RuntimeError(f"missing transform {parent} -> {child}")

    def _base_to_map(self, local_x: float, local_y: float) -> tuple[float, float]:
        pose = self._lookup_transform(self.map_frame, self.base_frame)
        c = math.cos(pose.yaw)
        s = math.sin(pose.yaw)
        return (
            pose.x + c * local_x - s * local_y,
            pose.y + s * local_x + c * local_y,
        )

    def _map_to_odom(self, map_x: float, map_y: float) -> tuple[float, float]:
        transform = self._lookup_transform(self.map_frame, self.odom_frame)
        inv = _invert(transform)
        c = math.cos(inv.yaw)
        s = math.sin(inv.yaw)
        return (
            inv.x + c * map_x - s * map_y,
            inv.y + s * map_x + c * map_y,
        )

    # --- occupancy helpers ---

    def _effective_map_metadata(self) -> Optional[dict]:
        grid = self._get_topic_data("map")
        if grid is not None and grid.get("info") is not None:
            return grid
        return self._get_topic_data("map_metadata")

    def _origin_dict(self, origin: dict) -> dict:
        position = origin.get("position", origin)
        orientation = origin.get("orientation", {})
        return {
            "x_m": float(position.get("x", 0.0)),
            "y_m": float(position.get("y", 0.0)),
            "yaw_rad": _yaw_from_quat(orientation) if orientation else 0.0,
        }

    def _world_to_grid(self, grid: dict, x: float, y: float) -> Optional[tuple[int, int]]:
        info = grid.get("info", {})
        width = int(info.get("width", 0))
        height = int(info.get("height", 0))
        resolution = float(info.get("resolution", 0.0))
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return None
        origin = self._origin_dict(info.get("origin", {}))
        dx = x - origin["x_m"]
        dy = y - origin["y_m"]
        c = math.cos(-origin["yaw_rad"])
        s = math.sin(-origin["yaw_rad"])
        local_x = c * dx - s * dy
        local_y = s * dx + c * dy
        col = int(math.floor(local_x / resolution))
        row = int(math.floor(local_y / resolution))
        if row < 0 or col < 0 or row >= height or col >= width:
            return None
        return row, col

    def _sample_occupancy(
        self,
        grid: dict,
        world_x: float,
        world_y: float,
        *,
        source_frame: str,
        map_frame: Optional[str] = None,
    ) -> int:
        sx, sy = world_x, world_y
        if source_frame == self.odom_frame and map_frame == self.map_frame:
            sx, sy = self._map_to_odom(world_x, world_y)
        idx = self._world_to_grid(grid, sx, sy)
        if idx is None:
            return UNKNOWN
        row, col = idx
        info = grid.get("info", {})
        width = int(info.get("width", 0))
        data = grid.get("data") or []
        flat_idx = row * width + col
        if flat_idx < 0 or flat_idx >= len(data):
            return UNKNOWN
        value = int(data[flat_idx])
        if value < 0:
            return UNKNOWN
        if value >= OCCUPIED_THRESHOLD:
            return OCCUPIED
        return FREE

    def _ray_costmap_distance(self, grid: dict, pose: _Transform2D, angle_rad: float, max_range_m: float) -> Optional[float]:
        step = max(0.1, float(grid.get("info", {}).get("resolution", 0.1)))
        distance = step
        while distance <= max_range_m:
            x = pose.x + distance * math.cos(pose.yaw + angle_rad)
            y = pose.y + distance * math.sin(pose.yaw + angle_rad)
            value = self._sample_occupancy(grid, x, y, source_frame=self.odom_frame)
            if value == OCCUPIED:
                return distance
            distance += step
        return None

    # --- scan helpers ---

    def _scan_points_base(self, scan: dict) -> list[dict]:
        ranges = list(scan.get("ranges") or [])
        angle_min = float(scan.get("angle_min", 0.0))
        angle_increment = float(scan.get("angle_increment", 0.0))
        range_min = float(scan.get("range_min", 0.0))
        range_max = float(scan.get("range_max", float("inf")))
        points = []
        for i, value in enumerate(ranges):
            try:
                r = float(value)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(r) or r < range_min or r > range_max:
                continue
            points.append({
                "angle_rad": angle_min + i * angle_increment,
                "range_m": r,
            })
        return points

    def _sector_for_angle(self, angle_rad: float, sectors: dict[str, tuple[float, float]]) -> str:
        angle = _norm_angle(angle_rad)
        for name, (lo, hi) in sectors.items():
            if lo <= hi:
                if lo <= angle < hi:
                    return name
            elif angle >= lo or angle < hi:
                return name
        return next(iter(sectors))

    def _bresenham(self, row0: int, col0: int, row1: int, col1: int) -> list[tuple[int, int]]:
        points: list[tuple[int, int]] = []
        drow = abs(row1 - row0)
        dcol = abs(col1 - col0)
        srow = 1 if row0 < row1 else -1
        scol = 1 if col0 < col1 else -1
        err = dcol - drow
        row, col = row0, col0
        while True:
            points.append((row, col))
            if row == row1 and col == col1:
                break
            e2 = 2 * err
            if e2 > -drow:
                err -= drow
                col += scol
            if e2 < dcol:
                err += dcol
                row += srow
        return points

    # --- generic topic access ---

    def _get_topic_data(self, key: str) -> Optional[dict]:
        with self._lock:
            entry = self._topics[key]
            if entry.data is None:
                return None
            return dict(entry.data)

    def _topic_age(self, key: str) -> Optional[float]:
        with self._lock:
            return self._topics[key].age_s()

    def _require_topic_fresh(self, key: str, max_age_s: float) -> dict:
        with self._lock:
            entry = self._topics[key]
            data = entry.data
            age = entry.age_s()
        if data is None:
            raise RuntimeError(f"{key.replace('_', ' ')} unavailable")
        if age is None or age > max_age_s:
            raise RuntimeError(f"{key.replace('_', ' ')} is stale ({age if age is not None else 'unknown'}s)")
        return dict(data)
