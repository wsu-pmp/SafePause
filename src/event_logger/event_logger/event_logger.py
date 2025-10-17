import json
import os
import socket
import threading
from datetime import datetime
from pathlib import Path
from time import time
from typing import Any

SCHEMA = 1

PARAM_NAME = "event_log_path"
ENV_NAME = "EVENT_LOG_PATH"
ENV_ROS_HOME = "ROS_HOME"
DEFAULT_SUBDIR = "event_logs"


class EventLogger:
    """
    Append-only jsonl event log, one file per logger per process.

    The output directory comes from the node's `event_log_path` parameter, or
    the EVENT_LOG_PATH environment variable if that parameter is unset.
    """

    # same name twice in one process would mean two handles appending to one
    # file with independent seq counters
    _open_names: set[str] = set()
    _open_names_lock = threading.Lock()

    def __init__(self, node, name: str, meta: dict[str, Any] | None = None):
        self._node = node
        self._name = name
        self._lock = threading.Lock()
        self._seq = 0
        self._closed = False
        self._write_errors = 0
        self._registered = False

        self._node_name = node.get_fully_qualified_name()
        self._pid = os.getpid()
        self._host = socket.gethostname()

        base_path = self._resolve_base_path(node)

        with EventLogger._open_names_lock:
            if name in EventLogger._open_names:
                raise ValueError(
                    f"EventLogger '{name}' already open in this process (pid "
                    f"{self._pid})"
                )
            EventLogger._open_names.add(name)
            self._registered = True

        started = datetime.now()
        log_dir = Path(base_path) / started.strftime("%Y-%m-%d")
        log_dir.mkdir(parents=True, exist_ok=True)

        # pid discriminates replicas launched from one launch file, which would
        # otherwise collide whenever two start within the same second
        stamp = started.strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"{name}_{stamp}_{self._pid}.jsonl"

        try:
            self._file = open(self.path, "x")
        except FileExistsError:
            self._release_name()
            raise FileExistsError(f"event log already exists: {self.path}") from None
        except OSError:
            self._release_name()
            raise

        self._emit("open", meta or {})

    @staticmethod
    def _resolve_base_path(node) -> str:
        # declared here so the parameter works without every consumer having to
        # declare it first
        if not node.has_parameter(PARAM_NAME):
            node.declare_parameter(PARAM_NAME, "")

        base_path = node.get_parameter(PARAM_NAME).value or os.environ.get(ENV_NAME)
        if not base_path:
            # defaults to ROS_HOME
            ros_home = os.environ.get(ENV_ROS_HOME) or Path.home() / ".ros"
            base_path = str(Path(ros_home) / DEFAULT_SUBDIR)
        return base_path

    def _release_name(self) -> None:
        if self._registered:
            with EventLogger._open_names_lock:
                EventLogger._open_names.discard(self._name)
            self._registered = False

    @staticmethod
    def _sorted(value: Any) -> Any:
        # cpp implementation stores caller payloads in a sorted map
        # give the same ordering here
        if isinstance(value, dict):
            return {k: EventLogger._sorted(value[k]) for k in sorted(value)}
        if isinstance(value, (list, tuple)):
            return [EventLogger._sorted(v) for v in value]
        return value

    def _emit(self, record_type: str, data: dict[str, Any]) -> None:
        stamp = self._node.get_clock().now().to_msg()
        record = {
            "schema": SCHEMA,
            "type": record_type,
            "logger": self._name,
            "node": self._node_name,
            "pid": self._pid,
            "host": self._host,
            "seq": self._seq,
            # include both ROS timestamp and wall time
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
            "wall": time(),
            "data": self._sorted(data),
        }
        self._seq += 1

        # compact separators so lines are byte-identical to the cpp output
        self._file.write(json.dumps(record, separators=(",", ":")) + "\n")

        self._file.flush()

    def log(self, event: dict[str, Any]) -> None:
        with self._lock:
            if self._closed:
                return
            try:
                self._emit("event", event)
            except (OSError, ValueError, TypeError) as e:
                self._write_errors += 1
                self._node.get_logger().error(
                    f"event log write failed ({self._write_errors} so far): {e}",
                    throttle_duration_sec=5.0,
                )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._emit("close", {"write_errors": self._write_errors})
            except (OSError, ValueError, TypeError):
                pass
            try:
                self._file.close()
            except OSError:
                pass
            self._release_name()

    def __enter__(self) -> "EventLogger":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __del__(self):
        # best effort only; callers should close() or use the context manager
        try:
            self.close()
        except Exception:
            pass
