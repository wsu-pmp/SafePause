import json
import os
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict


class EventLogger:
    def __init__(self, node, name: str, header: Dict[str, Any]):
        self.name = name
        self.node = node
        self.lock = threading.Lock()

        # base log path from node param or env var
        if self.node.has_parameter("event_log_path"):
            base_path = self.node.get_parameter("event_log_path").value
        else:
            base_path = os.environ.get("EVENT_LOG_PATH")
            if not base_path:
                raise ValueError(
                    "node's 'event_log_path' parameter or EVENT_LOG_PATH environment variable must be set"
                )

        # subdir structure: base_path/YYYY-MM-DD/
        date_str = datetime.now().strftime("%Y-%m-%d")
        log_dir = Path(base_path) / date_str
        log_dir.mkdir(parents=True, exist_ok=True)

        # log file naming: name_YYYYMMDD_HHMMSS.jsonl
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = log_dir / f"{name}_{timestamp}.jsonl"

        # write header as first line
        header_entry = {"header": header}

        with self.lock:
            with open(self.log_file, "w") as f:
                f.write(json.dumps(header_entry) + "\n")

    def log(self, event: Dict[str, Any]):
        timestamp = self.node.get_clock().now().to_msg()
        log_entry = {
            "timestamp": {"sec": timestamp.sec, "nanosec": timestamp.nanosec},
            "event": event,
        }

        # write with lock
        with self.lock:
            with open(self.log_file, "a") as f:
                f.write(json.dumps(log_entry) + "\n")
