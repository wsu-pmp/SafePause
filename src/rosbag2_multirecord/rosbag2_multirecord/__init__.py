RECORDER_NODE_NAME: str = "rosbag2_recorder"


def recorder_node_name(namespace: str | None = None) -> str:
    return f"{RECORDER_NODE_NAME}_{namespace}" if namespace else RECORDER_NODE_NAME
