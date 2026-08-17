# SafePause

This is the SafePause ROS 2 Workspace.


## Setup
### Prerequisites
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Doosan Robotics ROS 2 Dependencies](https://github.com/DoosanRobotics/doosan-robot2?tab=readme-ov-file#installation)

### Instructions
#### 1. Clone with Submodules
```bash
git clone --recurse-submodules https://github.com/wsu-pmp/SafePause.git
```

#### 2. Build
```bash
cd ./SafePause
source /opt/ros/humble/setup.bash
colcon build
```

#### 3. *(Optional)* Install Doosan Robot Emulator
```bash
cd ./src/doosan-robot2
chmod +x ./install_emulator.sh
sudo ./install_emulator.sh
```


## Use

Source the workspace:

```bash
cd ./SafePause
source install/setup.bash
```

Launch file arguments can always be listed with `--show-args`, e.g.
`ros2 launch bringup bringup.launch.py --show-args`.

Packages to launch the robot, gripper or camera themselves belong to the
`pmp-a0509` submodule.

The `pmp-a0509` MoveIt config can be launched on its own with:

```bash
ros2 launch pmp_a0509_moveit_config start.launch.py mode:=virtual
```

or, if perception etc. is required, the complete stack can be launched with:

```bash
ros2 launch pmp_a0509_bringup bringup.launch.py dsr_mode:=virtual
```


### `perception_pkg`
`PerceptionNode` synchronizes messages from multiple topics into logical
bundles, based on
[`message_filters::sync_policies::ApproximateTime`](https://docs.ros.org/en/humble/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html),
which requires types to be known at compile-time. `PerceptionNode` works with
arbitrary topic types specified via a YAML config by storing type-erased
messages that can later be downcast for processing. Time synchronisation will
use message
[`header`](https://docs.ros.org/en/humble/p/std_msgs/msg/Header.html) if one
exists, defaulting to the message arrival time otherwise.

The node waits for all topics to be discovered with a matching type and at least
one publisher before subscribing to any. Discovery is retried for 10 seconds, at
which point, if all topics have not been discovered, the missing topics are
logged and the node shuts down.

Bundles are matched by taking the earliest queued message across all topics
as an anchor and pulling the closest match within `slop` from every other
topic's queue. A topic with no match within `slop` drops the anchor and retries
the process with the next message. Both the per-topic input queues and the
bundle processing queue are bounded (`queue_size`, `processing_queue_size`),
dropping their oldest messages when queue sizes are exceeded. Bundles can be
processed internally (currently placeholder logic), and are published as
lightweight `safepause_msgs/MessageBundleIndex` bundle indices to
allow external consumers to identify messages belonging to bundles.

A topic with `requires_tf: true` whose transform can't be resolved is still
bundled. Degradation is indicated via `has_transform = false` on the bundle's
index entry, and transitions in and out of the degraded state are logged
(`bundle_degraded` / `bundle_recovered`).

The optional `--namespace` argument allows multiple nodes to run simultaneously,
allowing for topics to be combined arbitrarily across more than one bundle (e.g.
separating low and high rate topics such that slow messages don't bottleneck
faster ones). The namespace is applied as a suffix on the node's own name rather
than a ROS namespace remap, as such, it doesn't affect the topics
`perception_pkg` subscribes to, only the name it runs under and its
`~/bundle_index` publisher.

**Arguments** (passed via `--ros-args`)

| Argument | Default | Meaning |
|---|---|---|
| `--namespace` | *(none)* | Appended to the node name; also affects the `~/bundle_index` topic. |

```bash
ros2 run perception_pkg perception_node --ros-args --namespace foo
```

**Parameters** (passed via `--ros-args -p`)

| Name | Default | Meaning |
|---|---|---|
| `config_file` | — | Path to the YAML configuration describing input topics, message types, and TF requirements. Required. |
| `queue_size` | `10` | Maximum number of messages retained per input topic queue. |
| `slop` | `0.1` | Maximum allowed time difference (in seconds) between messages in a bundle. |
| `processing_rate` | `10.0` | Maximum rate (Hz) at which bundles are processed. Bundles form as fast as the slowest input topic allows, so a value below that rate throttles processing and drops the excess. |
| `processing_queue_size` | `100` | Maximum number of bundles held in queue between forming and processing. |

```bash
ros2 run perception_pkg perception_node --ros-args -p config_file:=./src/perception_pkg/config/example.yaml
```

**Example config YAML**
```yaml
variables:
  - &TARGET_FRAME base_link
  - &MAX_TF_AGE 0.05 # seconds

topics:
  - name: /scan
    type: sensor_msgs/msg/LaserScan
    requires_tf: true
    target_frame: *TARGET_FRAME
    max_tf_age: *MAX_TF_AGE

  - name: /pose
    type: geometry_msgs/msg/PoseStamped
    requires_tf: true
    target_frame: *TARGET_FRAME
    max_tf_age: *MAX_TF_AGE

  # String has no header, so bundling falls back to arrival time
  - name: /chatter
    type: std_msgs/msg/String
    requires_tf: false
```

`topics` must be non-empty and free of duplicate names, and any topic with
`requires_tf: true` must also set `target_frame`.


### `rosbag2_multirecord`
Records a set of topic groups with one recorder node per group, then merges the
per-recorder MCAP files into a single bag on `~/stop`. Splitting the recording
across per-group processes is intended to prevent one saturated topic's
subscription queue from causing message loss on other topics.

Each recorder adapts its subscription QoS to the publishers already present on
each topic, taking the most permissive reliability and durability any publisher
offers. If no publisher is present at subscribe time it falls back to
`BEST_EFFORT`/`VOLATILE`. `qos_overrides` can be set in the config to pin a
topic's QoS explicitly rather than relying on discovery.

Bag timestamps are **message receipt time** taken from the node clock, matching
`ros2 bag record`. They are not header stamps, and they follow sim time when
`use_sim_time` is set.

A topic may appear in only one `topic_groups` entry, duplicate entries will
report an error. By default, if any of a recorder's topics cannot be discovered
within 5 seconds it will prevent recording. `require_all_topics` can be set to
`false` to downgrade this to an error log and record any remaining topics that
are discovered. This is distinct from the coordinator's own
`recorder_discovery_timeout` (default 10s), which is how long it waits for the
recorder nodes themselves (as opposed to the recorded topics) to come up and
register their services.

#### Start a recording session
```bash
ros2 launch rosbag2_multirecord start.launch.py config_file:=./src/rosbag2_multirecord/config/example.yaml
```

| Argument | Default | Meaning |
|---|---|---|
| `config_file` | *(package's `example.yaml`)* | Path to the topic-group/QoS config below. |
| `output_bag_dir` | *(config's `output_bag_dir`, or `./multirecord-out`)* | Where the final merged bag is written. Overrides the config if set. |

**Config YAML**
```yaml
topic_groups:
  - - /joint_states
    - /tf
    - /tf_static

  - - /zed/zed_node/obj_det/objects
    - /obj_det/small_objects

qos_overrides:
  /tf_static:
    reliability: reliable
    durability: transient_local
    history: keep_last
    depth: 1

output_bag_dir: ./multirecord-out
```

`topic_groups` is a list of topic-name lists, with one recorder per group.
`qos_overrides` keys are topic names; `reliability`/`durability`/`history`
accept the usual ROS 2 QoS policy names (case-insensitive), `depth` an integer.

#### Controlling a run
The coordinator (not the individual recorders) is the entrypoint:

```bash
ros2 service call /multirecord_coordinator/start std_srvs/srv/Trigger
ros2 service call /multirecord_coordinator/pause std_srvs/srv/Trigger
ros2 service call /multirecord_coordinator/stop  std_srvs/srv/Trigger
```

`start`/`pause` can be called repeatedly to pause and resume recording. `stop`
merges every successful recorder's MCAP files with `mcap merge`, reindexes the
result with `ros2 bag reindex`, deletes the recorders' temporary output
directories, and shuts the whole session down. A recorder that fails to stop is
excluded from the merge rather than failing the whole run.


### `event_logger`
A small append-only [jsonl](https://jsonlines.org/) event log, provided in both
Python and C++ with matching interfaces. It is used for things that are
difficult to capture via rosbag, such as discovery outcomes, resolved QoS, state
transitions, degradation and error counts.

```python
from event_logger import EventLogger

self.events = EventLogger(self, "my_logger", {"config": "..."})   # run metadata
self.events.log({"event": "started"})
self.events.close()                       # or use it as a context manager
```

```cpp
#include <event_logger/event_logger.hpp>

event_logger::EventLogger events(this, "my_logger", nlohmann::json{{"config", "..."}});
events.log(nlohmann::json{{"event", "started"}});
events.close();                           // also called by the destructor
```

Output goes to `<base>/<YYYY-MM-DD>/<logger>_<YYYYMMDD_HHMMSS>_<pid>.jsonl`,
where `<base>` is the node's `event_log_path` parameter, falling back to an
`EVENT_LOG_PATH` environment variable, followed by `$ROS_HOME/event_logs`, when
params/vars are unset.

**Record format**

```jsonc
{"schema":1,"type":"open","logger":"recorder","node":"/rosbag2_recorder_0",
 "pid":4711,"host":"pmp-ros2-base","seq":0,
 "stamp":{"sec":1755167412,"nanosec":183000000},"wall":1755167412.183,
 "data":{ /* caller-supplied run metadata */ }}
```

| Field | Meaning |
|---|---|
| `schema` | format version, to facilitate possible schema changes and migration |
| `type` | `open` (first record, carries the run metadata), `event`, or `close` (last record, carries the write error count) |
| `logger` | which logger within the node emitted this |
| `node` | fully qualified node name |
| `pid` | process id |
| `host` | hostname, for container vs host and multi-machine runs |
| `seq` | sequential counter per logger, so dropped or reordered records are detectable |
| `stamp` | ROS time. May be sim time, and may be 0 before the first `/clock` |
| `wall` | epoch seconds; the field that correlates with bag files and system logs |
| `data` | the caller's payload. Object keys within `data` are sorted |


### `interrupt_ctrl`
`interrupt_node` is currently a placeholder for where a safety/compliance check
would receive `perception_pkg`'s bundle indices and issue warn/pause/stop/etc.
commands back to the controller, if one were implemented.

```bash
ros2 run interrupt_ctrl interrupt_node
```


### `object_approach`
A one-shot demo node, built to exercise the other packages' capture path
(throughput, reliability) under real motion. It expects
`object-detection-node`'s filter running upstream, publishing filtered ZED
detections to `objects_topic` (default `/obj_det/small_objects`).

It accumulates `min_accumulation` detection messages, picks a random object
within `max_range` of `base_frame`, and approaches it. The approach goal is
constructed by transforming the object's 3D bounding box into `base_frame`, and
selecting the vertical face whose normal points most directly back at the base.
A pose, `approach_distance` out along the vertical face's normal, is set as a
position constraint on `end_effector_link` and sent as a MoveIt goal. The node
shuts down once the goal resolves, regardless of its success/failure.

> [!WARNING]
> This package **executes its goal immediately by default**. Use caution when
> running it against real hardware.

```bash
ros2 launch object_approach start.launch.py
```

| Parameter | Default | Meaning |
|---|---|---|
| `max_range` | `1.05` | Max distance (m) from `base_frame` for an object to be considered. |
| `approach_distance` | `0.05` | Standoff (m) from the chosen face along its normal. |
| `base_frame` | `base_link` | Frame objects and the approach pose are computed in. |
| `planning_group` | `manipulator` | MoveIt planning group for the goal. |
| `end_effector_link` | `link_6` | Link the position/orientation constraints are applied to. |
| `objects_topic` | `/obj_det/small_objects` | `zed_msgs/ObjectsStamped` topic to accumulate from. |
| `move_action` | `/move_action` | MoveIt `MoveGroup` action server. |
| `min_accumulation` | `3` | Messages accumulated before an object is selected and a goal sent. |
| `move_action_timeout` | `5.0` | Seconds to wait for `move_action` to appear before giving up (and retrying on the next detection cycle). |
| `tf_timeout` | `1.0` | Seconds to wait on each individual transform lookup. |


## Dev Setup
### Prerequisites
- [CMake](https://cmake.org/download/)
- [Python](https://www.python.org/downloads/) >= 3.10
- [pip](https://pypi.org/project/pip/)

### Instructions
#### 1. Install Dev Dependencies
*Optionally: Create a virtual environment for Python (e.g. [venv](https://docs.python.org/3/library/venv.html), [virtualenv](https://virtualenv.pypa.io/en/latest/))*

```bash
pip install -r requirements-dev.txt
```

#### 2. Install pre-commit Git Hooks Locally
```bash
pre-commit install
pre-commit install --hook-type commit-msg
```

### Python Formatting / Linting
Formatting, linting, import sorting with [Ruff](https://github.com/astral-sh/ruff).

#### Check Formatting / Linting
```
make check
```

#### Fix Formatting / Linting
```
make fix
```

### Commit Hooks
#### On Pre-Commit
- [no-commit-to-branch](https://github.com/pre-commit/pre-commit-hooks)
    - targeting `main`
- [check-yaml](https://github.com/pre-commit/pre-commit-hooks)
- [check-xml](https://github.com/pre-commit/pre-commit-hooks)
- [ruff-check](https://github.com/astral-sh/ruff-pre-commit)
- [ruff-format](https://github.com/astral-sh/ruff-pre-commit)


#### On Commit Message
- [commit-msg](https://github.com/jorisroovers/gitlint)
    - Ignore [`title-must-not-contain-word`](https://jorisroovers.com/gitlint/latest/rules/builtin_rules/#t5-title-must-not-contain-word)
    - Ignore [`body-is-missing`](https://jorisroovers.com/gitlint/latest/rules/builtin_rules/#b6-body-is-missing)
    - Ignore [`body-changed-file-mention`](https://jorisroovers.com/gitlint/latest/rules/builtin_rules/#b7-body-changed-file-mention)
