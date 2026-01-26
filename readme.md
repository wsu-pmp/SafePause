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
### Launch with MoveIt
#### With Emulator
```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=a0509 host:=127.0.0.1
```

#### With Robot
```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=a0509 host:=<robot ip>
```

### `perception_pkg`
`PerceptionNode` synchronizes messages from multiple topics into logical bundles, based on [`message_filters::sync_policies::ApproximateTime`](https://docs.ros.org/en/humble/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html), which requires types to be known at compile-time. `PerceptionNode` works with arbitrary topic types specified via a YAML config by storing type-erased messages that can later be downcast for processing. Time synchronisation will use message [`header`](https://docs.ros.org/en/humble/p/std_msgs/msg/Header.html) if one exists, defaulting to the message arrival time otherwise.

Bundles can be processed internally (currently placeholder logic), and are published as lightweight bundle indices to allow external consumers to identify messages belonging to bundles.

The optional `--namespace` argument allows multiple nodes to run simultaneously, allowing for topics to be combined arbitrarily across more than one bundle (e.g. separating low and high rate topics such that slow messages don't bottleneck faster ones).

**Arguments** (passed via `--ros-args`)

| Argument        | Type   | Required | Description |
|-----------------|--------|----------|-------------|
| `--namespace`   | string | No       | Namespace under which the node is created, affects topic name of bundle index pub |

Example:
```bash
ros2 run perception_pkg perception_node --ros-args --namespace foo
```

**Parameters** (passed via `--ros-args -p`)
| Name              | Type     | Required | Default | Description |
|-------------------|----------|----------|---------|-------------|
| `config_file`     | string   | Yes      | —       | Path to the YAML configuration describing input topics, message types, and TF requirements |
| `queue_size`      | integer  | No       | `10`    | Maximum number of messages retained per input topic queue |
| `slop`            | double   | No       | `0.1`   | Maximum allowed time difference (in seconds) between messages in a bundle |
| `processing_rate` | double   | No       | `10.0`  | Optional rate (Hz) at which processing of bundles occurs |


Example:
```bash
ros2 run perception_pkg perception_node --ros-args -p config_file:=./src/perception_pkg/config/example.yaml
```

**Example config YAML**
```yaml
variables:
  - &TARGET_FRAME base_link
  - &MAX_TF_AGE 0.05 # seconds

topics:
  - name: /point_cloud
    type: geometry_msgs/msg/PointCloud2
    requires_tf: true
    target_frame: *TARGET_FRAME
    max_tf_age: *MAX_TF_AGE
    
  - name: /pose
    type: geometry_msgs/msg/PoseStamped
    requires_tf: true
    target_frame: *TARGET_FRAME
    max_tf_age: *MAX_TF_AGE

  - name: /chatter
    type: std_msgs/msg/String
    requires_tf: false
```

## Dev Setup
### Prerequisites
- [CMake](https://cmake.org/download/)
- [Python](https://www.python.org/downloads/) >= 3.10
- [pip](https://pypi.org/project/pip/)

### Instructions
#### 1. Clone with Submodules
```bash
git clone --recurse-submodules https://github.com/wsu-pmp/SafePause.git
```

#### 2. Install Dev Dependencies
*Optionally: Create a virtual environment for Python (e.g. [venv](https://docs.python.org/3/library/venv.html), [virtualenv](https://virtualenv.pypa.io/en/latest/))*

```bash
pip install -r requirements-dev.txt
```

#### 3. Install pre-commit Git Hooks Locally
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
