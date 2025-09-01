# SafePause

This is the SafePause ROS 2 Workspace.


## Setup
### Prerequisites
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Doosan Robotics ROS 2 Dependencies](https://github.com/DoosanRobotics/doosan-robot2?tab=readme-ov-file#installation)

### Instructions
#### 1. Clone with Submodules
```bash
git clone --recurse-submodules https://github.com/jd-u/SafePause.git
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


## Dev Setup
### Prerequisites
- [CMake](https://cmake.org/download/)
- [Python](https://www.python.org/downloads/) >= 3.10
- [pip](https://pypi.org/project/pip/)

### Instructions
#### 1. Clone with Submodules
```bash
git clone --recurse-submodules https://github.com/jd-u/SafePause.git
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
Formatting with [Black](https://pypi.org/project/black/) and [isort](https://pycqa.github.io/isort/), linting with [flake8](https://pypi.org/project/flake8/)/[autoflake](https://pypi.org/project/autoflake/).

#### Check Formatting / Linting
```
make check
```

#### Apply Formatting / Linting
```
make fmt
```

### Commit Hooks
#### On Pre-Commit
- [no-commit-to-branch](https://github.com/pre-commit/pre-commit-hooks)
    - targeting `main`
- [check-yaml](https://github.com/pre-commit/pre-commit-hooks)
- [check-xml](https://github.com/pre-commit/pre-commit-hooks)
- [Black](https://github.com/psf/black)
- [isort](https://github.com/pycqa/isort)
- [flake8](https://github.com/pycqa/flake8)

#### On Commit Message
- [commit-msg](https://github.com/jorisroovers/gitlint)
    - Ignore [`title-must-not-contain-word`](https://jorisroovers.com/gitlint/latest/rules/builtin_rules/#t5-title-must-not-contain-word)
    - Ignore [`body-is-missing`](https://jorisroovers.com/gitlint/latest/rules/builtin_rules/#b6-body-is-missing)
    - Ignore [`body-changed-file-mention`](https://jorisroovers.com/gitlint/latest/rules/builtin_rules/#b7-body-changed-file-mention)
