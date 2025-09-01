# SafePause

This is the SafePause ROS 2 Workspace.


## Setup
### Prerequisites
- [Ubuntu 22.04 (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)

### Instructions
#### 1. Clone with Submodules
```bash
git clone --recurse-submodules https://github.com/wsu-pmp/SafePause.git
```

#### 2. Install ROS 2 and Doosan Robot 2 Dependencies
```
chmod +x ./setup.sh
./setup.sh
```

#### 3. Build
```bash
cd ./SafePause
source /opt/ros/humble/setup.bash
colcon build
```

#### 4. *(Optional)* Install Doosan Robot Emulator
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
