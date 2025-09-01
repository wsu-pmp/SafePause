# SafePause

This is the SafePause ROS 2 Workspace.


## Dev Setup
### Prerequisites
- [CMake](https://cmake.org/download/)
- [Python](https://www.python.org/downloads/) >= 3.10
- [pip](https://pypi.org/project/pip/)

### Instructions
#### 1. Clone Repo
```bash
git clone https://github.com/jd-u/SafePause.git
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
