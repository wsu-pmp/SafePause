ifeq ($(OS), Windows_NT)
    PYTHON := python.exe
else
    PYTHON := python3
endif


.PHONY: check
check: check-black check-isort check-flake

.PHONY: fmt
fmt: fmt-black fmt-isort fmt-autoflake


.PHONY: check-black
check-black:
	$(PYTHON) -m black --check --diff . --exclude="(\.?.venv|install|build|log)/"

.PHONY: check-isort
check-isort:
	$(PYTHON) -m isort --check-only --diff . --skip-glob="venv/**" --skip-glob=".venv/**" --skip-glob="**/install/**" --skip-glob="**/build/**"  --skip-glob="**/log/**"

.PHONY: check-flake
check-flake:
	$(PYTHON) -m flake8 .  --exclude=venv,.venv,install,build,log


.PHONY: fmt-black
fmt-black:
	$(PYTHON) -m black . --exclude="(\.?.venv|install|build|log)/"

.PHONY: fmt-isort
fmt-isort:
	$(PYTHON) -m isort . --skip-glob="venv/**" --skip-glob=".venv/**" --skip-glob="**/install/**" --skip-glob="**/build/**" --skip-glob="**/log/**"

.PHONY: fmt-autoflake
fmt-autoflake:
	$(PYTHON) -m autoflake --remove-all-unused-imports --remove-unused-variables --in-place --recursive . --exclude=venv,.venv,install,build,log
