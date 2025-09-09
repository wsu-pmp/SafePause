ifeq ($(OS), Windows_NT)
    PYTHON := python.exe
else
    PYTHON := python3
endif

# git submodule patterns for excludes
# for black (pipe-separated)
SUBMODULES_PIPE := $(shell $(PYTHON) -c "import subprocess; print('|'.join([line.strip() for line in subprocess.check_output(['git','submodule','foreach','--quiet','echo $$path'], shell=True).decode().splitlines()]))")

# for isort (space-separated, appended with /**)
SUBMODULES_GLOB := $(shell $(PYTHON) -c "import subprocess; print(' '.join([f'{line.strip()}/**' for line in subprocess.check_output(['git','submodule','foreach','--quiet','echo $$path'], shell=True).decode().splitlines()])))

# for flake8/autoflake (comma-separated)
SUBMODULES_COMMA := $(shell $(PYTHON) -c "import subprocess; print(','.join([line.strip() for line in subprocess.check_output(['git','submodule','foreach','--quiet','echo $$path'], shell=True).decode().splitlines()]))")

SUBMODULES := $(shell $(PYTHON) -c "import subprocess; print(','.join([line.strip() for line in subprocess.check_output(['git','submodule','foreach','--quiet','echo $$path'], shell=True).decode().splitlines()]))")


.PHONY: check
check: check-black check-isort check-flake

.PHONY: fmt
fmt: fmt-black fmt-isort fmt-autoflake


.PHONY: check-black
check-black:
	$(PYTHON) -m black --check --diff . --exclude="(\.?.venv/|install/|build/|log/|$(SUBMODULES_PIPE)|.*/test/test_copyright\.py|.*/test/test_flake8\.py|.*/test/test_pep257\.py)"

.PHONY: check-isort
check-isort:
	$(PYTHON) -m isort --check-only --diff . --skip-glob="venv/**" --skip-glob=".venv/**" --skip-glob="**/install/**" --skip-glob="**/build/**"  --skip-glob="**/log/**" $(foreach g,$(SUBMODULES_GLOB),--skip-glob="$(g)") --skip-glob="**/test/test_copyright.py" --skip-glob="**/test/test_flake8.py" --skip-glob="**/test/test_pep257.py"
	
.PHONY: check-flake
check-flake:
	$(PYTHON) -m flake8 .  --exclude=venv,.venv,install,build,log,$(SUBMODULES_COMMA),**/test/test_copyright.py,**/test/test_flake8.py,**/test/test_pep257.py


.PHONY: fmt-black
fmt-black:
	$(PYTHON) -m black . --exclude="(\.?.venv/|install/|build/|log/|$(SUBMODULES_PIPE)|.*/test/test_copyright\.py|.*/test/test_flake8\.py|.*/test/test_pep257\.py)"

.PHONY: fmt-isort
fmt-isort:
	$(PYTHON) -m isort . --skip-glob="venv/**" --skip-glob="venv/**" --skip-glob=".venv/**" --skip-glob="**/install/**" --skip-glob="**/build/**"  --skip-glob="**/log/**" $(foreach g,$(SUBMODULES_GLOB),--skip-glob="$(g)") --skip-glob="**/test/test_copyright.py" --skip-glob="**/test/test_flake8.py" --skip-glob="**/test/test_pep257.py"

.PHONY: fmt-autoflake
fmt-autoflake:
	$(PYTHON) -m autoflake --remove-all-unused-imports --remove-unused-variables --in-place --recursive . --exclude=venv,.venv,install,build,log,$(SUBMODULES_COMMA),**/test/test_copyright.py,**/test/test_flake8.py,**/test/test_pep257.py
