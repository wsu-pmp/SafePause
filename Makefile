ifeq ($(OS), Windows_NT)
    PYTHON := python.exe
else
    PYTHON := python3
endif


.PHONY: check
check: check-lint check-fmt 

.PHONY: fix
fix: fix-lint fix-fmt


.PHONY: check-lint
check-lint:
	$(PYTHON) -m ruff check --no-fix .

.PHONY: check-fmt
check-fmt:
	$(PYTHON) -m ruff format --diff .


.PHONY: fix-lint
fix-lint:
	$(PYTHON) -m ruff check --fix .

.PHONY: fix-fmt
fix-fmt:
	$(PYTHON) -m ruff format . 
