# Run after: micromamba activate ompl_env
# Install build tools in that environment: python -m pip install cmake nanobind
PYTHON ?= python3
BUILD_DIR = build

.PHONY: all clean
all:
	$(PYTHON) -m cmake -S . -B $(BUILD_DIR) -DPython_EXECUTABLE=$$($(PYTHON) -c "import sys; print(sys.executable)")
	$(PYTHON) -m cmake --build $(BUILD_DIR) -j

clean:
	$(RM) -r $(BUILD_DIR)
	$(RM) project3_cpp*.so project3_cpp*.pyd
