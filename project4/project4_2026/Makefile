# Run after: micromamba activate ompl_env
# Install CMake in that environment: python -m pip install cmake
PYTHON ?= python3
BUILD_DIR = build

.PHONY: all clean
all:
	$(PYTHON) -m cmake -S . -B $(BUILD_DIR) -DPython_EXECUTABLE=$$($(PYTHON) -c "import sys; print(sys.executable)")
	$(PYTHON) -m cmake --build $(BUILD_DIR) -j

clean:
	$(RM) -r $(BUILD_DIR)
	$(RM) Project4Car Project4Pendulum *.o
