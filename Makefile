# Makefile for LaneZero

.PHONY: all build clean test install lint help

all: build

build:
	@echo "Building LaneZero..."
	@mkdir -p build
	@cd build && cmake .. && make -j$$(sysctl -n hw.ncpu)
	@cp build/_core*.so LaneZero/
	@echo "Build completed!"

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf build
	@rm -f LaneZero/_core*.so
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	@echo "Clean completed!"

install: build
	@echo "Installing LaneZero in development mode..."
	@pip install -e .
	@echo "Installation completed!"

test: build
	@echo "Running tests..."
	@python3 -m pytest test/ -v
	@echo "Tests completed!"

lint:
	@echo "Linting Python files..."
	@flake8 LaneZero/ test/ --max-line-length=79 --exclude=__pycache__
	@echo "Linting C++ files..."
	@find src -name "*.cpp" -o -name "*.h" | xargs clang-format -i
	@echo "Linting completed!"

help:
	@echo "LaneZero Makefile"
	@echo ""
	@echo "Usage:"
	@echo "  make build    - Build the C++ library and Python bindings"
	@echo "  make clean    - Remove build artifacts"
	@echo "  make install  - Build and install in development mode"
	@echo "  make test     - Build and run tests"
	@echo "  make lint     - Run linters on code"
	@echo "  make help     - Show this help message"

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
