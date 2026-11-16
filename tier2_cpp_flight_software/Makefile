BUILD_DIR := build

.PHONY: all build test run_sitl clean

all: build

build:
	@mkdir -p $(BUILD_DIR)
	@cmake -B $(BUILD_DIR) -S .
	@cmake --build $(BUILD_DIR) -j

test: build
	@ctest --test-dir $(BUILD_DIR) --output-on-failure

run_sitl: build
	@./$(BUILD_DIR)/test_sitl_closed_loop

clean:
	@rm -rf $(BUILD_DIR)