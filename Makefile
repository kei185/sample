.PHONY: build init

init:
	colcon build --symlink-install


build:
	colcon build
