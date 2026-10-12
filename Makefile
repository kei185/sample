.PHONY: build init clean

init:
	colcon build --symlink-install


build:
	colcon build

clean:
	rm -rf build
