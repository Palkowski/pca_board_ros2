ROS2_PKGS = pca_board_bringup pca_board_interfaces
DIRS_TO_CLEAN = build/ install/ log/

SHELL := /bin/bash  # to make `source` available

.PHONY: clean all $(ROS2_PKGS) $(DIRS_TO_CLEAN)

all: $(ROS2_PKGS)

clean: $(DIRS_TO_CLEAN)
	rm -rf $^

pca_board_interfaces:
	colcon build --packages-select $@

pca_board_bringup: pca_board_interfaces
	source install/setup.bash && \
	colcon build --packages-select $@
