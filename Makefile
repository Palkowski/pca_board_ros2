#ARCH = $(shell uname -m)
#
#ifeq ($(ARCH), x86_64)
#    BCAM_ROS2 = berxel_camera_ros2_x86_64
#else ifeq ($(ARCH), aarch64)
#    BCAM_ROS2 = berxel_camera_ros2_arm64
#else
#    $(error Unsupported CPU architecture: $(ARCH))
#endif

ROS2_PKGS = pca_board_bringup pca_board_interfaces
DIRS_TO_CLEAN = build/ install/ log/

.PHONY: clean all $(ROS2_PKGS) $(DIRS_TO_CLEAN)

all: $(ROS2_PKGS)

clean: $(DIRS_TO_CLEAN)
	rm -rf $^

pca_board_interfaces:
	colcon build --packages-select $@

pca_board_bringup: pca_board_interfaces
	colcon build --packages-select $@
