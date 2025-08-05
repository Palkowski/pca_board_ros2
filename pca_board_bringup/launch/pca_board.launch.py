from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pca_board_params = [
        {'i2c': 1},  # 7 for jetson orin nano 3 and 5 pins
        {'slave_addr': 0x40},  # default PCA slave address
        {'freq': 50},
        {'angle_min': 0},
        {'angle_max': 180},
        {'pl_min': 0.4},
        {'pl_max': 2.4},
        {'angle_min_0': 0},
        {'angle_max_0': 90},
        {'pl_min_0': 1.0},
        {'pl_max_0': 2.0},
        {'angle_min_1': 0},
        {'angle_max_1': 90},
        {'pl_min_1': 1.0},
        {'pl_max_1': 2.0},
    ]
    pca_board = Node(
        package='pca_board_bringup',
        executable='pca_board',
        parameters=pca_board_params,
    )
    ld.add_action(pca_board)
    return ld
