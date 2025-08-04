cmake_minimum_required(VERSION 3.8)
project(pca_board_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -pedantic -Wfatal-errors
        -Wfloat-equal -Wnon-virtual-dtor -Wshadow -Werror=vla -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pca_board_interfaces REQUIRED)

include_directories(include/)

add_executable(pca_board src/pca_board.cpp src/i2c_pca_utils.c)
target_link_libraries(pca_board -li2c)
ament_target_dependencies(pca_board rclcpp pca_board_interfaces)

#add_executable(pca_board_keyboard_control src/pca_board_keyboard_control.cpp)
#ament_target_dependencies(pca_board_keyboard_control rclcpp)

install(TARGETS
    pca_board
    DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
