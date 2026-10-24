cmake_minimum_required(VERSION 3.8)
project(fishbot_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# 把地图、参数、launch 装到 share/fishbot_navigation2/
# （不写这段，ros2 launch 找不到 launch 文件，map_server 也找不到地图）
install(DIRECTORY maps param launch
        DESTINATION share/${PROJECT_NAME})

# 巡检脚本装到 lib/fishbot_navigation2/
# （不写这段，ros2 run fishbot_navigation2 patrol_inspector 会说找不到可执行文件）
# RENAME 很重要：不写的话装出来叫 patrol_inspector.py，
# 而 ros2 run / launch 里写的 executable 是 patrol_inspector，名字对不上就找不到
install(PROGRAMS scripts/patrol_inspector.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME patrol_inspector)

# 世界文件 → 真值地图 的渲染脚本（SLAM 建图不理想时用它生成 maps/room.*）
install(PROGRAMS scripts/make_map_from_world.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME make_map_from_world)

ament_package()
