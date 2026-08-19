cmake_minimum_required(VERSION 3.8)
project(robot_launch_files)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Node(executable=...) is resolved from lib/<package>, so the scripts have to be installed there.
install(
  PROGRAMS
    script/bodypart_resetter.py
    script/emoticon_handler
    script/joke.py
    script/rqt.bash
    script/scan_gmapping.py
    script/ssl_dummy
    script/topic_monitor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_lint_cmake(MAX_LINE_LENGTH 120)
  ament_xmllint(MAX_LINE_LENGTH 120)
endif()

ament_package()
