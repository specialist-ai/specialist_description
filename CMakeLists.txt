cmake_minimum_required(VERSION 3.16)
project(specialist_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY specialist_description/robots DESTINATION share/${PROJECT_NAME} PATTERN cad EXCLUDE)

ament_package()
