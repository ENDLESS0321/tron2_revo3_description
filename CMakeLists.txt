cmake_minimum_required(VERSION 3.22)
project(tron2_revo3_description NONE)
find_package(ament_cmake REQUIRED)
# Mesh paths in the URDFs are relative to assets/, so install the directory unchanged.
install(DIRECTORY assets licenses DESTINATION share/${PROJECT_NAME})
install(FILES THIRD_PARTY_NOTICES.md DESTINATION share/${PROJECT_NAME})
ament_package()
