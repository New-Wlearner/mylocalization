add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/)
# 添加libGeographiccc到ALL_TARGET_LIBRARIES
# libGeographiccc是Geo库生成的
list(APPEND ALL_TARGET_LIBRARIES libGeographiccc )   