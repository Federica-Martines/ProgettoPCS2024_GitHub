list(APPEND src_sources "${CMAKE_CURRENT_SOURCE_DIR}/GeometryLibrary.cpp")
list(APPEND src_sources "${CMAKE_CURRENT_SOURCE_DIR}/GeometryLibrary.hpp")

list(APPEND src_sources "${CMAKE_CURRENT_SOURCE_DIR}/Utils.cpp")
list(APPEND src_sources "${CMAKE_CURRENT_SOURCE_DIR}/Utils.hpp")

list(APPEND src_sources "${CMAKE_CURRENT_SOURCE_DIR}/input-output.cpp")
list(APPEND src_sources "${CMAKE_CURRENT_SOURCE_DIR}/input-output.hpp")

list(APPEND src_includes "${CMAKE_CURRENT_SOURCE_DIR}")

set(src_sources ${src_sources} PARENT_SCOPE)
set(src_headers ${src_headers} PARENT_SCOPE)
set(src_includers ${src_includes} PARENT_SCOPE)
