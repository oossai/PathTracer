cmake_minimum_required(VERSION 3.2.0)

cmake_policy(SET CMP0076 NEW)

file(GLOB SRC_FILES CONFIGURE_DEPENDS "*.h" "*.cpp")

target_sources(${PROJECT_NAME} PUBLIC ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})