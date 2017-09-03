cmake_minimum_required(VERSION 2.8)

project (pico_flex_driver)

set(CMAKE_PREFIX_PATH ".")

find_package(royale REQUIRED)
find_package(OpenCV REQUIRED)

set(pico_flex_driver_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

link_directories(${royale_LIB_DIR} ${OpenCV_INCLUDE_DIRS})

include_directories(${pico_flex_driver_INCLUDE_DIR} ${royale_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(pico_flex_driver src/driver.cc)
target_link_libraries(pico_flex_driver "${royale_LIBS}" ${OpenCV_LIBS})

add_executable(demo_main src/demo_main.cc)
target_link_libraries(demo_main pico_flex_driver)
