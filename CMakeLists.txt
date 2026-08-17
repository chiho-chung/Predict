cmake_minimum_required(VERSION 3.16)
project(drone_chase_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(drone_chase_sim
  src/app/main.cpp
  src/sim/sim.cpp
  src/ui/render.cpp
  src/estimator/predictor.cpp
  src/ui/plot.cpp
  src/io/csv_log.cpp
  src/app/bench.cpp
  src/ui/history.cpp
)

target_include_directories(drone_chase_sim PRIVATE src)

if(WIN32)
  target_link_libraries(drone_chase_sim PRIVATE gdi32 user32)
endif()
