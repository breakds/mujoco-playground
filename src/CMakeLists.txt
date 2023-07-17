add_executable(testspeed)
target_sources(testspeed PRIVATE testspeed.cc)
target_link_libraries(testspeed PRIVATE mujoco::mujoco spdlog::spdlog)

add_library(go1_recorded_control STATIC)
target_sources(go1_recorded_control
    PUBLIC data/go1_recorded_control.h
    PRIVATE data/go1_recorded_control.cc)
target_link_libraries(go1_recorded_control mujoco::mujoco)

add_executable(test_go1_terrain)
target_sources(test_go1_terrain PRIVATE test_go1_terrain.cc)
target_link_libraries(test_go1_terrain PRIVATE mujoco::mujoco spdlog::spdlog go1_recorded_control)
