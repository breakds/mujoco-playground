add_executable(testspeed)
target_sources(testspeed PRIVATE testspeed.cc)
target_link_libraries(testspeed PRIVATE mujoco::mujoco spdlog::spdlog)
