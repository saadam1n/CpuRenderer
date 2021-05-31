file(GLOB_RECURSE Sources "*.cpp" "*.h")

add_executable(CpuRenderer ${Sources})

target_link_libraries(CpuRenderer PRIVATE glm assimp)