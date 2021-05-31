set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

add_subdirectory(glm)
add_subdirectory(assimp)

set_target_properties("assimp" PROPERTIES FOLDER "extern/assimp")
set_target_properties("assimp_cmd" PROPERTIES FOLDER "extern/assimp")
set_target_properties("unit" PROPERTIES FOLDER "extern/assimp")
set_target_properties("zlibstatic" PROPERTIES FOLDER "extern/assimp")
set_target_properties("UpdateAssimpLibsDebugSymbolsAndDLLs" PROPERTIES FOLDER "extern/assimp")