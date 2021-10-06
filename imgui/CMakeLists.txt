project(imgui)
add_library(${PROJECT_NAME} 
			imconfig.h
			imgui.cpp
			imgui_demo.cpp
			imgui_draw.cpp
			imgui.h
			imgui_internal.h
			imgui_tables.cpp
			imgui_widgets.cpp
			imstb_rectpack.h
			imstb_textedit.h
			imstb_truetype.h
			backends/imgui_impl_glfw.cpp
			backends/imgui_impl_glfw.h
			backends/imgui_impl_opengl3.cpp
			backends/imgui_impl_opengl3.h
			)
target_compile_definitions(${PROJECT_NAME}  PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)
target_include_directories(${PROJECT_NAME}  PUBLIC .)

# GLAD
find_package(glad CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME}  PUBLIC glad::glad)