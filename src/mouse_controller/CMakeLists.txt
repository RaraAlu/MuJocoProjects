# 创建鼠标控制器库
add_library(mouse_controller
    windows_controller/MWindowsController.cpp
)

# 设置包含目录
target_include_directories(mouse_controller
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 链接库
target_link_libraries(
    mouse_controller
    PRIVATE
    OpenGL::GL
    glfw
)

# 添加编译选项
target_compile_options(mouse_controller
    PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# 设置库属性
set_target_properties(mouse_controller PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# 安装库和头文件
install(
    TARGETS mouse_controller
    # ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
    FILES windows_controller/MWindowsController.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

