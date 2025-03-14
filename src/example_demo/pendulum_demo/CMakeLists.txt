# 这是pendulum_demo目录下的CMakeLists.txt

# 设置可执行文件名称
set(DEMO_NAME pendulum_demo)

# 首先创建库部分
add_library(${DEMO_NAME}_lib
  pendulum_demo.cpp
)

# 设置库的包含目录
target_include_directories(${DEMO_NAME}_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  PRIVATE
    ${MUJOCO_INCLUDE_DIR}
)

# 设置库的链接
target_link_libraries(${DEMO_NAME}_lib
  PUBLIC
    mouse_controller
  PRIVATE
    mujoco
    OpenGL::GL
    glfw
)

# 设置库的编译选项
target_compile_options(${DEMO_NAME}_lib
  PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# 设置库的属性
set_target_properties(${DEMO_NAME}_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# 创建可执行文件
add_executable(${DEMO_NAME} main.cpp)

# 设置可执行文件的包含目录
target_include_directories(${DEMO_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 设置可执行文件的链接
target_link_libraries(${DEMO_NAME}
  PRIVATE
    ${DEMO_NAME}_lib
    mujoco
    OpenGL::GL
    glfw
    $<$<CXX_COMPILER_ID:GNU>:stdc++fs>
)

# 设置可执行文件的编译定义
target_compile_definitions(${DEMO_NAME}
  PRIVATE
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Release>:NDEBUG>
)

# 设置可执行文件的编译选项
target_compile_options(${DEMO_NAME}
  PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# 复制demo特定的模型文件到可执行文件目录
add_custom_command(
  TARGET ${DEMO_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_CURRENT_SOURCE_DIR}/../model/${DEMO_NAME}
  $<TARGET_FILE_DIR:${DEMO_NAME}>/model/${DEMO_NAME}
  COMMENT "Copying ${DEMO_NAME} model files to build directory"
)

# 安装设置
install(
  TARGETS ${DEMO_NAME} ${DEMO_NAME}_lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  FILES pendulum_demo.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/${DEMO_NAME}
)
