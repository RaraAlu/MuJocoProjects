# 创建控制器库
# 使用add_library命令创建一个名为 sphere_demo 的库
# 该库的源文件为 sphere_demo.cpp
add_library(sphere_demo
  sphere_demo.cpp
)

# 设置包含目录
# 为sphere_demo目标设置包含目录
# PUBLIC表示该包含目录对使用sphere_demo库的目标也可见
# 当前源文件所在目录为公共包含目录
# PRIVATE表示该包含目录仅对sphere_demo库自身可见
# MuJoCo的头文件目录为私有包含目录
target_include_directories(sphere_demo
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  PRIVATE
    ${MUJOCO_INCLUDE_DIR}
)

# 链接库
target_link_libraries(sphere_demo
  PUBLIC
    mouse_controller
  PRIVATE
    mujoco
    OpenGL::GL
    glfw
)

# 添加编译选项
# 为sphere_demo目标添加编译选项
# 根据不同的编译器，添加不同的警告选项
# 如果使用GNU编译器，添加-Wall和-Wextra警告选项
# 如果使用Clang编译器，添加-Wall和-Wextra警告选项
# 如果使用MSVC编译器，添加/W4警告选项
target_compile_options(sphere_demo
  PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# 设置库属性
# 为sphere_demo目标设置属性
# POSITION_INDEPENDENT_CODE ON表示生成位置无关代码，适用于动态链接库
set_target_properties(sphere_demo PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

# 安装库和头文件
# 安装sphere_demo库到标准的库安装目录
# ARCHIVE表示静态库，LIBRARY表示动态库
install(
  TARGETS sphere_demo
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# 安装sphere_demo.h头文件到标准的包含目录下的项目名子目录
install(
  FILES sphere_demo.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
