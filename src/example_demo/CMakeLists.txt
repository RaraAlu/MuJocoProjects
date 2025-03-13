# 添加控制器库子目录
# 此操作会让CMake处理 sphere_demo 子目录下的CMakeLists.txt文件
add_subdirectory(sphere_demo)

set(LOCAL_PKG
  example_demo
)

# 添加可执行文件
# 创建一个名为inverted_pendulum的可执行文件，其源文件为main.cpp
add_executable(${LOCAL_PKG} main.cpp)

# 设置包含目录
# 为${LOCAL_PKG}目标设置私有包含目录，这里指定为当前源文件所在目录
target_include_directories(${LOCAL_PKG}
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# 链接库
# 为${LOCAL_PKG}目标链接所需的库
target_link_libraries(${LOCAL_PKG}
  PRIVATE
  # 链接example库
  sphere_demo
  # 链接mujoco库
  mujoco
  # 链接OpenGL库
  OpenGL::GL
  # 链接glfw库
  glfw
  # 如果使用GNU编译器，链接stdc++fs库
  $<$<CXX_COMPILER_ID:GNU>:stdc++fs>
)

# 添加编译定义
# 为${LOCAL_PKG}目标添加编译定义
target_compile_definitions(${LOCAL_PKG}
  PRIVATE
  # 如果是Debug配置，定义DEBUG宏
  $<$<CONFIG:Debug>:DEBUG>
  # 如果是Release配置，定义NDEBUG宏
  $<$<CONFIG:Release>:NDEBUG>
)

# 添加编译选项
# 为${LOCAL_PKG}目标添加编译选项
target_compile_options(${LOCAL_PKG}
  PRIVATE
  # 如果使用GNU编译器，添加-Wall和-Wextra警告选项
  $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra>
  # 如果使用Clang编译器，添加-Wall和-Wextra警告选项
  $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra>
  # 如果使用MSVC编译器，添加/W4警告选项
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# 在构建时复制模型文件到构建目录
# 添加一个自定义命令，在${LOCAL_PKG}目标构建完成后执行
add_custom_command(
  TARGET ${LOCAL_PKG} POST_BUILD
  # 执行复制目录的命令，将当前源文件目录下的model目录复制到${LOCAL_PKG}目标文件所在目录下的model目录
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_CURRENT_SOURCE_DIR}/model
  $<TARGET_FILE_DIR:${LOCAL_PKG}>/model
  # 注释信息，显示在构建过程中
  COMMENT "Copying model files to build directory"
)

# 安装目标和模型文件
# 安装${LOCAL_PKG}可执行文件到标准的二进制安装目录
install(
  TARGETS ${LOCAL_PKG}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 安装模型文件到标准的数据安装目录下的项目名/model目录
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/model/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/model
)
