# 设定CMake的最低版本要求，确保使用的CMake版本不低于3.24
cmake_minimum_required(VERSION 3.24)

# 定义一个新的CMake项目
# 项目名称为MojocoProjects
# 项目版本为1.0.0
# 项目描述为 "MuJoCo Projects Collection"
# 项目支持的编程语言为C和C++
project(MojocoProjects
  VERSION 1.0.0
  DESCRIPTION "MuJoCo Projects Collection"
  LANGUAGES C CXX
)

# 设置CMake策略
# CMP0135策略用于控制下载提取时间戳的行为
if(POLICY CMP0135)
  # 设置下载提取时间戳策略为新版本
  cmake_policy(SET CMP0135 NEW) 
endif()

# 设置全局编译选项
# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)
# 要求必须使用指定的C++标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 不使用特定编译器的C++扩展
set(CMAKE_CXX_EXTENSIONS OFF)
# 设置C标准为C99
set(CMAKE_C_STANDARD 99)
# 要求必须使用指定的C标准
set(CMAKE_C_STANDARD_REQUIRED ON)
# 不使用特定编译器的C扩展
set(CMAKE_C_EXTENSIONS OFF)
# 导出编译命令到compile_commands.json文件，方便其他工具使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 默认构建类型
# 如果没有指定构建类型，则默认设置为Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
  # 设置构建类型的可选值
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# 设置输出目录
# 设置静态库的输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# 设置动态库的输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# MuJoCo库定位
# 设置MuJoCo的安装目录，默认从环境变量中获取
set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.3.0" CACHE PATH "MuJoCo installation directory")
# 查找MuJoCo库文件
find_library(MUJOCO_LIB
  # 库文件的名称
  NAMES mujoco
  # 查找库文件的路径
  PATHS
    "${MUJOCO_DIR}/lib"
    "${MUJOCO_DIR}/bin"
    /usr/local/lib
  # 查找库文件的子目录
  PATH_SUFFIXES lib libs
  # 库文件的描述信息
  DOC "MuJoCo library"
  # 要求必须找到该库文件
  REQUIRED
)

# 查找MuJoCo头文件
find_path(MUJOCO_INCLUDE_DIR
  # 头文件的名称
  NAMES mujoco/mujoco.h
  # 查找头文件的路径
  PATHS
    "${MUJOCO_DIR}/include"
    /usr/local/include
  # 头文件目录的描述信息
  DOC "MuJoCo include directory"
  # 要求必须找到该头文件目录
  REQUIRED
)

# 创建MuJoCo导入目标
add_library(mujoco SHARED IMPORTED)
# 设置导入目标的属性
set_target_properties(mujoco PROPERTIES
  # 导入库文件的位置
  IMPORTED_LOCATION "${MUJOCO_LIB}"
  # 导入库的头文件目录
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
)

# 查找依赖库
# 查找OpenGL库，要求必须找到
find_package(OpenGL REQUIRED)
# 查找glfw3库，要求必须找到
find_package(glfw3 REQUIRED)

# 添加子目录，CMake会递归处理该子目录下的CMakeLists.txt文件
add_subdirectory(src)

# 安装配置
# 包含GNUInstallDirs模块，用于定义标准的安装目录
include(GNUInstallDirs)
# 安装目标文件
install(TARGETS
  # 可执行文件安装到标准的二进制目录
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  # 动态库安装到标准的库目录
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  # 静态库安装到标准的库目录
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

