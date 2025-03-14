# 这是example_demo目录下的顶层CMakeLists.txt
# 这个文件作为总控文件，添加各个demo子目录

# 添加所有demo子目录
add_subdirectory(sphere_demo)
# 将来可以添加更多demo
# add_subdirectory(pendulum_demo)
# add_subdirectory(robot_arm_demo)
# 等等...

# 在构建时复制共享的模型文件到构建目录
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/model
     DESTINATION ${CMAKE_BINARY_DIR})

# 安装共享的模型文件
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/model/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/model
)
