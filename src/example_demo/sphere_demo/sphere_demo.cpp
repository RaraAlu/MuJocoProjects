#include "sphere_demo.h"
#include <iostream>

SphereDemo::SphereDemo(mjModel *model, mjData *data) : m(model), d(data)
{
    initVisualization();
}

SphereDemo::~SphereDemo()
{
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    if (window) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

void SphereDemo::render()
{
    // 渲染数据准备
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // 更新和渲染场景
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
}

void SphereDemo::updateCameraLookAt(double x)
{
    // 使用鼠标控制器更新相机位置
    mouseController.updateCameraLookAt(&cam, x);
}

void SphereDemo::initVisualization()
{
    // 初始化GLFW和窗口
    mouseController.initializeGLFW(&window, "MuJoCo Inverted Pendulum", 1200, 900);

    // 初始化可视化组件
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);

    // 检查输入有效性
    if (!m || !d) {
        std::cerr << "Invalid MuJoCo model or data pointers" << std::endl;
        throw std::invalid_argument("MuJoCo model or data is null");
    }

    // 初始化MuJoCo渲染器
    mjv_makeScene(m, &scn, 1000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 设置相机参数
    mouseController.setupCamera(&cam, 90, -20, 4.0, 0.0, 0.0, 0.3);

    void* pointer = glfwGetWindowUserPointer(window);
    
    // 将相机对象指针关联到窗口
    glfwSetWindowUserPointer(window, pointer);
}
