#include "pendulum_controller.h"
#include <iostream>

PendulumController::PendulumController(mjModel *model, mjData *data) : m(model), d(data)
{
    initVisualization();
}

PendulumController::~PendulumController()
{
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    if (window) {
        glfwDestroyWindow(window);
    }
    glfwTerminate();
}

void PendulumController::computeControl()
{
    // 状态获取：小车位置[0], 摆杆角度[1]
    double angle = d->qpos[1];  // 获取铰链角度
    double angular_velocity = d->qvel[1];  // 获取角速度

    // PD控制参数 - 正参数用于倒立摆控制
    const double Kp = 10.5;  // 比例系数
    const double Kd = 0.0;  // 微分系数

    // 计算角度误差 - 目标是保持在倒立位置(π)
    double angle_error = angle - M_PI;  
    
    // 归一化角度误差到[-π,π]区间
    while (angle_error > M_PI) angle_error -= 2*M_PI;
    while (angle_error < -M_PI) angle_error += 2*M_PI;
    
    // 计算控制力
    double control = Kp * angle_error + Kd * angular_velocity;

    // 施加控制（限幅在±1之间）
    d->ctrl[0] = mju_clip(control, -1.0, 1.0);
}

void PendulumController::render()
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

void PendulumController::updateCameraLookAt(double x)
{
    // 使用鼠标控制器更新相机位置
    mouseController.updateCameraLookAt(&cam, x);
}

void PendulumController::initVisualization()
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
