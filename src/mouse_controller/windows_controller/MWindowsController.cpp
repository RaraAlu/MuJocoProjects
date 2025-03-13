#include "MWindowsController.h"
#include <iostream>
#include <unordered_map>

// 静态变量存储相机指针映射
static std::unordered_map<GLFWwindow*, mjvCamera*> windowCameraMap;

MWindowsController::MWindowsController() : window(nullptr),
                                           mouse_left_pressed(false),
                                           mouse_middle_pressed(false),
                                           user_camera_moved(false),
                                           lastx(0.0),
                                           lasty(0.0)
{
}

MWindowsController::~MWindowsController()
{
    // 清理映射
    if (window && windowCameraMap.count(window)) {
        windowCameraMap.erase(window);
    }
}

void MWindowsController::initializeGLFW(GLFWwindow **window, const char *title, int width, int height)
{
    // 初始化GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        throw std::runtime_error("GLFW initialization failed");
    }
    // 开启反走样（在创建窗口前设置）
    glfwWindowHint(GLFW_SAMPLES, 4);
    // 创建GLFW窗口
    GLFWwindow* window_ptr = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!window_ptr) {
        glfwTerminate();
        std::cerr << "Failed to create GLFW window" << std::endl;
        throw std::runtime_error("GLFW window creation failed");
    } 
    // 保存窗口引用
    this->window = window_ptr;
    *window = window_ptr; // 更新传入的指针
    
    // 设置回调函数
    glfwMakeContextCurrent(this->window);
    glfwSetWindowUserPointer(this->window, this); // 将 this 指针与窗口关联
    glfwSetScrollCallback(this->window, scrollCallback);
    glfwSetWindowSizeCallback(this->window, windowResizeCallback);
    glfwSetMouseButtonCallback(this->window, mouseButtonCallback);
    glfwSetCursorPosCallback(this->window, cursorPosCallback);
}

void MWindowsController::setupCamera(mjvCamera *cam, double azimuth, double elevation, double distance, double lookat_x, double lookat_y, double lookat_z)
{
    if (!cam) {
        std::cerr << "Invalid camera pointer" << std::endl;
        return;
    }
    // 设置相机参数
    cam->azimuth = azimuth;
    cam->elevation = elevation;
    cam->distance = distance;
    cam->lookat[0] = lookat_x;
    cam->lookat[1] = lookat_y;
    cam->lookat[2] = lookat_z;
    
    // 将相机指针存储到全局映射中
    windowCameraMap[window] = cam;
}

void MWindowsController::updateCameraLookAt(mjvCamera *cam, double x)
{
    if (!cam) return;
    
    // 只在用户没有手动移动相机时更新相机位置
    if (!user_camera_moved) {
        cam->lookat[0] = x;
    }
}

bool MWindowsController::isUserCameraMoved() const
{
    return user_camera_moved;
}

void MWindowsController::resetUserCameraMoved()
{
    user_camera_moved = false;
}

GLFWwindow *MWindowsController::getWindow() const
{
    return window;
}

void MWindowsController::mouseButtonCallback(GLFWwindow *window, int button, int act, int mods)
{
    (void)mods;
    MWindowsController* controller = static_cast<MWindowsController*>(glfwGetWindowUserPointer(window));
    if (!controller) return;
    // 鼠标左键：旋转
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        controller->mouse_left_pressed = (act == GLFW_PRESS);
        if (act == GLFW_PRESS) {
            controller->user_camera_moved = true;
        }
    }
    // 鼠标中键：平移
    else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        controller->mouse_middle_pressed = (act == GLFW_PRESS);
        if (act == GLFW_PRESS) {
            controller->user_camera_moved = true;
        }
    }
    // 鼠标右键：平移
    else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        controller->mouse_right_pressed = (act == GLFW_PRESS);
        if (act == GLFW_PRESS) {
            controller->user_camera_moved = true;
        }
    }

    // 更新鼠标位置
    glfwGetCursorPos(window, &controller->lastx, &controller->lasty);
}

void MWindowsController::cursorPosCallback(GLFWwindow *window, double xpos, double ypos)
{
    // 获取控制器实例和窗口尺寸
    MWindowsController* controller = static_cast<MWindowsController*>(glfwGetWindowUserPointer(window));
    if (!controller) return;
    
    // 从映射中获取相机
    mjvCamera* cam = nullptr;
    if (windowCameraMap.count(window)) {
        cam = windowCameraMap[window];
    }
    if (!cam) return;
    
    int win_width, win_height;
    glfwGetWindowSize(window, &win_width, &win_height);
    // 计算鼠标位移
    double dx = xpos - controller->lastx;
    double dy = ypos - controller->lasty;
    // 左键旋转
    if (controller->mouse_left_pressed) {
        cam->azimuth -= 0.5 * dx;
        cam->elevation -= 0.5 * dy; // 修正：反转方向使其更直观
        cam->elevation = mju_clip(cam->elevation, -89.0, 89.0);
    }
    // 中键平移
    else if (controller->mouse_middle_pressed) {
        // 计算动态灵敏度参数
        const double scale = cam->distance * 0.01;
        // 通过欧拉角计算前向向量和侧向向量
        const double azimuth_rad = mjPI * cam->azimuth / 180.0;
        const double elevation_rad = mjPI * cam->elevation / 180.0;
        // 计算前向和侧向向量
        mjtNum forward[3] = {
            cos(elevation_rad) * cos(azimuth_rad),
            cos(elevation_rad) * sin(azimuth_rad),
            sin(elevation_rad)};
        // 计算右向量 (侧向)
        mjtNum right[3] = {
            -sin(azimuth_rad),
            cos(azimuth_rad),
            0};
        // 计算上向量
        mjtNum up[3];
        mju_cross(up, right, forward);
        mju_normalize3(up);
        // 应用平移
        for (int i = 0; i < 3; i++) {
            cam->lookat[i] += right[i] * (-dx * scale) + up[i] * (dy * scale);
        }
    }
    // 右键平移
    else if (controller->mouse_right_pressed) {
        // 计算动态灵敏度参数
        const double scale = cam->distance * 0.001;
        // 通过欧拉角计算前向向量和侧向向量
        const double azimuth_rad = mjPI * cam->azimuth / 180.0;
        const double elevation_rad = mjPI * cam->elevation / 180.0;
        // 计算前向和侧向向量
        mjtNum forward[3] = {
            cos(elevation_rad) * cos(azimuth_rad),
            cos(elevation_rad) * sin(azimuth_rad),
            sin(elevation_rad)};
        // 计算右向量 (侧向)
        mjtNum right[3] = {
            -sin(azimuth_rad),
            cos(azimuth_rad),
            0};
        // 计算上向量
        mjtNum up[3];
        mju_cross(up, right, forward);
        mju_normalize3(up);
        // 应用平移
        for (int i = 0; i < 3; i++) {
            cam->lookat[i] -= right[i] * (-dx * scale) + up[i] * (dy * scale);
        }
    }


    // 更新历史位置
    controller->lastx = xpos;
    controller->lasty = ypos;
}

void MWindowsController::scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
    (void)xoffset;
    MWindowsController* controller = static_cast<MWindowsController*>(glfwGetWindowUserPointer(window));
    if (!controller) return;
    
    // 从映射中获取相机
    mjvCamera* cam = nullptr;
    if (windowCameraMap.count(window)) {
        cam = windowCameraMap[window];
    }
    if (!cam) return;
    
    // 修改相机距离，并确保在合理范围内
    cam->distance = mju_clip(cam->distance - 0.1 * yoffset, 0.5, 10.0);
    // 标记用户已手动调整相机
    controller->user_camera_moved = true;
}

void MWindowsController::windowResizeCallback(GLFWwindow *window, int width, int height)
{
    (void)width;
    (void)height;
    (void)window;
    // 这里不需要处理渲染上下文重置，因为具体的渲染器在使用者那里
}

void MWindowsController::quatToForward(mjtNum forward[3], const mjtNum quat[4])
{
    mjtNum mat[9];
    mju_quat2Mat(mat, quat);
    mju_copy3(forward, mat + 6); // 提取Z轴方向
}
