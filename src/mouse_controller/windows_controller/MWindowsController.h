#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

class MWindowsController {
public:

    MWindowsController();
    ~MWindowsController();

    // 初始化 GLFW 和 鼠标控制
    void initializeGLFW(GLFWwindow **window, const char *title, int width, int height);

    // 设置相机参数
    void setupCamera(mjvCamera *cam, double azimuth = 90, double elevation = -20,
                     double distance = 2.0,
                     double lookat_x = 0.0, double lookat_y = 0.0, double lookat_z = 0.0);

    // 更新想即位置跟随物体
    void updateCameraLookAt(mjvCamera *cam, double x);

    // 检查是否由用户手动移动了相机
    bool isUserCameraMoved() const;

    // 重置用户相机移动状态
    void resetUserCameraMoved();

    // 获取关联的窗口
    GLFWwindow *getWindow() const;

private:

    GLFWwindow *window;
    bool mouse_left_pressed;   // 鼠标左键是否按下
    bool mouse_right_pressed;  // 鼠标右键是否按下
    bool mouse_middle_pressed; // 鼠标中键是否按下
    bool user_camera_moved;    // 用户是否手动移动了相机
    double lastx, lasty;       // 上一次鼠标位置

    // GLFW 回调函数
    static void mouseButtonCallback(GLFWwindow *window, int button, int act, int mods);
    static void cursorPosCallback(GLFWwindow *window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset);
    static void windowResizeCallback(GLFWwindow *window, int width, int height);

    // 辅助函数
    static void quatToForward(mjtNum forward[3], const mjtNum quat[4]);
};