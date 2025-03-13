#pragma once

#include "windows_controller/MWindowsController.h"

class SphereDemo {
public:
    SphereDemo(mjModel *model, mjData *data);
    ~SphereDemo();

    void render();
    void updateCameraLookAt(double x); // 更新相机跟随
    
    GLFWwindow *window;

private:
    mjModel *m;
    mjData *d;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjvCamera cam;
    
    MWindowsController mouseController; // 使用鼠标控制器

    void initVisualization();
};

