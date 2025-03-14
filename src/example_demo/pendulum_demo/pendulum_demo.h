#pragma once

#include "windows_controller/MWindowsController.h"

class Pendulum {
public:
    Pendulum(mjModel *model, mjData *data);
    ~Pendulum();

    void render();
    void updateCameraLookAt(double x); // 更新相机跟随

    inline mjvCamera *getCamera() { return &cam; }
    inline mjData *Data() { return d; }
    
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

