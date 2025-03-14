#include "pendulum_demo.h"
#include <cmath>
#include <cstring>
#include <experimental/filesystem>
#include <iostream>
#include <mujoco/mujoco.h>

namespace fs = std::experimental::filesystem;

// 用于判断两个浮点数是否相等(考虑浮点误差)
bool isEqual(double a, double b, double epsilon = 1e-6)
{
    return std::fabs(a - b) < epsilon;
}

// 判断相机是否移动
bool cameraChanged(const mjvCamera *cam, const mjvCamera *lastCam)
{
    if (!isEqual(cam->azimuth, lastCam->azimuth) ||
        !isEqual(cam->elevation, lastCam->elevation) ||
        !isEqual(cam->distance, lastCam->distance) ||
        !isEqual(cam->lookat[0], lastCam->lookat[0]) ||
        !isEqual(cam->lookat[1], lastCam->lookat[1]) ||
        !isEqual(cam->lookat[2], lastCam->lookat[2])) {
        return true;
    }
    return false;
}

int main()
{
    // 初始化MuJoCo
    char error[1000] = "Could not load binary model";



    
    // 获取根目录地址的上一级
    fs::path rootPath = fs::current_path().parent_path();
    printf("Parent_path: %s\n", rootPath.string().c_str());
    std::string project_path = rootPath.string() + "/src/example_demo/model/pendulum_demo/pendulum.xml";
    std::cout << "Root Path: " << project_path << std::endl;

    // 加载模型
    mjModel *model = mj_loadXML(project_path.c_str(), nullptr, error, 1000);
    if (!model) {
        std::cerr << "Error loading model: " << error << std::endl;
        return -1;
    }

    mjData *data = mj_makeData(model);




    // 初始化控制器
    Pendulum controller(model, data);

    // 仿真参数
    int stabilizationSteps = 300000; // 给系统足够时间稳定
    int currentStep = 0;


    double realTimeStart = glfwGetTime(); // 实时同步变量
    double simTime = 0.0;

    double speedFactor = 1.0; // 1.0=实时，2.0=两倍速，0.5=半速

    // 存储上一帧相机状态
    mjvCamera lastCam;
    mjv_defaultCamera(&lastCam); // 初始化为默认值

    // 获取初始相机状态
    mjvCamera *currentCam = controller.getCamera();
    memcpy(&lastCam, currentCam, sizeof(mjvCamera));




    /* =============================================================================================== */

    // 仿真循环
    while (!glfwWindowShouldClose(controller.window)) {

        // 获取当前真实时间
        double realTime = (glfwGetTime() - realTimeStart) * speedFactor;

        // data = controller.Data();

        // 执行物理仿真直到追赶上实时时钟
        while (simTime < realTime) {
            mj_step(model, data);

            // drag force = -c * v^2 * unit_vector(v); v = sqrt(vx^2 + vy^2 + vz^2)
            // vector(v) = [vx, vy, vz]
            // unit_vector(v) = vector(v) / v
            // fx = -c * v * vx;
            // fy = -c * v * vy;
            // fz = -c * v * vz;

            // double vx, vy, vz;
            // vx = data->qvel[0]; vy = data->qvel[1]; vz = data->qvel[2]; 

            // double v;
            // v = sqrt(vx*vx + vy*vy + vz*vz);

            // double fx, fy, fz;
            // double c = -0.1;
            // fx = c * v * vx;
            // fy = c * v * vy;
            // fz = c * v * vz;

            // data->qfrc_applied[0] = fx;
            // data->qfrc_applied[1] = fy;
            // data->qfrc_applied[2] = fz;

            simTime += model->opt.timestep;
        }

        // 更新相机跟随 (放在主循环中, 确保鼠标拖动流畅)
        controller.updateCameraLookAt(data->qpos[0]); // 使用新方法

        // 获取当前相机状态
        mjvCamera *cam = controller.getCamera();

        // 检查相机是否移动
        if (cameraChanged(cam, &lastCam)) {
            printf("%f, %f, %f, %f, %f, %f\n",
                   cam->azimuth, cam->elevation, cam->distance,
                   cam->lookat[0], cam->lookat[1], cam->lookat[2]);

            // 更新上一帧相机状态
            memcpy(&lastCam, cam, sizeof(mjvCamera));
        }

        // 渲染处理
        controller.render();

        currentStep++;

        // 如果达到稳定步数，或窗口关闭则退出
        if (currentStep >= stabilizationSteps) {
            break;
        }
    }

    // 资源释放
    mj_deleteData(data);
    mj_deleteModel(model);

    return 0;
}
