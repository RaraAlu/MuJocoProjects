#include "pendulum_controller/pendulum_controller.h"
#include <experimental/filesystem>
#include <iostream>
#include <mujoco/mujoco.h>


namespace fs = std::experimental::filesystem;


int main()
{
    // 初始化MuJoCo
    char error[1000] = "Could not load binary model";

    // 获取根目录地址的上一级


    fs::path rootPath = fs::current_path().parent_path().parent_path();
    std::string project_path = rootPath.string() + "/src/inverted_pendulum/model/inverted_pendulum/pendulum.xml";
    std::cout << "Root Path: " << project_path << std::endl;

    // 加载模型
    mjModel *model = mj_loadXML(project_path.c_str(), nullptr, error, 1000);
    if (!model) {
        std::cerr << "Error loading model: " << error << std::endl;
        return -1;
    }
    mjData *data = mj_makeData(model);

    // 设置初始状态为倒立位置（倒立摆初始状态）
    data->qpos[1] = M_PI - 0.1; // 设置摆杆角度为略微偏离垂直向上位置，增加控制难度和可观察性
    data->qvel[1] = 0.0;        // 初始角速度为0
    data->qpos[0] = 0.0;        // 小车初始位置为0
    data->qvel[0] = 0.0;        // 小车初始速度为0


    // 初始化控制器
    PendulumController controller(model, data);

    // 仿真参数
    int stabilizationSteps = 300000; // 给系统足够时间稳定
    int currentStep = 0;

    // 仿真循环
    while (!glfwWindowShouldClose(controller.window)) {
        // 执行控制计算
        controller.computeControl();

        // 物理仿真步进
        mj_step(model, data);

        // 更新相机跟随 (放在主循环中, 确保鼠标拖动流畅)
        controller.updateCameraLookAt(data->qpos[0]); // 使用新方法

        // 渲染处理
        controller.render();

        // 打印当前状态（可选）
        if (currentStep % 100 == 0) {
            std::cout << "Step: " << currentStep
                      << ", Angle: " << data->qpos[1]
                      << ", Angle Velocity: " << data->qvel[1] << std::endl;
        }

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
