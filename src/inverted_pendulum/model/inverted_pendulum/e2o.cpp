#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

// 使用常量表达式定义圆周率
constexpr double PI = 3.14159265358979323846;

// 欧拉角类（单位：度）
class EulerAngle {
public:

    double roll = 0.0;  // X轴旋转
    double pitch = 0.0; // Y轴旋转
    double yaw = 0.0;   // Z轴旋转

    // 方便初始化的构造函数
    constexpr EulerAngle(double r = 0.0, double p = 0.0, double y = 0.0)
        : roll(r), pitch(p), yaw(y) {}

    // 用于比较的运算符
    bool operator==(const EulerAngle &other) const
    {
        constexpr double epsilon = 1e-6;
        return std::abs(roll - other.roll) < epsilon &&
               std::abs(pitch - other.pitch) < epsilon &&
               std::abs(yaw - other.yaw) < epsilon;
    }

    bool operator!=(const EulerAngle &other) const
    {
        return !(*this == other);
    }
};

// 四元数类
class Quaternion {
public:

    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

    // 方便初始化的构造函数
    constexpr Quaternion(double w_ = 1.0, double x_ = 0.0, double y_ = 0.0, double z_ = 0.0)
        : w(w_), x(x_), y(y_), z(z_) {}

    // 计算四元数的范数
    [[nodiscard]] double norm() const
    {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    // 归一化并返回新的四元数
    [[nodiscard]] Quaternion normalized() const
    {
        double n = norm();
        if (n > 1e-10) {
            return {w / n, x / n, y / n, z / n};
        }
        return {1.0, 0.0, 0.0, 0.0}; // 默认单位四元数
    }

    // 原地归一化并返回自身引用（方便链式调用）
    Quaternion &normalize()
    {
        double n = norm();
        if (n > 1e-10) {
            w /= n;
            x /= n;
            y /= n;
            z /= n;
        }
        return *this;
    }

    // 四元数乘法
    [[nodiscard]] Quaternion operator*(const Quaternion &q) const
    {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w};
    }

    // 共轭四元数
    [[nodiscard]] Quaternion conjugate() const
    {
        return {w, -x, -y, -z};
    }

    // 用于比较的运算符
    bool operator==(const Quaternion &other) const
    {
        constexpr double epsilon = 1e-6;
        return std::abs(w - other.w) < epsilon &&
               std::abs(x - other.x) < epsilon &&
               std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon;
    }

    bool operator!=(const Quaternion &other) const
    {
        return !(*this == other);
    }
};

// 将角度与弧度转换的辅助函数
constexpr double degreesToRadians(double degrees)
{
    return degrees * PI / 180.0;
}

constexpr double radiansToDegrees(double radians)
{
    return radians * 180.0 / PI;
}

// 旋转类 - 同时支持四元数和欧拉角表示
class Rotation {
public:

    // 默认构造函数
    Rotation() = default;

    // 通过欧拉角构造 (RPY顺序: X-Y-Z)
    explicit Rotation(const EulerAngle &euler)
    {
        setEuler(euler);
    }

    // 通过欧拉角值构造 (RPY顺序: X-Y-Z)
    Rotation(double roll, double pitch, double yaw)
    {
        setEuler(roll, pitch, yaw);
    }

    // 通过四元数构造
    explicit Rotation(const Quaternion &quat)
    {
        setQuaternion(quat);
    }

    // 通过四元数值构造
    Rotation(double w, double x, double y, double z)
    {
        setQuaternion(w, x, y, z);
    }

    // 打印四元数
    void printQuaternion() const
    {
        std::cout << "四元数: ["
                  << std::fixed << std::setprecision(6)
                  << m_quat.w << " " << m_quat.x << " "
                  << m_quat.y << " " << m_quat.z << "]\n";
    }

    // 打印欧拉角
    void printEuler() const
    {
        std::cout << "欧拉角 (度):\n"
                  << std::fixed << std::setprecision(2)
                  << "Roll (X): " << m_euler.roll << "°\n"
                  << "Pitch (Y): " << m_euler.pitch << "°\n"
                  << "Yaw (Z): " << m_euler.yaw << "°\n";
    }

    // 打印完整信息
    void print(const std::string &label = "") const
    {
        if (!label.empty()) {
            std::cout << "===== " << label << " =====" << std::endl;
        }
        printEuler();
        printQuaternion();
        std::cout << std::endl;
    }

    // 获取四元数
    [[nodiscard]] const Quaternion &getQuaternion() const
    {
        return m_quat;
    }

    // 获取欧拉角
    [[nodiscard]] const EulerAngle &getEuler() const
    {
        return m_euler;
    }

    // 设置欧拉角
    void setEuler(double roll, double pitch, double yaw)
    {
        setEuler(EulerAngle(roll, pitch, yaw));
    }

    // 设置欧拉角（使用EulerAngle对象）
    void setEuler(const EulerAngle &euler)
    {
        m_euler = euler;
        m_quat = eulerToQuaternion();
    }

    // 设置四元数
    void setQuaternion(double w, double x, double y, double z)
    {
        setQuaternion(Quaternion(w, x, y, z));
    }

    // 设置四元数（使用Quaternion对象）
    void setQuaternion(const Quaternion &quat)
    {
        m_quat = quat;
        m_quat.normalize();
        m_euler = quaternionToEuler();
    }

    // 组合旋转（右乘）
    Rotation &operator*=(const Rotation &other)
    {
        setQuaternion(m_quat * other.m_quat);
        return *this;
    }

    // 组合旋转（返回新对象）
    [[nodiscard]] Rotation operator*(const Rotation &other) const
    {
        Rotation result(*this);
        result *= other;
        return result;
    }

    // 检查是否接近相等
    bool isApprox(const Rotation &other, double epsilon = 1e-6) const
    {
        // 四元数可能有符号差异但表示相同旋转
        const Quaternion &q1 = m_quat;
        const Quaternion &q2 = other.m_quat;

        // 检查q1和q2或q1和-q2是否足够相近
        bool directMatch =
            std::abs(q1.w - q2.w) < epsilon &&
            std::abs(q1.x - q2.x) < epsilon &&
            std::abs(q1.y - q2.y) < epsilon &&
            std::abs(q1.z - q2.z) < epsilon;

        bool inverseMatch =
            std::abs(q1.w + q2.w) < epsilon &&
            std::abs(q1.x + q2.x) < epsilon &&
            std::abs(q1.y + q2.y) < epsilon &&
            std::abs(q1.z + q2.z) < epsilon;

        return directMatch || inverseMatch;
    }

private:

    // 成员变量
    EulerAngle m_euler;
    Quaternion m_quat{1.0, 0.0, 0.0, 0.0}; // 初始为单位四元数

    // 欧拉角转四元数（X-Y-Z顺序，即RPY顺序）
    [[nodiscard]] Quaternion eulerToQuaternion() const
    {
        // 转换为弧度
        double roll = degreesToRadians(m_euler.roll);
        double pitch = degreesToRadians(m_euler.pitch);
        double yaw = degreesToRadians(m_euler.yaw);

        // 计算半角的正弦和余弦值
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);

        // 四元数乘法顺序：X * Y * Z (RPY)
        return Quaternion(
                   cr * cp * cy + sr * sp * sy,
                   sr * cp * cy - cr * sp * sy,
                   cr * sp * cy + sr * cp * sy,
                   cr * cp * sy - sr * sp * cy)
            .normalize(); // 确保归一化
    }

    // 四元数转欧拉角（X-Y-Z顺序，即RPY顺序）
    [[nodiscard]] EulerAngle quaternionToEuler() const
    {
        EulerAngle euler;

        // 使用临时变量方便计算
        double w = m_quat.w;
        double x = m_quat.x;
        double y = m_quat.y;
        double z = m_quat.z;

        // 计算X轴旋转（roll）
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        euler.roll = radiansToDegrees(std::atan2(sinr_cosp, cosr_cosp));

        // 计算Y轴旋转（pitch）- 检查奇点（万向锁）
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            euler.pitch = radiansToDegrees(std::copysign(PI / 2, sinp)); // 处理±90度情况
        else
            euler.pitch = radiansToDegrees(std::asin(sinp));

        // 计算Z轴旋转（yaw）
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        euler.yaw = radiansToDegrees(std::atan2(siny_cosp, cosy_cosp));

        return euler;
    }
};

// 测试辅助函数
void runAllTests()
{
    std::cout << "\n===== 开始旋转测试 =====\n"
              << std::endl;

    // 测试1: 通过欧拉角构造 (只旋转Y轴30度)
    Rotation r1(0.0, 90.0, 0.0);
    r1.print("测试1: 通过欧拉角构造 (只旋转Y轴30度)");

    // // 测试2: 通过欧拉角构造 (X和Y轴同时旋转30度)
    // Rotation r2(30.0, 30.0, 0.0);
    // r2.print("测试2: 通过欧拉角构造 (X和Y轴同时旋转30度)");

    // // 测试3: 通过四元数构造 (等价于绕Y轴旋转30度)
    // Rotation r3(0.9659, 0.0, 0.2588, 0.0);
    // r3.print("测试3: 通过四元数构造 (等价于绕Y轴旋转30度)");

    // // 测试4: 修改已有旋转
    // Rotation r4;
    // r4.print("测试4: 初始状态 (无旋转)");

    // r4.setEuler(30.0, 45.0, 60.0);
    // r4.print("修改欧拉角后 (RPY: 30,45,60)");

    // // 使用高级特性获取四元数
    // const Quaternion &q = r4.getQuaternion();
    // std::cout << "当前四元数: w=" << q.w << ", x=" << q.x
    //           << ", y=" << q.y << ", z=" << q.z << std::endl
    //           << std::endl;

    // r4.setQuaternion(0.7071, 0.7071, 0.0, 0.0);
    // r4.print("直接修改为新四元数 (绕X轴旋转90度)");

    // // 测试5: 组合旋转
    // Rotation rx(90.0, 0.0, 0.0); // 绕X轴旋转90度
    // Rotation ry(0.0, 90.0, 0.0); // 绕Y轴旋转90度

    // rx.print("绕X轴旋转90度");
    // ry.print("绕Y轴旋转90度");

    // // 按照先X后Y的顺序组合旋转
    // Rotation combined = rx * ry;
    // combined.print("组合旋转 (先X后Y)");

    // // 测试6: 欧拉角和四元数转换的一致性
    // std::cout << "\n===== 测试6: 欧拉角和四元数转换一致性 =====" << std::endl;

    // // 创建一些随机旋转进行测试
    // std::array<EulerAngle, 3> testAngles = {
    //     EulerAngle(30, 45, 60),
    //     EulerAngle(180, 0, 0), // 绕X轴180度
    //     EulerAngle(0, 90, 90)  // 绕Y轴90度再绕Z轴90度
    // };

    // for (const auto &euler : testAngles) {
    //     Rotation r1(euler);
    //     Quaternion q = r1.getQuaternion();
    //     Rotation r2(q);

    //     std::cout << "原始欧拉角: Roll=" << euler.roll
    //               << ", Pitch=" << euler.pitch
    //               << ", Yaw=" << euler.yaw << std::endl;

    //     std::cout << "四元数转换: w=" << q.w << ", x=" << q.x
    //               << ", y=" << q.y << ", z=" << q.z << std::endl;

    //     std::cout << "重新计算的欧拉角: Roll=" << r2.getEuler().roll
    //               << ", Pitch=" << r2.getEuler().pitch
    //               << ", Yaw=" << r2.getEuler().yaw << std::endl;

    //     std::cout << "转换一致性: "
    //               << (r1.isApprox(r2) ? "通过✓" : "失败✗") << std::endl
    //               << std::endl;
    // }

    std::cout << "===== 旋转测试完成 =====" << std::endl;
}

int main()
{
    // 运行所有测试
    runAllTests();

    return 0;
}
