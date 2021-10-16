//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct Pose {
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};

int main() {

    int poseNums = 10;
    double radius = 8;
    std::vector<Pose> camera_pose;
    for (int n = 0; n < poseNums; ++n) {
        double theta = n * 2 * M_PI / (poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius,
                                            radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.emplace_back(R, t);
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

#define ADD_MEASURE_NOISE
#ifdef ADD_MEASURE_NOISE
    std::normal_distribution<double> uv_rand(0., 1 / 1000.0);
#endif

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x / z, y / z);
#ifdef ADD_MEASURE_NOISE
        camera_pose[i].uv[0] += uv_rand(generator);
        camera_pose[i].uv[1] += uv_rand(generator);
#endif
    }

    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */
    Eigen::MatrixXd D;
    const unsigned int N = camera_pose.size() - start_frame_id;
    D.resize(2 * N, 4);
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        const Eigen::Matrix3d &R = camera_pose[i].Rwc.transpose();
        const Eigen::Vector3d &t = -R * camera_pose[i].twc;
        const double &u = camera_pose[i].uv[0];
        const double &v = camera_pose[i].uv[1];
        Eigen::RowVector4d T_0;
        T_0 << R.block(0, 0, 1, 3), t[0];
        Eigen::RowVector4d T_1;
        T_1 << R.block(1, 0, 1, 3), t[1];
        Eigen::RowVector4d T_2;
        T_2 << R.block(2, 0, 1, 3), t[2];

        D.block(2 * (i - start_frame_id), 0, 1, 4) = u * T_2 - T_0;
        D.block(2 * (i - start_frame_id) + 1, 0, 1, 4) = v * T_2 - T_1;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::MatrixXd &V = svd.matrixV();
    P_est = (V.col(3) / V(3, 3)).head(3);
    /* your code end */

    std::cout << "ground truth: \n" << Pw.transpose() << std::endl;
    std::cout << "\nyour result: \n" << P_est.transpose() << std::endl;
    std::cout << "\neigen value: \n" << svd.singularValues().eval().transpose() << std::endl;
    std::cout << "\nerror(norm): " << (P_est - Pw).norm() << std::endl;
    std::cout << "ratio: " << svd.singularValues()[3] / svd.singularValues()[2] << std::endl;
    // TODO:: 请如课程讲解中提到的判断三角化结果好坏的方式，绘制奇异值比值变化曲线
    return 0;
}
