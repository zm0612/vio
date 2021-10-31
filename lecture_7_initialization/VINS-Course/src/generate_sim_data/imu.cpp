//
// Created by hyj on 18-1-19.
//

#include <random>
#include <utility>
#include "generate_sim_data/imu.h"
#include "generate_sim_data/utilities.h"

/*!
 * 将Body frame -> Inertial frame的欧拉角转换成旋转矩阵
 * @param eulerAngles
 * @return
 */
Eigen::Matrix3d euler2Rotation(Eigen::Vector3d eulerAngles) {
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb << cy * cp, cy * sp * sr - sy * cr, sy * sr + cy * cr * sp,
            sy * cp, cy * cr + sy * sr * sp, sp * sy * cr - cy * sr,
            -sp, cp * sr, cp * cr;
    return RIb;
}

/*!
 * Inertial frame下的欧拉角速度转换到Body frame下
 * @param eulerAngles
 * @return
 */
Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles) {
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);

    Eigen::Matrix3d R;
    R << 1, 0, -sp,
            0, cr, sr * cp,
            0, -sr, cr * cp;

    return R;
}

IMU::IMU(Param p) : param_(std::move(p)) {
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}

/*!
 * 给IMU真实数据增加噪声，增加bias和高斯白噪声，另外还更新bias
 * @param data
 */
void IMU::addIMUnoise(MotionData &data) {
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_gyro(noise(generator_), noise(generator_), noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();

    // 这里应用了高斯白噪声离散时的公式推算.  IMU的误差考虑了bias和高斯白噪声
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt(param_.imu_timestep) + gyro_bias_;

    Eigen::Vector3d noise_acc(noise(generator_), noise(generator_), noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt(param_.imu_timestep) + acc_bias_;

    // gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_), noise(generator_), noise(generator_));
    gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep) * noise_gyro_bias;
    data.imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_), noise(generator_), noise(generator_));
    acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep) * noise_acc_bias;
    data.imu_acc_bias = acc_bias_;
}

/*!
 * 根据时间t求对应时刻的位置
 * @param t
 * @return
 */
MotionData IMU::MotionModel(double t) {

    MotionData data;
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;// z轴做sin运动
    float K1 = 10;// z轴的正弦频率是x，y的k1倍
    float K = M_PI / 10;// 20 * K = 2pi 由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    // translation
    // twb:  body frame in world frame
    Eigen::Vector3d position(ellipse_x * cos(K * t) + 5,
                             ellipse_y * sin(K * t) + 5,
                             z * sin(K1 * K * t) + 5);//位移

    Eigen::Vector3d dp(-K * ellipse_x * sin(K * t),
                       K * ellipse_y * cos(K * t),
                       z * K1 * K * cos(K1 * K * t));//速度　in world frame
    double K2 = K * K;
    Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t),
                        -K2 * ellipse_y * sin(K * t),
                        -z * K1 * K1 * K2 * sin(K1 * K * t));//加速度

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    //roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAngles(k_roll * cos(t), k_pitch * sin(t), K * t);
    // euler angles 的导数
    Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t), k_pitch * cos(t), K);

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);// body frame to world frame
    //euler rates trans to body gyro
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;

    //gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d gn(0, 0, -9.81);
    Eigen::Vector3d imu_acc = Rwb.transpose() * (ddp - gn);  //Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t;
    return data;
}

//读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::string src, const std::string &dist) {
    std::vector<MotionData> imu_data;
    LoadPose(std::move(src), imu_data);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;// position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);// quaternion double:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;// velocity  :   from imu measurements
    Eigen::Vector3d gw(0, 0, -9.81);// ENU frame
    for (unsigned int i = 1; i < imu_data.size(); ++i) {
        MotionData imu_pose_curr = imu_data[i];

//#define USE_EULER
#ifdef USE_EULER
        /// imu 动力学模型 欧拉积分
        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half = imu_pose_curr.imu_gyro * dt / 2.0;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
        dq.normalize();

        Eigen::Vector3d acc_w = Qwb * (imu_pose_curr.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
        Qwb = Qwb * dq;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
        Vw = Vw + acc_w * dt;
#else
        /// 中值积分
        MotionData imu_pose_last = imu_data[i - 1];
        Eigen::Vector3d omega = 0.5 * (imu_pose_last.imu_gyro + imu_pose_curr.imu_gyro);
        Eigen::Vector3d dtheta_half = omega * dt / 2.0;
        Eigen::Quaterniond dq;
        dq.w() = 1.0;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
        dq.normalize();

        Eigen::Vector3d acc_w_last = Qwb * imu_pose_last.imu_acc + gw;
        Qwb = Qwb * dq;
        Eigen::Vector3d acc_w_curr = Qwb * imu_pose_curr.imu_acc + gw;
        Eigen::Vector3d acc_mid_w = 0.5 * (acc_w_last + acc_w_curr);
        Eigen::Vector3d velocity_last_w = Vw;
        Vw = Vw + acc_mid_w * dt;
        Eigen::Vector3d velocity_mid_w = 0.5 * (velocity_last_w + Vw);
        Pwb = Pwb + velocity_mid_w * dt + 0.5 * acc_mid_w * dt * dt;
#endif
        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points << imu_pose_curr.timestamp << " "
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << std::endl;
    }
    save_points.close();

    std::cout << "test　end" << std::endl;
}