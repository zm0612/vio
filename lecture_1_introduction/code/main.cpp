#include <Eigen/Dense>

#include <iostream>
#include <cmath>

using namespace std;
using namespace Eigen;

Matrix3d SkewSymmetricMatrix(const Vector3d &vec) {
    Matrix3d matrix;
    matrix << 0, -vec(2), vec(1),
            vec(2), 0, -vec(0),
            -vec(1), vec(0), 0;

    return matrix;
}

Matrix3d RodriguesFormula(const Vector3d &omega) {
    const double theta = omega.norm();
    const Vector3d n = omega.normalized();

    Matrix3d R;
    R = std::cos(theta) * Matrix3d::Identity()
        + (1 - std::cos(theta)) * n * n.transpose()
        + std::sin(theta) * SkewSymmetricMatrix(n);

    return R;
}

int main() {
    const Vector3d OMEGA = {0.03, 0.06, 0.09};
    cout << "theta: " << OMEGA.norm() / M_PI * 360.0 << " degree" << endl;
    cout << "n " << OMEGA.normalized().transpose() << endl << endl;

    const Matrix3d R = Matrix3d::Identity();
    const Quaterniond q = Quaterniond::Identity();

    const Matrix3d updated_R = R * RodriguesFormula(OMEGA);

    const Vector3d imaginary_part = 0.5 * OMEGA;
    const Quaterniond delta_q = Quaterniond(1, imaginary_part[0], imaginary_part[1],
                                            imaginary_part[2]).normalized();
    const Quaterniond updated_q = q * delta_q;

    const AngleAxisd omega_angle_axis(OMEGA.norm(), OMEGA.normalized());
    const Quaterniond delta_completed_q(omega_angle_axis);
    const Quaterniond updated_completed_q = q * delta_completed_q;

    cout << "  R " << endl;
    cout << R << endl << endl;

    cout << "            updated R       " << endl;
    cout << updated_R << endl << endl;

    cout << "            updated q          " << endl;
    cout << updated_q.toRotationMatrix() << endl << endl;

    cout << "        updated q completely      " << endl;
    cout << updated_completed_q.toRotationMatrix() << endl;

    return 0;
}
