#include "Utils.hpp"

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond q)
{
    Eigen::Vector3d euler;

    Eigen::Matrix<double, 4, 1> coeff = q.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double t0 = 2.0 * (w * x + y * z);
    double t1 = 1.0 - 2.0 * (x * x + y*y);

    euler[0] = atan2(t0, t1);

    double t2 = 2.0 * (w * y - z * x);
    if(t2 > 1.0)
        t2 = 1.0;
    if(t2 < -1.0)
        t2= -1.0;

    euler[1] = asin(t2);

    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (y*y + z * z);
    euler[2] = atan2(t3, t4);
    return euler;
}

Eigen::Matrix3d Utils::skew(Eigen::Vector3d v)
{
    Eigen::Matrix3d skew;
    skew << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return skew;
}