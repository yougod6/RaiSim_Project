#pragma once

#include <Eigen/Dense>

class Utils {
    public:
        static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond q);
        static Eigen::Matrix3d skew(Eigen::Vector3d v);
};