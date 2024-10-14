#pragma once

#include <Eigen/Dense>

class Utils {
    public:
        static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond q);
        static Eigen::Quaterniond euler_to_quat(Eigen::Vector3d euler);
        static Eigen::Vector3d box_minus_operator(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
        static Eigen::Matrix3d skew(Eigen::Vector3d v);
        static Eigen::MatrixXd compute_nullspace_QR(const Eigen::MatrixXd& A);
        static void write_label_to_csv(std::string filename, std::vector<std::string> labels);
        static void write_data_to_csv(std::string& filename, double time, Eigen::VectorXd& data, bool append=true);
        static void write_data_to_csv(std::string& filename, std::vector<double>& time, std::vector<Eigen::VectorXd>& data, bool append=true);
        static std::string get_current_date_time();
};