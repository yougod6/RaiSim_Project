#pragma once
#include "TaskLS.hpp"

class TaskLS_LinearMomentum: public TaskLS
{
    public:
        TaskLS_LinearMomentum(RobotState* robot_state_, const int task_dim, const int var_dim, Eigen::Vector3d rate_weight, Eigen::Vector3d position_weight);
        ~TaskLS_LinearMomentum();

        void updateVector()override;
        void updateMatrix()override;
        void updeateDesiredCoMPosition();
        void QR_decomposition();

    private:
        RobotState* robot_state_;
        int dof_;
        int actuated_dof_;
        int contact_dim_;
        double CoM_mass_;
        std::vector<Eigen::Vector3d> contact_points_;
        std::vector<bool> contact_state_;
        Eigen::MatrixXd M_;
        Eigen::VectorXd h_;
        Eigen::VectorXd g_;

        Eigen::MatrixXd J_c_FR_;
        Eigen::MatrixXd J_c_FL_;
        Eigen::MatrixXd J_c_RR_;
        Eigen::MatrixXd J_c_RL_;
        Eigen::MatrixXd J_c_;
        Eigen::MatrixXd S_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd Qc_;
        Eigen::MatrixXd R_;
        Eigen::MatrixXd A_tmp;
        Eigen::MatrixXd P_; // change 3(nc) x 1 vec to 3 x 1

        Eigen::Matrix3d rate_weight_;
        Eigen::Matrix3d position_weight_;

        Eigen::VectorXd desired_linear_momentum_rate_;
        Eigen::VectorXd desired_CoM_position_;
        Eigen::VectorXd CoM_position_;
        Eigen::VectorXd center_of_contacts_;
        Eigen::VectorXd desired_CoM_position_rate_;
        Eigen::VectorXd CoM_position_rate_;
};