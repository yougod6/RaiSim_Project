#pragma once    
#include "TaskLS.hpp"
#include "Utils.hpp"

class TaskLS_AngularMomentum: public TaskLS
{
    public:
        TaskLS_AngularMomentum(RobotState* robot_state, const int task_dim, const int var_dim, Eigen::Vector3d weight);
        ~TaskLS_AngularMomentum();

        void updateVector()override;
        void updateMatrix()override;
        void updateAngularMomentum();
        void QR_decomposition();

    private:
        RobotState* robot_state_;
        int dof_;
        int actuated_dof_;
        int contact_dim_;
        Eigen::MatrixXd M_;
        Eigen::VectorXd h_;
        Eigen::Vector3d g_;

        Eigen::MatrixXd J_c_FR_;
        Eigen::MatrixXd J_c_FL_;
        Eigen::MatrixXd J_c_RR_;
        Eigen::MatrixXd J_c_RL_;
        Eigen::MatrixXd J_c_;
        Eigen::MatrixXd S_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd Qc_;
        Eigen::MatrixXd R_;
        Eigen::VectorXd lambda_; // contact forces (stacked)
        Eigen::MatrixXd A_tmp_;

        std::vector<Eigen::Vector3d> contact_points_;
        Eigen::Vector3d r_c_FR_;
        Eigen::Vector3d r_c_FL_;
        Eigen::Vector3d r_c_RR_;
        Eigen::Vector3d r_c_RL_;
        Eigen::Vector3d CoM_position_;
        double CoM_mass_;
        Eigen::MatrixXd D_; // Matrix for sum of cross-product

        Eigen::Matrix3d weight_;
        Eigen::VectorXd desired_angular_momentum_;
        Eigen::VectorXd desired_angular_momentum_rate_;
        Eigen::VectorXd angular_momentum_;
};