#pragma once
#include "TaskLS.hpp"

class TaskLS_LinearMomentum: public TaskLS
{
    public:
        TaskLS_LinearMomentum(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim, const int var_dim, Eigen::Vector3d rate_weight, Eigen::Vector3d position_weight);
        ~TaskLS_LinearMomentum();

        void updateVector()override;
        void updateMatrix()override;
        void updateCoMPositionRate();
        void updeateDesiredCoMPosition();
        void QR_decomposition();

    private:
        raisim::World* world_;
        raisim::ArticulatedSystem* robot_;
        int dof_;
        int contact_dim_;
        raisim::Vec<3> gravity_;
        Eigen::MatrixXd M_;
        Eigen::VectorXd h_;

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
        double CoM_mass_;
};