#pragma once
#include "TaskLS.hpp"

class TaskLS_ID: public TaskLS
{
    public:
        TaskLS_ID(RobotState* robot_state, const int task_dim=6, const int var_dim=30);
        ~TaskLS_ID();
        void updateVector()override;
        void updateMatrix()override;
        void QR_decomposition();
    private:
        RobotState* robot_state_;
        int dof_;
        int actuated_dof_;
        int contact_dim_;
        Eigen::MatrixXd M_;
        Eigen::VectorXd h_;
        
        Eigen::MatrixXd J_c_FR_;
        Eigen::MatrixXd J_c_FL_;
        Eigen::MatrixXd J_c_RR_;
        Eigen::MatrixXd J_c_RL_;
        Eigen::MatrixXd J_c_;
        Eigen::MatrixXd S_;
        Eigen::MatrixXd Sc_;
        Eigen::MatrixXd Su_;
        Eigen::MatrixXd Qc_;
        Eigen::MatrixXd Qu_;


};