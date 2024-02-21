#pragma once
#include "TaskLS.hpp"

class TaskLS_StationaryFeet: public TaskLS
{

    public:
        TaskLS_StationaryFeet(raisim::World* world, raisim::ArticulatedSystem* robot);
        ~TaskLS_StationaryFeet();

        void update_dJ_c(const double dt=0.001);
        void update_J_c();
        void updateVector()override;
        void updateMatrix()override;


    private:
        raisim::World* world_;
        raisim::ArticulatedSystem* robot_;
        raisim::Vec<3> gravity_;
        
        Eigen::MatrixXd J_c_FR_;
        Eigen::MatrixXd J_c_FL_;
        Eigen::MatrixXd J_c_RR_;
        Eigen::MatrixXd J_c_RL_;
        Eigen::MatrixXd J_c_;
        Eigen::MatrixXd dJ_c_;
        Eigen::VectorXd dq_;
};
