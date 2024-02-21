#pragma once
#include "TaskLS.hpp"

class TaskLS_MoveBase: public TaskLS
{

    public:
        TaskLS_MoveBase(raisim::World* world, raisim::ArticulatedSystem* robot, const double kp=10, const double kd=(2*sqrt(10)));
        ~TaskLS_MoveBase();

        void update_dJ_B(const double dt=0.001);
        void update_J_B();
        void updateVector()override;
        void updateMatrix()override;
        //retrun desired_q_b, desired_dq_b 
        void makeBaseTrajectory(double time);
        //return desired_ddq_b by pd control
        void updateDesiredBaseAcceleration(const double kp, const double kd);
    
    private:
        raisim::World* world_;
        raisim::ArticulatedSystem* robot_;
        raisim::Vec<3> gravity_;
        double kp_;
        double kd_;
        
        Eigen::MatrixXd J_B_position;
        Eigen::MatrixXd J_B_rotation;
        Eigen::MatrixXd J_B_;
        Eigen::MatrixXd dJ_B_;
        Eigen::VectorXd dq_;

        Eigen::VectorXd desired_q_B_;
        Eigen::VectorXd desired_dq_B_;
        Eigen::VectorXd desired_ddq_B_;
};