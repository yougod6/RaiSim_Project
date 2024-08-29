#include "TaskLS_FrictionCone.hpp"

TaskLS_FrictionCone::TaskLS_FrictionCone(raisim::World* world, raisim::ArticulatedSystem* robot, int task_dim, int var_dim, double mu)
: TaskLS(task_dim,var_dim), world_(world), robot_(robot) ,mu_(mu)
{
    task_name_ = "Friction Cone Constraint";
    lambda_max_ = 1000.;
    lambda_min_ = 0.;
    dof_ = robot_->getDOF();
    S_ = Eigen::MatrixXd::Zero(dof_-6, dof_);
    S_.block(0,6,dof_-6,dof_-6) = Eigen::MatrixXd::Identity(dof_-6,dof_-6);

    M_ = Eigen::MatrixXd::Zero(dof_, dof_);
    h_ = Eigen::VectorXd::Zero(dof_);
    J_c_FR_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_FL_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_RR_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_RL_ = Eigen::MatrixXd::Zero(3, dof_);
    J_c_ = Eigen::MatrixXd::Zero(12, dof_);

    F_block_ = Eigen::MatrixXd::Zero(6, 3);
    F_block_ <<  1, 0, 0,
                -1, 0, 0,
                 0, 1, 0,
                 0,-1, 0,
                 0, 0, 1,
                 0, 0,-1;    

    F_ = Eigen::MatrixXd::Zero(task_dim_, 12);
    F_.block(0, 0, 6, 3) = F_block_;
    F_.block(6, 3, 6, 3) = F_block_;
    F_.block(12, 6, 6, 3) = F_block_;
    F_.block(18, 9, 6, 3) = F_block_;
}

TaskLS_FrictionCone::~TaskLS_FrictionCone(){}

void TaskLS_FrictionCone::updateMatrix(){
    M_ = robot_->getMassMatrix().e();
    gravity_ = world_->getGravity();
    h_ = robot_->getNonlinearities(gravity_).e();
    robot_->getDenseFrameJacobian("FR_foot_fixed", J_c_FR_);
    robot_->getDenseFrameJacobian("FL_foot_fixed", J_c_FL_);
    robot_->getDenseFrameJacobian("RR_foot_fixed", J_c_RR_);
    robot_->getDenseFrameJacobian("RL_foot_fixed", J_c_RL_);
    J_c_.block(0, 0, 3, dof_) = J_c_FR_;
    J_c_.block(3, 0, 3, dof_) = J_c_FL_;
    J_c_.block(6, 0, 3, dof_) = J_c_RR_;
    J_c_.block(9, 0, 3, dof_) = J_c_RL_;

    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_c_.transpose());
    Q_ = qr.householderQ();
    R_ = qr.matrixQR().triangularView<Eigen::Upper>();
    R_ = R_.block(0, 0, 12, 12);
    Qc_ = Q_.block(0, 0, dof_, 12);
    Eigen::MatrixXd E = Eigen::MatrixXd::Zero(dof_, dof_+(dof_-6));
    E.block(0, 0, dof_, dof_) = M_;
    E.block(0, dof_, dof_, dof_-6) = -S_.transpose();
    A_c_ = R_.inverse()*Qc_.transpose()*E;
    b_c_ = R_.inverse()*Qc_.transpose()*h_;
    Eigen::VectorXd u_dot = robot_->getGeneralizedAcceleration().e();  
    Eigen::VectorXd tau = robot_->getGeneralizedForce().e().tail(dof_-6);  
    Eigen::VectorXd x = Eigen::VectorXd::Zero(dof_+(dof_-6));
    x.head(dof_) = u_dot;
    x.tail(dof_-6) = tau;
    lambda_ = A_c_*x + b_c_;
    A_ = F_*A_c_;
}

void TaskLS_FrictionCone::updateVector(){
    
    Eigen::VectorXd k = Eigen::VectorXd::Zero(task_dim_);
    double cone=0.;
    for(size_t i=0; i<4; i++)
    {
        cone = mu_*lambda_(3*(i+1)-1);
        k.segment(6*i, 6) << mu_*lambda_(3*(i+1)-1),
                             mu_*lambda_(3*(i+1)-1),
                             mu_*lambda_(3*(i+1)-1),
                             mu_*lambda_(3*(i+1)-1),
                             lambda_max_, 
                             lambda_min_;
    }
    b_ = k-F_*b_c_;
}