#include "TaskLS_LinearMomentum.hpp"

TaskLS_LinearMomentum::TaskLS_LinearMomentum(raisim::World* world, raisim::ArticulatedSystem* robot, const int task_dim, const int var_dim, Eigen::Vector3d rate_weight, Eigen::Vector3d position_weight)
: TaskLS(task_dim, var_dim), world_(world), robot_(robot)
{
    task_name_ = "Linear Momentum";
    rate_weight_ = rate_weight.asDiagonal();
    position_weight_ = position_weight.asDiagonal();    
    contact_dim_ = 12;
    gravity_ = world_->getGravity();
    dof_ = robot_->getDOF();

    M_ = Eigen::MatrixXd::Zero(dof_,dof_);
    h_ = Eigen::VectorXd::Zero(dof_);

    J_c_FR_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_FL_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_RR_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_RL_ = Eigen::MatrixXd::Zero(3,dof_);
    J_c_ = Eigen::MatrixXd::Zero(12,dof_);

    S_ = Eigen::MatrixXd::Zero(dof_-6, dof_);
    P_ = Eigen::MatrixXd::Zero(3,contact_dim_);
    for(int i=0; i<(contact_dim_/3); i++){
        P_.block(0,3*i,3,3) = Eigen::MatrixXd::Identity(3,3);
    }
    CoM_mass_=0;
    auto mass_list = robot_->getMass();  
    for(auto mass : mass_list){
        CoM_mass_ += mass;
    }
    desired_linear_momentum_rate_ = Eigen::VectorXd::Zero(3);
    desired_CoM_position_ = Eigen::VectorXd::Zero(3);
    desired_CoM_position_rate_ = Eigen::VectorXd::Zero(3);
    center_of_contacts_ = Eigen::VectorXd::Zero(3);
    center_of_contacts_ << -0.277321, 0., 0.625;
}

TaskLS_LinearMomentum::~TaskLS_LinearMomentum(){}


void TaskLS_LinearMomentum::updateMatrix(){
    M_ = robot_->getMassMatrix().e();
    
    robot_->getDenseFrameJacobian("FR_foot_fixed", J_c_FR_); // 3x18
    robot_->getDenseFrameJacobian("FL_foot_fixed", J_c_FL_); // 3x18
    robot_->getDenseFrameJacobian("RR_foot_fixed", J_c_RR_); // 3x18
    robot_->getDenseFrameJacobian("RL_foot_fixed", J_c_RL_); // 3x18

    J_c_.block(0, 0, 3, dof_) = J_c_FR_;
    J_c_.block(3, 0, 3, dof_) = J_c_FL_;
    J_c_.block(6, 0, 3, dof_) = J_c_RR_;
    J_c_.block(9, 0, 3, dof_) = J_c_RL_;
    
    QR_decomposition();

    A_tmp = Eigen::MatrixXd::Zero(dof_, var_dim_);
    A_tmp.block(0, 0, dof_, dof_) = M_;
    A_tmp.block(0, dof_, dof_, dof_-6) = -S_.transpose();
    A_ = P_*R_.inverse()*Qc_.transpose()*A_tmp;
}

void TaskLS_LinearMomentum::updateVector(){
    auto CoM_position_tmp = robot_->getCompositeCOM();
    CoM_position_ = CoM_position_tmp[0].e();
    updateCoMPositionRate();
    updeateDesiredCoMPosition();
    desired_linear_momentum_rate_ = rate_weight_*(desired_CoM_position_rate_ - CoM_position_rate_) + position_weight_*(desired_CoM_position_ - CoM_position_);
    desired_linear_momentum_rate_ *= CoM_mass_;
    b_ = desired_linear_momentum_rate_ -(P_*R_.inverse()*Qc_.transpose()*(robot_->getNonlinearities(gravity_).e()) + CoM_mass_*gravity_.e());
    // std::cout << "desired_linear_momentum_rate_ : " << desired_linear_momentum_rate_.transpose() << std::endl;
}

void TaskLS_LinearMomentum::updateCoMPositionRate(){
    auto CoM_position = CoM_position_;
    CoM_position_ = robot_->getCompositeCOM()[0].e();
    CoM_position_rate_ = (CoM_position_ - CoM_position)/world_->getTimeStep();
}

void TaskLS_LinearMomentum::updeateDesiredCoMPosition(){
    auto contacts = robot_->getContacts();
    Eigen::VectorXd center_of_contacts_tmp = Eigen::VectorXd::Zero(3);
    std::vector<bool> contact_feet = {false,false,false,false};//FR, FL, RR, RL
    for(auto contact : contacts){
        auto idx = contact.getlocalBodyIndex();
        if(idx == 3) {
            contact_feet[0] = true;
            center_of_contacts_tmp += contact.getPosition().e();
            }
        if(idx == 6) {
            contact_feet[1] = true;
            center_of_contacts_tmp += contact.getPosition().e();
            }
        if(idx == 9){
            contact_feet[2] = true;
            center_of_contacts_tmp += contact.getPosition().e();
        }
        if(idx == 12){
            contact_feet[3] = true;
            center_of_contacts_tmp += contact.getPosition().e();
        }
    }
    if(contact_feet[0] && contact_feet[1] && contact_feet[2] && contact_feet[3]){
        center_of_contacts_ = center_of_contacts_tmp/4;
    }
    desired_CoM_position_(0) = center_of_contacts_(0);
    desired_CoM_position_(1) = center_of_contacts_(1);
    desired_CoM_position_(2) = CoM_position_(2);
    // std::cout << "desired_CoM_position_ : " << desired_CoM_position_.transpose() << std::endl;
}

void TaskLS_LinearMomentum::QR_decomposition(){
    //QR Decomposition
    Eigen::MatrixXd J_c_T = J_c_.transpose();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_c_T);
    Q_ = qr.householderQ();
    R_ = qr.matrixQR().triangularView<Eigen::Upper>();
    R_ = R_.block(0, 0, contact_dim_, contact_dim_);
    Qc_ = Q_.block(0, 0, dof_, 12);
}
