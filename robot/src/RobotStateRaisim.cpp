#include "RobotStateRaisim.hpp"

RobotStateRaisim::RobotStateRaisim(raisim::World* world, raisim::ArticulatedSystem* robot)
    : world_(world), robot_(robot){
    if (!robot_) {
        throw std::runtime_error("Robot pointer is null.");
    }
    dof_ = robot_->getDOF();
    actuated_dof_ = dof_-6;
    contact_dim_ = 12;
    generalized_coordinates_ = Eigen::VectorXd::Zero(dof_);
    generalized_velocities_ = Eigen::VectorXd::Zero(dof_);
    generalized_accelerations_ = Eigen::VectorXd::Zero(dof_);
    generalized_forces_ = Eigen::VectorXd::Zero(dof_);
    
    J_C_FR_ = Eigen::MatrixXd::Zero(3,dof_);
    J_C_FL_ = Eigen::MatrixXd::Zero(3,dof_);
    J_C_RR_ = Eigen::MatrixXd::Zero(3,dof_);
    J_C_RL_ = Eigen::MatrixXd::Zero(3,dof_);
    J_C_ = Eigen::MatrixXd::Zero(contact_dim_,dof_);
    dJ_C_ = Eigen::MatrixXd::Zero(contact_dim_,dof_);
    J_B_position_ = Eigen::MatrixXd::Zero(3,dof_);
    J_B_orientation_ = Eigen::MatrixXd::Zero(3,dof_);
    J_B_ = Eigen::MatrixXd::Zero(6,dof_);
    dJ_B_ = Eigen::MatrixXd::Zero(6,dof_);
    J_EE_position_ = Eigen::MatrixXd::Zero(3,dof_);
    J_EE_orientation_ = Eigen::MatrixXd::Zero(3,dof_);
    J_EE_ = Eigen::MatrixXd::Zero(6,dof_);
    dJ_EE_ = Eigen::MatrixXd::Zero(6,dof_);

    M_ = Eigen::MatrixXd::Zero(dof_, dof_);
    h_ = Eigen::VectorXd::Zero(dof_);
    g_ = Eigen::VectorXd::Zero(dof_);
    linear_momentum_ = Eigen::Vector3d::Zero();
    angular_momentum_ = Eigen::Vector3d::Zero();
    
    auto mass_list = robot->getMass();
    for(auto mass : mass_list){
        total_mass_ += mass;
    }
}

RobotStateRaisim::~RobotStateRaisim() = default;

void RobotStateRaisim::updateState() {
    J_C_prev = J_C_;
    J_B_prev = J_B_;
    J_EE_prev = J_EE_;
    CoM_position_prev = CoM_position_;

    generalized_coordinates_ = robot_->getGeneralizedCoordinate().e();
    generalized_velocities_ = robot_->getGeneralizedVelocity().e();
    generalized_accelerations_ = robot_->getGeneralizedAcceleration().e();
    generalized_forces_ = robot_->getGeneralizedForce().e();

    robot_->getDenseFrameJacobian("FR_foot_fixed", J_C_FR_);
    robot_->getDenseFrameJacobian("FL_foot_fixed", J_C_FL_);
    robot_->getDenseFrameJacobian("RR_foot_fixed", J_C_RR_);
    robot_->getDenseFrameJacobian("RL_foot_fixed", J_C_RL_);
    J_C_.block(0, 0, 3, dof_) = J_C_FR_;
    J_C_.block(3, 0, 3, dof_) = J_C_FL_;
    J_C_.block(6, 0, 3, dof_) = J_C_RR_;
    J_C_.block(9, 0, 3, dof_) = J_C_RL_;
    dJ_C_ = (J_C_ - J_C_prev)/world_->getTimeStep();

    robot_->getDenseFrameJacobian("floating_base", J_B_position_);
    robot_->getDenseFrameRotationalJacobian("floating_base", J_B_orientation_);
    J_B_.block(0, 0, 3, dof_) = J_B_position_;
    J_B_.block(3, 0, 3, dof_) = J_B_orientation_;
    dJ_B_ = (J_B_ - J_B_prev)/world_->getTimeStep();

    raisim::Vec<3> base_position;
    raisim::Vec<4> base_quat;
    robot_->getBasePosition(base_position);
    robot_->getBaseOrientation(base_quat);
    base_position_ = base_position.e();
    base_quaternion_ = Eigen::Quaterniond(base_quat[0], base_quat[1], base_quat[2], base_quat[3]);

    robot_->getDenseFrameJacobian("joint6", J_EE_position_);
    robot_->getDenseFrameRotationalJacobian("joint6", J_EE_orientation_);
    J_EE_.block(0, 0, 3, dof_) = J_EE_position_;
    J_EE_.block(3, 0, 3, dof_) = J_EE_orientation_;
    dJ_EE_ = (J_EE_ - J_EE_prev)/world_->getTimeStep();

    raisim::Vec<3> ee_position = Eigen::VectorXd::Zero(3);
    robot_->getFramePosition("joint6",ee_position);
    ee_position_ = ee_position.e();

    raisim::Mat<3,3> wRee;
    robot_->getFrameOrientation("joint6",wRee);
    ee_quaternion_ = Eigen::Quaterniond(wRee.e());
    ee_quaternion_.normalize();

    raisim::Vec<3> ee_velocity;
    robot_->getFrameVelocity("joint6",ee_velocity);
    ee_velocity_ = ee_velocity.e();

    raisim::Vec<3> ee_angular_velocity;
    robot_->getFrameAngularVelocity("joint6",ee_angular_velocity);
    ee_angular_velocity_ = ee_angular_velocity.e();

    M_ = robot_->getMassMatrix().e();
    h_ = robot_->getNonlinearities(world_->getGravity()).e();
    g_ = world_->getGravity().e();


    auto CoM_position_tmp = robot_->getCompositeCOM();
    CoM_position_ = CoM_position_tmp[0].e();

    CoM_position_rate_ = (CoM_position_ - CoM_position_prev)/world_->getTimeStep();

    raisim::Vec<3> angular_momentum_tmp;
    robot_->getAngularMomentum(CoM_position_,angular_momentum_tmp);
    angular_momentum_ = angular_momentum_tmp.e();   

    auto contacts = robot_->getContacts();
    contact_points_.clear();
    contact_state_.clear();
    for(auto contact : contacts){
        auto idx = contact.getlocalBodyIndex();
        if(idx == 3){
            contact_points_.push_back(contact.getPosition().e()); 
            contact_state_.push_back(true);
        }  
        if(idx == 6){
            contact_points_.push_back(contact.getPosition().e()); 
            contact_state_.push_back(true);
        }  
        if(idx == 9){
            contact_points_.push_back(contact.getPosition().e()); 
            contact_state_.push_back(true);
        }  
        if(idx == 12){
            contact_points_.push_back(contact.getPosition().e()); 
            contact_state_.push_back(true);
        } 
    }
}
