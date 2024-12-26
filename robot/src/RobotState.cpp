#include "RobotState.hpp"

RobotState::RobotState() 
    : dof_(0), 
      actuated_dof_(0), 
      contact_dim_(0), 
      total_mass_(0.0), 
      generalized_coordinates_(Eigen::VectorXd::Zero(0)), 
      generalized_velocities_(Eigen::VectorXd::Zero(0)), 
      generalized_accelerations_(Eigen::VectorXd::Zero(0)), 
      generalized_forces_(Eigen::VectorXd::Zero(0)) {}

      
RobotState::~RobotState() = default;

int RobotState::getDOF() const {
    return dof_;
}

int RobotState::getActuatedDOF() const {
    return actuated_dof_;
}

int RobotState::getContactDim() const {
    return contact_dim_;
}

double RobotState::getTotalMass() const {
    return total_mass_;
}

Eigen::VectorXd RobotState::getGeneralizedCoordinates() const {
    return generalized_coordinates_;
}

Eigen::VectorXd RobotState::getGeneralizedVelocities() const {
    return generalized_velocities_;
}

Eigen::VectorXd RobotState::getGeneralizedAccelerations() const {
    return generalized_accelerations_;
}

Eigen::VectorXd RobotState::getGeneralizedForces() const {
    return generalized_forces_;
}

Eigen::MatrixXd RobotState::getContactJacobian() const {
    return J_C_;
}

Eigen::MatrixXd RobotState::getBaseJacobian() const {
    return J_B_;
}

Eigen::Vector3d RobotState::getBasePosition() const {
    return base_position_;
}

Eigen::Quaterniond RobotState::getBaseQuaternion() const {
    return base_quaternion_;
}

Eigen::MatrixXd RobotState::getEEJacobian() const {
    return J_EE_;
}

Eigen::MatrixXd RobotState::getEEJacobianRate() const {
    return dJ_EE_;
}

Eigen::Vector3d RobotState::getEEPosition() const {
    return ee_position_;
}

Eigen::Quaterniond RobotState::getEEQuaternion() const {
    return ee_quaternion_;
}

Eigen::Vector3d RobotState::getEELinearVelocity() const {
    return ee_velocity_;
}

Eigen::Vector3d RobotState::getEEAngularVelocity() const {
    return ee_angular_velocity_;
}

Eigen::MatrixXd RobotState::getContactJacobianRate() const {
    return dJ_C_;
}

Eigen::MatrixXd RobotState::getBaseJacobianRate() const {
    return dJ_B_;
}

Eigen::MatrixXd RobotState::getMassMatrix() const {
    return M_;
}

Eigen::VectorXd RobotState::getNonlinearVector() const {
    return h_;
}

Eigen::VectorXd RobotState::getGravityVector() const {
    return g_;
}

Eigen::Vector3d RobotState::getLinearMomentum() const {
    return linear_momentum_;
}

Eigen::Vector3d RobotState::getAngularMomentum() const {
    return angular_momentum_;
}

Eigen::Vector3d RobotState::getCompositeCoM() const {
    return CoM_position_;
}

Eigen::Vector3d RobotState::getCompositeCoMRate() const {
    return CoM_position_rate_;
}

std::vector<Eigen::Vector3d> RobotState::getContactPoints() const {
    return contact_points_;
}

std::vector<bool> RobotState::getContactState() const {
    return contact_state_;
}
