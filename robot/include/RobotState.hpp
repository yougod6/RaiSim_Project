#pragma once

#include <Eigen/Dense>

/**
 * @brief Abstract base class for robot state representation.
 * This class provides a common interface for kinematic and dynamic terms.
 * Derived classes (e.g., RobotStateRaisim) must implement the updateState() method.
 */
class RobotState {
public:
    RobotState();
    ~RobotState();

    // Pure virtual function to update the robot state (should be implemented by derived classes)
    virtual void updateState() = 0;

    int getDOF() const;
    int getActuatedDOF() const;
    int getContactDim() const;
    double getTotalMass() const;

    // Getters for kinematic terms
    Eigen::VectorXd getGeneralizedCoordinates() const;
    Eigen::VectorXd getGeneralizedVelocities() const;
    Eigen::VectorXd getGeneralizedAccelerations() const;
    Eigen::VectorXd getGeneralizedForces() const;
    Eigen::MatrixXd getContactJacobian() const;
    Eigen::MatrixXd getBaseJacobian() const;
    Eigen::Vector3d getBasePosition() const;
    Eigen::Quaterniond getBaseQuaternion() const;
    Eigen::MatrixXd getEEJacobian() const;
    Eigen::MatrixXd getEEJacobianRate() const;
    Eigen::Vector3d getEEPosition() const;
    Eigen::Quaterniond getEEQuaternion() const;
    Eigen::Vector3d getEELinearVelocity() const;
    Eigen::Vector3d getEEAngularVelocity() const;
    Eigen::MatrixXd getContactJacobianRate() const;
    Eigen::MatrixXd getBaseJacobianRate() const;

    // Getters for dynamic terms
    Eigen::MatrixXd getMassMatrix() const;
    Eigen::VectorXd getNonlinearVector() const; // Coriolis, Centrifugal terms
    Eigen::VectorXd getGravityVector() const;

    Eigen::Vector3d getCompositeCoM() const;
    Eigen::Vector3d getCompositeCoMRate() const;
    Eigen::Vector3d getLinearMomentum() const;
    Eigen::Vector3d getAngularMomentum() const;
    std::vector<Eigen::Vector3d> getContactPoints() const;
    std::vector<bool> getContactState() const;

protected:
    // Member variables for derived classes to use
    int dof_;
    int actuated_dof_;
    int contact_dim_;   // 3x[number of contact points]. (point contact)
    double total_mass_;
    Eigen::VectorXd generalized_coordinates_;   // Generalized joint positions
    Eigen::VectorXd generalized_velocities_; // Generalized joint velocities
    Eigen::VectorXd generalized_accelerations_; // Generalized joint accelerations
    Eigen::VectorXd generalized_forces_; // Generalized joint forces
    Eigen::MatrixXd J_C_FR_;
    Eigen::MatrixXd J_C_FL_;
    Eigen::MatrixXd J_C_RR_;
    Eigen::MatrixXd J_C_RL_;
    Eigen::MatrixXd J_C_;   // Stacked Contact Jacobian 
    Eigen::MatrixXd J_C_prev; // Previous Contact Jacobian  
    Eigen::MatrixXd dJ_C_;  // time-derivative of J_C_
    Eigen::Vector3d base_position_; 
    Eigen::Quaterniond base_quaternion_;
    Eigen::MatrixXd J_B_position_;
    Eigen::MatrixXd J_B_orientation_;
    Eigen::MatrixXd J_B_;   
    Eigen::MatrixXd J_B_prev; // Previous Base Jacobian   
    Eigen::MatrixXd dJ_B_;  // time-derivative of J_B_

    Eigen::MatrixXd J_EE_position_;
    Eigen::MatrixXd J_EE_orientation_;
    Eigen::MatrixXd J_EE_;
    Eigen::MatrixXd J_EE_prev;
    Eigen::MatrixXd dJ_EE_;
    Eigen::Vector3d ee_position_;
    Eigen::Quaterniond ee_quaternion_; 
    Eigen::Vector3d ee_velocity_;
    Eigen::Vector3d ee_angular_velocity_;

    Eigen::MatrixXd M_;      // Mass/inertia matrix
    Eigen::VectorXd h_;  // Coriolis, Centrifigual 
    Eigen::VectorXd g_;   // Gravity forces
    Eigen::Vector3d linear_momentum_;
    Eigen::Vector3d angular_momentum_;

    Eigen::Vector3d CoM_position_;
    Eigen::Vector3d CoM_position_prev;
    Eigen::Vector3d CoM_position_rate_;
    std::vector<Eigen::Vector3d> contact_points_;
    std::vector<bool> contact_state_; // true if contact, false otherwise (FR, FL, RR, RL)
};