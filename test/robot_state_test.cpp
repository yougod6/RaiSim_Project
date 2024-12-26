#include "RobotStateRaisim.hpp"
#include "raisim/RaisimServer.hpp"
#include <iostream>

int main (int argc, char* argv[]) {
    raisim::World world;
    auto ground = world.addGround();
    const double hz = 100;
    world.setTimeStep(1/hz);
    ground->setAppearance("1,1,1,1");   

    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\z1\\aliengo_z1.urdf");

    std::unique_ptr<RobotStateRaisim> robot_state_ptr = std::make_unique<RobotStateRaisim>(&world, robot);
    int dof = robot_state_ptr->getDOF();
    int actuated_dof = robot_state_ptr->getActuatedDOF();
    int contact_dim = robot_state_ptr->getContactDim();
    std::cout << "DoFs : " << dof << std::endl;
    std::cout << "Actuated DoFs : " << actuated_dof << std::endl;
    std::cout << "Contact Dimensions : " << contact_dim << std::endl;
    std::cout << "Z1's DoF: " << robot->getDOF() << std::endl;
   
    Eigen::VectorXd q;
    Eigen::VectorXd u;
    Eigen::MatrixXd J_C;
    Eigen::MatrixXd J_B;
    Eigen::MatrixXd M;
    Eigen::VectorXd h;
    Eigen::VectorXd g;
    Eigen::Vector3d l; //linear momentum
    Eigen::Vector3d a; //angular momentum
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);
    for (int i=0; i<100000; i++) {
        RS_TIMED_LOOP(1e3)
        robot_state_ptr->updateState();
        q = robot_state_ptr->getGeneralizedCoordinates();
        u = robot_state_ptr->getGeneralizedVelocities();
        J_C = robot_state_ptr->getContactJacobian();
        J_B = robot_state_ptr->getBaseJacobian();
        M = robot_state_ptr->getMassMatrix();
        h = robot_state_ptr->getNonlinearVector();
        g = robot_state_ptr->getGravityVector();
        l = robot_state_ptr->getLinearMomentum();
        a = robot_state_ptr->getAngularMomentum();
        // std::cout << "Generalized Coordinates : " << q.transpose() << std::endl;
        // std::cout << "Generalized Velocities : " << u.transpose() << std::endl;
        // std::cout << "Contact Jacobian : " << J_C << std::endl;
        // std::cout << "Base Jacobian : " << J_B << std::endl;
        // std::cout << "Mass Matrix : " << M << std::endl;
        // std::cout << "Nonlinear Vector : " << h.transpose() << std::endl;
        // std::cout << "Gravity Vector : " << g.transpose() << std::endl;
        // std::cout << "Linear Momentum : " << l.transpose() << std::endl;
        // std::cout << "Angular Momentum : " << a.transpose() << std::endl;


        server.integrateWorldThreadSafe();
    }
    server.killServer();

    return 0;
}