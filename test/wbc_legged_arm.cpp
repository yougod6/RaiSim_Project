#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "TaskLS_ID.hpp"
#include "TaskLS_StationaryFeet.hpp"
#include "TaskLS_MoveBase.hpp"
#include "TaskLS_StationaryEE.hpp"
#include "TaskLS_MinMotion.hpp"
#include "TaskLS_EnergyOpt.hpp"
#include "HOQP.hpp" 
#include "HOQP_Slack.hpp"
#include "Utils.hpp"

int main (int argc, char* argv[]) {
    raisim::World world;
    auto ground = world.addGround();
    raisim::Vec<3> gravity = world.getGravity();
    const double hz = 400;
    world.setTimeStep(1/hz);
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\z1\\aliengo_z1.urdf");
    // robot->printOutBodyNamesInOrder();  
    // std::cout << "----------------- Frame Names -----------------" << std::endl;
    // robot->printOutFrameNamesInOrder();
    // std::cout << "Z1's DoF: " << robot->getDOF() << std::endl;
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim()),jointVelocityTarget(robot->getDOF());
    jointNominalConfig << 0.0, 0.0, 0.45, //base position
                          1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3,
                          0.0, 0.6, -1.3,
                          0, 0.7, -0.6, 0, 0, 0; // arm joints

    jointVelocityTarget.setZero();
    robot->setGeneralizedCoordinate(jointNominalConfig);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF())); 
    
    TaskLS *task1 = new TaskLS_ID(&world, robot, 12, 42);    
    TaskLS *task2 = new TaskLS_StationaryFeet(&world, robot, 12, 42);
    TaskLS *task3 = new TaskLS_StationaryEE(&world, robot, 6, 42);
    TaskLS *task4 = new TaskLS_MoveBase(&world, robot, 6, 42);
    TaskLS *task5 = new TaskLS_EnergyOpt(&world, robot, 42);
    
    HOQP* hoqp = new HOQP();
    hoqp->addTask(task1);
    hoqp->addTask(task2);
    hoqp->addTask(task3);
    hoqp->addTask(task4);
    hoqp->addTask(task5);
    hoqp->init();   
    hoqp->solveAllTasks(); 

    Eigen::VectorXd solution_vector;
    Eigen::VectorXd qddot;
    Eigen::VectorXd tau;
    Eigen::VectorXd g;
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(robot->getDOF());

    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);
    // robot->printOutMovableJointNamesInOrder();
    for (int i=0; i<100000; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6)

        hoqp->solveAllTasks();
        solution_vector = hoqp->getSolution();
        qddot = solution_vector.head(24);
        tau = solution_vector.tail(18);
        g = robot->getNonlinearities(gravity).e();

        generalizedForce.tail(18) = tau;
        robot->setGeneralizedForce(generalizedForce);
        server.integrateWorldThreadSafe();
    }
    server.killServer();

    return 0;
}
