#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "TaskLS_ID.hpp"
#include "TaskLS_StationaryFeet.hpp"
#include "TaskLS_MoveBase.hpp"
#include "TaskLS_StationaryEE.hpp"
#include "TaskLS_MinMotion.hpp"
#include "TaskLS_EnergyOpt.hpp"
#include "TaskLS_TorqueLimits.hpp"
#include "TaskLS_FrictionCone.hpp"
#include "HOQP.hpp" 
#include "HOQP_Slack.hpp"
#include "Utils.hpp"

int main (int argc, char* argv[]) {
    raisim::World world;
    auto ground = world.addGround();
    raisim::Vec<3> gravity = world.getGravity();
    const double hz = 100;
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
    robot->setName("aliengo + Z1");
    auto sphere_body = world.addSphere(0.05, 1.0, "default",raisim::COLLISION(2), raisim::COLLISION(2));
    sphere_body->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_body->setAppearance("green");
    sphere_body->setPosition(raisim::Vec<3>{0.15, 0., 0.8});
 
    std::unique_ptr<TaskLS> task1 = std::make_unique<TaskLS_ID>(&world, robot, 12, 42);
    std::unique_ptr<TaskLS> task2 = std::make_unique<TaskLS_StationaryFeet>(&world, robot, 12, 42);
    std::unique_ptr<TaskLS> task3 = std::make_unique<TaskLS_StationaryEE>(&world, robot, 6, 42);
    std::unique_ptr<TaskLS> task4 = std::make_unique<TaskLS_MoveBase>(&world, robot, 6, 42);
    std::unique_ptr<TaskLS> task5 = std::make_unique<TaskLS_EnergyOpt>(&world, robot, 42);
    
    Eigen::VectorXd tau_min = -40*Eigen::VectorXd::Ones(18);
    Eigen::VectorXd tau_max = 40*Eigen::VectorXd::Ones(18);
    std::unique_ptr<TaskLS> constraints1 = std::make_unique<TaskLS_TorqueLimits>(tau_min, tau_max);
    std::unique_ptr<TaskLS> constraints2 = std::make_unique<TaskLS_FrictionCone>(&world, robot, 24,42);

    std::unique_ptr<TaskSet> task_set1 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set2 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set3 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set4 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set5 = std::make_unique<TaskSet>(42);
    task_set1->addEqualityTask(task1.get());
    task_set1->addInequalityTask(constraints1.get());
    task_set2->addEqualityTask(task2.get());
    task_set2->addInequalityTask(constraints2.get());
    task_set3->addEqualityTask(task3.get());
    task_set4->addEqualityTask(task4.get());
    task_set5->addEqualityTask(task5.get());

    std::unique_ptr<HOQP_Slack> hoqp = std::make_unique<HOQP_Slack>();

    hoqp->addTask(task_set1.get());
    hoqp->addTask(task_set2.get());
    hoqp->addTask(task_set3.get());
    hoqp->addTask(task_set4.get());
    hoqp->addTask(task_set5.get());
    hoqp->init();   

    Eigen::VectorXd solution_vector;
    Eigen::VectorXd tau;
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(robot->getDOF());
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);
    robot->printOutBodyNamesInOrder();
    // robot->printOutMovableJointNamesInOrder();
    for (int i=0; i<100000; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6)
        hoqp->solveAllTasks();
        solution_vector = hoqp->getSolution();
        tau = solution_vector.tail(18);
        generalizedForce.tail(18) = tau;
        robot->setGeneralizedForce(generalizedForce);
        // 10 seconds later, exert external force on the robot
        if(i>1000 && i<2000){
            robot->setExternalForce(18, {0,0,0}, {10, 0, -10});
        }
        
        server.integrateWorldThreadSafe();
    }
    server.killServer();

    return 0;
}
