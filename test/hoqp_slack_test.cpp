#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "TaskLS_ID.hpp"
#include "TaskLS_StationaryFeet.hpp"
#include "TaskLS_MoveBase.hpp"
#include "TaskLS_StationaryEE.hpp"
#include "TaskLS_MinMotion.hpp"
#include "TaskLS_EnergyOpt.hpp"
#include "HOQP_Slack.hpp" 
#include <memory>

int main (int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World world;
    world.setTimeStep(0.01);
    auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");

    robot->setComputeInverseDynamics(true);
    auto ground = world.addGround();
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim());

    jointNominalConfig << 0.0, 0.0, 0.42, //base position
                        1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        0.0, 0.6, -1.3, //
                        0.0, 0.6, -1.3,
                        0.0, 0.6, -1.3,
                        0.0, 0.6, -1.3;

    robot->setGeneralizedCoordinate(jointNominalConfig);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
    robot->setName("go1");

    std::unique_ptr<TaskLS> task1 = std::make_unique<TaskLS_ID>(&world, robot);
    std::unique_ptr<TaskLS> task2 = std::make_unique<TaskLS_StationaryFeet>(&world, robot);
    std::unique_ptr<TaskLS> task3 = std::make_unique<TaskLS_MoveBase>(&world, robot);
    std::unique_ptr<TaskLS> task4 = std::make_unique<TaskLS_EnergyOpt>(&world, robot);
    std::unique_ptr<HOQP_Slack> hoqp = std::make_unique<HOQP_Slack>();
    hoqp->addTask(task1.get());
    hoqp->addTask(task2.get());
    hoqp->addTask(task3.get());
    hoqp->addTask(task4.get());
    hoqp->init();
    // hoqp->solveAllTasks();
    Eigen::VectorXd solution_vector;
    Eigen::VectorXd qddot;
    Eigen::VectorXd tau;

    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);
    const int totalT = 1000000;
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(robot->getDOF());
    
    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);
      
        hoqp->solveAllTasks();
        solution_vector = hoqp->getSolution();

        tau = solution_vector.tail(12);
        generalizedForce.tail(12) = tau;
        robot->setGeneralizedForce(generalizedForce);

        server.integrateWorldThreadSafe();
    }    

    return 0;
}