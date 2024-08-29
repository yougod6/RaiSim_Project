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
#include "HOQP_Slack.hpp" 
#include "TaskSet.hpp"
#include <memory>

int main (int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World world;
    world.setTimeStep(0.01);
   
    auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\aliengo\\aliengo.urdf");
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim()),jointVelocityTarget(robot->getDOF());
    jointNominalConfig << 0.0, 0.0, 0.45, //base position
                          1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3,
                          0.0, 0.6, -1.3;
    robot->setComputeInverseDynamics(true);
    auto ground = world.addGround();

    robot->setGeneralizedCoordinate(jointNominalConfig);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
    robot->setName("aliengo");
    std::unique_ptr<TaskLS> task1 = std::make_unique<TaskLS_ID>(&world, robot);
    std::unique_ptr<TaskLS> task2 = std::make_unique<TaskLS_StationaryFeet>(&world, robot);
    std::unique_ptr<TaskLS> task3 = std::make_unique<TaskLS_MoveBase>(&world, robot);
    std::unique_ptr<TaskLS> task4 = std::make_unique<TaskLS_EnergyOpt>(&world, robot);
    
    Eigen::VectorXd tau_min = -40*Eigen::VectorXd::Ones(12);
    Eigen::VectorXd tau_max = 40*Eigen::VectorXd::Ones(12);
    std::unique_ptr<TaskLS> constraint1 = std::make_unique<TaskLS_TorqueLimits>(tau_min,tau_max);
    std::unique_ptr<TaskLS> constraint2 = std::make_unique<TaskLS_FrictionCone>(&world, robot);   
    int var_dim = 30;
    std::unique_ptr<TaskSet> task_set1 = std::make_unique<TaskSet>(var_dim);
    std::unique_ptr<TaskSet> task_set2 = std::make_unique<TaskSet>(var_dim);
    std::unique_ptr<TaskSet> task_set3 = std::make_unique<TaskSet>(var_dim);
    std::unique_ptr<TaskSet> task_set4 = std::make_unique<TaskSet>(var_dim);
    task_set1->addEqualityTask(task1.get());
    task_set2->addEqualityTask(task2.get());
    task_set3->addEqualityTask(task3.get());
    task_set4->addEqualityTask(task4.get());
    task_set2->addInequalityTask(constraint1.get());
    task_set3->addInequalityTask(constraint2.get());

    std::unique_ptr<HOQP_Slack> hoqp = std::make_unique<HOQP_Slack>();
    hoqp->addTask(task_set1.get());
    hoqp->addTask(task_set2.get());
    hoqp->addTask(task_set3.get());
    hoqp->addTask(task_set4.get());
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
    server.killServer();

    return 0;
}