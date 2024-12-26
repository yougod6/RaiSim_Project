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
#include "RobotStateRaisim.hpp"
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

    Eigen::VectorXd base_desired_x = Eigen::VectorXd::Zero(6);
    base_desired_x <<   -0.0490382 ,
                    0.00157048,
                    0.401518,// + amplitude*sin(2*M_PI*freq*time) ,
                    0,
                    0,
                    0;

    std::unique_ptr<RobotStateRaisim> robot_state_ptr = std::make_unique<RobotStateRaisim>(&world, robot);
    robot_state_ptr->updateState();
    std::unique_ptr<TaskLS> task1 = std::make_unique<TaskLS_ID>(robot_state_ptr.get());
    std::unique_ptr<TaskLS> task2 = std::make_unique<TaskLS_StationaryFeet>(robot_state_ptr.get());
    std::unique_ptr<TaskLS> task3 = std::make_unique<TaskLS_MoveBase>(robot_state_ptr.get(),6,30,base_desired_x);
    std::unique_ptr<TaskLS> task4 = std::make_unique<TaskLS_EnergyOpt>(robot_state_ptr.get());
    
    Eigen::VectorXd tau_min = -40*Eigen::VectorXd::Ones(12);
    Eigen::VectorXd tau_max = 40*Eigen::VectorXd::Ones(12);
    std::unique_ptr<TaskLS> constraint1 = std::make_unique<TaskLS_TorqueLimits>(tau_min,tau_max);
    std::unique_ptr<TaskLS> constraint2 = std::make_unique<TaskLS_FrictionCone>(robot_state_ptr.get());   
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
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-10.0, 10.0);
    std::uniform_real_distribution<double> x_distribution(-0.3, 0.3);
    std::uniform_real_distribution<double> y_distribution(-0.12, 0.12);
    std::uniform_real_distribution<double> z_distribution(-0.05, 0.05);
    double rand_x, rand_y, rand_z;
    double rand_x_posi, rand_y_posi, rand_z_posi;

    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);
        robot_state_ptr->updateState();
        hoqp->solveAllTasks();
        solution_vector = hoqp->getSolution();

        tau = solution_vector.tail(12);
        generalizedForce.tail(12) = tau;
        robot->setGeneralizedForce(generalizedForce);

        if(i%500==0){
            rand_x = distribution(generator);
            rand_y = distribution(generator);
            rand_z = distribution(generator);
            rand_x_posi = x_distribution(generator);
            rand_y_posi = y_distribution(generator);
            rand_z_posi = z_distribution(generator);
        }

        if(i>500){
            robot->setExternalForce(0, {rand_x_posi,rand_y_posi,rand_z_posi}, {rand_x, rand_y, rand_z});
        }


        server.integrateWorldThreadSafe();
    }    
    server.killServer();

    return 0;
}