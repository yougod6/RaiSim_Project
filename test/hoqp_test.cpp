#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "TaskLS_ID.hpp"
#include "TaskLS_StationaryFeet.hpp"
#include "TaskLS_MoveBase.hpp"
#include "TaskLS_EnergyOpt.hpp"
#include "HOQP.hpp" 
#include <memory>


int main (int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World world;
    world.setTimeStep(0.01);
    raisim::Vec<3> gravity = world.getGravity();
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");

    go1->setComputeInverseDynamics(true);
    auto ground = world.addGround();
    Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointVelocityTarget(go1->getDOF());

    jointNominalConfig << 0.0, 0.0, 0.42, //base position
                        1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        0.0, 0.6, -1.3, //
                        0.0, 0.6, -1.3,
                        0.0, 0.6, -1.3,
                        0.0, 0.6, -1.3;

    jointVelocityTarget.setZero();

    go1->setGeneralizedCoordinate(jointNominalConfig);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF())); 
    go1->setName("go1");

    go1->printOutFrameNamesInOrder();

    Eigen::VectorXd base_desired_x = Eigen::VectorXd::Zero(6);
    base_desired_x <<   -0.0490382 ,
                    0.00157048,
                    0.401518,// + amplitude*sin(2*M_PI*freq*time) ,
                    0,
                    0,
                    0;

    // std::cout << "generalizedCoordinate : " <<go1->getGeneralizedCoordinate().e().size() <<std::endl;
    // std::cout << "generalizedVelocity : " <<go1->getGeneralizedVelocity().e().size() <<std::endl;
    std::unique_ptr<TaskLS> task1 = std::make_unique<TaskLS_ID>(&world, go1);
    std::unique_ptr<TaskLS> task2 = std::make_unique<TaskLS_StationaryFeet>(&world, go1);
    std::unique_ptr<TaskLS> task3 = std::make_unique<TaskLS_MoveBase>(&world, go1,6,30,base_desired_x);
    std::unique_ptr<TaskLS> task4 = std::make_unique<TaskLS_EnergyOpt>(&world, go1);
    std::unique_ptr<HOQP> hoqp = std::make_unique<HOQP>();
    hoqp->addTask(task1.get());
    hoqp->addTask(task2.get());
    hoqp->addTask(task3.get());
    hoqp->addTask(task4.get());
    Eigen::VectorXd solution_vector;
    Eigen::VectorXd qddot;
    Eigen::VectorXd tau;
    Eigen::VectorXd g;
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(go1);
    const int totalT = 1000000;
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(go1->getDOF());
    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);
      
        hoqp->solveAllTasks();
        solution_vector = hoqp->getSolution();
       
        qddot = solution_vector.head(18);
        
        tau = solution_vector.tail(12);
        g = go1->getNonlinearities(gravity).e();

        generalizedForce.tail(12) = tau;
        go1->setGeneralizedForce(generalizedForce);
        
        server.integrateWorldThreadSafe();

    }

    return 0;
}