#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include "TaskLS.hpp"
#include "TaskLS_ID.hpp"
#include "TaskLS_StationaryFeet.hpp"
#include "TaskLS_MoveBase.hpp"
#include "HOQP.hpp" 


int main (int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World world;
    world.setTimeStep(0.001);
    raisim::Vec<3> gravity = world.getGravity();
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");

    go1->setComputeInverseDynamics(true);
    auto ground = world.addGround();
    Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointVelocityTarget(go1->getDOF());

    jointNominalConfig << 0.0, 0.0, 0.40, //base position
                        1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        0.03, 0.2, -1.2, //
                        -0.03, 0.2, -1.2,
                        0.03, -0.2, 1.2,
                        -0.03, -0.2, 1.2;
    jointVelocityTarget.setZero();

    Eigen::VectorXd jointPgain(go1->getDOF()), jointDgain(go1->getDOF());
    double kp = 100;
    double kd = 2*sqrt(kp);
    jointPgain.tail(12).setConstant(kp);
    jointDgain.tail(12).setConstant(kd);

    // go1->setGeneralizedCoordinate(jointNominalConfig);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF())); 
    go1->setName("go1");

    go1->printOutFrameNamesInOrder();
    // std::cout << "generalizedCoordinate : " <<go1->getGeneralizedCoordinate().e().size() <<std::endl;
    // std::cout << "generalizedVelocity : " <<go1->getGeneralizedVelocity().e().size() <<std::endl;
    TaskLS *task1 = new TaskLS_ID(&world, go1);
    TaskLS *task2 = new TaskLS_StationaryFeet(&world, go1);
    TaskLS *task3 = new TaskLS_MoveBase(&world, go1);
    HOQP* hoqp = new HOQP();
    hoqp->addTask(task1);
    // hoqp->addTask(task2);
    hoqp->addTask(task3);

    hoqp->init();
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(go1);
    const int totalT = 1000000;
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(go1->getDOF());
    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);
      
        hoqp->updateAllTasks();
        hoqp->solveAllTasks();
        Eigen::VectorXd solution_vector = hoqp->getSolution();
       
        Eigen::VectorXd qddot = solution_vector.head(18);
        Eigen::VectorXd tau = solution_vector.tail(12);
        Eigen::VectorXd g = go1->getNonlinearities(gravity).e();

        std::cout << "qddot : " << qddot.transpose() << std::endl;
        std::cout << "tau : " << tau.transpose() << std::endl;
    
        generalizedForce.tail(12) = tau;

        // generalizedForce.tail(12) = tau;
        // generalizedForce.tail(12) += base_toq.tail(12);
        // std::cout << "torque : " << generalizedForce.tail(12).transpose() << std::endl;

        go1->setGeneralizedForce(generalizedForce);
        
        server.integrateWorldThreadSafe();

    }

    return 0;
}