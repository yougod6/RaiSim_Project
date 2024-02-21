#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
// #include "osqp.h"
#include "InverseDynamicsQPSolver.hpp"

int main(int argc, char* argv[]) {

    auto binaryPath = raisim::Path::setFromArgv(argv[0]);

    raisim::World world;
    world.setTimeStep(0.001);
    raisim::Vec<3> gravity = world.getGravity();
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");
    go1->setComputeInverseDynamics(true);
    
    auto ground = world.addGround();

    Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim());

    jointNominalConfig << 0.0, 0.0, 0.54, //base position
                        1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        0.03, 0.2, -1.2, //
                        -0.03, 0.2, -1.2,
                        0.03, -0.2, 1.2,
                        -0.03, -0.2, 1.2;

    go1->setGeneralizedCoordinate(jointNominalConfig);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF())); 
    go1->setName("go1");
    go1->printOutFrameNamesInOrder();
    Eigen::VectorXd genco, genvel;
    
    // go1->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

    InverseDynamicsQPSolver solver(&world, go1);
    Eigen::SparseMatrix<double> P;
    Eigen::VectorXd q;
    Eigen::SparseMatrix<double> A;
    Eigen::VectorXd lb(12);
    Eigen::VectorXd ub(12);
    solver.init(P, q, A, lb, ub,false);
    
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(go1);
    // Eigen::VectorXd torqueFromInverseDynamics(go1->getDOF());
    Eigen::VectorXd actuatedTorque(12);
    Eigen::VectorXd generalizedForce(go1->getDOF());
    const int totalT = 100000;
    for (int i=0; i<totalT; i++) {

        RS_TIMED_LOOP(world.getTimeStep()*1e6);

        //update robot state
        go1->setState(go1->getGeneralizedCoordinate().e(), go1->getGeneralizedVelocity().e());
        go1->getState(genco,genvel); 
        std::cout << "go1 generalized coordinate :" << std::endl << genco.transpose() << std::endl;
        std::cout << "go1 generalized velocity :" << std::endl << genvel.transpose() << std::endl;
        solver.updateVectors(q, lb, ub);
        solver.updateMatrices(P, A);
        solver.solve();

        actuatedTorque = solver.getSolution().tail(12);

        generalizedForce.tail(12) = actuatedTorque;
        

        go1->setGeneralizedForce(generalizedForce.dot(go1->getGeneralizedVelocity().e()));

        // print time every 1s
        if(i%1000==0)
          std::cout << "time " << i*world.getTimeStep() << std::endl;
        

        server.integrateWorldThreadSafe();
    }

    server.killServer();
}
