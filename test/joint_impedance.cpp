#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

int main (int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World world;
    world.setTimeStep(0.0001);
    raisim::Vec<3> gravity = world.getGravity();
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");

    go1->setComputeInverseDynamics(true);
    auto ground = world.addGround();
    Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointVelocityTarget(go1->getDOF());

    jointNominalConfig << 0.0, 0.0, 0.54, //base position
                        1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        0.0, 0.6, -1.3, //
                        0.0, 0.6, -1.3,
                        0.0, 0.6, -1.3,
                        0.0, 0.6, -1.3;

    // jointNominalConfig << 0.0, 0.0, 0.32, //base position
    //                     0.7071068, 0.0, 0.0, 0.7071068, //base orientation(quaternion)
    //                     0.03, 0.1, -1.4, //
    //                     -0.03, 0.1, -1.4,
    //                     0.03, -0.1, 1.4,
    //                     -0.03, -0.1, 1.4;
    jointVelocityTarget.setZero();


    Eigen::MatrixXd jointPgain = Eigen::MatrixXd::Identity(12, 12);
    Eigen::MatrixXd jointDgain = Eigen::MatrixXd::Identity(12, 12);
    const double kp = 100.0, kd = 2*sqrt(kp);
    jointPgain*=kp;
    jointDgain*=kd;

    go1->setGeneralizedCoordinate(jointNominalConfig);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF())); 
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(go1->getDOF());

    generalizedForce.tail(12) = go1->getNonlinearities(gravity).e().tail(12);

    go1->setName("go1");

    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(go1);
    const int totalT = 1000000;
    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);
        Eigen::VectorXd actuated_toq = Eigen::VectorXd::Zero(12);
        raisim::Vec<3> base_position;
        go1->getFramePosition("floating_base", base_position);
        std::cout << "base_position: " << base_position.e().transpose() << std::endl;
        actuated_toq += go1->getNonlinearities(gravity).e().tail(12);
        actuated_toq += jointPgain*(jointNominalConfig.tail(12) - go1->getGeneralizedCoordinate().e().tail(12)) - jointDgain*(go1->getGeneralizedVelocity().e().tail(12));
        generalizedForce.tail(12) = actuated_toq;
        // std::cout << "generalizedForce: " << generalizedForce << std::endl;
        go1->setGeneralizedForce(generalizedForce);
        if(i>50000 && i<100000){
            go1->setExternalForce(3, {0,0,-0.1}, {15, 0, 0});
            go1->setExternalForce(2, {0,0,-0.1}, {15, 0, 0});
            go1->setExternalForce(0, {-0.05,0.1,0.05}, {1.5, -30., -10.});
        }
        
        if(i>150000 && i<200000){
            go1->setExternalForce(3, {0,0,0}, {-10, 0, -300});
            go1->setExternalForce(6, {0,0,0}, {-10, 0, -300});
            go1->setExternalForce(9, {0,0,0}, {100, 0, -300});
            go1->setExternalForce(12, {0,0,0}, {100, 0, -300});
        }

        server.integrateWorldThreadSafe();
    }

    return 0;
}