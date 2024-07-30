#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "Utils.hpp"

int main (int argc, char* argv[]) {
    raisim::World world;
    auto ground = world.addGround();
    const double hz = 200;
    world.setTimeStep(1/hz);
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    // auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go2\\go2.urdf");
    // auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\b1\\b1.urdf");
    // auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\b2\\urdf\\b2_z1.urdf");
    auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\z1\\aliengo_z1.urdf");
    // auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\z1\\go1z1.urdf");
    // auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\kinova\\urdf\\kinova.urdf");
    robot->printOutBodyNamesInOrder();  
    std::cout << "Z1's DoF: " << robot->getDOF() << std::endl;
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim()),jointVelocityTarget(robot->getDOF());
    jointNominalConfig << 0.0, 0.0, 0.42, //base position
                          1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3,
                          0.0, 0.6, -1.3,
                          0, 0.7, -0.6, 0, 0, 0; // arm joints

    jointVelocityTarget.setZero();
    
    Eigen::VectorXd jointPgain(robot->getDOF()), jointDgain(robot->getDOF());
    jointPgain.tail(18).setConstant(100.0);
    jointDgain.tail(18).setConstant(1.0);
    robot->setGeneralizedCoordinate(jointNominalConfig);
    robot->setPdGains(jointPgain, jointDgain);
    robot->setPdTarget(jointNominalConfig, jointVelocityTarget);

    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);

    raisim::Vec<3> ee_posi;
    raisim::Mat<3,3> ee_rot;
    raisim::Vec<3> base_posi;
    raisim::Vec<4> base_quat;

    for (int i=0; i<100000; i++) {
        RS_TIMED_LOOP(1e3)
        robot->getFramePosition("joint6",ee_posi);  
        robot->getFrameOrientation("joint6",ee_rot);
        Eigen::Quaterniond ee_quat(ee_rot.e());
        Eigen::Vector3d ee_euler = Utils::quat_to_euler(ee_quat);   
        std::cout << "EE Position: \n" << ee_posi.e() << std::endl;
        std::cout << "EE Euler : \n" << ee_euler << std::endl;
        robot->getBasePosition(base_posi);
        robot->getBaseOrientation(base_quat);
        Eigen::Quaterniond base_quat_e(base_quat[0], base_quat[1], base_quat[2], base_quat[3]);
        Eigen::Vector3d base_euler = Utils::quat_to_euler(base_quat_e);
        std::cout << "Base Position: \n" << base_posi.e() << std::endl;
        std::cout << "Base Euler : \n" << base_euler << std::endl;

        server.integrateWorldThreadSafe();

    }
    server.killServer();

    return 0;
}
