#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "includeTest.hpp"

int main(int argc, char* argv[]) {
  includeTest();
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  world.setTimeStep(0.001);
  
  auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");
  auto ground = world.addGround();

  // go1 joint PD controller
  Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim());
  Eigen::VectorXd jointVelocityTarget(go1->getDOF());
  // std::cout << "getGeneralizedCoordinateDim(): " << go1->getGeneralizedCoordinateDim() << std::endl;
  // std::cout << "getDOF(): " << go1->getDOF() << std::endl;
  // std::cout << "getGeneralizedCoordinate()" << go1->getGeneralizedCoordinate() << std::endl;
  // std::cout << jointNominalConfig.size() << std::endl;  
  jointNominalConfig << 0.0, 0.0, 0.54, //base position
                        1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        0.03, 0.2, -1.2, //
                        -0.03, 0.2, -1.2,
                        0.03, -0.2, 1.2,
                        -0.03, -0.2, 1.2;
  jointVelocityTarget.setZero();
  std::cout << go1->getGeneralizedCoordinate().e().transpose() << std::endl;  

  for (int i=0; i<go1->getDOF(); i++)
    jointVelocityTarget[i] = (i+4)*0.01;

  Eigen::VectorXd jointPgain(go1->getDOF());
  Eigen::VectorXd jointDgain(go1->getDOF());
  const double kp = 350.0, kd = 2*sqrt(kp);
  jointPgain.tail(12).setConstant(kp);
  jointDgain.tail(12).setConstant(kd);

  go1->setGeneralizedCoordinate(jointNominalConfig);
  go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF())); 
  go1->setPdGains(jointPgain, jointDgain);
  go1->setPdTarget(jointNominalConfig, jointVelocityTarget);
  go1->setName("go1");
  
  // allow inverse dynamics computation
  go1->setComputeInverseDynamics(true);
  go1->printOutBodyNamesInOrder();
  go1->printOutFrameNamesInOrder();

  /// launch raisim server for visualization. Can be visualized using raisimUnity
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(go1);
  Eigen::VectorXd torqueFromInverseDynamics(go1->getDOF());
  const int totalT = 100000;
  for (int i=0; i<totalT; i++) {

    RS_TIMED_LOOP(world.getTimeStep()*1e6);
    std::vector<Eigen::Vector3d> axes(go1->getDOF()-6);

    // Index 0 is the base joint, so we start at 1
    for (int j=0; j<go1->getDOF()-6; j++)
      axes[j] = go1->getJointAxis(j+1).e();

    // print time every 1s
    // if(i%1000==0)
    //   std::cout << "time " << i*world.getTimeStep() << std::endl;
    
    if(i>5000 && i<10000){
      go1->setExternalForce(3, {0,0,-0.1}, {15, 0, 0});
      go1->setExternalForce(2, {0,0,-0.1}, {15, 0, 0});
      go1->setExternalForce(0, {-0.05,0.1,0.05}, {1.5, -30., -10.});
    }
   
    if(i>15000 && i<20000){
      go1->setExternalForce(3, {0,0,0}, {-10, 0, -300});
      go1->setExternalForce(6, {0,0,0}, {-10, 0, -300});
      go1->setExternalForce(9, {0,0,0}, {100, 0, -300});
      go1->setExternalForce(12, {0,0,0}, {100, 0, -300});
    }

    server.integrateWorldThreadSafe();
    /// retrieve force/torque acting at joints
    torqueFromInverseDynamics.head(3) = go1->getForceAtJointInWorldFrame(0).e();
    torqueFromInverseDynamics.segment<3>(3) = go1->getTorqueAtJointInWorldFrame(0).e();
    
    //j=1 ~ 12
    for (size_t j=1; j<go1->getDOF()-5; j++)
      torqueFromInverseDynamics(j+5) = go1->getTorqueAtJointInWorldFrame(j).dot(axes[j-1]);

  }

  server.killServer();
}
