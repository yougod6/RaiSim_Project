#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include "TaskLS.hpp"
#include "TaskLS_ID.hpp"
#include "TaskLS_StationaryFeet.hpp"
#include "TaskLS_MoveBase.hpp"
#include "HOQP.hpp" 

Eigen::MatrixXd moor_penrose_pseudo_inverse(Eigen::MatrixXd A) {
    Eigen::MatrixXd At = A.transpose();
    return At*((A*At).inverse());
}

Eigen::VectorXd make_base_trajectory(const double time){
    const double amplitude = 0.05; //0.15
    const double freq = 0.5;

    Eigen::VectorXd desired_q_B_ = Eigen::VectorXd::Zero(6);
    // desired_q_B_ << -0.00270028 + amplitude*sin(2*M_PI*freq*time),
    //                 0.000455424 ,
    //                 0.345183,
    //                 0,0,0.7071068;

    desired_q_B_ << -0.00270028 + amplitude*sin(2*M_PI*freq*time),
                    0.000455424 ,
                    0.32,
                    0,0,0.7071068;
    return desired_q_B_;
}
// optimization variable is the joint acceleration
// solution of qp prblem is used to compute the joint torque

int main (int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World world;
    world.setTimeStep(0.001); //1kHz
    raisim::Vec<3> gravity = world.getGravity();
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");
    
    auto sphere_body = world.addSphere(0.1, 1.0, "default",raisim::COLLISION(2), raisim::COLLISION(2));
    sphere_body->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_body->setAppearance("green");
    sphere_body->setPosition(raisim::Vec<3>{-0.00270028, 0.000455424, 0.345183});
    
    auto ground = world.addGround();
    
    Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointVelocityTarget(go1->getDOF());
    
    jointNominalConfig << 0.0, 0.0, 0.36, //base position
                        0.7071068, 0.0, 0.0, 0.7071068, //base orientation(quaternion)
                        0.03, 0.2, -1.2, //
                        -0.03, 0.2, -1.2,
                        0.03, -0.2, 1.2,
                        -0.03, -0.2, 1.2;

    jointVelocityTarget.setZero();

    go1->setGeneralizedCoordinate(jointNominalConfig);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF())); 
    
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(go1->getDOF());
    Eigen::MatrixXd J_c_FR = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_FL = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_RR = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_RL = Eigen::MatrixXd::Zero(3, go1->getDOF());

    Eigen::MatrixXd J_c_ = Eigen::MatrixXd::Zero(12, go1->getDOF());
    Eigen::MatrixXd J_c_prev = Eigen::MatrixXd::Zero(12, go1->getDOF()); 
    Eigen::MatrixXd J_c_dot = Eigen::MatrixXd::Zero(12, go1->getDOF()); 
    
    Eigen::MatrixXd J_B_p = Eigen::MatrixXd::Zero(3, go1->getDOF()); 
    Eigen::MatrixXd J_B_r = Eigen::MatrixXd::Zero(3, go1->getDOF()); 
    Eigen::MatrixXd J_B = Eigen::MatrixXd::Zero(6, go1->getDOF()); 
    Eigen::MatrixXd J_B_prev = Eigen::MatrixXd::Zero(6, go1->getDOF()); 
    Eigen::MatrixXd J_B_dot = Eigen::MatrixXd::Zero(6, go1->getDOF()); 
    
    Eigen::MatrixXd Su= Eigen::MatrixXd::Zero(6,18); //6x18
    Su.block(0,12,6,6) = Eigen::MatrixXd::Identity(6,6);   
    
    Eigen::MatrixXd Sc = Eigen::MatrixXd::Zero(12, go1->getDOF()); //12x18
    Sc.block(0,0,12,12) = Eigen::MatrixXd::Identity(12,12);
    
    // Selection Matrix
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(go1->getDOF()-6, go1->getDOF()); //12x18
    S.block(0,6,12,12) = Eigen::MatrixXd::Identity(go1->getDOF()-6,go1->getDOF()-6); 
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(go1);

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(go1->getDOF(), go1->getDOF());
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(go1->getDOF());
    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(go1->getDOF());
    Eigen::VectorXd h = go1->getNonlinearities(gravity).e();

    raisim::Vec<3> base_position;
    raisim::Vec<4> base_quat;

    Eigen::VectorXd base_pose = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd desired_base_pose = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd desired_qddot = Eigen::VectorXd::Zero(18);
    const int Kp_base_position = 100;
    const int Kp_base_orientation = 100;
    const int Kd_base_position = 2*sqrt(Kp_base_position);
    const int Kd_base_orientation = 2*sqrt(Kp_base_orientation);
    
    Eigen::MatrixXd Kp_base = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd Kd_base = Eigen::MatrixXd::Zero(6,6);
    Kp_base.block(0,0,3,3) = Kp_base_position*Eigen::MatrixXd::Identity(3,3);
    Kp_base.block(3,3,3,3) = Kp_base_orientation*Eigen::MatrixXd::Identity(3,3);
    Kd_base.block(0,0,3,3) = Kd_base_position*Eigen::MatrixXd::Identity(3,3);
    Kd_base.block(3,3,3,3) = Kd_base_orientation*Eigen::MatrixXd::Identity(3,3);
    
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);

    TaskLS *task1 = new TaskLS_StationaryFeet(&world, go1);
    TaskLS *task2 = new TaskLS_MoveBase(&world, go1);

    HOQP* hoqp = new HOQP();
    hoqp->addTask(task1);
    // hoqp->addTask(task2);
    hoqp->init();

    const int totalT = 1000000;
    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);
        
        go1->getDenseFrameJacobian("FR_foot_fixed", J_c_FR); // 3x18    
        go1->getDenseFrameJacobian("FL_foot_fixed", J_c_FL); // 3x18
        go1->getDenseFrameJacobian("RR_foot_fixed", J_c_RR); // 3x18
        go1->getDenseFrameJacobian("RL_foot_fixed", J_c_RL); // 3x18

        J_c_.block(0, 0, 3, go1->getDOF()) = J_c_FR;
        J_c_.block(3, 0, 3, go1->getDOF()) = J_c_FL;
        J_c_.block(6, 0, 3, go1->getDOF()) = J_c_RR;
        J_c_.block(9, 0, 3, go1->getDOF()) = J_c_RL;

        go1->getDenseFrameJacobian("floating_base", J_B_p); // 3x18
        go1->getDenseFrameRotationalJacobian("floating_base", J_B_r); // 3x18
        J_B.block(0, 0, 3, go1->getDOF()) = J_B_p;
        J_B.block(3, 0, 3, go1->getDOF()) = J_B_r;
        
        // Get Dynamics
        M = go1->getMassMatrix().e();
        qdot = go1->getGeneralizedVelocity().e();
        qddot = go1->getGeneralizedAcceleration().e();
        h = go1->getNonlinearities(gravity).e();

        // Get Base Pose
        go1->getBasePosition(base_position);
        go1->getBaseOrientation(base_quat);
        // Update Base Pose
        base_pose.head(3) = base_position.e();
        base_pose.tail(3) = base_quat.e().tail(3);

        // QR Decomposition
        Eigen::MatrixXd J_c_T = J_c_.transpose(); // 18x9

        Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_c_T);
        qr.compute(J_c_T);
        Eigen::MatrixXd Q = qr.householderQ();
        Eigen::MatrixXd QT = Q.transpose();
        Eigen::MatrixXd Qu = Su*QT;
        Eigen::MatrixXd R = qr.matrixQR().template triangularView<Eigen::Upper>();
        Eigen::MatrixXd QuS_inv = moor_penrose_pseudo_inverse(Qu*S.transpose());

        hoqp->updateAllTasks();
        hoqp->solveAllTasks();
        Eigen::VectorXd des_qddot = hoqp->getSolution();
        tau = (QuS_inv)*Qu*(M*des_qddot + h);
        generalizedForce.tail(12) = tau;
        go1->setGeneralizedForce(generalizedForce);

        server.integrateWorldThreadSafe();
    }
    server.killServer();

    return 0;
}