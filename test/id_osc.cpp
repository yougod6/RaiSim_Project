#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

Eigen::MatrixXd moor_penrose_pseudo_inverse(Eigen::MatrixXd A) {
    return A.transpose()*((A*A.transpose()).inverse());
}

Eigen::MatrixXd QR_decompostion(Eigen::MatrixXd &J_c_){
    Eigen::MatrixXd J_c_T = J_c_.transpose();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_c_T);
    Eigen::MatrixXd Q = qr.householderQ();
    Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
    return Q;
}

Eigen::VectorXd make_base_trajectory(const double time){
    const double amplitude = 0.1; //0.15
    const double freq = 1.0;

    Eigen::VectorXd desired_q_B_ = Eigen::VectorXd::Zero(3);
    
    // Base X-Y references
    // desired_q_B_ << 0,0,0.30+amplitude*cos(2*M_PI*freq*time);
    desired_q_B_ << amplitude*sin(2*M_PI*freq*time),0,0.4;
    // desired_q_B_ << amplitude*sin(2*M_PI*freq*time),0,0.30+amplitude*0.5*cos(2*M_PI*freq*time);
                    // 0.5*amplitude*cos(2*M_PI*freq*time);

    return desired_q_B_;
}

Eigen::MatrixXd compute_Jdot(Eigen::MatrixXd J, Eigen::MatrixXd J_prev, const double dt){
    return (J - J_prev)/dt;
}


int main (int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World world;
    world.setTimeStep(0.0001);
    raisim::Vec<3> gravity = world.getGravity();
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");
    auto sphere1 = world.addSphere(0.05, 1.0, "default",raisim::COLLISION(2), raisim::COLLISION(2));
    sphere1->setBodyType(raisim::BodyType::KINEMATIC);
    sphere1->setAppearance("green");

    // auto box1 = world.addBox(0.1, 0.1, 0.1, 10, "default");
    // box1->setPosition(-0.05, 0, 1.5);
    // box1->setAppearance("gray");

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
    go1->setGeneralizedCoordinate(jointNominalConfig);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF())); 
    
    Eigen::MatrixXd jointPgain = Eigen::MatrixXd::Identity(12, 12);
    Eigen::MatrixXd jointDgain = Eigen::MatrixXd::Identity(12, 12);
    const double kp = 30.0, kd = 2*sqrt(kp);
    jointPgain*=kp;
    jointDgain*=kd;
    
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(go1->getDOF());
    Eigen::MatrixXd J_c_FR_ = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_FL_ = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_RR_ = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_RL_ = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_ = Eigen::MatrixXd::Zero(12, go1->getDOF());
    
    Eigen::MatrixXd Sc = Eigen::MatrixXd::Zero(go1->getDOF()-6, go1->getDOF()); //12x18
    Sc.block(0,0,12,12) = Eigen::MatrixXd::Identity(go1->getDOF()-6,go1->getDOF()-6);
    
    Eigen::MatrixXd Su= Eigen::MatrixXd::Zero(6,18); //6x18
    Su.block(0,12,6,6) = Eigen::MatrixXd::Identity(6,6);   
    
    // Selection Matrix
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(go1->getDOF()-6, go1->getDOF()); //12x18
    S.block(0,6,12,12) = Eigen::MatrixXd::Identity(go1->getDOF()-6,go1->getDOF()-6); 
    std::cout << "start " << std::endl;
    
    go1->setName("go1");
    go1->printOutBodyNamesInOrder();   
    go1->printOutFrameNamesInOrder(); 
    std::cout << go1->getName() << std::endl;
    Eigen::VectorXd actuated_toq = Eigen::VectorXd::Zero(12);
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(go1);
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(go1->getDOF(), go1->getDOF());
    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(go1->getDOF());
    Eigen::MatrixXd J_B = Eigen::MatrixXd::Zero(3, go1->getDOF()); 
    Eigen::MatrixXd J_B_prev = Eigen::MatrixXd::Zero(3, go1->getDOF()); 
    Eigen::MatrixXd J_B_dot = Eigen::MatrixXd::Zero(3, go1->getDOF()); 
    Eigen::MatrixXd J_c_prev = Eigen::MatrixXd::Zero(12, go1->getDOF()); 
    Eigen::MatrixXd J_c_dot = Eigen::MatrixXd::Zero(12, go1->getDOF()); 

    const int Kp_base = 500;
    const int Kd_base = 2*sqrt(Kp_base);
    const int totalT = 1000000;
    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);

        std::vector<Eigen::Vector3d> axes(go1->getDOF()-6);

        go1->getDenseFrameJacobian("FR_foot_fixed", J_c_FR_); // 3x18
        go1->getDenseFrameJacobian("FL_foot_fixed", J_c_FL_); // 3x18
        go1->getDenseFrameJacobian("RR_foot_fixed", J_c_RR_); // 3x18
        go1->getDenseFrameJacobian("RL_foot_fixed", J_c_RL_); // 3x18
        J_c_.block(0, 0, 3, go1->getDOF()) = J_c_FR_;
        J_c_.block(3, 0, 3, go1->getDOF()) = J_c_FL_;
        J_c_.block(6, 0, 3, go1->getDOF()) = J_c_RR_;
        J_c_.block(9, 0, 3, go1->getDOF()) = J_c_RL_;

        Eigen::MatrixXd Q =  QR_decompostion(J_c_);
        Eigen::MatrixXd Qc = Sc*Q;
        Eigen::MatrixXd Qu = Su*Q;
        Eigen::MatrixXd S_tmp = Su*Q.transpose()*S.transpose();
        Eigen::MatrixXd SQS = moor_penrose_pseudo_inverse(S_tmp);

        Eigen::MatrixXd M = go1->getMassMatrix().e();
        Eigen::VectorXd qdot = go1->getGeneralizedVelocity().e();
        Eigen::VectorXd qddot = go1->getGeneralizedAcceleration().e();
        Eigen::VectorXd h = go1->getNonlinearities(gravity).e();
        
        Eigen::VectorXd desired_qddot = Eigen::VectorXd::Zero(18);
        Eigen::VectorXd desired_xddot = Eigen::VectorXd::Zero(3);
        Eigen::MatrixXd J_tmp = Eigen::MatrixXd::Zero(15, 18);
        go1->getDenseFrameJacobian("floating_base",J_B);
        // std::cout << std::endl << "J_B : " << std::endl << J_B << std::endl;
        //현재 코드 라인 출력
        // std::cout << "line : " << __LINE__ << std::endl;
        J_tmp.block(0,0,3,18) = J_B.block(0,0,3,18);
        J_tmp.block(3,0,12,18) = J_c_;

        raisim::Vec<3> base_position;
        go1->getBasePosition(base_position);
        Eigen::VectorXd base_velocity = go1->getGeneralizedVelocity().e().head(3);
        Eigen::VectorXd desired_base_position = make_base_trajectory(world.getWorldTime());
        desired_xddot = Kp_base*(desired_base_position - base_position.e().head(3)) - Kd_base*(base_velocity);
        sphere1->setPosition(raisim::Vec<3>{desired_base_position(0), desired_base_position(1), desired_base_position(2)});

        Eigen::VectorXd x_tmp = Eigen::VectorXd::Zero(15);
        x_tmp.head(3) = desired_xddot;

        J_B_dot = compute_Jdot(J_B, J_B_prev, world.getTimeStep());
        J_c_dot = compute_Jdot(J_c_, J_c_prev, world.getTimeStep());
        Eigen::MatrixXd J_tmp_dot = Eigen::MatrixXd::Zero(15,18);
        J_tmp_dot.block(0,0,3,18) = J_B_dot.block(0,0,3,18);
        J_tmp_dot.block(3,0,12,18) = J_c_dot;
        qdot = go1->getGeneralizedVelocity().e();
        desired_qddot = moor_penrose_pseudo_inverse(J_tmp)*(x_tmp - J_tmp_dot*qdot);

        // tau = (Su*Q^T*S^T)^(-1)*Su*Q^T*[Mqddot + h]
        Eigen::VectorXd tau = (SQS)*Su*Q.transpose()*(M*desired_qddot + h);
        // Eigen::VectorXd tau = (SQS)*Su*Q.transpose()*(M*qddot + h);
        
        Eigen::VectorXd tau_pd = (jointPgain*(jointNominalConfig.tail(12) - go1->getGeneralizedCoordinate().e().tail(12)) - jointDgain*(go1->getGeneralizedVelocity().e().tail(12)));
        tau += tau_pd;
        
        std::cout << "tau : " << tau.transpose() << std::endl;
        
        generalizedForce.tail(12) = tau;
        // go1->getCOM();
        go1->setGeneralizedForce(generalizedForce);
        J_B_prev = J_B;
        J_c_prev = J_c_;
        server.integrateWorldThreadSafe();
    }

    return 0;
}