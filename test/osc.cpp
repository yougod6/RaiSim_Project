#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include <matplot/matplot.h>


Eigen::MatrixXd get_operational_space_inertia(Eigen::MatrixXd M, Eigen::MatrixXd J){
    return (J*M.inverse()*J.transpose()).inverse();
}

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
    const double amplitude = 0.05; //0.15
    const double freq = 0.5;

    Eigen::VectorXd desired_q_B_ = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd desired_q_B_traj = Eigen::VectorXd::Zero(3);
    
    // Base X-Y references
    // desired_q_B_ << 0,0,0.30+amplitude*cos(2*M_PI*freq*time);
    desired_q_B_ << -0.00270028 + amplitude*sin(2*M_PI*freq*time),
                    0.000455424 ,
                    0.345183;
    // desired_q_B_ << amplitude*sin(2*M_PI*freq*time),0,0.30+amplitude*0.5*cos(2*M_PI*freq*time);
                    // 0.5*amplitude*cos(2*M_PI*freq*time);

    return desired_q_B_;
}

Eigen::MatrixXd compute_Jdot(Eigen::MatrixXd J, Eigen::MatrixXd J_prev, const double dt){
    return (J - J_prev)/dt;
}


Eigen::MatrixXd get_actuated_inertia(Eigen::MatrixXd M, Eigen::MatrixXd S){
    return (S*M.inverse()*S.transpose()).inverse();
}

Eigen::MatrixXd get_inverse_seleciton_matrix(Eigen::MatrixXd S, Eigen::MatrixXd M){
    Eigen::MatrixXd M_act = get_actuated_inertia(M, S);
    return M.inverse()*S.transpose()*M_act;
}

// dynamical consistent task null space 
Eigen::MatrixXd get_nullspace(Eigen::MatrixXd J,Eigen::MatrixXd M_act){
    Eigen::MatrixXd Lambda = (J*M_act.inverse()*J.transpose()).inverse();
    // Dynamically consistent generalized inverse Jacobian
    Eigen::MatrixXd J_inv = M_act.inverse()*J.transpose()*Lambda;
    Eigen::MatrixXd JinvJ = J_inv*J;
    return Eigen::MatrixXd::Identity(JinvJ.rows(),JinvJ.cols()) - JinvJ;
}

Eigen::MatrixXd get_projected_J(Eigen::MatrixXd Jsys, Eigen::MatrixXd Msys){
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(18,12);
    S.block(0,0,6,12) = - Msys.block(0,0,6,6).inverse() * Msys.block(0,6,6,12);
    S.block(6,0,12,12) = Eigen::MatrixXd::Identity(12,12);
    return Jsys*S;
}

int main (int argc, char* argv[]) {
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World world;
    world.setTimeStep(0.001); //1kHz
    // world.setERP(0,0);
    raisim::Vec<3> gravity = world.getGravity();
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");
    auto sphere1 = world.addSphere(0.1, 1.0, "default",raisim::COLLISION(2), raisim::COLLISION(2));
    // auto sphere2 = world.addSphere(0.1, 1.0, "default",raisim::COLLISION(3), raisim::COLLISION(3));
    sphere1->setBodyType(raisim::BodyType::KINEMATIC);
    sphere1->setAppearance("green");
    sphere1->setPosition(raisim::Vec<3>{-0.00270028, 0.000455424, 0.345183});
    // sphere2->setBodyType(raisim::BodyType::KINEMATIC);
    // sphere2->setAppearance("green");
    // sphere2->setPosition(raisim::Vec<3>{-0.0999941, -0.00767566, 0.354407});
    // auto box1 = world.addBox(0.1, 0.1, 0.1, 10, "default");
    // box1->setPosition(-0.05, 0, 1.5);
    // box1->setAppearance("gray");

    go1->setComputeInverseDynamics(true);
    auto ground = world.addGround();
    Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointVelocityTarget(go1->getDOF());
    
    jointNominalConfig << 0.0, 0.0, 0.4, //base position
                        1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        0.03, 0.2, -1.2, //
                        -0.03, 0.2, -1.2,
                        0.03, -0.2, 1.2,
                        -0.03, -0.2, 1.2;
    jointVelocityTarget.setZero();
    go1->setGeneralizedCoordinate(jointNominalConfig);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF())); 
    
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(go1->getDOF());
    Eigen::MatrixXd J_c_FR_ = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_FL_ = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_RR_ = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_RL_ = Eigen::MatrixXd::Zero(3, go1->getDOF());
    Eigen::MatrixXd J_c_ = Eigen::MatrixXd::Zero(12, go1->getDOF());
   
    Eigen::MatrixXd Su= Eigen::MatrixXd::Zero(6,18); //6x18
    Su.block(0,12,6,6) = Eigen::MatrixXd::Identity(6,6);   
    
    Eigen::MatrixXd Sc = Eigen::MatrixXd::Zero(12, go1->getDOF()); //12x18
    Sc.block(0,0,12,12) = Eigen::MatrixXd::Identity(12,12);
    
    // Selection Matrix
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(go1->getDOF()-6, go1->getDOF()); //12x18
    S.block(0,6,12,12) = Eigen::MatrixXd::Identity(go1->getDOF()-6,go1->getDOF()-6); 
    std::cout << "start " << std::endl;
    
    go1->setName("go1");
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(go1);
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(go1->getDOF(), go1->getDOF());
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(go1->getDOF());
    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(go1->getDOF());
    Eigen::VectorXd h = go1->getNonlinearities(gravity).e();

    Eigen::MatrixXd J_B = Eigen::MatrixXd::Zero(3, go1->getDOF()); 
    Eigen::MatrixXd J_B_prev = Eigen::MatrixXd::Zero(3, go1->getDOF()); 
    Eigen::MatrixXd J_B_dot = Eigen::MatrixXd::Zero(3, go1->getDOF()); 
    Eigen::MatrixXd J_c_prev = Eigen::MatrixXd::Zero(12, go1->getDOF()); 
    Eigen::MatrixXd J_c_dot = Eigen::MatrixXd::Zero(12, go1->getDOF()); 

    raisim::Vec<3> base_position;
    Eigen::VectorXd desired_base_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd desired_qddot_contact = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd desired_qddot_base = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd desired_xddot = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd J_base = Eigen::MatrixXd::Zero(3, 18);
    const int Kp_base = 400;
    const int Kd_base = 2*sqrt(Kp_base);
    const int totalT = 100000;


    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tau_contact = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tau_base = Eigen::VectorXd::Zero(12);
    Eigen::MatrixXd N1 = Eigen::MatrixXd::Zero(12,12);
    namespace plt = matplot;
    std::vector<double> time = plt::linspace(0, totalT);
    std::vector<double> desired_base_x(totalT);
    std::vector<double> base_x(totalT);

    // auto graph = server.addTimeSeriesGraph("desired_base_x", {"desired_base_x"}, "time", "desired_base_x");


    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);

       
        go1->getDenseFrameJacobian("FR_foot_fixed", J_c_FR_); // 3x18
        go1->getDenseFrameJacobian("FL_foot_fixed", J_c_FL_); // 3x18
        go1->getDenseFrameJacobian("RR_foot_fixed", J_c_RR_); // 3x18
        go1->getDenseFrameJacobian("RL_foot_fixed", J_c_RL_); // 3x18
        J_c_.block(0, 0, 3, go1->getDOF()) = J_c_FR_;
        J_c_.block(3, 0, 3, go1->getDOF()) = J_c_FL_;
        J_c_.block(6, 0, 3, go1->getDOF()) = J_c_RR_;
        J_c_.block(9, 0, 3, go1->getDOF()) = J_c_RL_;
        Eigen::MatrixXd Q =  QR_decompostion(J_c_);
        // Eigen::MatrixXd Qc = Sc*Q;
        // Eigen::MatrixXd Qu = Su*Q;
        Eigen::MatrixXd S_tmp = Su*Q.transpose()*S.transpose();
        Eigen::MatrixXd SQS = moor_penrose_pseudo_inverse(S_tmp);

        M = go1->getMassMatrix().e();
        qdot = go1->getGeneralizedVelocity().e();
        qddot = go1->getGeneralizedAcceleration().e();
        h = go1->getNonlinearities(gravity).e();
        
        go1->getDenseFrameJacobian("floating_base",J_B);
    
        go1->getBasePosition(base_position);
        Eigen::VectorXd base_velocity = go1->getGeneralizedVelocity().e().head(3);
        desired_base_position = make_base_trajectory((world.getWorldTime()));
        desired_xddot = Kp_base*(desired_base_position - base_position.e().head(3)) - Kd_base*(base_velocity);
        sphere1->setPosition(raisim::Vec<3>{desired_base_position(0), desired_base_position(1), desired_base_position(2)});
        // sphere2->setPosition(raisim::Vec<3>{base_position(0), base_position(1), base_position(2)});

        // Eigen::VectorXd desired_xddot_base = Eigen::VectorXd::Zero(3);
        // desired_xddot_base(0) = desired_xddot(0); // only x direction

        J_B_dot = compute_Jdot(J_B, J_B_prev, world.getTimeStep());
        J_c_dot = compute_Jdot(J_c_, J_c_prev, world.getTimeStep());
        
        qdot = go1->getGeneralizedVelocity().e();
        desired_qddot_base = moor_penrose_pseudo_inverse(J_B)*(desired_xddot - J_B_dot*qdot);
        Eigen::VectorXd Z = Eigen::VectorXd::Zero(12);
        desired_qddot_contact = moor_penrose_pseudo_inverse(J_c_)*(Z - J_c_dot*qdot);

        tau_contact = (SQS)*Su*Q.transpose()*(M*desired_qddot_contact + h);
        // tau_base = (SQS)*Su*Q.transpose()*(M*desired_qddot_base + h);
        Eigen::MatrixXd S_inv = get_inverse_seleciton_matrix(S, M);


        Eigen::MatrixXd J = get_projected_J(J_B,M);
        Eigen::MatrixXd Jc_proj = get_projected_J(J_c_,M);
        std::cout << __LINE__ << std::endl;
        Eigen::MatrixXd M_act = get_actuated_inertia(M, S);
        std::cout << __LINE__ << std::endl;
        Eigen::MatrixXd Lambda = get_operational_space_inertia(M_act, J);
        std::cout << __LINE__ << std::endl;
        Eigen::MatrixXd J_inv = M_act.inverse()*J.transpose()*Lambda;
        std::cout << "J_inv: " << J_inv.rows() << "x" << J_inv.cols() << std::endl;
        std::cout << __LINE__ << std::endl;
        // Eigen::MatrixXd nonlinearities  = J_inv.transpose()*h;
        std::cout << __LINE__ << std::endl;
        Eigen::MatrixXd F = Lambda*desired_xddot;
        std::cout << __LINE__ << std::endl;
        tau_base = J.transpose()*F;
        std::cout << __LINE__ << std::endl;
        N1 = get_nullspace(Jc_proj, M_act);
        tau_base = N1.transpose()*tau_base;
        J_B_prev = J_B;
        J_c_prev = J_c_;
       
        desired_base_x[i] = desired_base_position(0);
        base_x[i] = base_position.e().head(3)(0);
        tau.setZero();
        tau += tau_contact;
        tau += tau_base;
        generalizedForce.tail(12) = tau;
        go1->setGeneralizedForce(generalizedForce);
        server.integrateWorldThreadSafe();
    }
    
    return 0;
}