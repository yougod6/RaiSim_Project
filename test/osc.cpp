#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include <matplot/matplot.h>

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

Eigen::MatrixXd getAdjointMap(raisim::ArticulatedSystem* robot, const std::string &frame_name){
    //get parent body idx of the frame
    size_t parent_idx = robot->getFrameByName(frame_name).parentId;
    //get parent name
    auto name = robot->getFrameByName(frame_name).parentName;
    Eigen::MatrixXd Ad_map = Eigen::MatrixXd::Zero(6,6);
    raisim::Mat<3,3> rot;
    raisim::Vec<3> pos;
    robot->getBodyPose(parent_idx, rot, pos);
    Eigen::MatrixXd rot_e = rot.e();
    Eigen::MatrixXd pos_skew = Eigen::MatrixXd::Zero(3,3);
    pos_skew << 0, -pos(2), pos(1),
                pos(2), 0, -pos(0),
                -pos(1), pos(0), 0;
    Ad_map.block(0,0,3,3) = rot_e;
    Ad_map.block(3,3,3,3) = rot_e;
    Ad_map.block(0,3,3,3) = pos_skew*rot_e;
    return Ad_map;
}

Eigen::MatrixXd getBodyJacobian(raisim::ArticulatedSystem* robot, const std::string &frame_name){
    Eigen::MatrixXd Jb_p = Eigen::MatrixXd::Zero(3, robot->getDOF());
    Eigen::MatrixXd Jb_r = Eigen::MatrixXd::Zero(3, robot->getDOF());
    Eigen::MatrixXd Jb = Eigen::MatrixXd::Zero(6, robot->getDOF());
    robot->getDenseFrameJacobian(frame_name, Jb_p);
    robot->getDenseFrameRotationalJacobian(frame_name, Jb_r);
    Jb.block(0, 0, 3, robot->getDOF()) = Jb_p;
    Jb.block(3, 0, 3, robot->getDOF()) = Jb_r;
    return Jb;
}

Eigen::MatrixXd getSpaceJacobian(raisim::ArticulatedSystem* robot, const std::string &frame_name){
    Eigen::MatrixXd Jb = getBodyJacobian(robot, frame_name);
    Eigen::MatrixXd Ad_map = getAdjointMap(robot, frame_name);
    return Jb;
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
    raisim::Vec<3> gravity = world.getGravity();
    // auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\aliengo\\aliengo.urdf");
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");
    
    auto sphere_body = world.addSphere(0.1, 1.0, "default",raisim::COLLISION(2), raisim::COLLISION(2));
    sphere_body->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_body->setAppearance("green");
    sphere_body->setPosition(raisim::Vec<3>{-0.00270028, 0.000455424, 0.345183});
    
    // go1->setComputeInverseDynamics(true);
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

    Eigen::MatrixXd J_B = Eigen::MatrixXd::Zero(6, go1->getDOF()); 
    Eigen::MatrixXd J_B_prev = Eigen::MatrixXd::Zero(6, go1->getDOF()); 
    Eigen::MatrixXd J_B_dot = Eigen::MatrixXd::Zero(6, go1->getDOF()); 
    
    Eigen::MatrixXd J_c_prev = Eigen::MatrixXd::Zero(12, go1->getDOF()); 
    Eigen::MatrixXd J_c_dot = Eigen::MatrixXd::Zero(12, go1->getDOF()); 

    Eigen::MatrixXd J_t = Eigen::MatrixXd::Zero(18, go1->getDOF());
    Eigen::MatrixXd J_t_dot = Eigen::MatrixXd::Zero(18, go1->getDOF());

    raisim::Vec<3> base_position;
    raisim::Vec<4> base_quat;

    Eigen::VectorXd base_pose = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd desired_base_pose = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd desired_qddot_contact = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd desired_qddot_base = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd desired_xddot_base = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd des_xddot = Eigen::VectorXd::Zero(18);

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
    const int totalT = 1000000;

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tau_pd = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tau_contact = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tau_base = Eigen::VectorXd::Zero(12);
    Eigen::MatrixXd N1 = Eigen::MatrixXd::Zero(12,12);
    
    namespace plt = matplot;
    std::vector<double> time = plt::linspace(0, totalT);
    std::vector<double> desired_base_x(totalT);
    std::vector<double> base_x(totalT);
    // 소수점 첫째자리까지 출력
    std::cout.precision(1);

    size_t DOF = go1->getDOF();
    size_t FR_calf_idx = go1->getBodyIdx("FR_calf");
    size_t FL_calf_idx = go1->getBodyIdx("FL_calf");
    size_t RR_calf_idx = go1->getBodyIdx("RR_calf");
    size_t RL_calf_idx = go1->getBodyIdx("RL_calf");
    raisim::Vec<3> FR_foot_position;
    raisim::Vec<3> FL_foot_position;
    raisim::Vec<3> RR_foot_position;
    raisim::Vec<3> RL_foot_position;
    go1->printOutBodyNamesInOrder();
    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);
        go1->getFramePosition("FR_foot_fixed", FR_foot_position);
        go1->getFramePosition("FL_foot_fixed", FL_foot_position);
        go1->getFramePosition("RR_foot_fixed", RR_foot_position);
        go1->getFramePosition("RL_foot_fixed", RL_foot_position);
        // Update Foot Jacobian
        // J_c_FR = getSpaceJacobian(go1, "FR_foot_fixed");
        // J_c_FL = getSpaceJacobian(go1, "FL_foot_fixed");
        // J_c_RR = getSpaceJacobian(go1, "RR_foot_fixed");
        // J_c_RL = getSpaceJacobian(go1, "RL_foot_fixed");
        
        go1->getDenseJacobian(FR_calf_idx,FR_foot_position, J_c_FR);
        go1->getDenseJacobian(FL_calf_idx,FL_foot_position, J_c_FL);
        go1->getDenseJacobian(RR_calf_idx,RR_foot_position, J_c_RR);
        go1->getDenseJacobian(RL_calf_idx,RL_foot_position, J_c_RL);
        
        //Get Base Jacobian
        J_B = getSpaceJacobian(go1, "floating_base");

        // Stack Contact Jacobians
        J_c_.block(0, 0, 3, DOF) = J_c_FR.block(0, 0, 3, DOF);
        J_c_.block(3, 0, 3, DOF) = J_c_FL.block(0, 0, 3, DOF);
        J_c_.block(6, 0, 3, DOF) = J_c_RR.block(0, 0, 3, DOF);
        J_c_.block(9, 0, 3, DOF) = J_c_RL.block(0, 0, 3, DOF);


        // QR Decomposition
        Eigen::MatrixXd Q =  QR_decompostion(J_c_);
        Eigen::MatrixXd S_tmp = Su*Q.transpose()*S.transpose();
        Eigen::MatrixXd SQS = moor_penrose_pseudo_inverse(S_tmp);

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

        // Get Base Velocity
        Eigen::VectorXd base_velocity = go1->getGeneralizedVelocity().e().head(6);
       
        // Desired Trajectory
        desired_base_pose = make_base_trajectory((world.getWorldTime()));
       
        // Get Desired Base Acceleration (in world frame)
        desired_xddot_base = Kp_base*(desired_base_pose - base_pose) - Kd_base*(base_velocity);
        sphere_body->setPosition(raisim::Vec<3>{desired_base_pose(0), desired_base_pose(1), desired_base_pose(2)});

        // Compute Jacobian Derivative
        J_B_dot = compute_Jdot(J_B, J_B_prev, world.getTimeStep());
        J_c_dot = compute_Jdot(J_c_, J_c_prev, world.getTimeStep());
        
        // Stack Jacobians
        J_t.block(0, 0, 12, go1->getDOF()) = J_c_;
        J_t.block(12, 0, 6, go1->getDOF()) = J_B;

        J_t_dot.block(0, 0, 12, go1->getDOF()) = J_c_dot;
        J_t_dot.block(12, 0, 6, go1->getDOF()) = J_B_dot;

        // Stack Desired Acceleration (in world frame)
        des_xddot.tail(6) = desired_xddot_base;
        desired_qddot_base = moor_penrose_pseudo_inverse(J_t)*(des_xddot - J_t_dot*qdot);
        
        // Inverse Dynamics
        tau_base = (SQS)*Su*Q.transpose()*(M*desired_qddot_base + h);
        
        J_B_prev = J_B;
        J_c_prev = J_c_;

        desired_base_x[i] = desired_base_pose(0);
        base_x[i] = base_position.e().head(3)(0);
        tau.setZero();
        
        tau += tau_base;

        generalizedForce.tail(12) = tau;
        go1->setGeneralizedForce(generalizedForce);
        server.integrateWorldThreadSafe();
    }
    server.killServer();
    auto fig = plt::figure();   
    auto ax = fig->current_axes();
    plt::plot(ax,desired_base_x,"--r",base_x,"-b"); //plot the x,y
    // plt::plot(base_x,"-b"); //plot the x,y
    plt::show();
    return 0;
}