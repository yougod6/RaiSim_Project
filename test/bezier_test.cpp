#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include <matplot/matplot.h>

Eigen::MatrixXd moor_penrose_pseudo_inverse(Eigen::MatrixXd A) {
    Eigen::MatrixXd At = A.transpose();
    return At*((A*At).inverse());
}

Eigen::MatrixXd weighted_pseudo_inverse(Eigen::MatrixXd J, Eigen::MatrixXd W) {
    Eigen::MatrixXd JT = J.transpose();
    return (JT*W*J).inverse()*JT*W;
}

Eigen::MatrixXd QR_decompostion(const Eigen::MatrixXd &J_c_){
    Eigen::MatrixXd J_c_T = J_c_.transpose();
    // std::cout << "column rank of J_c_T: " << Eigen::FullPivLU<Eigen::MatrixXd>(J_c_T).rank() << std::endl;
    // QR Decomposition J_c_T = Q*R , where Q is mxn orthogonal matrix, R is nxn upper triangular matrix
    // full QR decomposition
    Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr(J_c_T);
    Eigen::MatrixXd Q = qr.matrixQ();
    return Q;
}

Eigen::VectorXd make_base_trajectory(const double time){
    const double amplitude = 0.05; //0.15
    const double freq = 0.5;

    Eigen::VectorXd desired_q_B_ = Eigen::VectorXd::Zero(6);
    // desired_q_B_ << -0.00270028 + amplitude*sin(2*M_PI*freq*time),
    //                 0.000455424 ,
    //                 0.345183,
    //                 0,0,0;
   desired_q_B_ << -0.00270028,
                    0.000455424 ,
                    0.355,
                    0,0,0;
    return desired_q_B_;
}

// 2-order bezier curve
Eigen::VectorXd make_bezier_trajectory(std::vector<Eigen::VectorXd> P,const double t){
    Eigen::VectorXd bezier_position = Eigen::VectorXd::Zero(3);
    if(t<3){return P[0];}
    
    const double totalT = 3;
    const double time = (t-3)/totalT;
    if(time>1){return P[2];}
    bezier_position = P[1] + (1-time)*(1-time)*(P[0]-P[1]) + time*time*(P[2]-P[1]);
    return bezier_position;
}

Eigen::MatrixXd compute_Jdot(Eigen::MatrixXd J, Eigen::MatrixXd J_prev, const double dt){
    return (J - J_prev)/dt;
}

Eigen::MatrixXd get_nullspace(Eigen::MatrixXd J,Eigen::MatrixXd M){
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(18,18);
    Eigen::MatrixXd operational_inertia = J*M.inverse()*J.transpose();
    return I - M.inverse()*J.transpose()*operational_inertia.inverse()*J;
}

void contact_scheduler(std::vector<bool> contact_status){
    // Contact Scheduler (FR, FL, RR, RL)
    contact_status[0] = true;
    contact_status[1] = true;
    contact_status[2] = true;
    contact_status[3] = true;
}
    
Eigen::MatrixXd get_contact_jacobian(std::vector<bool> contact_status,raisim::ArticulatedSystem *robot){
    // contact 개수 (contact_status의 true 개수)
    std::vector<int> contact_idx;
    for(bool contact : contact_status){
        if(contact){
            contact_idx.push_back(contact);
        }
    }
    int contact_num = contact_idx.size();
    Eigen::MatrixXd J_c = Eigen::MatrixXd::Zero(3*contact_num, 18);

    // Contact Jacobian
    int stacked_num = 0;    
    // for(int i=0; i<contact_num; i++){
    //     if(contact_idx[i] == 0){
    //         robot->getDenseFrameJacobian("FR_foot_fixed", J_c.block(stacked_num,0,3,18));
    //         stacked_num += 3;
    //     }
    //     else if(contact_idx[i] == 1){
    //         robot->getDenseFrameJacobian("FL_foot_fixed", J_c.block(stacked_num,0,3,18));
    //         stacked_num += 3;
    //     }
    //     else if(contact_idx[i] == 2){
    //         robot->getDenseFrameJacobian("RR_foot_fixed", J_c.block(stacked_num,0,3,18));
    //         stacked_num += 3;
    //     }
    //     else if(contact_idx[i] == 3){
    //         robot->getDenseFrameJacobian("RL_foot_fixed", J_c.block(stacked_num,0,3,18));
    //         stacked_num += 3;
    //     }
    // }
    return J_c;
}

int main (int argc, char* argv[]) {
    raisim::World world;
    auto ground = world.addGround();
    world.setTimeStep(0.0001); //10kHz

    raisim::Vec<3> gravity = world.getGravity();
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    // auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\aliengo\\aliengo.urdf");
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");
    // auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\a1\\urdf\\a1.urdf");
    // go1->printOutMovableJointNamesInOrder();

    const int DOF = go1-> getDOF();
    const int jointNum = DOF - 6;

    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(go1);

    auto sphere_body = world.addSphere(0.1, 1.0, "default",raisim::COLLISION(2), raisim::COLLISION(2));
    sphere_body->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_body->setAppearance("green");
    sphere_body->setPosition(raisim::Vec<3>{-0.00270028, 0.000455424, 0.345183});

    Eigen::VectorXd StartPoint = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd EndPoint = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd z_offset = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd WayPoint1 = Eigen::VectorXd::Zero(3);

    StartPoint << 0.358693,-0.122684,0.135906;
    EndPoint << 0.429162,-0.122845,0.1199188;
    z_offset << 0,0,0.1;
    WayPoint1 = (StartPoint + EndPoint)/2 + z_offset;

    std::vector<Eigen::VectorXd> bezier_points = {StartPoint, WayPoint1, EndPoint};

    Eigen::VectorXd bezier_position = Eigen::VectorXd::Zero(3);
    
    auto sphere_traj = world.addSphere(0.025, 1.0, "default",raisim::COLLISION(6), raisim::COLLISION(6));
    sphere_traj->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_traj->setAppearance("blue");
    sphere_traj->setPosition(raisim::Vec<3>{StartPoint(0), StartPoint(1), StartPoint(2)});

    auto sphere_start = world.addSphere(0.03, 1.0, "default",raisim::COLLISION(4), raisim::COLLISION(4));
    sphere_start->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_start->setAppearance("green");
    sphere_start->setPosition(raisim::Vec<3>{StartPoint(0), StartPoint(1), StartPoint(2)});

    auto sphere_goal = world.addSphere(0.03, 1.0, "default",raisim::COLLISION(4), raisim::COLLISION(4));
    sphere_goal->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_goal->setAppearance("red");
    sphere_goal->setPosition(raisim::Vec<3>{EndPoint(0), EndPoint(1), EndPoint(2)});

    auto sphere_waypoint = world.addSphere(0.03, 1.0, "default",raisim::COLLISION(4), raisim::COLLISION(4));
    sphere_waypoint->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_waypoint->setAppearance("gray");
    sphere_waypoint->setPosition(raisim::Vec<3>{WayPoint1(0), WayPoint1(1), WayPoint1(2)});

    Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim());
    
    jointNominalConfig << 0.0, 0.0, 0.355, //base position
                        1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        0.02, 0.2, -1.8, //
                        -0.03, 0.2, -1.2,
                        0.03, -0.2, 1.2,
                        -0.03, -0.2, 1.2;

    go1->setGeneralizedCoordinate(jointNominalConfig);

    // Dynamic Terms
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(DOF, DOF);
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(DOF);
    Eigen::VectorXd h = go1->getNonlinearities(gravity).e();

    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(DOF);

    // Contact Jacobian
    Eigen::MatrixXd J_c_ = Eigen::MatrixXd::Zero(9, DOF);
    Eigen::MatrixXd J_c_prev = Eigen::MatrixXd::Zero(9, DOF); 
    Eigen::MatrixXd J_c_dot = Eigen::MatrixXd::Zero(9, DOF);
    Eigen::MatrixXd J_c_FL = Eigen::MatrixXd::Zero(3, DOF);
    Eigen::MatrixXd J_c_RR = Eigen::MatrixXd::Zero(3, DOF);
    Eigen::MatrixXd J_c_RL = Eigen::MatrixXd::Zero(3, DOF);
    
    // FR Foot Jacobian
    Eigen::MatrixXd J_FR = Eigen::MatrixXd::Zero(3, DOF);
    Eigen::MatrixXd J_FR_prev = Eigen::MatrixXd::Zero(3, DOF); 
    Eigen::MatrixXd J_FR_dot = Eigen::MatrixXd::Zero(3, DOF);

    // Base Jacobian
    Eigen::MatrixXd J_B = Eigen::MatrixXd::Zero(6, DOF); 
    Eigen::MatrixXd J_B_prev = Eigen::MatrixXd::Zero(6, DOF); 
    Eigen::MatrixXd J_B_dot = Eigen::MatrixXd::Zero(6, DOF); 
    Eigen::MatrixXd J_Bp = Eigen::MatrixXd::Zero(3, DOF); 
    Eigen::MatrixXd J_Br = Eigen::MatrixXd::Zero(3, DOF); 
    
    // Task Jacobian
    Eigen::MatrixXd J_t = Eigen::MatrixXd::Zero(18, DOF);
    Eigen::MatrixXd J_t_prev = Eigen::MatrixXd::Zero(18, DOF);
    Eigen::MatrixXd J_t_dot = Eigen::MatrixXd::Zero(18, DOF);

    // Selection Matrix
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(DOF-6, DOF); //12x18
    S.block(0,6,12,12) = Eigen::MatrixXd::Identity(DOF-6,DOF-6); 
    
    // Selection Matrix for Contact Jacobian (QR Decomposition)
    Eigen::MatrixXd Sc = Eigen::MatrixXd::Zero(9, 18); //9x18
    Sc.block(0,0,9,9) = Eigen::MatrixXd::Identity(9,9);
    Eigen::MatrixXd Su= Eigen::MatrixXd::Zero(9,18); 
    Su.block(0,9,9,9) = Eigen::MatrixXd::Identity(9,9);   

    raisim::Vec<3> base_position;
    raisim::Vec<4> base_quat;

    Eigen::VectorXd base_pose = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd desired_base_pose = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd base_velocity = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd desired_qddot_contact = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd desired_qddot_base = Eigen::VectorXd::Zero(18);

    raisim::Vec<3> FR_foot_position;
    raisim::Vec<3> FR_foot_velocity;
    Eigen::VectorXd des_xddot = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd desired_xddot_base = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd desired_xddot_foot = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd des_qddot = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd desired_qddot_foot = Eigen::VectorXd::Zero(18);
    
    Eigen::MatrixXd Kp_base = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd Kd_base = Eigen::MatrixXd::Zero(6,6);
    
    const int Kp_base_position = 100;
    const int Kp_base_orientation = 100;
    const int Kd_base_position = 2*sqrt(Kp_base_position);
    const int Kd_base_orientation = 2*sqrt(Kp_base_orientation);
    
    Kp_base.block(0,0,3,3) = Kp_base_position*Eigen::MatrixXd::Identity(3,3);
    Kp_base.block(3,3,3,3) = Kp_base_orientation*Eigen::MatrixXd::Identity(3,3);
    Kd_base.block(0,0,3,3) = Kd_base_position*Eigen::MatrixXd::Identity(3,3);
    Kd_base.block(3,3,3,3) = Kd_base_orientation*Eigen::MatrixXd::Identity(3,3);

    const int Kp_foot_ = 50;
    const int Kd_foot_ = 2*sqrt(Kp_foot_);
    Eigen::MatrixXd Kp_foot = Kp_foot_*Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd Kd_foot = Kd_foot_*Eigen::MatrixXd::Identity(3,3);
    
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tau_total = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tau_contact = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tau_base = Eigen::VectorXd::Zero(12);

    const int totalT = 100000; //100s

    for (int i=0; i<totalT; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6);

        // Get Dynamics
        M = go1->getMassMatrix().e();
        qdot = go1->getGeneralizedVelocity().e();
        qddot = go1->getGeneralizedAcceleration().e();
        h = go1->getNonlinearities(gravity).e();

        // Get Base Pose
        go1->getBasePosition(base_position);
        go1->getBaseOrientation(base_quat);
        base_pose.head(3) = base_position.e();
        base_pose.tail(3) = base_quat.e().tail(3);

        // Get Base Velocity
        base_velocity = go1->getGeneralizedVelocity().e().head(6);

        // Get Front-Right Foot Position and Velocity
        go1->getFramePosition("FR_foot_fixed", FR_foot_position);
        go1->getFrameVelocity("FR_foot_fixed", FR_foot_velocity);

        // Get FR Foot and Contact Jacobian
        go1->getDenseFrameJacobian("FR_foot_fixed", J_FR);
        go1->getDenseFrameJacobian("FL_foot_fixed", J_c_FL);
        go1->getDenseFrameJacobian("RR_foot_fixed", J_c_RR);
        go1->getDenseFrameJacobian("RL_foot_fixed", J_c_RL);

        //Get Base Jacobian
        go1->getDenseFrameJacobian("floating_base", J_Bp);
        go1->getDenseFrameRotationalJacobian("floating_base", J_Br);
        J_B.block(0, 0, 3, DOF) = J_Bp;
        J_B.block(3, 0, 3, DOF) = J_Br;
        
        // Stack Contact Jacobians
        J_c_.block(0, 0, 3, DOF) = J_c_FL;
        J_c_.block(3, 0, 3, DOF) = J_c_RR;
        J_c_.block(6, 0, 3, DOF) = J_c_RL;

        // Desired Trajectory
        desired_base_pose = make_base_trajectory((world.getWorldTime()));
        sphere_body->setPosition(raisim::Vec<3>{desired_base_pose(0), desired_base_pose(1), desired_base_pose(2)});

        bezier_position = make_bezier_trajectory(bezier_points, (world.getWorldTime()));
        sphere_traj->setPosition(raisim::Vec<3>{bezier_position(0), bezier_position(1), bezier_position(2)});

        // Get Desired Base and Foot Acceleration (represented in world frame)
        desired_xddot_base = Kp_base*(desired_base_pose - base_pose) - Kd_base*(base_velocity);
        desired_xddot_foot = Kp_foot*(bezier_position - FR_foot_position.e()) - Kd_foot*(FR_foot_velocity.e());

        // Stack Jacobians
        J_t.block(0, 0, 3, DOF) = J_FR;
        J_t.block(3, 0, 9, DOF) = J_c_;
        J_t.block(12, 0, 6,DOF) = J_B;

        // Compute Jacobian Derivative
        J_B_dot = compute_Jdot(J_B, J_B_prev, world.getTimeStep());
        J_c_dot = compute_Jdot(J_c_, J_c_prev, world.getTimeStep());  
        J_FR_dot = compute_Jdot(J_FR, J_FR_prev, world.getTimeStep());
        J_t_dot = compute_Jdot(J_t, J_t_prev, world.getTimeStep());

        // Stack Desired Acceleration (in world frame)
        des_xddot.setZero();
        des_xddot.head(3) = desired_xddot_foot;
        des_xddot.tail(6) = desired_xddot_base;

        // Compute Desired Joint Acceleration
        Eigen::MatrixXd W = Eigen::MatrixXd::Identity(18,18);
        const int FR_weight = 1;
        const int contact_weight = 1;
        const int base_weight = 1;
        W.block(0,0,3,3) = FR_weight*Eigen::MatrixXd::Identity(3,3);
        W.block(3,3,9,9) = contact_weight*Eigen::MatrixXd::Identity(9,9);
        W.block(12,12,6,6) = base_weight*Eigen::MatrixXd::Identity(6,6);

        // des_qddot = weighted_pseudo_inverse(J_t,W)*(des_xddot - J_t_dot*qdot);
        des_qddot = moor_penrose_pseudo_inverse(J_t)*(des_xddot - J_t_dot*qdot);

        // QR Decomposition
        Eigen::MatrixXd J_c_T = J_c_.transpose(); // 18x9

        Eigen::HouseholderQR<Eigen::MatrixXd> qr(J_c_T);
        qr.compute(J_c_T);
        Eigen::MatrixXd Q = qr.householderQ();
        Eigen::MatrixXd QT = Q.transpose();
        Eigen::MatrixXd Qu = Su*QT;
        Eigen::MatrixXd R = qr.matrixQR().template triangularView<Eigen::Upper>();
        Eigen::MatrixXd QuS_inv = moor_penrose_pseudo_inverse(Qu*S.transpose());

        // Inverse Dynamics
        tau_total = (QuS_inv)*Qu*(M*des_qddot + h);

        J_B_prev = J_B;
        J_c_prev = J_c_;
        J_FR_prev = J_FR;
        J_t_prev = J_t;

        tau.setZero();
        tau += tau_total;

        generalizedForce.setZero();
        generalizedForce.tail(12) = tau;
        go1->setGeneralizedForce(generalizedForce);
        server.integrateWorldThreadSafe();
    }
    
    return 0;
}