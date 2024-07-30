#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "ConvexMPC.hpp"
#include "OsqpEigenSolver.hpp"

Eigen::VectorXd make_base_vel_trajectory(const double time){
    const double amplitude = 0.15;
    const double freq = 0.5;

    Eigen::VectorXd desired_v_B_ = Eigen::VectorXd::Zero(3);
   
    desired_v_B_ << 0.0,
                    0.0,
                    -amplitude*sin(2*M_PI*freq*time);

    return desired_v_B_;
}

int main (int argc, char* argv[]) {
    raisim::World world;
    auto ground = world.addGround();
    const double hz = 200;
    world.setTimeStep(1/hz); //1kHz
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");
    // auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\aliengo\\aliengo.urdf");
    
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);

    double dt = world.getTimeStep();
   
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim());
    
    jointNominalConfig << 0.0, 0.0, 0.42, //base position
                        1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        0.0, 0.6, -1.3, //
                        0.0, 0.6, -1.3,
                        0.0, 0.6, -1.3,
                        0.0, 0.6, -1.3;

    robot->setGeneralizedCoordinate(jointNominalConfig);

    Eigen::VectorXd l_weights(MPC_STATE_DIM);
    Eigen::VectorXd k_weights(NUM_ACTUATED_DOF);
    l_weights << 1.0,1.0,1.0, //theta
                 100.0,100.0,1000.0, // p
                //  10.0,10.0,200.0, // p
                 0.0,0.0,1.0, // d(theta)
                 1.0,1.0,1.0, // d(p)
                 1.0;         // g

    for(auto k: k_weights){
        k = 1e-6;
    }
    ConvexMPC mpc_solver = ConvexMPC(l_weights, k_weights);

    auto mass_list = robot->getMass();
    robot->printOutBodyNamesInOrder();
    std::cout << mass_list.size() << std::endl; 
    double base_mass=0.0;
    base_mass = mass_list[0]+mass_list[1]+mass_list[4]+mass_list[7]+mass_list[10];
    std::cout << "base_mass : " << base_mass << std::endl;

    Eigen::Vector3d base_euler_prev;
    
    OsqpEigenSolver *solver = new OsqpEigenSolver();
    Eigen::VectorXd p_base_COM;
    Eigen::VectorXd x0(MPC_STATE_DIM);
    raisim::Vec<4> base_quat;
    Eigen::Vector3d base_euler;
    raisim::Vec<3> base_lin_vel;

    raisim::Mat<3,3> wRb_tmp;
    Eigen::Matrix3d wRb;
    raisim::Mat<3,3> base_inertia;

    std::vector<raisim::Vec<3>> p_foot_list(4);
    Eigen::Matrix<double,3,4> com_p_foot;
    Eigen::Matrix<double,3,4> foot_grf_body;
    Eigen::Matrix<double,3,4> foot_grf;

    Eigen::MatrixXd J_c_FR = Eigen::MatrixXd::Zero(3,18);
    Eigen::MatrixXd J_c_FL = Eigen::MatrixXd::Zero(3,18);
    Eigen::MatrixXd J_c_RR = Eigen::MatrixXd::Zero(3,18);
    Eigen::MatrixXd J_c_RL = Eigen::MatrixXd::Zero(3,18);

    Eigen::MatrixXd H;
    Eigen::VectorXd g;
    Eigen::MatrixXd C;
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;

    Eigen::VectorXd B_v_ref = Eigen::VectorXd::Zero(3*MPC_HORIZON);                
    Eigen::VectorXd v_ref = Eigen::VectorXd::Zero(3*MPC_HORIZON);                
    Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(MPC_STATE_DIM*MPC_HORIZON);
    Eigen::VectorXd u;
    
    std::vector<raisim::Mat<3,3>> wR_foot(4);

    Eigen::Vector3d tauFR;
    Eigen::Vector3d tauFL;
    Eigen::Vector3d tauRR;
    Eigen::Vector3d tauRL;
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd generalized_force = Eigen::VectorXd::Zero(18);
    const int totalT = 100000; //100s
    auto com_list = robot->getBodyCOM_W();

    p_base_COM = com_list[0].e();
    // std::cout << "p_base_COM : " << p_base_COM.transpose() << std::endl;    

    auto sphere_traj = world.addSphere(0.01, 1.0, "default",raisim::COLLISION(6), raisim::COLLISION(6));
    sphere_traj->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_traj->setAppearance("green");
    sphere_traj->setPosition(raisim::Vec<3>{p_base_COM(0), p_base_COM(1), p_base_COM(2)});
    auto sphere_COM = world.addSphere(0.01, 1.0, "default",raisim::COLLISION(5), raisim::COLLISION(5));
    sphere_COM->setBodyType(raisim::BodyType::KINEMATIC);
    sphere_COM->setAppearance("red");
    sphere_COM->setPosition(raisim::Vec<3>{p_base_COM(0), p_base_COM(1), p_base_COM(2)});
    std::cout << "initialization done" << std::endl;
    for (int i=0; i<totalT; i++) {

        auto com_list = robot->getBodyCOM_W();
        p_base_COM = com_list[0].e();
        sphere_COM->setPosition(raisim::Vec<3>{p_base_COM(0), p_base_COM(1), p_base_COM(2)});
        robot->getBaseOrientation(base_quat);
    
        Eigen::Quaterniond q(base_quat[0], base_quat[1], base_quat[2], base_quat[3]);
        base_euler = Utils::quat_to_euler(q);  
        x0.segment(0,3) = base_euler;
        x0.segment(3,3) = p_base_COM;
        x0.segment(6,3) = (base_euler-base_euler_prev)/dt;
        robot->getFrameVelocity("floating_base", base_lin_vel);
        x0.segment(9,3) = base_lin_vel.e();
        x0(12) = -9.81; 
        auto inertia_list = robot->getInertia();
        base_inertia = inertia_list[0]; // base inertia (body inertia)

        robot->getFrameOrientation("floating_base", wRb_tmp);
        wRb = wRb_tmp.e();
        
        robot->getFramePosition("FR_foot_fixed", p_foot_list[0]);
        robot->getFramePosition("FL_foot_fixed", p_foot_list[1]);
        robot->getFramePosition("RR_foot_fixed", p_foot_list[2]);
        robot->getFramePosition("RL_foot_fixed", p_foot_list[3]);
        for(auto p: p_foot_list){
            p(2)=0.0;
        }
        for(int i = 0; i<4; i++){
            com_p_foot.block(0,i,3,1) = p_foot_list[i].e() - p_base_COM;
        }   
        B_v_ref = make_base_vel_trajectory(world.getWorldTime());
        sphere_traj->setVelocity(raisim::Vec<3>{B_v_ref(0), B_v_ref(1), B_v_ref(2)}, raisim::Vec<3>{0.0, 0.0, 0.0});
        v_ref = B_v_ref;
        // v_ref = wRb*B_v_ref;
        // std::cout << "v_ref : " << v_ref.transpose() << std::endl;
        x_ref(5) = 0.355;
        for(int i=0; i<MPC_HORIZON; i++){
            x_ref.segment(3+i*MPC_STATE_DIM,3) = sphere_traj->getPosition();//p_base_COM + (i+1)*v_ref*dt; // base position
            x_ref.segment(9+i*MPC_STATE_DIM,3) = v_ref; // base velocity
            x_ref(12+i*MPC_STATE_DIM) = -9.81;
        }
        std::cout.precision(3);
        for(int i=0; i<MPC_HORIZON; i++){
            // std::cout << "x_ref z-position : " << x_ref(5+i*MPC_STATE_DIM) << std::endl;
            // std::cout <<"x_ref z_velocity : " << x_ref(11+i*MPC_STATE_DIM) << std::endl;
        }
        std::cout << "x_ref z-position : " << x_ref(5) << std::endl;
        std::cout << "x0 z-position : " << x0(5) << std::endl;

        // std::cout << "x_ref : " << x_ref.segment(0,MPC_STATE_DIM).transpose() << std::endl;
        // std::cout << "x_ref : " << x_ref.segment(MPC_STATE_DIM,MPC_STATE_DIM).transpose() << std::endl;
        
        // Set up the convex MPC solver
        mpc_solver.calculate_Ac(base_euler(2));
        mpc_solver.calculate_Bc(base_mass,base_inertia.e(), wRb, com_p_foot);
        mpc_solver.state_discretization(dt);
        mpc_solver.calculate_qp_matrices();
        mpc_solver.calculate_hessian();
        mpc_solver.calculate_gradient(x0,x_ref);
        mpc_solver.calculate_constraints();
        
        // MPC QP Conversion
        H = mpc_solver.get_hessian();
        g = mpc_solver.get_gradient();
        C = mpc_solver.get_constraint_matrix();
        lb = mpc_solver.get_lb();
        ub = mpc_solver.get_ub();

        // QP Solver
        solver->init(H, g, C, lb, ub, false);
        solver->solve();
        u = solver->getSolution();
        

        for(int i=0; i<4; i++){
            foot_grf.block(0,i,3,1) = u.segment(i*3,3);
        }
        // std::cout << "foot_grf : " << std::endl<<foot_grf << std::endl;
       
        
        robot->getDenseFrameJacobian("FR_foot_fixed", J_c_FR);
        robot->getDenseFrameJacobian("FL_foot_fixed", J_c_FL);
        robot->getDenseFrameJacobian("RR_foot_fixed", J_c_RR);
        robot->getDenseFrameJacobian("RL_foot_fixed", J_c_RL);
        Eigen::MatrixXd J_c_FR_r = Eigen::MatrixXd::Zero(3,18);
        Eigen::MatrixXd J_c_FL_r = Eigen::MatrixXd::Zero(3,18);
        Eigen::MatrixXd J_c_RR_r = Eigen::MatrixXd::Zero(3,18);
        Eigen::MatrixXd J_c_RL_r = Eigen::MatrixXd::Zero(3,18);
        robot->getDenseFrameRotationalJacobian("FR_foot_fixed", J_c_FR_r);
        robot->getDenseFrameRotationalJacobian("FL_foot_fixed", J_c_FL_r);
        robot->getDenseFrameRotationalJacobian("RR_foot_fixed", J_c_RR_r);
        robot->getDenseFrameRotationalJacobian("RL_foot_fixed", J_c_RL_r);
        robot->getFrameOrientation("FR_foot_fixed", wR_foot[0]);
        robot->getFrameOrientation("FL_foot_fixed", wR_foot[1]);
        robot->getFrameOrientation("RR_foot_fixed", wR_foot[2]);
        robot->getFrameOrientation("RL_foot_fixed", wR_foot[3]);


        // Convert grf to body frame
        for(int i=0; i<4; i++){
            foot_grf_body.block(0,i,3,1) = wRb.transpose()*foot_grf.block(0,i,3,1);
        }
        foot_grf_body = -foot_grf_body;

        tauFR = J_c_FR.block(0,6,3,3).transpose()*foot_grf_body.block(0,0,3,1);
        tauFL = J_c_FL.block(0,9,3,3).transpose()*foot_grf_body.block(0,1,3,1);
        tauRR = J_c_RR.block(0,12,3,3).transpose()*foot_grf_body.block(0,2,3,1);
        tauRL = J_c_RL.block(0,15,3,3).transpose()*foot_grf_body.block(0,3,3,1);


        tau.setZero();
        tau.segment(0,3) = tauFR;
        tau.segment(3,3) = tauFL;
        tau.segment(6,3) = tauRR;
        tau.segment(9,3) = tauRL;

        generalized_force.tail(12) = tau;
        robot->setGeneralizedForce(generalized_force);
        base_euler_prev = base_euler;
        server.integrateWorldThreadSafe();
    }

    // Eigen::VectorXd tauFF = J_c_FF.block(0,6,3,3).transpose()*foot_grf_body.block(0,0,3,1);

    delete solver;
    return 0;   
}