#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "osqp.h"
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

int main(int argc, char* argv[]) {

    auto binaryPath = raisim::Path::setFromArgv(argv[0]);

    raisim::World world;
    world.setTimeStep(0.001);
    
    auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");
    auto ground = world.addGround();

    Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointVelocityTarget(go1->getDOF());

        jointNominalConfig << 0.0, 0.0, 0.54, //base position
                                                1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                                                0.03, 0.2, -1.2, //
                                                -0.03, 0.2, -1.2,
                                                0.03, -0.2, 1.2,
                                                -0.03, -0.2, 1.2;
    jointVelocityTarget.setZero();

    for (int i=0; i<go1->getDOF(); i++)
        jointVelocityTarget[i] = (i+4)*0.01;


    go1->setGeneralizedCoordinate(jointNominalConfig);
    go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF())); 
    go1->setName("go1");
    
    Eigen::MatrixXd M;      // Mass matrix (18x18)
    Eigen::VectorXd h;      // Coriolis and gravity forces

    Eigen::MatrixXd A_eom;  // System dynamics matrix
    Eigen::VectorXd b_eom;  // Constraints vector
    Eigen::VectorXd xopt;      // Decision variables vector
    Eigen::VectorXd qddot;  // Joint accelerations
    Eigen::VectorXd f_c; // Contact forces
    Eigen::VectorXd tau;  // Joint torques
    constexpr double tolerance = 1e-4;

    M = go1->getMassMatrix().e();
    std::cout << "mass matrix's size : " << M.rows() << "x" << M.cols() << std::endl;
    h = go1->getNonlinearities(world.getGravity()).e();
    qddot = go1->getGeneralizedAcceleration().e();
    
    Eigen::MatrixXd J_c_FR = Eigen::MatrixXd::Zero(6, go1->getDOF());
    Eigen::MatrixXd J_c_FL = Eigen::MatrixXd::Zero(6, go1->getDOF());
    Eigen::MatrixXd J_c_RR = Eigen::MatrixXd::Zero(6, go1->getDOF());
    Eigen::MatrixXd J_c_RL = Eigen::MatrixXd::Zero(6, go1->getDOF());

    Eigen::MatrixXd J_c = Eigen::MatrixXd::Zero(24, go1->getDOF());// 24x18
    J_c.block(0, 0, 6, go1->getDOF()) = J_c_FR;
    J_c.block(6, 0, 6, go1->getDOF()) = J_c_FL;
    J_c.block(12, 0, 6, go1->getDOF()) = J_c_RR;
    J_c.block(18, 0, 6, go1->getDOF()) = J_c_RL;
    std::cout << "J_c's size : " << J_c.rows() << "x" << J_c.cols() << std::endl;

    // go1->getForceAtJointInWorldFrame


    go1->getDenseFrameJacobian("FR_foot_fixed", J_c_FR); // 6x18
    go1->getDenseFrameJacobian("FL_foot_fixed", J_c_FL); // 6x18
    go1->getDenseFrameJacobian("RR_foot_fixed", J_c_RR); // 6x18
    go1->getDenseFrameJacobian("RL_foot_fixed", J_c_RL); // 6x18

    const int actuated_joint_num = go1->getNumberOfJoints()-1;

    //selection matrix that selects the joint torques from the generalized coordinates
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(actuated_joint_num, go1->getGeneralizedCoordinateDim()-1); // 12x18
    // 12x6 zero matrix, 12x12 identity matrix
    S.block(0,0,12,6) = Eigen::MatrixXd::Zero(12,6);
    S.block(0,6,12,12) = Eigen::MatrixXd::Identity(12,12);
    std::cout << "S's size : " << S.rows() << "x" << S.cols() << std::endl;

    // M, -J', -S' 를 열로 붙인 행렬
    A_eom = Eigen::MatrixXd::Zero(18,54);
    A_eom.block(0,0,18,18) = M;
    A_eom.block(0,18,18,24) = -J_c.transpose();
    A_eom.block(0,42,18,12) = -S.transpose();
    std::cout << "A_eom's size : " << A_eom.rows() << "x" << A_eom.cols() << std::endl;

    b_eom = Eigen::VectorXd::Zero(18);
    b_eom = h;

    Eigen::MatrixXd H = A_eom.transpose()*A_eom;
    Eigen::VectorXd g = -A_eom.transpose()*b_eom;

    OsqpEigen::Solver solver;
    Eigen::SparseMatrix<double> P = H.sparseView();
    Eigen::VectorXd q = g;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12,54);
    A.block(0,42,12,12) = Eigen::MatrixXd::Identity(12,12);
    Eigen::SparseMatrix<double> A_sparse = A.sparseView();

    solver.settings()->setVerbosity(false);  // Adjust verbosity as needed
    solver.data()->setNumberOfVariables(54);
    solver.data()->setNumberOfConstraints(12);
    solver.data()->setHessianMatrix(P);
    solver.data()->setGradient(q);
    Eigen::VectorXd lb(12), ub(12);
    lb = go1->getActuationLowerLimits().e().tail(actuated_joint_num);
    ub = go1->getActuationUpperLimits().e().tail(actuated_joint_num);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);
    solver.data()->setLinearConstraintsMatrix(A_sparse);

    if (!solver.initSolver()) {
        std::cerr << "Error initializing the solver." << std::endl;
        return -1;
    }

    // Solve the QP problem and get the solution
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "Error solving the QP problem." << std::endl;
        return -1;
    }

    // Get the solution vector
    xopt = solver.getSolution();
    qddot  = xopt.segment(0, 18);
    f_c = xopt.segment(18, 24);
    tau = xopt.segment(42, 12);
    std::cout << "qddot : " << qddot << std::endl;
    std::cout << "f_c : " << f_c << std::endl;
    std::cout << "tau : " << tau << std::endl;

    // Extract the control input from the solution vector (adjust indices accordingly)
    // Eigen::VectorXd controlInput = solution.segment(/* Define the appropriate segment indices */, /* Define the appropriate segment size */);



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
