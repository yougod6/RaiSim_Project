#include "ConvexMPC.hpp"

/**
 * <Linear Discrete Time Dynamics>
 * x = [theta, p, w, pdot, g] (13x1)
 * u = [f1, f2, f3, f4] (12x1)
 * x(k+1) = A*x(k) + B*u(k)
 * A(yaw,dt)
 * B(I_hat,m,r,dt), I_hat = Rz(yaw)* B_I * Rz(yaw)'
 * -----------------------------------
 * <QP Formulation>
 * Aqp(A)
 * Bqp(A,B_list)
 * H(Bqp,L,K)
 * g(Bqp,L,Aqp,x0,x_ref)
 * C(mu)
 * lb,ub(fz_min,fz_max)
 * -----------------------------------
 * <GRF Control>
 * foot_grt <- u.block(0,0,12,1), single-step input 
 * where, foot_grf = [f1, f2, f3, f4] (12x1)
 * tau_i = J_c_i' * R_i' *foot_grf_i (i=FR,FL,RR,RL // convert to body frame)
 * tau = sum(tau_i)
 * -----------------------------------
 * Recalculate A, B, H, g, C, lb, ub for each time step
*/



ConvexMPC::ConvexMPC(Eigen::VectorXd& l_weights_, Eigen::VectorXd& k_weights)
{
    mu_ = 0.6;
    fz_min_ = 10;
    fz_max_ = 10000;
    Eigen::VectorXd l_weights_tmp(MPC_STATE_DIM*MPC_HORIZON);
    Eigen::VectorXd k_weights_tmp(NUM_ACTUATED_DOF*MPC_HORIZON);
    for(int i=0; i<MPC_HORIZON; i++)
    {
        l_weights_tmp.segment(i*MPC_STATE_DIM, MPC_STATE_DIM) = l_weights_;
        k_weights_tmp.segment(i*NUM_ACTUATED_DOF, NUM_ACTUATED_DOF) = k_weights;
    }   
    L_ = Eigen::DiagonalMatrix<double, MPC_STATE_DIM*MPC_HORIZON>(l_weights_tmp);
    K_ = Eigen::DiagonalMatrix<double, NUM_ACTUATED_DOF*MPC_HORIZON>(k_weights_tmp);

    // Linear Constraint Matrix C (20k x 3nk) , lb <= CU <= ub
    C_.resize(MPC_CONSTRAINT_DIM*MPC_HORIZON, NUM_ACTUATED_DOF*MPC_HORIZON);
    for(int i=0; i<NUM_LEG*MPC_HORIZON; i++)
    {
        C_(5*i, 3*i) = 1;
        C_(1 + 5*i, 3*i) = 1;
        C_(2 + 5*i, 1 + 3*i) = 1;
        C_(3 + 5*i, 1 + 3*i) = 1;
        C_(4 + 5*i, 2 + 3*i) = 1;

        C_(5*i, 2 + 3*i) = mu_;
        C_(1 + 5*i, 2 + 3*i) = -mu_;
        C_(2 + 5*i, 2 + 3*i) = mu_;
        C_(3 + 5*i, 2 + 3*i) = -mu_;
    }
    std::cout << "--- Convex MPC Solver Constructed ---" << std::endl;
    std::cout << "State Dimension: " << MPC_STATE_DIM << std::endl;
    std::cout << "Horizon: " << MPC_HORIZON << std::endl;
    std::cout << "mu: " << mu_ << std::endl;
    std::cout << "fz_min: " << fz_min_ << std::endl;
    std::cout << "fz_max: " << fz_max_ << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}

void ConvexMPC::reset()
{
    Ac_.setZero();
    Bc_.setZero();
    Bc_list_.setZero();

    Ad_.setZero();
    Bd_.setZero();
    Bd_list_.setZero();

    A_qp_.setZero();
    B_qp_.setZero();

    H_.setZero();
    g_.setZero();
    lb_.setZero();
    ub_.setZero();
}

void ConvexMPC::calculate_Ac(double yaw)
{
    Eigen::Matrix3d R_yaw;
    double c_yaw = cos(yaw);
    double s_yaw = sin(yaw);
    R_yaw << c_yaw, s_yaw, 0,
             -s_yaw, c_yaw, 0,
             0, 0, 1;

    Ac_.block(0,6,3,3) = R_yaw;
    Ac_.block(3,9,3,3) = Eigen::Matrix3d::Identity();
}

void ConvexMPC::calculate_Bc(double base_mass, const Eigen::Matrix3d& base_inertia, Eigen::Matrix3d& wRb, Eigen::Matrix<double,3,4> foot_position)
{
    Eigen::Matrix3d inertia_world;
    inertia_world = wRb * base_inertia * wRb.transpose();
    for (int i = 0; i < NUM_LEG; ++i) {
        Bc_.block(6,3*i,3,3) =
                inertia_world.inverse() * Utils::skew(foot_position.block(0,i,3,1));
        Bc_.block(9,3*i,3,3) =
                (1 / base_mass) * Eigen::Matrix3d::Identity();
    }
}

void ConvexMPC::state_discretization(double dt)
{
    Ad_ = Eigen::Matrix<double,MPC_STATE_DIM,MPC_STATE_DIM>::Identity() + Ac_*dt;
    Bd_ = Bc_*dt;
    for(int i=0; i<MPC_HORIZON; i++){
        Bd_list_.block(i*MPC_STATE_DIM, 0, MPC_STATE_DIM, NUM_ACTUATED_DOF) = Bd_;
    }
}

/**
 * minU (1/2)U'HU + U'g
 * s.t. lb <= CU <= ub
 * A_qp_ = [A', A'^2, ... , A'^k]'
 * B_qp_ = [A^0*B(0)
 *          A^1*B(0),  B(1)
 *          A^2*B(0),  A^1*B(2),  B(2)
 *         ...
 *         A^(k-1)*B(0),  A^(k-2)*B(1), ... , B(k-1)]
*/
void ConvexMPC::calculate_qp_matrices(){
    // caculate A_qp_, B_qp_
    for(int i=0; i<MPC_HORIZON; i++)
    {
        // A_qp_
        if(i==0){
            A_qp_.block(0, 0, MPC_STATE_DIM, MPC_STATE_DIM) = Ad_;
        }
        else{
            A_qp_.block(i*MPC_STATE_DIM, 0, MPC_STATE_DIM, MPC_STATE_DIM) = A_qp_.block((i-1)*MPC_STATE_DIM, 0, MPC_STATE_DIM, MPC_STATE_DIM)*Ad_;
        }

        // B_qp_ , if j>i then zeros
        for(int j=0; j<i+1; j++)
        {
            // B_d_(j)
            if(i==j){
                B_qp_.block(i*MPC_STATE_DIM, j*NUM_ACTUATED_DOF, MPC_STATE_DIM, NUM_ACTUATED_DOF) 
                    = Bd_list_.block(j*MPC_STATE_DIM, 0, MPC_STATE_DIM, NUM_ACTUATED_DOF);
            }
            else{
                B_qp_.block(i*MPC_STATE_DIM, j*NUM_ACTUATED_DOF, MPC_STATE_DIM, NUM_ACTUATED_DOF) 
                    = A_qp_.block((i-j-1)*MPC_STATE_DIM, 0, MPC_STATE_DIM, MPC_STATE_DIM)*B_qp_.block(j*MPC_STATE_DIM,0, MPC_STATE_DIM, NUM_ACTUATED_DOF);
            }
        }
    }
}

void ConvexMPC::calculate_hessian()
{
    H_ = 2 * B_qp_.transpose()*L_*B_qp_;
    H_ += K_;
}   

void ConvexMPC::calculate_gradient(Eigen::VectorXd& x0,Eigen::VectorXd& x_ref)
{
    x0_ = x0;   
    g_ = 2 * B_qp_.transpose()*L_*(A_qp_*x0_ - x_ref);
}   

/**
 * Calculate lb and ub for the constraints
*/
void ConvexMPC::calculate_constraints()
{
    Eigen::VectorXd lb_tmp(MPC_CONSTRAINT_DIM);
    Eigen::VectorXd ub_tmp(MPC_CONSTRAINT_DIM);
    for(int i=0; i<NUM_LEG; i++){
        lb_tmp.segment(i*5,5) << 0.0, -OsqpEigen::INFTY,0.0, -OsqpEigen::INFTY, fz_min_;
        ub_tmp.segment(i*5,5) << OsqpEigen::INFTY,0.0, OsqpEigen::INFTY, 0.0, fz_max_;
    }

    for(int i=0; i<MPC_HORIZON; i++)
    {
        lb_.segment(i*MPC_CONSTRAINT_DIM, MPC_CONSTRAINT_DIM) = lb_tmp;
        ub_.segment(i*MPC_CONSTRAINT_DIM, MPC_CONSTRAINT_DIM) = ub_tmp;
    }
}

Eigen::MatrixXd ConvexMPC::get_hessian()
{
    return H_;
}

Eigen::VectorXd ConvexMPC::get_gradient()
{
    return g_;
}

Eigen::MatrixXd ConvexMPC::get_constraint_matrix()
{
    return C_;
}

Eigen::VectorXd ConvexMPC::get_lb()
{
    return lb_;
}

Eigen::VectorXd ConvexMPC::get_ub()
{
    return ub_;
}
