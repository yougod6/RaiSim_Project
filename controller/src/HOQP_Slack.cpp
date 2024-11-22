#include "HOQP_Slack.hpp"

HOQP_Slack::HOQP_Slack()
{
    solver_ = new OsqpEigenSolver();
    task_num_ = 0;
    inequality_contraints_dim_ = 0;
    stacked_constraints_dim_ = 0;
    var_dim_ = 0;
}

HOQP_Slack::~HOQP_Slack()
{
    delete solver_;
}

void HOQP_Slack::addTask(TaskSet* task)
{
    stacked_tasks_.push_back(task);
    task_num_++;
    inequality_contraints_dim_ += task->getInequalityTaskDim();
}


void HOQP_Slack::init()
{
    if(task_num_<1){
        std::cout << "No task added" << std::endl;
        return;
    }
    updateAllTasks();
    std::cout << "HOQP_Slack initialized" << std::endl;
    var_dim_ = stacked_tasks_[0]->getVarDim();
    xopt_ = Eigen::VectorXd::Zero(var_dim_);
    Z_ = Eigen::MatrixXd::Identity(var_dim_, var_dim_);
    // for(auto task : stacked_tasks_){
    //     for(auto name : task->getEqualityTaskNames()){
    //         std::cout << name <<std::endl;
    //     }
    //     for(auto name : task->getInequalityTaskNames()){
    //         std::cout << name <<std::endl;
    //     }
    // }
}

void HOQP_Slack::solve()
{
    A_eq_ = current_task_->getEqualityMatrix();
    b_eq_ = current_task_->getEqualityVector();
    // std::cout << current_task_->getEqualityTaskNames()[0] << std::endl;
    // std::cout << "A_eq_ : " << A_eq_.rows() << " x " << A_eq_.cols() << std::endl;
    // std::cout << "b_eq_ : " << b_eq_.rows() << std::endl;
    if(inequality_contraints_dim_ == task_num_){
        H_ = Z_.transpose()*A_eq_.transpose()*A_eq_*Z_;
        g_ = Z_.transpose()*A_eq_.transpose()*(A_eq_*xopt_-b_eq_);
        solver_->init(H_, g_,false);
        solver_->solve();
        zopt_ = solver_->getSolution();
    }
    else{
        D_ineq_ = current_task_->getInequalityMatrix();
        D_ineq_vec_.push_back(D_ineq_);
        f_ineq_ = current_task_->getInequalityVector();
        f_ineq_vec_.push_back(f_ineq_);
        z_dim_ = Z_.cols();
        v_dim_ = D_ineq_.rows();
        H_ = Eigen::MatrixXd::Zero(z_dim_+v_dim_, z_dim_+v_dim_);
        H_.block(0,0,z_dim_,z_dim_) = Z_.transpose()*A_eq_.transpose()*A_eq_*Z_;
        H_.block(z_dim_,z_dim_,v_dim_,v_dim_) = Eigen::MatrixXd::Identity(v_dim_,v_dim_);
        g_ = Eigen::VectorXd::Zero(z_dim_+v_dim_);
        g_.segment(0,z_dim_) = Z_.transpose()*A_eq_.transpose()*(A_eq_*xopt_-b_eq_);
        stacked_constraints_dim_ += current_task_->getInequalityTaskDim();
        D_tilde = Eigen::MatrixXd::Zero(stacked_constraints_dim_ + v_dim_, z_dim_+v_dim_);
        f_tilde = Eigen::VectorXd::Zero(stacked_constraints_dim_ + v_dim_);
        int row=0;
        for(int i=D_ineq_vec_.size()-1; i>=0; i--){
            D_tilde.block(row,0,D_ineq_vec_[i].rows(), Z_.cols()) = D_ineq_vec_[i]*Z_;
            if(i==D_ineq_vec_.size()-1){
                f_tilde.segment(0,D_ineq_vec_[i].rows()) = f_ineq_vec_[i] - D_ineq_vec_[i]*xopt_;
            }
            else{
                f_tilde.segment(row,D_ineq_vec_[i].rows()) = f_ineq_vec_[i] - D_ineq_vec_[i]*xopt_ + vopt_vec_[i];
            }
            // std::cout << "i : " << i << std::endl;
            // std::cout << "row : " << row << std::endl;  
            row += D_ineq_vec_[i].rows();
        } 
        D_tilde.block(0,z_dim_,v_dim_,v_dim_) = -Eigen::MatrixXd::Identity(v_dim_,v_dim_);
        D_tilde.block(row,z_dim_,v_dim_,v_dim_) = -Eigen::MatrixXd::Identity(v_dim_,v_dim_);
        
        ub_vec_ = f_tilde;
        lb_vec_ = -std::numeric_limits<double>::infinity()*Eigen::VectorXd::Ones(ub_vec_.rows());
        // std::cout << "D_tilde : " << D_tilde.rows() << " x " << D_tilde.cols() << std::endl;
        // std::cout << "lb_vec_ : " << lb_vec_.rows() << std::endl;
        // std::cout << "ub_vec_ : " << ub_vec_.rows() << std::endl;
        // 출력되는 소수점 1자리로 제한
        // std::cout << std::fixed << std::setprecision(1); // 소수점 이하 자릿수 설정
        // std::cout << "D : \n" << D_tilde << std::endl;
        // std::cout << "lb_vec_ : \n" << lb_vec_ << std::endl;
        // std::cout << "ub_vec_ : \n" << ub_vec_ << std::endl;
        // std::cout << "inequality constraints dimension : " << inequality_contraints_dim_ << std::endl;
        // std::cout << "stacked_constraints_dim_ : " << stacked_constraints_dim_ << std::endl;
        
        // std::cout << "z_dim : " << z_dim_ << std::endl;
        // std::cout << "v_dim : " << v_dim_ << std::endl;
        solver_->init(H_, g_, D_tilde, lb_vec_, ub_vec_, false);
        solver_->solve();
        Eigen::VectorXd x = solver_->getSolution();
        zopt_ = x.segment(0,Z_.cols());
        vopt_vec_.push_back(x.segment(z_dim_,v_dim_));
        // std::cout << "zopt : " << zopt_ << std::endl;
        // for(auto v : vopt_vec_){
        //     std::cout << "v : " << v << std::endl;
        // }
        // std::cout << "--------------------------------" << std::endl;
        // std::cout << "D_ineq_vec_ size : " << D_ineq_vec_.size() << std::endl;
        // std::cout << "f_ineq_vec_ size : " << f_ineq_vec_.size()<< std::endl;
        // std::cout << "vopt_vec_ size : " << vopt_vec_.size()<< std::endl;
    }
    
}

void HOQP_Slack::solveAllTasks()
{
    // std::cout << " ------------------------ Start solving all tasks ------------------------ " << std::endl;
    updateAllTasks();
    xopt_ = Eigen::VectorXd::Zero(var_dim_);
    Z_ = Eigen::MatrixXd::Identity(var_dim_, var_dim_);
    D_ineq_vec_.clear();
    f_ineq_vec_.clear();
    vopt_vec_.clear();  
    stacked_constraints_dim_ = 0;
    for (TaskSet* task : stacked_tasks_) {
        current_task_ = task;
        if(current_task_->getEqualityTaskDim() != 0){
            solve();
            xopt_ = xopt_ + Z_*zopt_;
            if(current_task_!=stacked_tasks_[task_num_-1])
                Z_ = Z_*Utils::compute_nullspace_QR(A_eq_*Z_);
        }
        
        // auto name = current_task_->getEqualityTaskNames();
        // std::cout << "##############  "<<name[0] <<" is solved!  ##############" << std::endl;
    }
    // std::cout << " ------------------------ All tasks solved ------------------------ " << std::endl;
}

void HOQP_Slack::updateAllTasks()
{
    for (TaskSet* task : stacked_tasks_) {
        task->updateAllTasks();
    }
}

Eigen::VectorXd HOQP_Slack::getSolution()
{
    return xopt_;
}