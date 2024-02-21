#include "HOQP.hpp"

HOQP::HOQP()
{
    solver_ = new OsqpEigenSolver();
    task_num_ = 0;
    
}

HOQP::~HOQP()
{
    delete solver_;
}


void HOQP::addTask(TaskLS* task)
{
    stacked_tasks_.push_back(task);
    task_num_++;
}

void HOQP::init()
{
    A_eqs_ = Eigen::MatrixXd::Zero(12,30);
    A_eqs_.block(0,18,12,12) = Eigen::MatrixXd::Identity(12,12);
    lb_eqs_ = Eigen::VectorXd::Zero(12);
    ub_eqs_ = Eigen::VectorXd::Zero(12);
    lb_eqs_ << -23.7, -23.7, -35.55, -23.7, -23.7, -35.55, -23.7, -23.7, -35.55, -23.7, -23.7, -35.55;
    ub_eqs_ = -lb_eqs_;
}

double HOQP::getTaskNum()
{
    return task_num_;
}   

void HOQP::solve()
{
    // solver_->init(H, g, A, l, u, Aeq, beq);
    // solver_->solve();
}

void HOQP::solveAllTasks()
{
    init();
    for (TaskLS* task : stacked_tasks_) {
        // Set Objective Function
        Eigen::MatrixXd H_tmp;
        H_tmp = (task->getMatrix().transpose()*task->getMatrix());
        H_ = H_tmp.sparseView();
        g_ = -task->getMatrix().transpose()*task->getVector(); 
        std::cout << "------------------------------------------------" << std::endl; 
        std::cout << task->getTaskName() << std::endl;
        std::cout << "------------------------------------------------" << std::endl; 
        std::cout << "size of A_eqs_ is " << A_eqs_.rows() << " by " << A_eqs_.cols() << std::endl;
        std::cout << "size of lb_eqs_ is " << lb_eqs_.rows() << " by " << lb_eqs_.cols() << std::endl;
        std::cout << "size of ub_eqs_ is " << ub_eqs_.rows() << " by " << ub_eqs_.cols() << std::endl;
        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "size of H_ is " << H_.rows() << " by " << H_.cols() << std::endl; 
        std::cout << "size of g_ is " << g_.rows() << " by " << g_.cols() << std::endl;
        std::cout << "------------------------------------------------" << std::endl; 

        
        // // Solve QP
        Eigen::SparseMatrix<double> A_eqs = A_eqs_.sparseView();

        solver_->init(H_, g_, A_eqs, lb_eqs_, ub_eqs_,false);
        
        
        solver_->solve();
        xopt_ = solver_->getSolution();

        // Stack Equality Constraints
        A_eqs_.resize(A_eqs_.rows()+task->getMatrix().rows(), Eigen::NoChange);
        A_eqs_.bottomRows(task->getMatrix().rows()) = task->getMatrix();
        // Stack lower, upper bounds
        lb_eqs_.resize(lb_eqs_.rows()+task->getMatrix().rows(), Eigen::NoChange);
        Eigen::VectorXd epsilonVec = Eigen::VectorXd::Ones(task->getMatrix().rows());
        epsilonVec = epsilonVec*1;
        lb_eqs_.bottomRows(task->getMatrix().rows()) = task->getMatrix()*xopt_ - epsilonVec;

        ub_eqs_.resize(ub_eqs_.rows()+task->getMatrix().rows(), Eigen::NoChange);
        ub_eqs_.bottomRows(task->getMatrix().rows()) = task->getMatrix()*xopt_ + epsilonVec;

    }
}

void HOQP::updateAllTasks()
{
    for (TaskLS* task : stacked_tasks_) {
        task->updateMatrix();
        task->updateVector();
    }
}

Eigen::VectorXd HOQP::getSolution()
{
    return xopt_;
}