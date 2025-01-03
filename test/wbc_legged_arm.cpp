#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "TaskLS_ID.hpp"
#include "TaskLS_StationaryFeet.hpp"
#include "TaskLS_MoveBase.hpp"
#include "TaskLS_StationaryEE.hpp"
#include "TaskLS_MinMotion.hpp"
#include "TaskLS_EnergyOpt.hpp"
#include "TaskLS_TorqueLimits.hpp"
#include "TaskLS_FrictionCone.hpp"
#include "RobotStateRaisim.hpp"
#include "HOQP.hpp" 
#include "HOQP_Slack.hpp"
#include "Utils.hpp"

void make_ee_trajectory(const double time, Eigen::VectorXd& desired_x,double offset=0.0){
    const double amplitude = 0.2; //0.15
    const double freq = 0.5;
    desired_x << 0.65,
                 0.0+2.*amplitude*sin(2*M_PI*freq*time),
                 offset+0.6-0.5*(amplitude*cos(2*M_PI*freq*time)-amplitude),
                0.0, 0.0, 0.0;
}

/**
 * @brief Get the contact feet with the ground
 * @return std::vector<bool> contact_feet = {FR, FL, RR, RL} , true if contact, false otherwise
 */
void get_contact_feet(raisim::ArticulatedSystem* robot, std::vector<bool>& contact_feet){
    auto contacts = robot->getContacts();
    for(auto contact : contacts){
        auto idx = contact.getlocalBodyIndex();
        if(idx == 3) contact_feet[0] = true;
        if(idx == 6) contact_feet[1] = true;
        if(idx == 9) contact_feet[2] = true;
        if(idx == 12) contact_feet[3] = true;
    }
}


int main (int argc, char* argv[]) {
    bool is_attached = true;
    bool is_inclined = true;
    double hz = 1000;
    for(int i=1; i<argc; i++){
        if(i==1) is_attached = std::stoi(argv[i]);
        if(i==2) is_inclined = std::stoi(argv[i]);
        if(i==3) hz = std::stod(argv[i]);
    }
    std::cout << "is_attached : " << is_attached << std::endl;
    std::cout << "is_inclined : " << is_inclined << std::endl;
    std::cout << "hz : " << hz << std::endl;

    raisim::World world;
    auto ground = world.addGround(0., "default");//, raisim::COLLISION(-1));
    
    raisim::Vec<3> gravity = world.getGravity();
    world.setTimeStep(1/hz);
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\z1\\aliengo_z1.urdf");//, "",{}, raisim::COLLISION(-1), raisim::COLLISION(-1));
    // auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\z1\\aliengo_z1_obj.urdf", "",{}, raisim::COLLISION(-1), raisim::COLLISION(-1));
    
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim());
    jointNominalConfig << 0.0, 0.0, 0.43, //base position
                          1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3,
                          0.0, 0.6, -1.3,
                          0, 2.57, -1.5,  -1.0,  0. , 0.; // arm joints

    Eigen::VectorXd desired_x = Eigen::VectorXd::Zero(7);
    desired_x << 0.65, 0.0, 0.6,
                1.0, 0.0, 0.0, 0.0;

    Eigen::VectorXd base_desired_x = Eigen::VectorXd::Zero(6);
    base_desired_x <<   -0.0490382 ,
                        0.00157048,
                        0.401518,// + amplitude*sin(2*M_PI*freq*time) ,
                        0,
                        0,
                        0;
    double offset = 0.0;
    
    if(is_inclined){
        raisim::TerrainProperties terrainProperties;
        terrainProperties.frequency = 0.9;
        terrainProperties.zScale = 1.0;
        terrainProperties.xSize = 20.0;
        terrainProperties.ySize = 20.0;
        terrainProperties.xSamples = 100;
        terrainProperties.ySamples = 100;
        terrainProperties.fractalOctaves = 1;
        terrainProperties.fractalLacunarity = 5.0;
        terrainProperties.fractalGain = 0.25;
        
        offset = 0.45;

        auto hm = world.addHeightMap(0.0, 0.0, terrainProperties);
        hm->setAppearance("0.72, 0.49, 0.3, 1.0"); //"26/255, 72/255, 33/255, 1.0"
        jointNominalConfig << 0.0, 0.0, 0.43+offset, //base position
                          0.9914449 ,0, -0.130526, 0,//1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3,
                          0.0, 0.6, -1.3,
                          0, 2.57, -1.5,  -1.0,  0. , 0.; // arm joints

        desired_x << 0.65, 0.0, offset+0.6,
                    1.0, 0.0, 0.0, 0.0;
        
        base_desired_x <<   -0.0490382 ,
                    0.00157048,
                    offset+0.401518,// + amplitude*sin(2*M_PI*freq*time) ,
                    0,
                    0,
                    0;
    }
    
    std::unique_ptr<RobotStateRaisim> robot_state_ptr = std::make_unique<RobotStateRaisim>(&world, robot);
    robot_state_ptr->updateState();        
    std::unique_ptr<TaskLS> task1 = std::make_unique<TaskLS_ID>(robot_state_ptr.get(), 12, 42);
    std::unique_ptr<TaskLS> task2 = std::make_unique<TaskLS_StationaryFeet>(robot_state_ptr.get(), 12, 42);
    std::unique_ptr<TaskLS_StationaryEE> task3 = std::make_unique<TaskLS_StationaryEE>(robot_state_ptr.get(), 6, 42, desired_x,300,2*sqrt(150));
    std::unique_ptr<TaskLS_MoveBase> task4 = std::make_unique<TaskLS_MoveBase>(robot_state_ptr.get(), 6, 42, base_desired_x);
    std::unique_ptr<TaskLS> task5 = std::make_unique<TaskLS_EnergyOpt>(robot_state_ptr.get(), 42);
    
    Eigen::VectorXd tau_min = -40*Eigen::VectorXd::Ones(18);
    Eigen::VectorXd tau_max = 40*Eigen::VectorXd::Ones(18);
    tau_min.tail(6) = -33*Eigen::VectorXd::Ones(6);
    tau_max.tail(6) = 33*Eigen::VectorXd::Ones(6);
    std::unique_ptr<TaskLS> constraints1 = std::make_unique<TaskLS_TorqueLimits>(tau_min, tau_max);
    std::unique_ptr<TaskLS> constraints2 = std::make_unique<TaskLS_FrictionCone>(robot_state_ptr.get(), 24,42,0.2);

    std::unique_ptr<TaskSet> task_set1 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set2 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set3 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set4 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set5 = std::make_unique<TaskSet>(42);
    task_set1->addEqualityTask(task1.get());
    task_set2->addEqualityTask(task2.get());
    task_set2->addInequalityTask(constraints1.get());
    task_set3->addEqualityTask(task3.get());
    task_set3->addInequalityTask(constraints2.get());
    task_set4->addEqualityTask(task4.get());
    task_set5->addEqualityTask(task5.get());

    std::unique_ptr<HOQP_Slack> hoqp = std::make_unique<HOQP_Slack>();

    hoqp->addTask(task_set1.get());
    hoqp->addTask(task_set2.get());
    hoqp->addTask(task_set3.get());
    hoqp->addTask(task_set4.get());
    hoqp->addTask(task_set5.get());
    hoqp->init();   

    Eigen::VectorXd solution_vector;
    Eigen::VectorXd tau;
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(robot->getDOF());
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);
    
    auto ee_traj_line = server.addVisualPolyLine("EndEffectorTrajectory");
    ee_traj_line->color = {1,0,0,1};
    ee_traj_line->width = 0.01;
    ee_traj_line->points.push_back(desired_x.head(3));
    ee_traj_line->points.push_back(desired_x.head(3)+1e-3*Eigen::Vector3d::Identity());

    if(is_attached){
        // auto pin = world.addSphere(0.05, 0.01,"steel",raisim::COLLISION(1),raisim::COLLISION(1));
        // pin->setBodyType(raisim::BodyType::DYNAMIC);
        // pin->setPosition(raisim::Vec<3>{0.6,0.0,0.5});
        auto pin2 = world.addBox(0.1, 0.4, 0.1, 0.5,"steel",raisim::COLLISION(2),raisim::COLLISION(2));
        pin2->setBodyType(raisim::BodyType::DYNAMIC);
        pin2->setPosition(raisim::Vec<3>{0.6,0.0,0.5+offset});
        // auto wire = world.addCompliantWire(pin, 0, {0,0,0}, robot, 0, {0., 0, 0}, 2.0, 1000);
        auto wire = world.addStiffWire(pin2,0,{0,0,0},robot,18,{0.05,0,0},0.1);
        wire->setStretchType(raisim::LengthConstraint::StretchType::BOTH);
    }

    std::vector<bool> contact_feet = {false,false,false,false};//FR, FL, RR, RL
    robot->setGeneralizedCoordinate(jointNominalConfig);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF())); 
    robot->setName("aliengo + Z1");
    robot->printOutBodyNamesInOrder();

    for (int i=0; i<100000; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6)
        robot_state_ptr->updateState();        
        get_contact_feet(robot, contact_feet);
        hoqp->solveAllTasks();
        solution_vector = hoqp->getSolution();
        tau = solution_vector.tail(18);
        generalizedForce.tail(18) = tau;
        robot->setGeneralizedForce(generalizedForce);
        // // 10 seconds later, exert external force on the robot
        if(i>1000 && i<4000){
            // make_ee_trajectory(world.getWorldTime(), desired_x,offset);
            task3->updateDesiredEEPose(desired_x);
            server.lockVisualizationServerMutex();
            if (ee_traj_line->points.size() > 500){
                ee_traj_line->points.erase(ee_traj_line->points.begin());
            }
            ee_traj_line->points.push_back(desired_x.head(3));
            // if(is_attached && i>2000)
            //     pin2->setMass(0.2);
           
            server.unlockVisualizationServerMutex();
            // trajectory_visualization(ee_traj_line, desired_x);
        }
        
        server.integrateWorldThreadSafe();
    }
    server.killServer();

    return 0;
}