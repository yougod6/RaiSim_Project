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
#include "HOQP.hpp" 
#include "HOQP_Slack.hpp"
#include "Utils.hpp"

// auto contacts  = robot->getContacts();
// for(auto contact : contacts){
//     auto idx = contact.getlocalBodyIndex();
//     std::cout << "Contact Body Index : " << idx << std::endl;
//     auto contact_posi = contact.getPosition();
//     std::cout << "Contact Position : " << contact_posi.e().transpose() << std::endl;
//     auto force = contact.getImpulse();
//     force = force/world.getTimeStep();
//     std::cout << "Contact Force : " << force.e().transpose() << std::endl;
// }

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
    raisim::World world;
    auto ground = world.addGround(0., "default", raisim::COLLISION(-1));
    bool is_attached = true;
    bool is_inclined = true;
    
    raisim::Vec<3> gravity = world.getGravity();
    const double hz = 200;
    world.setTimeStep(1/hz);
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\z1\\aliengo_z1.urdf", "",{}, raisim::COLLISION(-1), raisim::COLLISION(-1));
    // auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\z1\\aliengo_z1_obj.urdf", "",{}, raisim::COLLISION(-1), raisim::COLLISION(-1));
    
    Eigen::VectorXd jointNominalConfig(robot->getGeneralizedCoordinateDim());
    jointNominalConfig << 0.0, 0.0, 0.43, //base position
                          1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3,
                          0.0, 0.6, -1.3,
                          0, 2.57, -1.5,  -1.0,  0. , 0.; // arm joints

    Eigen::VectorXd desired_x = Eigen::VectorXd::Zero(6);
    desired_x << 0.65, 0.0, 0.6,
                0.0, 0.0, 0.0;

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
        terrainProperties.frequency = 0.7;
        terrainProperties.zScale = 1.0;
        terrainProperties.xSize = 20.0;
        terrainProperties.ySize = 20.0;
        terrainProperties.xSamples = 100;
        terrainProperties.ySamples = 100;
        terrainProperties.fractalOctaves = 2;
        terrainProperties.fractalLacunarity = 5.0;
        terrainProperties.fractalGain = 0.25;
        
        offset = 0.45;

        auto hm = world.addHeightMap(0.0, 0.0, terrainProperties);

        jointNominalConfig << 0.0, 0.0, 0.43+offset, //base position
                          1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3,
                          0.0, 0.6, -1.3,
                          0, 2.57, -1.5,  -1.0,  0. , 0.; // arm joints

        desired_x << 0.65, 0.0, offset+0.6,
                    0.0, 0.0, 0.0;
        
        base_desired_x <<   -0.0490382 ,
                    0.00157048,
                    offset+0.401518,// + amplitude*sin(2*M_PI*freq*time) ,
                    0,
                    0,
                    0;
    }
            
    std::unique_ptr<TaskLS> task1 = std::make_unique<TaskLS_ID>(&world, robot, 12, 42);
    std::unique_ptr<TaskLS> task2 = std::make_unique<TaskLS_StationaryFeet>(&world, robot, 12, 42);
    std::unique_ptr<TaskLS_StationaryEE> task3 = std::make_unique<TaskLS_StationaryEE>(&world, robot, 6, 42, desired_x,150,2*sqrt(150));
    std::unique_ptr<TaskLS_MoveBase> task4 = std::make_unique<TaskLS_MoveBase>(&world, robot, 6, 42, base_desired_x);
    std::unique_ptr<TaskLS> task5 = std::make_unique<TaskLS_EnergyOpt>(&world, robot, 42);
    
    Eigen::VectorXd tau_min = -40*Eigen::VectorXd::Ones(18);
    Eigen::VectorXd tau_max = 40*Eigen::VectorXd::Ones(18);
    tau_min.tail(6) = -33*Eigen::VectorXd::Ones(6);
    tau_max.tail(6) = 33*Eigen::VectorXd::Ones(6);
    std::unique_ptr<TaskLS> constraints1 = std::make_unique<TaskLS_TorqueLimits>(tau_min, tau_max);
    std::unique_ptr<TaskLS> constraints2 = std::make_unique<TaskLS_FrictionCone>(&world, robot, 24,42,0.8);

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
        get_contact_feet(robot, contact_feet);
        hoqp->solveAllTasks();
        solution_vector = hoqp->getSolution();
        tau = solution_vector.tail(18);
        generalizedForce.tail(18) = tau;
        robot->setGeneralizedForce(generalizedForce);
        // // 10 seconds later, exert external force on the robot
        if(i>1000 && i<4000){
            make_ee_trajectory(world.getWorldTime(), desired_x,offset);
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