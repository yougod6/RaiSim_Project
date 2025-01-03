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

void make_ee_trajectory(const double time, Eigen::VectorXd& desired_x,double offset=0.0){
    const double amplitude = 0.2; //0.15
    const double freq = 0.5;
    desired_x << 0.65,
                 0.0+2.*amplitude*sin(2*M_PI*freq*time),
                 offset+0.6-0.5*(amplitude*cos(2*M_PI*freq*time)-amplitude),
                0.0, 0.0, 0.0;
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

void get_center_of_contact(raisim::ArticulatedSystem* robot, Eigen::VectorXd& center_of_contact){
    auto contacts = robot->getContacts();
    center_of_contact = Eigen::VectorXd::Zero(3);
    for(auto contact : contacts){
        auto idx = contact.getlocalBodyIndex();
        if(idx == 3) {
            // std::cout << "FR : " <<contact.getPosition().e().transpose() << std::endl;
            center_of_contact += contact.getPosition().e();
            }
        if(idx == 6) {
            // std::cout << "FL : " << contact.getPosition().e().transpose() << std::endl;
            center_of_contact += contact.getPosition().e();
            }
        if(idx == 9){
            // std::cout << "RR : " << contact.getPosition().e().transpose() << std::endl;
            center_of_contact += contact.getPosition().e();
        }
        if(idx == 12){
            // std::cout << "RL : " << contact.getPosition().e().transpose() << std::endl;
            center_of_contact += contact.getPosition().e();
        }
    }
    center_of_contact /= 4;
}

int main (int argc, char* argv[]) {
    bool is_attached = true;
    bool is_inclined = true;
    double hz = 100;
    for(int i=1; i<argc; i++){
        if(i==1) is_attached = std::stoi(argv[i]);
        if(i==2) is_inclined = std::stoi(argv[i]);
        if(i==3) hz = std::stod(argv[i]);
    }
    std::cout << "is_attached : " << is_attached << std::endl;
    std::cout << "is_inclined : " << is_inclined << std::endl;
    std::cout << "hz : " << hz << std::endl;

    raisim::World world;
    auto ground = world.addGround(0., "brass", raisim::COLLISION(-1));
    raisim::Vec<3> gravity = world.getGravity();
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
        double slope_anlge_deg = 15.;
        double robot_anlge_deg = 5.;

        double slope_angle_rad = slope_anlge_deg*M_PI/180;
        double robot_angle_rad = robot_anlge_deg*M_PI/180;
        Eigen::Matrix3d slope_rot = Eigen::Matrix3d::Zero(3,3);
        Eigen::Matrix3d robot_rot = Eigen::Matrix3d::Zero(3,3);
        slope_rot << cos(slope_angle_rad), 0, sin(slope_angle_rad),
                            0, 1, 0,
               -sin(slope_angle_rad), 0, cos(slope_angle_rad);
        robot_rot << cos(robot_angle_rad), 0, sin(robot_angle_rad),
                            0, 1, 0,
               -sin(robot_angle_rad), 0, cos(robot_angle_rad);
        double size_x = 2.0;
        double size_z = 0.05;
        auto slope = world.addBox(size_x,1,size_z,100,"brass",raisim::COLLISION(-1),raisim::COLLISION(-1));
        slope->setBodyType(raisim::BodyType::STATIC);
        slope->setPosition(raisim::Vec<3>{0.0,0.0,size_x*sin(slope_angle_rad)/2});
        slope->setOrientation(slope_rot);
        robot->getCollisionBody("FL_foot").setMaterial("steel");
        robot->getCollisionBody("FR_foot").setMaterial("steel");
        robot->getCollisionBody("RL_foot").setMaterial("steel");
        robot->getCollisionBody("RR_foot").setMaterial("steel");        
        offset =size_x*sin(slope_angle_rad)/2 + size_z/2;
        Eigen::Quaterniond slope_quat = Eigen::Quaterniond(slope_rot);
        Eigen::Quaterniond robot_quat = Eigen::Quaterniond(robot_rot);

        jointNominalConfig << 0.0, 0.0, 0.43+offset, //base position
                          robot_quat.w() ,robot_quat.x(), robot_quat.y(), robot_quat.z(),//1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3,
                          0.0, 0.6, -1.3,
                          0, 2.57, -1.5,  -1.0,  0. , 0.; // arm joints
        Eigen::Vector3d euler = Utils::quat_to_euler(slope_quat);


        desired_x << 0.65, 0.0, offset+0.6,
                    0.0, 0.0, 0.0;
        desired_x.tail(3) = euler;  
        base_desired_x <<   -0.0490382 ,
                    0.00157048,
                    offset+0.43,// + amplitude*sin(2*M_PI*freq*time) ,
                    0,
                    0,
                    0;
        base_desired_x.tail(3) = euler;
            
        auto sphere = world.addSphere(0.05, 0.01,"steel",raisim::COLLISION(1),raisim::COLLISION(1));
        sphere->setBodyType(raisim::BodyType::KINEMATIC);
        sphere->setPosition(raisim::Vec<3>{-0.0490382,0.00157048,offset+0.43});
    }
            
    std::unique_ptr<TaskLS> task1 = std::make_unique<TaskLS_ID>(&world, robot, 12, 42);
    std::unique_ptr<TaskLS> task2 = std::make_unique<TaskLS_StationaryFeet>(&world, robot, 12, 42);
    std::unique_ptr<TaskLS_StationaryEE> task3 = std::make_unique<TaskLS_StationaryEE>(&world, robot, 6, 42, desired_x,300,2*sqrt(150));
    std::unique_ptr<TaskLS_MoveBase> task4 = std::make_unique<TaskLS_MoveBase>(&world, robot, 6, 42, base_desired_x, 300, 2*sqrt(150));
    std::unique_ptr<TaskLS> task5 = std::make_unique<TaskLS_EnergyOpt>(&world, robot, 42);
    
    Eigen::VectorXd tau_min = -40*Eigen::VectorXd::Ones(18);
    Eigen::VectorXd tau_max = 40*Eigen::VectorXd::Ones(18);
    tau_min.tail(6) << -30, -60, -30, -30, -30, -30;
    tau_max.tail(6) << 30, 60, 30, 30, 30, 30;
    std::unique_ptr<TaskLS> constraints1 = std::make_unique<TaskLS_TorqueLimits>(tau_min, tau_max);
    std::unique_ptr<TaskLS> constraints2 = std::make_unique<TaskLS_FrictionCone>(&world, robot, 24,42,0.1);

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

    std::vector<bool> contact_feet = {false,false,false,false};//FR, FL, RR, RL
    robot->setGeneralizedCoordinate(jointNominalConfig);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF())); 
    robot->setName("aliengo + Z1");
    // robot->printOutBodyNamesInOrder();
    world.setMaterialPairProp("steel", "brass", 0.2, 0., 0., 0.95, 0.01);
    auto proper = world.getMaterialPairProperties("brass", "steel");
    std::cout << "friction btw steels : " << proper.c_static_f << std::endl;
    if(is_attached){
        // auto pin = world.addSphere(0.05, 0.01,"steel",raisim::COLLISION(1),raisim::COLLISION(1));
        // pin->setBodyType(raisim::BodyType::DYNAMIC);
        // pin->setPosition(raisim::Vec<3>{0.6,0.0,0.5});
        double wire_length = 0.2;
        raisim::Vec<3> ee_posi_tmp;
        robot->getFramePosition("joint6", ee_posi_tmp);
        ee_posi_tmp = ee_posi_tmp + raisim::Vec<3>{0,0,-wire_length};
        auto pin2 = world.addBox(0.15, 0.5, 0.15, 0.5,"steel",raisim::COLLISION(2),raisim::COLLISION(2));
        pin2->setBodyType(raisim::BodyType::DYNAMIC);
        pin2->setPosition(ee_posi_tmp);
        // auto wire = world.addCompliantWire(pin, 0, {0,0,0}, robot, 0, {0., 0, 0}, 2.0, 1000);
        auto wire = world.addStiffWire(pin2,0,{0,0,0},robot,18,{0.05,0,0},wire_length);
        wire->setStretchType(raisim::LengthConstraint::StretchType::BOTH);
    }

    auto CoM_Sphere = world.addSphere(0.05, 1.0, "default",raisim::COLLISION(6), raisim::COLLISION(6));
    CoM_Sphere->setBodyType(raisim::BodyType::KINEMATIC);
    CoM_Sphere->setAppearance("green");
    Eigen::VectorXd center_of_contact = Eigen::VectorXd::Zero(3);
    auto CoC_Sphere = world.addSphere(0.05, 1.0, "default",raisim::COLLISION(7), raisim::COLLISION(7));
    CoC_Sphere->setBodyType(raisim::BodyType::KINEMATIC);
    CoC_Sphere->setAppearance("red");


    for (int i=0; i<100000; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6)
        auto CoM = robot->getCompositeCOM();
        CoM_Sphere->setPosition(CoM[0]);
        get_center_of_contact(robot, center_of_contact);
        CoC_Sphere->setPosition(center_of_contact(0),center_of_contact(1),0);
        // get_contact_feet(robot, contact_feet);
        hoqp->solveAllTasks();
        solution_vector = hoqp->getSolution();
        tau = solution_vector.tail(18);
        // std::cout << "leg tau : " << tau.head(12).transpose() << std::endl;
        // std::cout << "main tau : " << tau.tail(6).transpose() << std::endl;
        generalizedForce.tail(18) = tau;
        robot->setGeneralizedForce(generalizedForce);
        // // 10 seconds later, exert external force on the robot
        if(i>5*hz && i<15*hz){
            make_ee_trajectory(world.getWorldTime()-5, desired_x,offset);
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