#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "TaskLS_ID.hpp"
#include "TaskLS_StationaryFeet.hpp"
#include "TaskLS_LinearMomentum.hpp"
#include "TaskLS_AngularMomentum.hpp"
#include "TaskLS_MoveBase.hpp"
#include "TaskLS_StationaryEE.hpp"
#include "TaskLS_MinMotion.hpp"
#include "TaskLS_EnergyOpt.hpp"
#include "TaskLS_TorqueLimits.hpp"
#include "TaskLS_FrictionCone.hpp"
#include "HOQP.hpp" 
#include "HOQP_Slack.hpp"
#include "Utils.hpp"
#include "yaml-cpp/yaml.h"

void make_ee_trajectory(const double time, Eigen::VectorXd& desired_x,double offset=0.0){
    const double amplitude = 0.2; //0.15
    const double freq = 0.5;
    desired_x << 0.65,
                 0.0+2.*amplitude*sin(2*M_PI*freq*time),
                 offset+0.6-0.5*(amplitude*cos(2*M_PI*freq*time)-amplitude),
                1., 0.0, 0.0, 0.0;
}

void make_circle_trajectory(const double time, Eigen::VectorXd& desired_x, double offset=0., double duration = 3, double angle = 90.0) {
    double x,y,z;
    // 입력된 각도를 라디안으로 변환
    double angle_rad = angle * (M_PI / 180.0); // 도를 라디안으로 변환
  
    // 현재 desired_x로부터 반지름 계산 (0.65)
    double radius = sqrt(desired_x[0] * desired_x[0] + desired_x[1] * desired_x[1]); // 0.65
    double theta;
    theta = angle_rad * sin(M_PI*time / duration); 
    x = radius * cos(theta);  // x 좌표
    y = radius * sin(theta);  // y 좌표
    double amplitude = 0.3;  
    Eigen::Matrix3d Rz = Eigen::MatrixXd::Zero(3,3);
    Rz << cos(theta), -sin(theta), 0,
          sin(theta), cos(theta), 0,
          0, 0, 1;
    Eigen::Quaterniond quat = Eigen::Quaterniond(Rz);
    quat.normalize();
    // std::cout << Utils::quat_to_euler(quat).transpose() << std::endl;
    // Eigen::Vector3d euler = Utils::quat_to_euler(quat);
    // z는 고정된 값
    // z = desired_x[2];
    z = offset+0.6-0.5*(amplitude*sin((M_PI*time)/(duration/2)));

    // desired_x에 좌표 값 업데이트
    desired_x << x, y, z, quat.w(), quat.x(), quat.y(), quat.z();  // x, y, z 좌표, 마지막 3개는 회전 (roll, pitch, yaw) 값
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

std::string make_file_dir(std::string date_time, bool is_attached,int terrain_type,int momentum_control, double hz, std::string file_name){
    std::stringstream ss1;
    ss1 << std::fixed << std::setprecision(0);
    ss1 << date_time << "_";
    if(is_attached)
        ss1 << "attached_";
    if(terrain_type==0)
        ss1 << "normal_";
    else if(terrain_type==1)
        ss1 << "inclined_";
    else if(terrain_type==2)
        ss1 << "heightmap_";
    else {}
        
    if(momentum_control==0)
        ss1 << "no_momentum_";
    else if(momentum_control==1)
        ss1 << "linear_";
    else if(momentum_control==2)
        ss1 << "angular_";
    else if(momentum_control==3)
        ss1 << "both_";
    else {}
        
    ss1 << hz << "hz_/";
    ss1 << file_name << ".csv";
    return ss1.str();   
}

int main (int argc, char* argv[]) {
    bool is_attached = 0;
    double obj_mass = 0;
    double wire_length = 0;
    int terrain_type = 0;
    int momentum_control = 0;
    double traj_duration = 0;
    double traj_angle = 0;
    double hz = 0;
    // YAML 파일 로드
    YAML::Node config = YAML::LoadFile("../yaml/momentum_test.yaml");

    // SimEnv 노드를 기준으로 파라미터 불러오기
    is_attached = config["SimEnv"]["is_attached"].as<bool>();
    obj_mass = config["SimEnv"]["obj_mass"].as<double>();
    wire_length = config["SimEnv"]["wire_length"].as<double>();
    terrain_type = config["SimEnv"]["terrain_type"].as<int>();
    momentum_control = config["SimEnv"]["momentum_control"].as<int>();
    traj_duration = config["SimEnv"]["traj_duration"].as<double>();
    traj_angle = config["SimEnv"]["traj_angle"].as<double>();
    hz = config["SimEnv"]["hz"].as<double>();

    // 파라미터 확인
    std::cout << "# is_attached : " << is_attached << std::endl;
    std::cout << "# obj_mass : " << obj_mass << std::endl;
    std::cout << "# wire_length : " << wire_length << std::endl; 
    std::cout << "# terrain_type : " << terrain_type << "\t (0 : normal / 1 : inclined / 2 : heightmap)"<<std::endl;
    std::cout << "# momentum_control : " << momentum_control << "\t (0 : no-momentum / 1 : linear / 2 : anguler/ 3 : both)"<<std::endl;
    std::cout << "# traj_duration : " << traj_duration << "\t trajectory cycle : " << 2*traj_duration << std::endl;
    std::cout << "# traj_angle : " << traj_angle << "\t trajectory total angle : " << 2*traj_angle << std::endl;
    std::cout << "# hz : " << hz << std::endl;


    // bool is_attached = true;
    // int terrain_type = 0; // 0 : normal, 1 : inclined, 2 : heightmap
    // int momentum_control = 3; // 0 : normal, 1 : linear, 2 : angular, 3 : both
    // double traj_duration = 2.0; // trajectory cycle = 2*traj_duration
    // double traj_angle = 90.0; // trajectory total angle = 2*traj_angle
    // double hz = 100;
    // bool save_data = false;
    // for(int i=1; i<argc; i++){
    //     if(i==1) is_attached = std::stoi(argv[i]);
    //     if(i==2) {
    //         if(terrain_type<0 || terrain_type>2){
    //             std::cout << "terrain_type should be 0, 1, or 2" << std::endl;
    //             return 0;
    //         }
    //         terrain_type = std::stoi(argv[i]);
    //     }
    //     if(i==3){
    //         if(momentum_control<0 || momentum_control>3){
    //             std::cout << "momentum_control should be 0, 1, 2, or 3" << std::endl;
    //             return 0;
    //         }
    //         momentum_control = std::stoi(argv[i]);
    //     } 
    //     if(i==4) traj_duration = std::stod(argv[i]);
    //     if(i==5) traj_angle = std::stod(argv[i]);
    //     if(i==6) hz = std::stod(argv[i]);
    //     if(i==7) save_data = std::stod(argv[i]);
    // }

    raisim::World world;
    auto ground = world.addGround(0., "brass", raisim::COLLISION(-1));
    raisim::Vec<3> gravity = world.getGravity();
    world.setTimeStep(1/hz);
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    auto robot = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\z1\\aliengo_z1.urdf", "",{}, raisim::COLLISION(-1), raisim::COLLISION(-1));
    
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
                1.0, 0.0, 0.0, 0.;

    Eigen::VectorXd base_desired_x = Eigen::VectorXd::Zero(6);
    base_desired_x <<   -0.0490382 ,
                        0.00157048,
                        0.401518,// + amplitude*sin(2*M_PI*freq*time) ,
                        0,
                        0,
                        0;
    double offset = 0.0;
    
    if(terrain_type==1){
        double slope_anlge_deg = 15.;
        double robot_anlge_deg = 5.;
        double slope_angle_rad = slope_anlge_deg*M_PI/180;
        double robot_angle_rad = robot_anlge_deg*M_PI/180;
        Eigen::Matrix3d slope_rot = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d robot_rot = Eigen::Matrix3d::Zero();
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
        slope->setPosition(raisim::Vec<3>{0.0,0.0,abs(size_x*sin(slope_angle_rad)/2)});
        slope->setOrientation(slope_rot);
        offset = abs(size_x*sin(slope_angle_rad)/2 + size_z/2);
        Eigen::Quaterniond slope_quat = Eigen::Quaterniond(slope_rot);
        Eigen::Quaterniond robot_quat = Eigen::Quaterniond(robot_rot);
        slope_quat.normalize();
        robot_quat.normalize();

        jointNominalConfig << 0.0, 0.0, 0.43+offset, //base position
                          robot_quat.w() ,robot_quat.x(), robot_quat.y(), robot_quat.z(),//1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3, 
                          0.0, 0.6, -1.3,
                          0.0, 0.6, -1.3,
                          0, 2.57, -1.5,  -1.0,  0. , 0.; // arm joints
        Eigen::Vector3d euler = Utils::quat_to_euler(slope_quat);


        desired_x << 0.65, 0.0, offset+0.6,
                    1.0, 0.0, 0.0, 0.0;
        // desired_x.tail(4) = slope_quat.coeffs();  
        base_desired_x <<   -0.0490382 ,
                    0.00157048,
                    offset+0.43-0.02,// + amplitude*sin(2*M_PI*freq*time) ,
                    0,
                    0,
                    0;
        base_desired_x.tail(3) = euler;
        // std::cout << "euler : " << euler.transpose() << std::endl;
        std::cout << offset+0.43 << std::endl;
    }

    else if(terrain_type == 2){
        raisim::TerrainProperties terrainProperties;
        terrainProperties.frequency = 0.9;
        terrainProperties.zScale = 1.0;
        terrainProperties.xSize = 20.0;
        terrainProperties.ySize = 20.0;
        terrainProperties.xSamples = 100;
        terrainProperties.ySamples = 100;
        terrainProperties.fractalOctaves = 1;
        terrainProperties.fractalLacunarity = 5.;
        terrainProperties.fractalGain = 0.25;
        auto hm = world.addHeightMap(0.0, 0.0, terrainProperties, "brass");
        hm->setAppearance("0.72, 0.49, 0.3, 1.0"); //"26/255, 72/255, 33/255, 1.0"
        offset = 0.4;

        double robot_anlge_deg = -5.;
        double robot_angle_rad = robot_anlge_deg*M_PI/180;
        Eigen::Matrix3d robot_rot = Eigen::Matrix3d::Zero();
        robot_rot << cos(robot_angle_rad), 0, sin(robot_angle_rad),
                            0, 1, 0,
               -sin(robot_angle_rad), 0, cos(robot_angle_rad);
        Eigen::Quaterniond robot_quat = Eigen::Quaterniond(robot_rot);
        robot_quat.normalize();

        Eigen::Vector3d euler = Utils::quat_to_euler(robot_quat);


        desired_x << 0.65, 0.0, offset+0.6,
                    1.0, 0.0, 0.0, 0.0;
        // desired_x.tail(4) = slope_quat.coeffs();  
        base_desired_x <<   -0.0490382 ,
                    0.00157048,
                    offset+0.43-0.02,// + amplitude*sin(2*M_PI*freq*time) ,
                    0,
                    0,
                    0;
        base_desired_x.tail(3) = euler;
        // std::cout << "euler : " << euler.transpose() << std::endl;
        // std::cout << offset+0.43 << std::endl;


        jointNominalConfig << 0.0, 0.0, 0.43+offset, //base position
                          robot_quat.w() ,robot_quat.x(), robot_quat.y(), robot_quat.z(),//1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                        //   0.9914449 ,0, -0.130526, 0,//1.0, 0.0, 0.0, 0.0, //base orientation(quaternion)
                          0.0, 0.6, -1.6, 
                          0.0, 0.6, -1.6, 
                          0.0, 0.6, -1.5,
                          0.0, 0.6, -1.5,
                          0, 2.57, -1.5,  -1.0,  0. , 0.; // arm joints

        // desired_x << 0.65, 0.0, offset+0.6,
        //             1.0, 0.0, 0.0, 0.0;

        // base_desired_x <<   -0.0490382 ,
        //                     0.00157048,
        //                     offset+0.401518,// + amplitude*sin(2*M_PI*freq*time) ,
        //                     0,
        //                     0,
        //                     0;
    }

    Eigen::Vector3d rate_weight = 50*Eigen::Vector3d::Ones(3);
    Eigen::Vector3d position_weight = 200*Eigen::Vector3d::Ones(3);
    Eigen::Vector3d angular_weight = 100*Eigen::Vector3d::Ones(3);

    std::unique_ptr<TaskLS> task_ID = std::make_unique<TaskLS_ID>(&world, robot, 12, 42);
    std::unique_ptr<TaskLS> task_feet = std::make_unique<TaskLS_StationaryFeet>(&world, robot, 12, 42);
    std::unique_ptr<TaskLS_StationaryEE> task_ee = std::make_unique<TaskLS_StationaryEE>(&world, robot, 6, 42, desired_x,300,2*sqrt(150));
    std::unique_ptr<TaskLS_MoveBase> task_base = std::make_unique<TaskLS_MoveBase>(&world, robot, 6, 42, base_desired_x, 100, 2*sqrt(100));
    std::unique_ptr<TaskLS> task_linear_momentum = std::make_unique<TaskLS_LinearMomentum>(&world, robot, 3, 42, rate_weight, position_weight);
    std::unique_ptr<TaskLS> task_angular_momentum = std::make_unique<TaskLS_AngularMomentum>(&world, robot, 3, 42, angular_weight);
    std::unique_ptr<TaskLS> task_Eopt = std::make_unique<TaskLS_EnergyOpt>(&world, robot, 42);
    
    Eigen::VectorXd tau_min = -40*Eigen::VectorXd::Ones(18);
    Eigen::VectorXd tau_max = 40*Eigen::VectorXd::Ones(18);
    tau_min.tail(6) << -30, -60, -30, -30, -30, -30;
    tau_max.tail(6) << 30, 60, 30, 30, 30, 30;
    std::unique_ptr<TaskLS> constraints_tau = std::make_unique<TaskLS_TorqueLimits>(tau_min, tau_max);
    std::unique_ptr<TaskLS> constraints_cone = std::make_unique<TaskLS_FrictionCone>(&world, robot, 24,42,0.1);
    std::unique_ptr<TaskSet> task_set1 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set2 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set3 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set4 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set5 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set6 = std::make_unique<TaskSet>(42);
    std::unique_ptr<TaskSet> task_set7 = std::make_unique<TaskSet>(42);
    task_set1->addEqualityTask(task_ID.get());
    task_set2->addInequalityTask(constraints_tau.get());
    task_set2->addEqualityTask(task_feet.get());
    task_set3->addInequalityTask(constraints_cone.get());
    task_set3->addEqualityTask(task_ee.get());
    task_set4->addEqualityTask(task_base.get());
    task_set5->addEqualityTask(task_linear_momentum.get());
    task_set6->addEqualityTask(task_angular_momentum.get());
    task_set7->addEqualityTask(task_Eopt.get());

    std::unique_ptr<HOQP_Slack> hoqp = std::make_unique<HOQP_Slack>();

    hoqp->addTask(task_set1.get()); // EoM
    hoqp->addTask(task_set2.get()); // Feet
    hoqp->addTask(task_set3.get()); // ee
    hoqp->addTask(task_set4.get()); // base
    if(momentum_control == 1){
        hoqp->addTask(task_set5.get()); // linear momentum
    }
    else if(momentum_control == 2){
        hoqp->addTask(task_set6.get()); // angular momentum
    }
    else if(momentum_control == 3){
        hoqp->addTask(task_set5.get()); // linear momentum
        hoqp->addTask(task_set6.get()); // angular momentum
    }
    else {}
    // hoqp->addTask(task_set7.get()); // energy optimization
    hoqp->init();   

    Eigen::VectorXd solution_vector;
    Eigen::VectorXd tau;
    Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(robot->getDOF());
    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(robot);
    
    auto ee_traj_line = server.addVisualPolyLine("EndEffectorTrajectory");
    ee_traj_line->color = {0,1,0,0.5};
    ee_traj_line->width = 0.01;
    ee_traj_line->points.push_back(desired_x.head(3));
    ee_traj_line->points.push_back(desired_x.head(3)+1e-3*Eigen::Vector3d::Identity());

    std::vector<bool> contact_feet = {false,false,false,false};//FR, FL, RR, RL
    robot->setGeneralizedCoordinate(jointNominalConfig);
    robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF())); 
    robot->setName("aliengo + Z1");
    robot->getCollisionBody("FL_foot/0").setMaterial("steel");    
    robot->getCollisionBody("FR_foot/0").setMaterial("steel");
    robot->getCollisionBody("RL_foot/0").setMaterial("steel");
    robot->getCollisionBody("RR_foot/0").setMaterial("steel");
    world.setMaterialPairProp("steel", "brass", 1., 0., 0.1, 1, 1.);
    auto proper = world.getMaterialPairProperties(robot->getCollisionBody("RR_foot/0").getMaterial(), "brass");
    /**
     * print friction properties
     * friction – the dynamic coefficient of friction
        restitution – the coefficient of restitution
        resThreshold – the minimum impact velocity to make the object bounce
        staticFriction – the static coefficient of friction
        staticFrictionThresholdVelocity – if the relative velocity of two points is bigger than this value, then the dynamic
          double c_f = 0.8; // coefficient of friction
  double c_r = 0.0; // coefficient of restitution
  double r_th = 0.01; // restitution threshold
  double c_static_f = 0.8;
  double v_static_speed = 1., v_static_speed_inv = 1.0;
     */
    // std::cout << "friction : " << proper.c_f << std::endl;
    // std::cout << "restitution : " << proper.c_r << std::endl;
    // std::cout << "resThreshold : " << proper.r_th << std::endl;
    // std::cout << "staticFriction : " << proper.c_static_f << std::endl;
    // std::cout << "staticFrictionThresholdVelocity : " << proper.v_static_speed << std::endl;
    auto unknown_obj = world.addBox(0.1, 0.4, 0.1, obj_mass,"steel",raisim::COLLISION(2),raisim::COLLISION(2));
    auto CoM_Sphere = world.addSphere(0.05, 0.01,"steel",raisim::COLLISION(111),raisim::COLLISION(111));
    Eigen::Vector3d r_obj_com;
    if(is_attached){
        raisim::Vec<3> ee_posi_tmp;
        robot->getFramePosition("joint6", ee_posi_tmp);
        ee_posi_tmp = ee_posi_tmp + raisim::Vec<3>{0,0,-wire_length};
        // std::cout << "Body Frame Inertia of object : " << unknown_obj->getInertiaMatrix_B().transpose() << std::endl;
        // Eigen::Matrix3d obj_inertia = Eigen::Matrix3d::Zero();
        // obj_inertia << (0.0113542), 2*(0.001875), 0, // (1/12)*0.5*(0.5^2 + 0.15^2)
        //                2*(0.001875), (0.001875), 0, // (1/12)*0.5*(0.15^2 + 0.15^2)
        //                0, 0, (0.0113542); // (1/12)*0.5*(0.5^2 + 0.15^2)
        // unknown_obj->setInertia(obj_inertia);
        unknown_obj->setBodyType(raisim::BodyType::DYNAMIC);
        unknown_obj->setAppearance("1,1,1,0.5");
        // unknown_obj->setAppearance("brown");
        unknown_obj->setPosition(ee_posi_tmp);
        // unknown_obj->setCom({0,((0.4)/2)/3,0});
        r_obj_com = unknown_obj->getComPosition();
        auto wire = world.addStiffWire(unknown_obj,0,{0,0,0.05},robot,18,{0.05,0,0},wire_length);
        wire->setStretchType(raisim::LengthConstraint::StretchType::BOTH);

        CoM_Sphere->setBodyType(raisim::BodyType::KINEMATIC);
        CoM_Sphere->setAppearance("red");
        CoM_Sphere->setPosition(r_obj_com);

    }
    Eigen::VectorXd obj_position = Eigen::VectorXd::Zero(3);
    std::vector<Eigen::VectorXd> obj_position_dataset;
    std::vector<Eigen::VectorXd> obj_CoM_position_dataset;
    raisim::Vec<3> base_position_tmp;
    Eigen::VectorXd base_position = Eigen::VectorXd::Zero(3);
    std::vector<Eigen::VectorXd> base_position_dataset;
    raisim::Mat<3,3> base_rot;
    Eigen::VectorXd base_euler = Eigen::VectorXd::Zero(3);
    std::vector<Eigen::VectorXd> base_euler_dataset;
    Eigen::VectorXd CoM_position = Eigen::VectorXd::Zero(3);
    std::vector<Eigen::VectorXd> CoM_position_dataset;

    std::vector<Eigen::VectorXd> desired_ee_position_dataset;
    std::vector<Eigen::VectorXd> ee_position_dataset;

    std::vector<Eigen::VectorXd> obj_force_dataset;
    std::vector<Eigen::VectorXd> joint_torques_dataset;
    std::vector<Eigen::VectorXd> linear_momentum_dataset;
    std::vector<Eigen::VectorXd> angular_momentum_dataset;

    std::vector<double> time_dataset;

    auto date_time = Utils::get_current_date_time(); 

    std::string filename1 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz, "EndEffectorDesiredTrajectory_");
    std::string filename2 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz, "EndEffectorActualTrajectory_");
    std::string filename3 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz, "ObjectPosition_");
    std::string filename4 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz, "ObjectCoMPosition_");
    std::string filename5 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz,  "BasePosition_");
    std::string filename6 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz,  "BaseEuler_");       
    std::string filename7 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz,  "CoMPosition_");
    std::string filename8 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz,  "ObjForce_");
    std::string filename9 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz,  "JointTorques_");
    std::string filename10 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz,  "LinearMomentum_");
    std::string filename11 = make_file_dir(date_time, is_attached, terrain_type, momentum_control, hz,  "AngularMomentum_");

    raisim::Vec<3> ee_position;
    raisim::Vec<3> obj_lm;
    raisim::Vec<3> obj_lm_prev;
    Eigen::VectorXd obj_force = Eigen::VectorXd::Zero(3);
    raisim::Vec<3> robot_angular_momentum;
    // auto sphere_tmp = world.addSphere(0.05, 0.01,"steel",raisim::COLLISION(111),raisim::COLLISION(111));
    // sphere_tmp->setPosition(raisim::Vec<3>{-0.0234187, 0., 0.725});
    // sphere_tmp->setBodyType(raisim::BodyType::KINEMATIC);
    // sphere_tmp->setAppearance("red");
    int i = 0;
    for (i=0; i<(10+traj_duration*5)*hz; i++) {
        RS_TIMED_LOOP(world.getTimeStep()*1e6)
        unknown_obj->setCom({0,((0.2)/3)*sin(4*i/hz),0});
        // auto w_R_obj = unknown_obj->getOrientation();
        obj_lm = unknown_obj->getLinearMomentum();
        r_obj_com = unknown_obj->getComPosition();
        CoM_Sphere->setPosition(r_obj_com);
        // get_contact_feet(robot, contact_feet);
        hoqp->solveAllTasks();
        solution_vector = hoqp->getSolution();
        tau = solution_vector.tail(18);
        // std::cout << "leg tau : " << tau.head(12).transpose() << std::endl;
        // std::cout << "main tau : " << tau.tail(6).transpose() << std::endl;
        std::cout << "desired base position : " << base_desired_x.head(3).transpose() << std::endl;
        std::cout << "desired base euler : " << base_desired_x.tail(3).transpose() << std::endl;
        generalizedForce.tail(18) = tau;
        robot->setGeneralizedForce(generalizedForce);
        // // 10 seconds later, exert external force on the robot
        if(i>5*hz && i<(5+traj_duration*5)*hz){
            make_circle_trajectory(world.getWorldTime()-5, desired_x,offset, traj_duration, traj_angle);
            // make_ee_trajectory(world.getWorldTime()-5, desired_x,offset);
            task_ee->updateDesiredEEPose(desired_x);
            server.lockVisualizationServerMutex();
            if(i%10==0){
                if (ee_traj_line->points.size() > 200){
                    ee_traj_line->points.erase(ee_traj_line->points.begin());
                }
            ee_traj_line->points.push_back(desired_x.head(3));
            }
          
           
            server.unlockVisualizationServerMutex();
        }
        // if(save_data){
            desired_ee_position_dataset.push_back(desired_x.head(3)); 
            robot->getFramePosition("joint6",ee_position);
            ee_position_dataset.push_back(ee_position.e());
            obj_position = unknown_obj->getPosition();
            obj_position_dataset.push_back(obj_position);
            obj_CoM_position_dataset.push_back(r_obj_com);
            robot->getFramePosition("floating_base",base_position_tmp);
            base_position = base_position_tmp.e();
            base_position_dataset.push_back(base_position);
            robot->getFrameOrientation("floating_base",base_rot);
            Eigen::Quaterniond base_quat = Eigen::Quaterniond(base_rot.e());
            base_quat.normalize();
            base_euler = Utils::quat_to_euler(base_quat);
            base_euler_dataset.push_back(base_euler);
            auto CoM_vec = robot->getCompositeCOM();
            CoM_position = CoM_vec[0].e();
            CoM_position_dataset.push_back(CoM_position);
            obj_force = (obj_lm.e() - obj_lm_prev.e())/world.getTimeStep();
            obj_force_dataset.push_back(obj_force);
            joint_torques_dataset.push_back(tau);
            time_dataset.push_back(world.getWorldTime());
            linear_momentum_dataset.push_back(robot->getLinearMomentum().e());
            robot->getAngularMomentum(CoM_vec[0],robot_angular_momentum);
            angular_momentum_dataset.push_back(robot_angular_momentum.e());
        // }
        obj_lm_prev = obj_lm;
        server.integrateWorldThreadSafe();
    }
    if(i==((10+traj_duration*5)*hz)){
        std::string save_data;
        std::cout << "Do you want to save data? (Y/n) : ";
        std::cin >> save_data;

        // 만약 입력이 없거나 Y를 입력하면 데이터 저장
        if(save_data == "" || save_data == "Y" || save_data == "y"){
            Utils::write_label_to_csv(filename1, {"time","x","y","z"});
            Utils::write_label_to_csv(filename2, {"time","x","y","z"});
            Utils::write_label_to_csv(filename3, {"time","x","y","z"});
            Utils::write_label_to_csv(filename4, {"time","x","y","z"});
            Utils::write_label_to_csv(filename5, {"time","x","y","z"});
            Utils::write_label_to_csv(filename6, {"time","roll","pitch","yaw"});
            Utils::write_label_to_csv(filename7, {"time","x","y","z"});
            Utils::write_label_to_csv(filename8, {"time","x","y","z"});
            Utils::write_label_to_csv(filename9, {"time","tau1","tau2","tau3","tau4","tau5","tau6","tau7","tau8","tau9","tau10","tau11","tau12","tau13","tau14","tau15","tau16","tau17","tau18"});
            Utils::write_label_to_csv(filename10, {"time","x","y","z"});
            Utils::write_label_to_csv(filename11, {"time","x","y","z"});

            Utils::write_data_to_csv(filename1, time_dataset, desired_ee_position_dataset);
            Utils::write_data_to_csv(filename2, time_dataset, ee_position_dataset);
            Utils::write_data_to_csv(filename3, time_dataset, obj_position_dataset);
            Utils::write_data_to_csv(filename4, time_dataset, obj_CoM_position_dataset);
            Utils::write_data_to_csv(filename5, time_dataset, base_position_dataset);
            Utils::write_data_to_csv(filename6, time_dataset, base_euler_dataset);
            Utils::write_data_to_csv(filename7, time_dataset, CoM_position_dataset);
            Utils::write_data_to_csv(filename8, time_dataset, obj_force_dataset);
            Utils::write_data_to_csv(filename9, time_dataset, joint_torques_dataset);
            Utils::write_data_to_csv(filename10, time_dataset, linear_momentum_dataset);
            Utils::write_data_to_csv(filename11, time_dataset, angular_momentum_dataset);
        }        

    }
   
    server.killServer();

    return 0;
}