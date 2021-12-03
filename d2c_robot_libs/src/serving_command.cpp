#include "d2c_robot_libs/serving_command.hpp"

ServingCommand::ServingCommand()
{}

ServingCommand::~ServingCommand()
{}


std::vector<std::vector<double>> ServingCommand::ReturnTargetJointPosition()
{
    std::vector<std::vector<double>> target_joint_position_;
    std::vector<double> object_position_;
    // target position 1. before object 2. on object 3. up the object  
    // target_joint_position_[0] = InverseKinematics(before_object_position_);
    // target_joint_position_[1] = InverseKinematics(on_object_position_);
    // target_joint_position_[2] = InverseKinematics(up_object_position_);
    return target_joint_position_;
}

std::vector<double> ServingCommand::InverseKinematics(std::vector<double> final_value)
{
    std::vector<double> joint_state_;
    double theta1, theta2, theta3, theta4;
    double link1 = 43.4;
    double link2 = 187.96;
    double link3 = 170.0;
    double theta2_off = 0.1603 ;
    
    double x= final_value[0];
    double y= final_value[1];
    double z= final_value[2];
    
    double F = sqrt(z*z + y*y - link1*link1);
    double H = sqrt(x*x + y*y + z*z);

    theta1 = 3.14/2 - acos(link1/sqrt(x*x + y*y)) -atan2(y,x);

    double D =(H*H-link2*link2-link3*link3)/(-2*link2*link3);

    theta3 = acos(D)-3.14/2-theta2_off;
    double alpha = asin(z/H);
    double beta = atan2(link3*sin(3.14/2-theta2_off-theta3), link2+link3*cos(3.14/2-theta2_off-theta3));
    theta2 =3.14/2 -beta -theta2_off -alpha;

    double a1 = 3.14 - beta - (theta3 + 3.14/2 + theta2_off);
    double a2 = atan2(sqrt(x*x+y*y),z);
    theta4 = 3.14/2-a1-a2;

    joint_state_ = {-theta1, -theta2, theta3, theta4 };

    return joint_state_;
}
// TODO 1. get the object position and transform the coordinate - TransformCoordinate()
// TODO 2. devide object position to 3 position and contain to variables - SetTargetPosition()

std::vector<int> ServingCommand::TransformCoordinate()
{
    std::vector<int> serving_position;
    int x = object_x_;
    int y = object_y_;
    // transform coordination 
    
    serving_position = {x,y};

    return serving_position;
}

std::vector<std::vector<double>> ServingCommand::SetTargetPosition()
{
    std::vector<std::vector<double>> object_position;
     
    // TODO : get the information of object, and save into object_position variable
     
    // TODO : target position 1. before object 2. on object 3. up the object 
    return object_position;
}

std::vector<float> ServingCommand::Initialize()
{
    std::vector<float> initialize_position;
    initialize_position = {1.52, 0.0, 0.0, 0.0};
    return initialize_position;
}



std::vector<float> ServingCommand::Cleaning()
{
    std::vector<float> cleaning_position;
    return cleaning_position;
}

Eigen::Matrix3d ServingCommand::Rx(double theta)
{
    double omega = theta;
    R_x <<  1.0, 0.0, 0.0,
           0.0, cos(omega),-sin(omega),
           0.0, sin(omega), cos(omega);

    return R_x;
}

Eigen::Matrix3d ServingCommand::Ry(double theta)
{
    double phi = theta;
    R_y <<  cos(phi), 0.0, sin(phi),
           0.0, 1.0, 0.0,
           -sin(phi), 0.0, cos(phi);

    return R_y;
}

Eigen::Matrix3d ServingCommand::Rz(double theta)
{
    double psi = theta;
    R_z <<  cos(psi), -sin(psi), 0.0,
           sin(psi), cos(psi), 0.0,
           0.0, 0.0, 1.0;

    return R_z;
}