#include "d2c_robot_libs/serving_command.hpp"

ServingCommand::ServingCommand()
{}

ServingCommand::~ServingCommand()
{}


std::vector<std::vector<double>> ServingCommand::ReturnTargetJointPosition()
{
    std::vector<std::vector<double>> target_joint_position_;
    std::vector<double> object_position_;

    object_position_ = TransformCoordinate();

    target_joint_position_ = SetTargetPosition(object_position_);
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

    joint_state_ = {-theta1, -theta2, theta3, -theta4 };

    return joint_state_;
}
// TODO 1. get the object position and transform the coordinate - TransformCoordinate()
// TODO 2. devide object position to 3 position and contain to variables - SetTargetPosition()

std::vector<double> ServingCommand::TransformCoordinate() // transform coordination 
{
    std::vector<double> serving_position;
    double x = (double)object_x_;
    double y = (double)object_y_;

    P_m << 1.83170838e+00, 8.09496928e-01, -2.84984280e+02,
           -2.69219180e-02, 4.73825757e+00,-1.52566509e+02, 
           -1.57744341e-04, 2.82588407e-03, 1.00000000e+00;


    double p11 = P_m(0,0);
    double p12 = P_m(0,1);
    double p13 = P_m(0,2);

    double p21 = P_m(1,0);
    double p22 = P_m(1,1);
    double p23 = P_m(1,2);

    double p31 = P_m(2,0);
    double p32 = P_m(2,1);
    double p33 = P_m(2,2);
    
    x_f = (p11*x +p12*y + p13)/(p31*x + p32*y + p33);
    y_f = (p21*x +p22*y + p23)/(p31*x + p32*y + p33);
    // double x_c, y_c, x_w, y_w;
    double x_f_ = (390.0-(x_f)*(330.0/634.0*2.0));
    double y_f_ = (390.0-(y_f)*(390.0/749.0));
    
    // Eigen::Matrix4d T_m_ = Tm(world2camera_theta, world2camera_x, world2camera_y, world2camera_z);
    
    // x_c = ((x-c_x)*Z_c)/focal_length;
    // y_c = ((y-c_y)*Z_c)/focal_length;
    ROS_INFO("%f, %f", x_f_, -y_f_);
    serving_position = {x_f_, -y_f_};
 
    return serving_position;
}

std::vector<std::vector<double>> ServingCommand::SetTargetPosition(std::vector<double> ob_position)
{
  
    std::vector<double> position_info_ = ob_position;
    std::vector<std::vector<double>> object_position;
    std::vector<double> p1;
    std::vector<double> p2;
    std::vector<double> p3;
    ROS_INFO("%f, %f",position_info_[0], position_info_[1]);
    std::vector<double> final_value1 = {position_info_[0]+20.0,position_info_[1]-20.0, 70.0};
    std::vector<double> final_value2 = {position_info_[0]+20.0,position_info_[1]-10.0, 70.0};
    std::vector<double> final_value3 = {position_info_[0]+30.0,position_info_[1]-30.0, 120.0};
 
    p1 = InverseKinematics(final_value1);
    p2 = InverseKinematics(final_value2);
    p3 = InverseKinematics(final_value3);
    object_position ={p1,p2,p3};
    ROS_INFO("%f, %f, %f,%f",p1[0], p1[1],p1[2],p1[3]);
    ROS_INFO("%f, %f, %f,%f",p2[0], p2[1],p2[2],p2[3]);
    ROS_INFO("%f, %f, %f,%f",p3[0], p3[1],p3[2],p3[3]);
    object_position[0][0]= object_position[0][0]-3.14/3;
    ROS_INFO("%f",object_position[0][0]);
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

Eigen::Matrix4d ServingCommand::Tm(double theta, double px,double py,double pz)
{
    R_m = Rx(theta)*Ry(3.14/2)*Rz(-3.14/2);
    T_m << 0.0, 0.0, 0.0, px,
           0.0, 0.0, 0.0, py,
           0.0, 0.0, 0.0, pz,
           0.0, 0.0, 0.0, 1.0;

    T_m.block(0,0,3,3) = R_m;
    return T_m;
}