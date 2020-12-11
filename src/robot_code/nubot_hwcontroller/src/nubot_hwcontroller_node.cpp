/// C与C++混合编程要在CPP文件中加extern “C”关键字，否则链接会出错
/// C and C++ mixed programming must add the extern "C" keyword in the CPP file, otherwise the link will be wrong
#include "nubot_hwcontroller/nubot_hwcontroller_node.h"
/// CAN模块位置
/// CAN module location
#define CAN_SLOT_POSITION 4
#define WHEELS 3
#define USE_BRUSH_MOTOR     0
#define DEFAULT_PRIO    (60)
#define MAX_SAFE_STACK (8*1024)
#define _USE_MATH_DEFINES
#define USE_CURSE 0
#define DribbleDebug 0
#define RADIUS 30.0f
/// 继电器导通到IGBT导通控制时间间隔，共10ms
/// The time interval between relay conduction and IGBT conduction control, 10ms in total
#define  Delay2IGBT     65
#ifndef min
#define min(x, y) (((x) > (y)) ? (y) : (x))
#define max(x, y) (((x) > (y)) ? (x) : (y))
#endif
#define deg(a) (PI*(a)/180.0f)
#define Limmit(Vmin,v,Vmax)			(min(max((v),(Vmin)),(Vmax)))
/// 用了比较多的全局变量
/// Use more global variables
using namespace boost::posix_time;
using namespace boost::filesystem;

bool  cmac_enable=false;
const char* msg_state[]={"TryCatch","Firm","Unstable","Stuck"};
const double WHEEL_DISTANCE=23.28;
const double MOTOR_GEAR_RATIO=10.0;
const double WHEEL_DIAMETER=12.5;
const double VEL_TO_RPM_RATIO=MOTOR_GEAR_RATIO*60.0/(M_PI*WHEEL_DIAMETER);
const double LIMITEDRPM=5500;
static std::string robot_name ;
int zero3[3]={0,0,0};
int zero2[2]={0,0};

/// 持球状态
/// Ball status
typedef enum {TryCatch=0,Firm,Unstable,Stuck} BallStates;

/// \brief Here we use list to initialize variables, but in fact, some newly added variables do not appear in the list, which is risky
Nubot_HWController::Nubot_HWController(int argc,char** argv)
    :n(),number(0),motor_left(motor_speed[0]),motor_right(motor_speed[1]),motor_up(0),
      Vx(0),Vy(0),w(0),P(7.0),I(0),D(2.5),BallSensor_IsHolding(false),
      BallHandleEnable(0),ShootEnable(false),shootcnt(0),ShootPos(0),RodPos(0),ShootPower(0),ShootDone(true),
      PowerState(true),PowerState_Last(true),
      acc_state(0),wacc_state(0),
      RotationMode(0),
      HoldingCnt(0),UnholdingCnt(0),
      move_action_(21),rotate_action_(21),
      LeverPos_SetPoint(0),FFRatio_Set(1)
{
//    ROS_INFO("initialize hw_node process");
    robot_name = argv[1];
    std::string num = robot_name.substr(robot_name.size()-1);
    std::string robot_prefix = robot_name.substr(0,robot_name.size()-1);
    robot_id = std::stoi(robot_name.substr(robot_name.size()-1));  //zdx_note:each robot's id
    ROS_FATAL("robot_name:%s",robot_name.c_str());
    //ROS_INFO("robot_name:%d",id_size);
    n = ros::NodeHandle(robot_name);

    motor_cmd_pub_ = n.advertise<nubot_common::VelCmd>("nubotcontrol/velcmd",1);
    /// 接受底盘速度指令
    /// Accept the chassis speed command
    actioncmd_sub_ = n.subscribe("nubotcontrol/actioncmd",1,&Nubot_HWController::SetAction,this);
    sendingoff_sub_ = n.subscribe("/"+robot_prefix+"/redcard/chatter",1,&Nubot_HWController::SendingOff_isvalid_flag,this);

    /// 建立周期回调函数
    /// Establish cycle callback function
    timer1=n.createTimer(ros::Rate(200),&Nubot_HWController::Timer1_Process,this);
    is_id_valid = true;
}

Nubot_HWController::~Nubot_HWController()
{

//    Ballhandle_Enable(false);
}

/// \brief 定时器，关系整个底层控制，控制频率500hz
/// \brief timer, related to the entire underlying control, the control frequency is 500hz
/// BaseController负责底层电机控制
/// BaseController is responsible for the underlying motor control
/// DribbleController负责抓球电机控制
/// DribbleController is responsible for the ball catch motor control
/// ShooterController负责升降机构控制
/// ShooterController is responsible for the lifting mechanism control
void Nubot_HWController::Timer1_Process(const ros::TimerEvent &e)
{
    std::string num = robot_name.substr(robot_name.size()-1);
//    std::cout<<"timer  "<<num<<std::endl;
    BaseController();
}



/// \brief 十分重要的底层控制，得到Vx，Vy，w后解算到各个电机，并读取当前电机的实际转速作为里程计融合
/// \brief is a very important underlying control. After obtaining Vx, Vy, w, it is calculated to each motor, and the actual speed of the current motor is read as the odometer fusion
void Nubot_HWController::BaseController()
{
    std::string num = robot_name.substr(robot_name.size()-1);
//    std::cout<<"received action cmd"<<num<<std::endl;
    /// 由上层得到的动作，目标，以及当前位置等得到机器人体坐标系下的速度
    /// Get the speed in the robot coordinate system from the action, target, and current position obtained from the upper layer
    calculateSpeed();
    nubot_common::VelCmd  vel_cmd;
//    Vx = 0;Vy=0;w = 0;
    vel_cmd.Vx = Vx;
    vel_cmd.Vy = Vy;
    vel_cmd.w  = w;
    if(move_action_==No_Action)
    {
        vel_cmd.Vy = 0;
        vel_cmd.Vx = 0;
    }
    if(rotate_action_==No_Action)
        vel_cmd.w = 0;
//    std::cout<<"num "<<num<<"  vx::"<<Vx<<"  vy:: "<<Vy<<"  w::"<<w<<std::endl;
    motor_cmd_pub_.publish(vel_cmd);
}

//zdx_note
void Nubot_HWController::SendingOff_isvalid_flag(const nubot_common::SendingOff::ConstPtr& cmd)
{
    //ROS_FATAL("get topic %d %d %d %s is  %s",cmd->id_maxvel_isvalid,robot_id,is_id_valid,cmd->TeamName.c_str(),robot_name.c_str());
    if(cmd->id_maxvel_isvalid==robot_id && is_id_valid == true && strcmp(cmd->TeamName.c_str(),robot_name.c_str())==0)
    {
        is_id_valid = false;
        //ROS_FATAL("allow robot in");
    }
    else if(cmd->id_maxvel_isvalid==robot_id+10 && is_id_valid == false && strcmp(cmd->TeamName.c_str(),robot_name.c_str())==0)
    {
        is_id_valid = true;
        //ROS_INFO("send robot off");
    }
}
//zdx_end
/// \brief 上层nubot_control发布的动作topic的回调函数，在自主运行时，订阅的是动作
/// \brief The callback function of the action topic published by the upper nubot_control. When it runs autonomously, it is subscribed to the action
void Nubot_HWController::SetAction(const nubot_common::ActionCmd::ConstPtr& cmd)
{
    std::string num = robot_name.substr(robot_name.size()-1);
    /// Action, current linear velocity, angular velocity, target position, target velocity, target angle, maximum linear velocity, maximum angular velocity
    /// 动作，当前线速度，角速度，目标位置，目标速度，目标角度，最大线速度，最大角速度
    move_action_=cmd->move_action;
    rotate_action_=cmd->rotate_action;
    rotate_mode_=cmd->rotate_mode;
    target_=nubot::DPoint2s(cmd->target.x,cmd->target.y);
    target_vel_=nubot::DPoint2s(cmd->target_vel.x,cmd->target_vel.y);

    target_ori_=cmd->target_ori;

    robot_vel_=nubot::DPoint2s(cmd->robot_vel.x,cmd->robot_vel.y);
    robot_w_=cmd->robot_w;

    //maxvel_=cmd->maxvel;
    //maxw_=cmd->maxw;
    //zdx_note
    if(is_id_valid)
    {
        maxvel_ = cmd->maxvel;
        maxw_ = cmd->maxw;
    }
    else
    {
        maxvel_ = 0;
        maxw_   = 0;
    }
        /*if(is_id_valid == id_size + 10 && strcmp(TeamName.c_str(),robot_name.c_str())==0)
    {
        maxvel_=cmd->maxvel;
        maxw_=cmd->maxw;
    }
    else if(is_id_valid == 0)
    {
        maxvel_=cmd->maxvel;
        maxw_=cmd->maxw;
    }*/

    //zdx_end
////    /// 带球及射门消息
    BallHandleEnable=cmd->handle_enable;
    ShootPos=cmd->shootPos;

    /// 当前一次射门结束时，赋予新的射门力量
    /// At the end of the current shot, new shooting power is given
    if(ShootDone)
        ShootPower=cmd->strength;
    /// 如果是带球状态，最大线速度进一步限制
    /// If it is dribbling, the maximum linear speed is further restricted
    if(BallSensor_IsHolding && maxvel_>300)
        maxvel_=300;
    /// 如果不是抓球动作，最大角速度也进一步限制
    /// If it is not a catching action, the maximum angular velocity is further restricted
    if(rotate_action_!=Catch_Positioned&&rotate_action_!=CatchBall&&rotate_action_!=CatchBall_slow&&rotate_action_!=TeleopJoy)
        maxw_=3;
    /// 根据动作可以灵活的设置不同动作的各种限制条件，明显优于以前传速度的形式
    /// Various restriction conditions of different actions can be flexibly set according to the action, which is obviously better than the previous form of speed
    robot_pos_ = nubot::DPoint2s(cmd->robot_pos.x,cmd->robot_pos.y);
    robot_ori_ = cmd->robot_ori;
    //更新topic时间
    TimeFromHighLevel_ = ros::Time::now();
}

/// \brief 根据上层节点的信息计算当前机器人的速度
void Nubot_HWController::calculateSpeed()
{
    //adjustPD(1,0);
    rotate2AbsOrienation();
    move2target();
//    if(rotate_action_==No_Action && move_action_==No_Action)
//    {
//        Vx=Vy=0;
//        w=0;
//    }
}

/// \brief 基本的PD控制器
float Nubot_HWController::basicPDControl(float pgain,float dgain, float err,float err1, float maxval)
{

    float _e1 = err1;
    float kp =  pgain;
    float kd=  dgain;
    float _e=err;
    float retval = 0;
    retval  = kp *_e +kd*(_e -_e1) ;
    if(fabs(retval) > maxval)
    {
        if(retval>0) retval= maxval;
        else    retval=-maxval;
    }
    return retval;
}

/// \brief 动态调节PD参数,pval参考P，dval参考D，weight可调的平动与转动权重
///Dynamically adjust PD parameters, pval refers to P, dval refers to D, weight can be adjusted for translation and rotation weights
void Nubot_HWController::adjustPD(float pval, float dval,float weight)
{
//    /// 目标点距离误差
//    /// Target distance error
//    float pos_e  = robot_pos_.distance(target_);
//    /// 目标点角度误差
//    /// Angle error of target point
//    float theta_e = fabs(target_ori_-robot_ori_.radian_);
//    /// 权重及归一化处理
//    /// Weight and normalization processing
//    pos_e=pos_e*weight/900;
//    theta_e=theta_e*(1-weight)/(M_PI);

//    /// 求得新的PD参数
//    /// Get new PD parameters
//    p_move_=pval/(pos_e+1);
//    d_move_=dval/(pos_e+1);
//    p_rotation_=pval;
//    d_rotation_=dval;
}

/// \brief 根据目标点计算当前的Vx，Vy
/// \brief Calculate the current Vx, Vy according to the target point
void Nubot_HWController::move2target()
{
    float _pos_e = robot_pos_.distance(target_);
    nubot::DPoint2s relposoftarget =  target_  - robot_pos_;
    float tar_theta = relposoftarget.angle().radian_;
    static float _pos_e1 = 0;
    float speed  = 0;
    p_move_ = 5; d_move_ = 0;
    speed = basicPDControl(p_move_,d_move_,_pos_e,_pos_e1,maxvel_);
    Vx =  speed*cos(tar_theta - robot_ori_.radian_) + target_vel_.x_;
    Vy =  speed*sin(tar_theta - robot_ori_.radian_) + target_vel_.y_;

    double v=sqrt(Vx*Vx+Vy*Vy);
    if(v>maxvel_)
    {
        Vx=Vx*maxvel_/v;
        Vy=Vy*maxvel_/v;
    }
    //if(robot_name.c_str() == "NuBot3") ROS_INFO("velocity:%d",Vx);
    _pos_e1  = _pos_e;
}

/// \brief 根据目标角度计算当前的w
/// \brief calculate the current w according to the target angle
void Nubot_HWController::rotate2AbsOrienation()
{
    float theta_e = target_ori_-robot_ori_.radian_;
    static float theta_e1 =  0;
    // rotate_mode: 0-根据小角度选择旋转方向，1-顺时针方向，-1-逆时针方向
    // rotate_mode: 0-Select the direction of rotation according to a small angle, 1-clockwise, -1-counterclockwise
    if(rotate_mode_==-1)
    {
        if(theta_e>0)
            theta_e = theta_e-2*SINGLEPI_CONSTANT;
    }
    else if(rotate_mode_==1)
    {
        if(theta_e<0)
            theta_e = theta_e+2*SINGLEPI_CONSTANT;
    }
    else
    {
        while(theta_e > SINGLEPI_CONSTANT) theta_e = theta_e-2*SINGLEPI_CONSTANT;
        while(theta_e <= -SINGLEPI_CONSTANT) theta_e = theta_e+2*SINGLEPI_CONSTANT;
    }
    p_rotation_=2; d_rotation_=0;
    w = basicPDControl(p_rotation_,d_rotation_,theta_e,theta_e1,maxw_);
    theta_e1 = theta_e ;
}

/// \brief 采用极坐标控制，只产生向前的平移和转动，模拟差动控制避免丢球
/// \brief adopts polar coordinate control, only produces forward translation and rotation, and simulates differential control to avoid losing the ball
void Nubot_HWController::movewithball()
{
//    nubot::DPoint2s robot2target=target_-robot_pos_;
//    float beta=robot2target.angle().radian_-target_ori_;
//    while(beta > SINGLEPI_CONSTANT) beta = beta-2*SINGLEPI_CONSTANT;
//    while(beta <= -SINGLEPI_CONSTANT) beta = beta+2*SINGLEPI_CONSTANT;

//    float alpha=robot2target.angle().radian_-robot_ori_.radian_;
//    while(alpha > SINGLEPI_CONSTANT) alpha = alpha-2*SINGLEPI_CONSTANT;
//    while(alpha <= -SINGLEPI_CONSTANT) alpha = alpha+2*SINGLEPI_CONSTANT;

//    float p=robot_pos_.distance(target_);

//    float k1=1.5,k2=1,k3=1;

//    Vx= k1*p*cos(alpha);
//    Vy= 0;
//    w = k2*alpha+k1*(sin(alpha)*cos(alpha)/alpha)*(alpha+k3*beta);

//    if(Vx>maxvel_)
//        Vx=maxvel_;

}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"nubot_hwcontroller_node");
    Nubot_HWController controller(argc,argv);
    ros::spin();
    return 0;
}
