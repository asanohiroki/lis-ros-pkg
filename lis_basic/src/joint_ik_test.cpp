#include <ros/ros.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_IK_joint");
    ros::NodeHandle n;
    std::cout << "Initializing node... " << std::endl;

    //publisher
    ros::Publisher pub_speed_ratio = n.advertise<std_msgs::Float64>("robot/limb/left/set_speed_ratio", 1);
    ros::Publisher pub_joint_cmd = n.advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1);
    //service
    ros::ServiceClient client = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");

    //srv変数
    baxter_core_msgs::SolvePositionIK iksvc;
    iksvc.request.pose_stamp.resize(1);
    iksvc.request.pose_stamp[0].header.stamp = ros::Time::now();
    iksvc.request.pose_stamp[0].header.frame_id = "base";  
    iksvc.request.pose_stamp[0].pose.position.x = 0.7;
    iksvc.request.pose_stamp[0].pose.position.y = 0.0;
    iksvc.request.pose_stamp[0].pose.position.z = 0.0;
    iksvc.request.pose_stamp[0].pose.orientation.x = -0.366894936773;
    iksvc.request.pose_stamp[0].pose.orientation.y = 0.885980397775;
    iksvc.request.pose_stamp[0].pose.orientation.z = 0.108155782462;
    iksvc.request.pose_stamp[0].pose.orientation.w = 0.262162481772;
    
    //Call IK service
    if(client.call(iksvc)){
        ROS_INFO("SUCCESS to call service");
    }
    else{
        ROS_ERROR("FAILED to call service");
        return 1;
    }
    //Check IK result
    if(iksvc.response.isValid[0]){
        ROS_INFO("SUCCESS - Valide Joint Sloution Found:");
    }
    else{
        ROS_ERROR("INVALID POSE");
    }

    //msg変数
    baxter_core_msgs::JointCommand joint_cmd;
    
    //names is a std::vector<std::string>
    joint_cmd.names.push_back("left_s0");//Joint 1
    joint_cmd.names.push_back("left_s1");//Joint 2
    joint_cmd.names.push_back("left_e0");//Joint 3
    joint_cmd.names.push_back("left_e1");//Joint 4
    joint_cmd.names.push_back("left_w0");//Joint 5
    joint_cmd.names.push_back("left_w1");//Joint 6
    joint_cmd.names.push_back("left_w2");//Joint 7
    joint_cmd.command.resize(joint_cmd.names.size());
    joint_cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;//positionモード
    
    //IKで求まった関節角を格納
    for(int i=0; i<joint_cmd.names.size(); i++){
        joint_cmd.command[i] = iksvc.response.joints[0].position[i];
    }

    //Set joint speed
    std_msgs::Float64 speed_ratio;
    speed_ratio.data=0.2; //set joint speed default =0.3 range= 0.0-1.0
    pub_speed_ratio.publish(speed_ratio);
    
    // publish joint_cmd at least 5 Hz, or else Baxter switches back to Position mode and holds position
    ros::Rate loop_rate(10);
    
    while(ros::ok()){
        pub_joint_cmd.publish(joint_cmd);
        ros::spinOnce();
        loop_rate.sleep(); //sleep
    }
    
    cout << "end command" << endl;

    return 0;
}
