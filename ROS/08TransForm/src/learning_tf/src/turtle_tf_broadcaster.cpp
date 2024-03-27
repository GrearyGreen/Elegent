#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
std::string turtle_name;

// 位姿回调函数
void poseCallback(const turtlesim::PoseConstPtr &msg)
{
        // 创建tf的广播器
        static tf::TransformBroadcaster br;

        // 初始化tf数据
        tf::Transform transform;
        //Vector3有4个参数，x,y,z,0（齐次运算）
        transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
        //初始化四元数（quaternion）
        tf::Quaternion q;
        //设置欧拉角（绕x,y,z轴旋转的参数，2d没有x,y的翻转）
        q.setRPY(0, 0, msg->theta);
        //更新transform的数据
        transform.setRotation(q);

        // 广播world与海龟坐标系之间的tf数据（Time::now()在launch中报错）
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
        // 使用launch启动，将海龟的坐标通过转移矩阵到世界坐标系
        br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", turtle_name));
}

int main(int argc, char **argv)
{
        // 初始化ROS节点
        ros::init(argc, argv, "my_tf_broadcaster");

        // 输入参数作为海龟的名字
        if (argc != 2)
        {
                ROS_ERROR("need turtle name as argument");
                return -1;
        }
        turtle_name = argv[1];

        ros::NodeHandle nh;

        // 订阅海龟的位姿话题
        ros::Subscriber sub = nh.subscribe(turtle_name + "/pose", 10, &poseCallback);

        ros::spin();

        return 0;
}