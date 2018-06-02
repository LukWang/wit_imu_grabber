#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>

sensor_msgs::Imu Imu_g;
void accCallback(sensor_msgs::Imu imu_msg)
{
    Imu_g = imu_msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "acc_viz");
    ros::NodeHandle nh;
    ros::Publisher acc_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber acc_sub = nh.subscribe("/imu_1/imu_stream", 1, accCallback);


    geometry_msgs::Point start_point, end_point;

    ros::Rate rate(50);
    while(ros::ok())
    {
        visualization_msgs::Marker arrow;
        /***
        start_point.type = visualization_msgs::Marker::POINTS;
        end_point.type = visualization_msgs::Marker::POINTS;
        start_point.pose.position.x = 0.0;
        start_point.pose.position.y = 0.0;
        start_point.pose.position.z = 0.0;
        end_point.pose.position.x = acc_g.accel.linear.x/10.0;
        end_point.pose.position.y = acc_g.accel.linear.y/10.0;
        end_point.pose.position.z = acc_g.accel.linear.z/10.0;
        ***/

        start_point.x = 0.0;
        start_point.y = 0.0;
        start_point.z = 0.0;
        end_point.x = Imu_g.linear_acceleration.x/10.0;
        end_point.y = Imu_g.linear_acceleration.y/10.0;
        end_point.z = Imu_g.linear_acceleration.z/10.0;

        arrow.header.frame_id = "world";
        arrow.header.stamp = Imu_g.header.stamp;
        arrow.ns = "imu";
        arrow.id = 0;
        arrow.color.a = 1.0;
        arrow.color.r = 1.0;
        arrow.scale.x = 0.05;
        arrow.scale.y = 0.08;
        arrow.scale.z = 0.05;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.points.push_back(start_point);
        arrow.points.push_back(end_point);

        acc_pub.publish(arrow);

        rate.sleep();

        ros::spinOnce();
    }


    return 0;
}
