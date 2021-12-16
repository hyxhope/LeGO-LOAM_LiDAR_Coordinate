//
// Created by hyx on 2021/12/4.
//

#include <utility.h>

class OdometrySaverNode {
private:
    ros::NodeHandle nh;
    ros::WallTimer timer;

    std::string endpoint_frame;
    std::string origin_frame;

    int saved_points;
    int saved_odometry;

    std::string dst_directory;

    queue<sensor_msgs::PointCloud2ConstPtr> points_save_queue;
    queue<nav_msgs::OdometryConstPtr> odometry_save_queue;

    ros::Subscriber points_sub;
    ros::Subscriber odometry_sub;

    ros::Publisher pubWorldOdometry;

    tf::TransformListener tf_listener;

    pcl::PointCloud<PointType>::Ptr globalMap;

    std::mutex mBuf;


public:

    OdometrySaverNode() : nh("~"), tf_listener(ros::DURATION_MAX) {

        endpoint_frame = nh.param<std::string>("endpoint_frame", "camera_init");
        origin_frame = nh.param<std::string>("origin_frame", "map");

        points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 128,
                                                            &OdometrySaverNode::points_callback, this);
        odometry_sub = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init", 128,
                                                        &OdometrySaverNode::odometry_callback, this);


        pubWorldOdometry = nh.advertise<nav_msgs::Odometry>("/world_odom", 5);

        dst_directory = nh.param<std::string>("dst_directory", "/tmp/odometry");
        boost::filesystem::create_directories(dst_directory);

        globalMap.reset(new pcl::PointCloud<PointType>());

        saved_points = 0;
        saved_odometry = 0;

        timer = nh.createWallTimer(ros::WallDuration(1.0), &OdometrySaverNode::timer_callback, this);

    }

    ~OdometrySaverNode() {
    }

    void timer_callback(const ros::WallTimerEvent &e) {
//        std::cout << "--- saver pcd ---" << std::endl;
//
//        std::cout << "points:" << points_save_queue.size() << "  odometry:" << odometry_save_queue.size() << std::endl;
//
//        ROS_INFO_STREAM("queue points:" << points_save_queue.size() << "  odometry:" << odometry_save_queue.size());
//        ROS_INFO_STREAM("saved points:" << saved_points << "  odometry:" << saved_odometry);

    }

    void points_callback(const sensor_msgs::PointCloud2ConstPtr &points_msg) {
        saved_points++;
//        points_save_queue.push(points_msg);
    }

    void odometry_callback(const nav_msgs::OdometryConstPtr &odometry_msg) {
        saved_odometry++;
        Eigen::Matrix4d origin2odom = lookup_eigen(odometry_msg->header.frame_id, origin_frame);
        Eigen::Matrix4d odom2base = lookup_eigen(endpoint_frame, odometry_msg->child_frame_id);

        const auto &pose = odometry_msg->pose.pose;
        Eigen::Matrix4d odombase2odom = Eigen::Matrix4d::Identity();
        odombase2odom.block<3, 1>(0, 3) = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
        odombase2odom.block<3, 3>(0, 0) = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                                             pose.orientation.z).toRotationMatrix();

        Eigen::Matrix4d result = odom2base * odombase2odom * origin2odom;
        Eigen::Quaterniond quat(result.block<3, 3>(0, 0));

        nav_msgs::OdometryPtr transformed(new nav_msgs::Odometry);
        *transformed = *odometry_msg;

        auto &dst_pose = transformed->pose.pose;
        dst_pose.position.x = result(0, 3);
        dst_pose.position.y = result(1, 3);
        dst_pose.position.z = result(2, 3);

        dst_pose.orientation.w = quat.w();
        dst_pose.orientation.x = quat.x();
        dst_pose.orientation.y = quat.y();
        dst_pose.orientation.z = quat.z();

        pubWorldOdometry.publish(transformed);

//        odometry_save_queue.push(transformed);
    }

    Eigen::Matrix4d
    lookup_eigen(const std::string &target, const std::string &source, const ros::Time &stamp = ros::Time(0)) {
        if (!tf_listener.waitForTransform(target, source, stamp, ros::Duration(5.0))) {
            return Eigen::Matrix4d::Identity();
        }

        tf::StampedTransform transform;
        tf_listener.lookupTransform(target, source, stamp, transform);

        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        transform.getOpenGLMatrix(matrix.data());

        return matrix;
    }


};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_saver");

    OdometrySaverNode node;

    ros::spin();

    return 0;
}