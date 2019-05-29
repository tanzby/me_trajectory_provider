#include <Eigen/Dense>
#include <deque>
#include <vector>
#include <flann/flann.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>

#include "Plotter.h"
#include "math/curve_fitting.h"
#include "nox_msgs/Location.h"
#include "nox_msgs/Lane.h"
#include "eyeq2_msgs/mobileye_lane.h"


class TrajectoryProvider
{
    using Point  = Eigen::Vector2f;
    using Points = std::vector<Point>;
    using Pose   = Eigen::Matrix4f;

    typedef message_filters::sync_policies::ApproximateTime
    <nav_msgs::Odometry, eyeq2_msgs::mobileye_lane> _sync_policies;

    message_filters::Subscriber<nav_msgs::Odometry>* syn_nav_sub;
    message_filters::Subscriber<nox_msgs::Location>* syn_loc_sub;
    message_filters::Subscriber<eyeq2_msgs::mobileye_lane>* syn_lane_sub;
    message_filters::Synchronizer<_sync_policies>* synchronizer;

    ros::Publisher trajectory_pub;

    ros::Publisher trajectory_pub_rviz;

    struct Scan
    {
        Pose pose;

        Points left_lane;   // 全局坐标系下
        Points right_lane;  // 全局坐标系下
        Points reference_points;

        Scan(const Pose& pose, const Points& lp, const Points& rp)
        {
            this->pose = pose;
            left_lane = lp;
            right_lane = rp;

            std::vector<double> dataset_vec;  // left
            std::vector<double> query_vec;    // right

            for(auto&p: left_lane)
            {
                dataset_vec.emplace_back(p[0]);
                dataset_vec.emplace_back(p[1]);
            }

            for(auto&p: right_lane)
            {
                query_vec.emplace_back(p[0]);
                query_vec.emplace_back(p[1]);
            }

            flann::Matrix<double> dataset(dataset_vec.data(), left_lane.size(),2);
            flann::Matrix<double>  query (query_vec.data(),  right_lane.size(),2);

            flann::Index<flann::L2<double>> index(dataset, flann::KDTreeIndexParams(1));
            index.buildIndex();

            std::vector<std::vector<int>> indices;
            std::vector<std::vector<double>> dists;

            index.knnSearch(query, indices, dists, 1, flann::SearchParams(128));

            if(indices.size() < 2)
            {
                ROS_INFO("FLANN cannot find enough pair to link reference line for current frame.");
            }

            for(int k = 0; k < indices.size(); ++k)
            {
                int i = indices[k][0];
                double sqdist = dists[k][0];

                // TODO 去除距离过大的连接

                reference_points.emplace_back( (left_lane[i] + right_lane[k]) * 0.5);
            }
        }

    };

    double lane_curve_sample_distance; // me 提取曲线使用的长度

    double lane_curve_sample_step;     // me 提供曲线中采样步长

    double scan_lane_angle_diff;       // 左右车道方向差距

    double add_keyframe_angle_diff;    // 添加关键帧的最大方向误差

    double add_keyframe_distance;      // 添加关键帧的最小距离

    double keep_back_distance;         // 保持尾部点的最大区域距离

    double trajectory_sample_distance; // 发布的引导线点采样间隔

    double max_reset_distance;         // 执行清除行为的最大帧间（关键帧与扫描帧）差距

    int    max_keyframe_size;          // 最大关键帧数量

    double me_to_imu_yaw;

    double me_to_imu_x;

    double me_to_imu_y;


    Pose   current_pose;

    Point  estimate_endpoint;

    std::deque<Scan>  key_scans;

    std::deque<Point> reference_points;


    float compute_accurate_odom(const Pose& prev, const Pose& current)
    {
        Pose acc_odom = prev.inverse() * current;

        return acc_odom.block(0,3,2,1).norm();
    }

    void transform_points(const Points& points,Points& output, Pose& pose)
    {
        output.resize(points.size());
        for (int i = points.size() - 1; i >= 0; --i) {
            output[i] = pose.block(0,0,2,2) * points[i] + pose.block(0,3,2,1);
        }
    }

    bool check_scan_points(const std::vector<Points>& lane_points)
    {
        // 1. 点是否齐全

        if(lane_points[0].size() < 5 || lane_points[1].size() < 5)
        {
            ROS_WARN("There are not enough points, no lane lines may be detected.");
            return false;
        }

        // 2. 两车道的方向在误差范围内

        auto a1 = std::atan(FitPolynomial<1>(lane_points[0])[1]);
        auto a2 = std::atan(FitPolynomial<1>(lane_points[1])[1]);

        if(scan_lane_angle_diff < std::abs(a1-a2))
        {
            ROS_WARN("The difference between the two lane lines' direction is too large, skipping");
            return false;
        }

        if(!key_scans.empty())
        {
            double pose_diff = compute_accurate_odom(key_scans.back().pose, current_pose);

            // 3. 如果两帧差距比较大，清空
            if(pose_diff > max_reset_distance)
            {
                key_scans.clear();
                reference_points.clear();
                ROS_WARN("The difference between the two frames is relatively large, clear.");
                return true;
            }
        }

        // 4. 如果发生大范围倒退
        if( !reference_points.empty())
        {
            Point ref_front = reference_points.front();
            Point current_position = current_pose.block(0,3,2,1);

            if( (reference_points.back() - ref_front).dot( current_position- ref_front) < - 1.2 * keep_back_distance )
            {
                key_scans.clear();
                reference_points.clear();
                ROS_WARN("Regressive behavior occurred, clear.");
                return true;
            }
        }

        return true;
    }

    void compute_reference_points()
    {
        std::vector<Point> candidate_ref_points;

        for(auto& keyscan: key_scans)
        {
            if(keyscan.reference_points.size() < 2)
            {
                ROS_INFO("A keyscan's reference points is not enough, skip");
                continue;
            }

            candidate_ref_points.insert(

                    candidate_ref_points.end(),
                    keyscan.reference_points.begin(),
                    keyscan.reference_points.end()
            );
        }

        // 判断方向
        auto  coef = FitPolynomial<1>(candidate_ref_points);

        double neg = key_scans.back().pose(0,0) > 0 ? 1.0 : -1.0;

        Eigen::Vector2f direction( neg, coef[1]*neg );

        Point current_position =  current_pose.block(0,3,2,1);

        Point reer_point       =  reference_points.empty()? current_position : reference_points.back();

        // 去除背后 keep_back_distance 的点
         while (!reference_points.empty())
        {
            if ( (reference_points.front() - current_position).dot(direction) < -keep_back_distance )
            {
                reference_points.pop_front();
            }
            else break;
        }

        // 以上一次路径的结尾点作为中心进行排序
        std::sort(candidate_ref_points.begin(), candidate_ref_points.end(),

            [&](Point &a, Point &b)
            {
                return (a - reer_point).dot(direction) < (b - reer_point).dot(direction);
            }
        );

        // 间隔采样
        double acc_dis = 0;
        for (auto &p: candidate_ref_points)
        {
            // 只取上一次 reference_point 之后的点
            if ((p - reer_point).dot(direction) > 0)
            {
                if (reference_points.empty()) {

                    reference_points.emplace_back(p);
                    continue;
                }

                acc_dis += (reference_points.back() - p).norm();

                if (acc_dis > trajectory_sample_distance)
                {
                    acc_dis = 0;
                    reference_points.emplace_back(p);
                }
            }
        }

        auto GetLineCorrsPoint = [&](Point &point) -> Point
        {
            double xt = (point[0] + coef[1] * point[1] - coef[0] * coef[1]) / (1.0 + coef[1] * coef[1]);
            double yt = coef[0] + coef[1] * xt;
            return {xt, yt};
        };

        // 不超过估计的终点后的点
        while(!reference_points.empty())
        {
            if((estimate_endpoint - reference_points.back()).dot(direction) < 0)
            {
                reference_points.pop_back();
            }
            else {
                break;
            }
        }

        // 使用局部直线进行优化
        for (auto &p: reference_points)
        {
            p = (p + GetLineCorrsPoint(p)) * 0.5;
        }
        
        // 往背后添加点

        while ((reference_points.front() - current_position).dot(direction) > -keep_back_distance)
        {
            Point p = reference_points.front() -direction * trajectory_sample_distance;
            reference_points.push_front(GetLineCorrsPoint(p));
        }

    }

    void optimize_keyscans(std::vector<Points>& lane_points_map)
    {
        if (!key_scans.empty())
        {
            // 1. EMA 调整车道结束点结束点
            static double beta = 0.98;

            Point new_end_point = (lane_points_map[0].back() + lane_points_map[1].back())* 0.5;
            estimate_endpoint = estimate_endpoint * beta + (1.0-beta) * new_end_point;
        }

        // 2. ME不好把控，关键帧的姿态很难进行调整，TODO

    }

    visualization_msgs::Marker convert_to_rviz_points(const Points& points, int r, int g, int b, double scale = 0.5, int marker_id = 0)
    {

        visualization_msgs::Marker reference_points_marker;

        reference_points_marker.header.frame_id = "/world";
        reference_points_marker.header.stamp = ros::Time::now();
        reference_points_marker.action = visualization_msgs::Marker::ADD;

        reference_points_marker.pose.orientation.w = 1;
        reference_points_marker.id = marker_id;
        reference_points_marker.type = visualization_msgs::Marker::POINTS;

        reference_points_marker.scale.x = scale;
        reference_points_marker.scale.y = scale;

        reference_points_marker.color.g = g/255.0;
        reference_points_marker.color.r = r/255.0;
        reference_points_marker.color.b = b/255.0;
        reference_points_marker.color.a = 1.0f;

        for(auto& point: points)
        {
            geometry_msgs::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = 0;
            reference_points_marker.points.push_back(p);
        }

        return reference_points_marker;
    }

    void publish_trajectory()
    {

        /**
         * 黄色：发布的引导线点
         * 白色：各关键帧的中心线点
         * 红色：右侧车道线点
         * 绿色：左侧车道线点
         * 蓝色：当前位置
         */

        Plotter::Instance()->clear();
        Plotter::Instance()->plot(reference_points,{255,255,0,255},1);
        Plotter::Instance()->plot(current_pose.block(0,3,2,1),{0,100,255,255},4);

        Plotter::Instance()->plot(reference_points.front(),{255,0,0,255},5); // 蓝色
        Plotter::Instance()->plot(reference_points.back(), {0,255,0,255},5); // 绿色

        visualization_msgs::Marker reference_points_marker = convert_to_rviz_points({},255,255,0,1.0,0);

        nox_msgs::Lane lane;

        for (auto &p: reference_points)
        {

            geometry_msgs::Point gpoint;
            geometry_msgs::Vector3 point = geometry_msgs::Vector3();

            gpoint.x = point.x = p[0];
            gpoint.y = point.y = p[1];
            gpoint.z = point.z = 0;

            lane.points.push_back(point);
            reference_points_marker.points.push_back(gpoint);
        }
        trajectory_pub.publish(lane);

        visualization_msgs::MarkerArray markerArray;

        auto t = ros::Time::now();

        for(auto& keyscan: key_scans)
        {
            markerArray.markers.push_back(convert_to_rviz_points(keyscan.reference_points, 255, 255, 255,0.5, 1));
            markerArray.markers.push_back(convert_to_rviz_points(keyscan.left_lane, 255, 0, 0, 0.5,2));
            markerArray.markers.push_back(convert_to_rviz_points(keyscan.right_lane, 0, 255, 0, 0.5, 3));

            Plotter::Instance()->plot(keyscan.reference_points,{255,255,255,255},0);
            Plotter::Instance()->plot(keyscan.left_lane,{255,0,0,255},1);
            Plotter::Instance()->plot(keyscan.right_lane,{0,255,0,255},1);
        }

        markerArray.markers.push_back(reference_points_marker);

        trajectory_pub_rviz.publish(markerArray);

        Plotter::Instance()->show(false);
    }

    Eigen::Matrix3f EularAngleToMatrix(float yaw, float pitch, float roll)
    {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;

        return q.matrix().cast<float>();
    }

    Pose convert_odom_to_mat(const nav_msgs::OdometryConstPtr &nav_msg)
    {

        auto &tranf = nav_msg->pose.pose.position;
        auto &orien = nav_msg->pose.pose.orientation;

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        Eigen::Quaterniond quad(orien.w, orien.x, orien.y, orien.z);

        pose.block(0, 0, 3, 3) = quad.matrix().cast<float>();
        pose.block(0, 3, 3, 1) = Eigen::Vector3f(tranf.x, tranf.y, tranf.z);


        Eigen::Matrix4f me_to_imu = Eigen::Matrix4f::Identity();
        me_to_imu.block(0, 0, 3, 3) = EularAngleToMatrix(me_to_imu_yaw, 0, 0);
        me_to_imu.block(0, 3, 3, 1) = Eigen::Vector3f(me_to_imu_x, me_to_imu_y, 0);

        return me_to_imu*pose;
    }

    void me_lane_callback(const nav_msgs::OdometryConstPtr &nav_msg,
                          const eyeq2_msgs::mobileye_laneConstPtr &lane_msg_ptr)
    {
        current_pose = convert_odom_to_mat(nav_msg);

        std::vector<Points> lane_points(2);

        for (int i = 0; i < 2; i++)
        {
            double t = 0, length = lane_msg_ptr->lane_range[i];

            const double limit_length = lane_curve_sample_distance < length ? lane_curve_sample_distance : length;

            while (t < limit_length) {
                double point_x = t;
                double point_y = t * t * t * lane_msg_ptr->lane_A3[i] +
                                 t * t * lane_msg_ptr->lane_A2[i] +
                                 t * lane_msg_ptr->lane_A1[i] +
                                 lane_msg_ptr->lane_A0[i];

                lane_points[i].emplace_back(point_x, -point_y);
                t += lane_curve_sample_step;
            }
        }

        // 在局部坐标系比较两车道线间
        if(!check_scan_points(lane_points))
        {
            if (!key_scans.empty())
            {
                static double beta = 0.995;
                estimate_endpoint = estimate_endpoint*beta + (1.0-beta)*current_pose.block(0,3,2,1);
            }
        }
        else
        {
            // 将点转变到全局坐标系下
            std::vector<Points>  lane_points_map(2); // map 坐标系下的lane points
            transform_points(lane_points[0], lane_points_map[0], current_pose);
            transform_points(lane_points[1], lane_points_map[1], current_pose);


            // 添加关键帧
            if(key_scans.empty())
            {
                key_scans.emplace_back(current_pose, lane_points_map[0], lane_points_map[1]);
                estimate_endpoint = key_scans.back().reference_points.back();
            }
            else
            {
                double accurate_distance = compute_accurate_odom(key_scans.back().pose, current_pose);

                if(accurate_distance > add_keyframe_distance)
                {
                    key_scans.emplace_back(current_pose, lane_points_map[0], lane_points_map[1]);
                    if(key_scans.size() > max_keyframe_size)
                    {
                        key_scans.pop_front();
                    }
                }
                else optimize_keyscans(lane_points_map);
            }
        }

        compute_reference_points();  // will modify reference points

        publish_trajectory(); // publish reference points and other values to RVIZ and OpenCV drawpad.
    }


public:

    void run(ros::NodeHandle& nh)
    {

        std::string nav_msg_topic  = "/gnss/odom";
        std::string loc_msg_topic  = "/Localization";
        std::string lane_msg_topic = "/mobileye_lane";

        nh.param<double>("lane_curve_sample_distance",lane_curve_sample_distance,20);
        ROS_INFO("lane_curve_sample_distance: %lf",lane_curve_sample_distance);

        nh.param<double>("lane_curve_sample_step", lane_curve_sample_step, 0.25);
        ROS_INFO("lane_curve_sample_step: %lf",lane_curve_sample_step);

        nh.param<double>("scan_lane_angle_diff", scan_lane_angle_diff, 3 * M_PI / 180.0);
        ROS_INFO("scan_lane_angle_diff: %lf",scan_lane_angle_diff);

        nh.param<double>("add_keyframe_angle_diff", add_keyframe_angle_diff, 6 * M_PI / 180.0);
        ROS_INFO("add_keyframe_angle_diff: %lf",add_keyframe_angle_diff);

        nh.param<double>("add_keyframe_distance", add_keyframe_distance, 4);
        ROS_INFO("add_keyframe_distance: %lf",add_keyframe_distance);

        nh.param<int>("max_keyframe_size", max_keyframe_size, 3);
        ROS_INFO("max_keyframe_size: %d",max_keyframe_size);

        nh.param<double>("keep_back_distance", keep_back_distance, 5.0);
        ROS_INFO("keep_back_distance: %lf",keep_back_distance);

        nh.param<double>("trajectory_sample_distance", trajectory_sample_distance, 0.4);
        ROS_INFO("trajectory_sample_distance: %lf",trajectory_sample_distance);

        nh.param<double>("max_reset_distance", max_reset_distance, 8);
        ROS_INFO("max_reset_distance: %lf",max_reset_distance);

        nh.param<double>("tran_yaw", me_to_imu_yaw, 0);
        ROS_INFO("transform me to imu: yaw: %lf", me_to_imu_yaw);
        nh.param<double>("tran_x", me_to_imu_x, 2.0);
        ROS_INFO("transform me to imu: x: %lf", me_to_imu_x);
        nh.param<double>("tran_y", me_to_imu_y, 0);
        ROS_INFO("transform me to imu: y: %lf", me_to_imu_y);


        trajectory_pub = nh.advertise<nox_msgs::Lane>("/mobileye_trajectory", 10, false);
        trajectory_pub_rviz = nh.advertise<visualization_msgs::MarkerArray>("/mobileye_trajectory_makers", 10, true);

        syn_nav_sub  = new message_filters::Subscriber<nav_msgs::Odometry>(nh, nav_msg_topic, 10);
        syn_loc_sub  = new message_filters::Subscriber<nox_msgs::Location>(nh, loc_msg_topic, 10);
        syn_lane_sub = new message_filters::Subscriber<eyeq2_msgs::mobileye_lane>(nh, lane_msg_topic, 10);
        synchronizer = new message_filters::Synchronizer<_sync_policies>(_sync_policies(10), *syn_nav_sub,
                                                                         *syn_lane_sub);
        synchronizer->registerCallback(boost::bind(&TrajectoryProvider::me_lane_callback, this, _1, _2));
    }

    ~TrajectoryProvider()
    {
//        if(syn_nav_sub) delete(syn_nav_sub);
//        if(syn_lane_sub) delete(syn_lane_sub);
//        if(synchronizer) delete(synchronizer);
    }


};


int main(int argc, char** argv)
{
    // init ros part
    ros::init(argc, argv, "me_trajectory_provider_node");

    ros::NodeHandle node("~");

    Plotter::Instance(1200, 300, 0.08, 120, -105); // opencv drawpad.

    TrajectoryProvider provider;

    provider.run(node);

    ros::spin();

    return 0;
}
