#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include<mav_trajectory_generation_ros/ros_conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_datatypes.h>

//nav_msgs::Odometry current_odom;

/*void odom_cb_(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_odom = msg;
}
*/

#define PI 3.14159265
geometry_msgs::PoseArray traj_with_yaw;
geometry_msgs::Pose traj_pose;
tf::Quaternion q;


int main(int argc,char** argv)
{
    ros::init(argc,argv,"tracking_node");

    ros::NodeHandle nh;
    ros::Publisher traj_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 10);
    ros::Publisher polynomial_trajectory_pub_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments", 1);
    ros::Publisher traj_with_yaw_pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_with_yaw", 10);
    //ros::Subscriber odom_sub_ = nh.subscribe("ground_truth/odometry", 10, odom_cb_);
    ros::Rate sleep_Rate(1);

    while(ros::ok())
    {
        mav_trajectory_generation::Vertex::Vector vertices;
        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        start.makeStartOrEnd(Eigen::Vector3d(2.0,-2.0,0.8), derivative_to_optimize);
        vertices.push_back(start);
        

        /**********************
        First Line 
        ************************/
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2.0, 0.0 , 0.8));
        vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.0, -1.8, 0.8));
        // vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2.0, 2.0, 0.8));
        vertices.push_back(middle);



        /**********************
        Second Line 
        ************************/
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 2.0, 0.8));
        vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-0.8, -1.0, 0.8));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-0.8, -0.4, 0.8));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-0.8, 0.2, 0.8));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-0.8, 0.8, 0.8));
        // vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.0, 2.0, 0.8));
        vertices.push_back(middle);

        /**********************
        Third Line 
        ************************/

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.0, 1.0, 0.8));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.2, 1.0, 0.8));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, 1.0, 0.8));
        // vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.0, 0.0, 0.8));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.0, -2.0, 0.8));
        vertices.push_back(middle);

         /**********************
            Last Line 
        ************************/

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, 0.8, 0.8));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, 0.4, 0.8));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, -0.2, 0.8));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, -0.8, 0.8));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, -1.2, 0.8));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, -1.6, 0.8));
        // vertices.push_back(middle);
        

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2,-1.8,0.8));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.4, -1.8, 0.8));
        // vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0.0, -2.0, 0.8));
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(2.0, -2.0, 0.8), derivative_to_optimize);
        vertices.push_back(end);

        /*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

        // start.makeStartOrEnd(Eigen::Vector3d(-1.6,-0.3,0.5), derivative_to_optimize);
        // vertices.push_back(start);
        

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, 0.3, 0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, 0.9 ,0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.0, 0.3, 0.5));
        // vertices.push_back(middle);

        
        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, -0.3, 0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, -0.9, 0.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, -1.5, 0.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.0, -0.9, 0.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, -0.3, 0.5));
        // vertices.push_back(middle);


        // // /* TRaj 2*/

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, 0.3, 0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, 0.9 ,0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.0, 0.3, 0.5));
        // vertices.push_back(middle);

        
        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, -0.3, 0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, -0.9, 0.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, -1.5, 0.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.0, -0.9, 0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, -0.3, 0.5));
        // vertices.push_back(middle);

        // //  /* TRaj 3*/

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, 0.3, 0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, 0.9 ,0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.0, 0.3, 0.5));
        // vertices.push_back(middle);

        
        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, -0.3, 0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2.2, -0.9, 0.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, -1.5, 0.5));
        // vertices.push_back(middle);

        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.0, -0.9, 0.5));
        // vertices.push_back(middle);


        // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-1.6, -0.3, 0.5));
        // vertices.push_back(middle);


        // end.makeStartOrEnd(Eigen::Vector3d(-2.2, 0.3, 0.5), derivative_to_optimize);
        // vertices.push_back(end);

        std::vector<double> segment_times;
        const double v_max = 1.0;
        const double a_max = 1.0;
        const double magic_fabian_constant = 6.5; // A tuning parameter.
        segment_times = estimateSegmentTimes(vertices, v_max, a_max);

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        mav_trajectory_generation::Segment::Vector segments;
        opt.getSegments(&segments);

        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        double sampling_time = 2.0;
        int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

        double t_start = 2.0;
        double t_end = 10.0;
        double dt = 0.01;
        std::vector<Eigen::VectorXd> result;
        std::vector<double> sampling_times; // Optional.
        trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);

        visualization_msgs::MarkerArray markers;
        double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";

        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        mav_planning_msgs::PolynomialTrajectory4D msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,&msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        polynomial_trajectory_pub_.publish(msg);

        traj_with_yaw.header.stamp = ros::Time::now();
        traj_with_yaw.header.frame_id = "world";


        float traj_marker_size = markers.markers.size();
        float trajectory_size = markers.markers[traj_marker_size - 1].points.size();

        for(int ii=0; ii < trajectory_size; ii++)
        {
            traj_pose.position.x = markers.markers[traj_marker_size - 1].points[ii].x;
            traj_pose.position.y = markers.markers[traj_marker_size - 1].points[ii].y;
            if(ii<trajectory_size-1)
            {
                float y_diff = (markers.markers[traj_marker_size - 1].points[ii+1].y - markers.markers[traj_marker_size - 1].points[ii].y);
                float x_diff = (markers.markers[traj_marker_size - 1].points[ii+1].x - markers.markers[traj_marker_size - 1].points[ii].x);
                float yaw = atan2(y_diff,x_diff);
                q.setRPY(0, 0, yaw);
            }
            else
            {
                float y_diff = (markers.markers[traj_marker_size - 1].points[ii].y - markers.markers[traj_marker_size - 1].points[ii-1].y);
                float x_diff = (markers.markers[traj_marker_size - 1].points[ii].x - markers.markers[traj_marker_size - 1].points[ii-1].x);
                float yaw = atan2(y_diff,x_diff);
                q.setRPY(0, 0, yaw);

            }

            
            traj_pose.orientation.z = q.z();
            traj_pose.orientation.w = q.w();
            //yaw set
            traj_with_yaw.poses.push_back(traj_pose);
        }
        
        traj_with_yaw_pub.publish(traj_with_yaw);
        traj_with_yaw.poses.clear();
        
        ros::spinOnce();

        traj_pub.publish(markers);

        vertices.clear();
        sleep_Rate.sleep();

    }
    return 0;
}
