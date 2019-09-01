#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <dorna_ros/DornaExecute.h>
#include <dorna_ros/DornaPlan.h>
#include <dorna_ros/DornaPlanCmd.h>
#include <dorna_ros/DornaPlay.h>
#include <dorna_ros/DornaView.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#define _ADVERTISE_NODE_SERVICE(Class, Name, NodeHandle) \
    m_##Name##Service = NodeHandle.advertiseService(     \
        "moveit_cmd/" #Name, &Class::m_##Name##ServiceCb, this);

struct MoveItInterface
{
  private:
    using Self = MoveItInterface;
  public:
    void onInit()
    {
        auto& nh = m_nh;
        auto& priv_nh = m_privNh;

        m_tfListener =
            std::make_unique<tf2_ros::TransformListener>(m_tfBuffer, nh);

        m_moveGroupPtr =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                "dorna");

        m_simulationModeService = priv_nh.advertiseService(
            "simulation", &Self::m_simulationModeServiceCb, this);

        _ADVERTISE_NODE_SERVICE(Self, execute, priv_nh);
        _ADVERTISE_NODE_SERVICE(Self, executeInput, priv_nh);
        _ADVERTISE_NODE_SERVICE(Self, moveExtend, priv_nh);
        _ADVERTISE_NODE_SERVICE(Self, moveHome, priv_nh);
        _ADVERTISE_NODE_SERVICE(Self, plan, priv_nh);
        _ADVERTISE_NODE_SERVICE(Self, view, priv_nh);

        m_displayTrajectoryPub =
            nh.template advertise<moveit_msgs::DisplayTrajectory>(
                "/move_group/display_planned_path", 1, true);
        m_executeClient = nh.template serviceClient<dorna_ros::DornaPlay>(
            "/dorna/robot_cmd/play");

        if (const std::string joint_param =
                "/move_group/hardware_interface/joints";
            nh.hasParam(joint_param)) {
            nh.getParam(joint_param, m_jointNames);
        } else {
            ROS_WARN_NAMED(ros::this_node::getName(), "No joints found");
        }

        ROS_INFO_STREAM("Waiting for service: " << m_executeClient.getService());
        m_executeClient.waitForExistence();
        ROS_INFO_STREAM(m_executeClient.getService() << " is available.");

        ROS_INFO_STREAM(ros::this_node::getName() << " init complete");
    }

    bool publishTrajectory() const
    {
        if (m_trajectory) {
            moveit_msgs::DisplayTrajectory trajectory_msg;
            trajectory_msg.trajectory.push_back(m_trajectory.value());
            m_displayTrajectoryPub.publish(trajectory_msg);
            return true;
        }
        ROS_WARN_NAMED(ros::this_node::getName(), "No trajectory to view");
        return false;
    }

    bool gotoNamedTarget(std::string target_name)
    {
        auto ptr = std::find(
            m_namedTargets.begin(), m_namedTargets.end(), target_name);
        if (ptr == m_namedTargets.end()) {
            ROS_WARN_STREAM_NAMED(ros::this_node::getName(),
                                  "No such name: " << target_name);
            return false;
        }
        m_moveGroupPtr->setNamedTarget(target_name);
        return plan();
    }

    bool gotoRandomTarget()
    {
        m_moveGroupPtr->setRandomTarget();
        return plan();
    }

    bool gotoJointValueTarget(std::vector<double> jointAngles)
    {
        m_moveGroupPtr->setJointValueTarget(jointAngles);
        return plan();
    }

    bool gotoRpyTarget(double roll,
                       double pitch,
                       double yaw,
                       std::string endEffectorLink = "")
    {
        m_moveGroupPtr->setRPYTarget(roll, pitch, yaw, endEffectorLink);
        return plan();
    }

    bool gotoOrientationTarget(double x,
                               double y,
                               double z,
                               double w,
                               std::string endEffectorLink = "")
    {
        m_moveGroupPtr->setOrientationTarget(x, y, z, w, endEffectorLink);
        return plan();
    }

    bool gotoPositionTarget(double x,
                            double y,
                            double z,
                            std::string endEffectorLink = "")
    {
        m_moveGroupPtr->setPositionTarget(x, y, z, endEffectorLink);
        return plan();
    }

    bool gotoPoseTarget(geometry_msgs::PoseStamped targetPose,
                        std::string endEffectorLink = "")
    {
        m_moveGroupPtr->setPoseTarget(targetPose, endEffectorLink);
        return plan();
    }

    bool gotoPoseTarget(std::vector<geometry_msgs::PoseStamped> targetPoses,
                        std::string endEffectorLink = "")
    {
        m_moveGroupPtr->setPoseTargets(targetPoses, endEffectorLink);
        return plan();
    }

    bool gotoShiftPoseTargets(
        std::vector<geometry_msgs::PoseStamped> /*targetPoses*/,
        std::string /*endEffectorLink*/ = "")
    {
        //@TODO
        // m_moveGroupPtr->sh(targetPoses, endEffectorLink);
        return plan();
    }

    bool planAndGo()
    {
        if (plan()) {
            return go();
        }
        return false;
    }

    bool plan()
    {
        ROS_INFO_NAMED(ros::this_node::getName(), "Moveit starting to plan");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = m_moveGroupPtr->plan(plan);

        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            m_trajectory = plan.trajectory_;
            ROS_INFO_STREAM_NAMED(
                ros::this_node::getName(),
                "Planned trajectory"); //<< m_trajectory.value());
            return true;
        }
        ROS_WARN_NAMED(ros::this_node::getName(),
                       "MoveIt failed to plan a trajectory");
        m_trajectory = {};
        return false;
    }

    bool go()
    {
        if (!m_trajectory) {
            ROS_WARN_NAMED(ros::this_node::getName(),
                       "No valid trajectory planned");
            return false;
        }
        ROS_INFO_NAMED(ros::this_node::getName(), "Moving soon");
        if (m_simulation) {
            // delegate to moveIt for hardware interface
            // picked up by gazebo
            auto result = m_moveGroupPtr->move();

            if (result ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                return true;
            }
            return false;
        }

        dorna_ros::DornaPlay srv;
        srv.request.joint_trajectory = m_trajectory.value().joint_trajectory;
        return m_executeClient.call(srv);
    }

  protected:
    ros::NodeHandle m_nh, m_privNh = {"~"};
    ros::ServiceServer m_executeService, m_executeInputService, m_planService,
        m_moveExtendService, m_moveHomeService, m_viewService;
    ros::ServiceServer m_simulationModeService;
    ros::Publisher m_displayTrajectoryPub;
    ros::ServiceClient m_executeClient;

    tf2_ros::Buffer m_tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tfListener;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
        m_moveGroupPtr;

    std::vector<std::string> m_jointNames;
    const std::array<std::string, 4> m_namedTargets = {
        "allZeros", "home", "doorPose", "ElbowDown"};

    std::optional<moveit_msgs::RobotTrajectory> m_trajectory;
    bool m_simulation = false;

    bool m_simulationModeServiceCb(std_srvs::SetBool::Request& req,
                                   std_srvs::SetBool::Response& res)
    {
        m_simulation = req.data;
        res.success = true;
        res.message = "Simulation mode: ";
        res.message += (m_simulation ? "on" : "off");
        ROS_INFO_NAMED(ros::this_node::getName(), res.message.c_str());
        return true;
    }

    bool m_viewServiceCb(dorna_ros::DornaView::Request&,
                         dorna_ros::DornaView::Response& res)
    {
        ROS_INFO_NAMED(ros::this_node::getName(), "View Service");
        bool success = publishTrajectory();
        res.response = success;
        return success;
    }

    bool m_executeServiceCb(std_srvs::Empty::Request&,
                            std_srvs::Empty::Response&)
    {
        ROS_INFO_NAMED(ros::this_node::getName(),
            "Execute Pre-planned Trajectory Service");
        return go();
    }

    bool m_executeInputServiceCb(dorna_ros::DornaExecute::Request& req,
                            dorna_ros::DornaExecute::Response&)
    {
        ROS_INFO_NAMED(ros::this_node::getName(), "Execute fron Input Service");
        if (m_simulation) {
            ROS_WARN_NAMED(ros::this_node::getName(),
                "Not implemented for simulation mode");
        }
        dorna_ros::DornaPlay srv;
        srv.request.joint_trajectory = req.trajectory.joint_trajectory;
        return m_executeClient.call(srv);
    }

    bool m_planServiceCb(dorna_ros::DornaPlan::Request& req,
                         dorna_ros::DornaPlan::Response& res)
    {
        ROS_INFO_NAMED(ros::this_node::getName(), "Plan Service");

        auto success = gotoJointValueTarget(req.joint_angles);
        if (m_trajectory) {
            res.trajectory = m_trajectory.value();
        }
        return success;
    }

    bool m_moveExtendServiceCb(dorna_ros::DornaPlanCmd::Request&,
                               dorna_ros::DornaPlanCmd::Response& res)
    {
        ROS_INFO_NAMED(ros::this_node::getName(), "Move Extend Service");
        std::string name = "allZeros";
        assert(m_namedTargets[0] == name);
        bool success = gotoNamedTarget(name);
        if (m_trajectory) {
            res.trajectory = m_trajectory.value();
        }
        return success;
    }

    bool m_moveHomeServiceCb(dorna_ros::DornaPlanCmd::Request&,
                             dorna_ros::DornaPlanCmd::Response& res)
    {
        ROS_INFO_NAMED(ros::this_node::getName(), "Move Home Service");
        std::string name = "home";
        assert(m_namedTargets[1] == name);
        bool success = gotoNamedTarget(name);
        if (m_trajectory) {
            res.trajectory = m_trajectory.value();
        }
        return success;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dorna_moveit_interface");
    std::string name = ros::this_node::getName();
    ROS_INFO_STREAM_NAMED(name, "Starting node: " + name);

    MoveItInterface interface;
    interface.onInit();

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}