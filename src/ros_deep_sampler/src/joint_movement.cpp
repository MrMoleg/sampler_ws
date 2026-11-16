#include "ros_deep_sampler/joint_movement.hpp"


namespace ros_deep_sampler{

JointMovement::JointMovement(rclcpp::Node *node)
: node_(node)
{
    position_client_ =
    rclcpp_action::create_client<FollowJointTrajectory>(
        node_,
        "/position_controller/follow_joint_trajectory");
    
    controller_switch_client = node_->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");
    
   
    joints_vel_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_command_controller/commands", 10);

    clamp_cmd_pub_ = 
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/clamp_position_controller/commands", 10);
    rotor_velocity_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/rotor_velocity_controller/commands", 10);
    vacuum_rotor_velocity_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/vacuum_rotor_velocity_controller/commands", 10);
    brush_rotor_velocity_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/brush_rotor_velocity_controller/commands", 10);

    joint_sub_ =
        node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(
                &JointMovement::jointStateCallback,
                this,
                std::placeholders::_1));



    can_id_to_joint_id[RosCanConstants::VescIds::sampler_platform] = JointsIds::PLATFORM;
    can_id_to_joint_id[RosCanConstants::VescIds::sampler_drill_mov] = JointsIds::DRILL;
    can_id_to_joint_id[RosCanConstants::VescIds::sampler_container_a] = JointsIds::CONTAINER;
    can_id_to_joint_id[RosCanConstants::VescIds::sampler_drill] = JointsIds::DRILL_ROTOR;
    can_id_to_joint_id[RosCanConstants::VescIds::sampler_vacuum_suction] = JointsIds::VACUUM_ROTOR;
    can_id_to_joint_id[RosCanConstants::VescIds::sampler_vacuum_a] = JointsIds::BRUSH_ROTOR;
    can_id_to_joint_id[RosCanConstants::VescIds::sampler_vacuum_b] = JointsIds::CLAMP_JOINT;
    
    // To DO: Rename sampler actuators

}

void JointMovement::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Received State Callback");
      for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string & name = msg->name[i];
        current_position_[name] = msg->position[i];
        current_velocity_[name] = msg->velocity[i];
    }
    
  }



double JointMovement::get_current_position(JointsIds id){
 
    if(id == JointsIds::PLATFORM){
        return current_position_["platform_joint"];
    }
    if(id == JointsIds::DRILL){
        return current_position_["drill_joint"];
    }
    if(id == JointsIds::CONTAINER){
        return current_position_["container_joint"];
    }
  

}
double JointMovement::get_current_velocity(JointsIds id){
 
    if(id == JointsIds::PLATFORM){
        return current_velocity_["platform_joint"];
    }
    if(id == JointsIds::DRILL){
        return current_velocity_["drill_joint"];
    }
    if(id == JointsIds::DRILL_ROTOR){
        return current_velocity_["rotor_joint"];
    }
    if(id == JointsIds::CONTAINER){
        return current_velocity_["container_joint"];
    }
    if(id == JointsIds::VACUUM_ROTOR){
        return current_velocity_["vacuum_rotor_joint"];
    }
    if(id == JointsIds::BRUSH_ROTOR){
        return current_velocity_["brush_rotor_joint"];
    }
  
}

bool JointMovement::get_end_switch_state(JointsIds id){
    return end_switch_state[id];

}

double JointMovement::get_distance_to_ground(){
    return distance_to_ground;
}

void JointMovement::switchToVelocityController()
{
    using SwitchController =
        controller_manager_msgs::srv::SwitchController;

    // Wait for the service.
    if (!controller_switch_client->wait_for_service(
            std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Controller manager service unavailable!");
        return;
    }

    auto request = std::make_shared<SwitchController::Request>();

    request->activate_controllers = {
        "velocity_command_controller"
    };

    request->deactivate_controllers = {
        "position_controller"
    };

    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;

    request->timeout.sec = 2;
    request->timeout.nanosec = 0;

    // Send request.
    controller_switch_client->async_send_request(
        request,
        [this](rclcpp::Client<SwitchController>::SharedFuture future)
        {
            auto response = future.get();

            if (response->ok)
            {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "Controller switch successful");
            }
            else
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Controller switch failed: %s",
                    response->message.c_str());
            }
        });

    RCLCPP_INFO(
        node_->get_logger(),
        "Requested switch to velocity controller");
}

void JointMovement::switchToPositionController()
{
    using SwitchController =
        controller_manager_msgs::srv::SwitchController;

    auto request = std::make_shared<SwitchController::Request>();

    request->activate_controllers = {
        "position_controller"
    };

    request->deactivate_controllers = {
        "velocity_command_controller"
    };

    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;

    request->timeout.sec = 2;
    request->timeout.nanosec = 0;

    controller_switch_client->async_send_request(
        request,
        [this](rclcpp::Client<SwitchController>::SharedFuture future)
        {
            auto response = future.get();

            if (response->ok)
            {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "Controller switch successful");
            }
            else
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Controller switch failed: %s",
                    response->message.c_str());
            }
        });

    RCLCPP_INFO(
        node_->get_logger(),
        "Requested switch to position controller");
}

void JointMovement::send_rotor_velocity(JointsIds rotor_id, double vel){
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {vel};
      if(rotor_id == JointsIds::DRILL_ROTOR){
        rotor_velocity_pub_->publish(msg);
      }
      if(rotor_id == JointsIds::VACUUM_ROTOR){
        vacuum_rotor_velocity_pub_->publish(msg);
      }
      if(rotor_id == JointsIds::BRUSH_ROTOR){
        brush_rotor_velocity_pub_->publish(msg);
      }
      
  }

void JointMovement::stopMotors(){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {0.0};
    rotor_velocity_pub_->publish(msg);
      
    vacuum_rotor_velocity_pub_->publish(msg);
      
    brush_rotor_velocity_pub_->publish(msg);
    
}

void JointMovement::open_clamp(){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {CLAMP_OPEN};
    clamp_cmd_pub_->publish(msg);
}
void JointMovement::close_clamp(){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {CLAMP_CLOSED};
    clamp_cmd_pub_->publish(msg);
}

// void JointMovement::setJointsVel(double platform, double drill, double container){
//     std_msgs::msg::Float64MultiArray msg;

//         msg.data = {
//             platform,   // platform [m/s]
//             drill,   // drill [m/s]
//             container    // container [rad/s]
//         };

//         joints_vel_pub_->publish(msg);
// }



void JointMovement::generateTraj(
    trajectory_msgs::msg::JointTrajectory &traj,
    const std::vector<JointCommand>& commands,
    bool use_same_time)
{
    const int N_points = 10;

    if (commands.empty()) {
        throw std::invalid_argument("No joint commands provided");
    }

    // Find the longest requested movement duration.
    double total_time = 0.0;

    for (const auto& cmd : commands) {
        if (cmd.traj_time <= 0.0 || !std::isfinite(cmd.traj_time)) {
            throw std::invalid_argument("Invalid trajectory duration");
        }

        total_time = std::max(total_time, cmd.traj_time);
    }

    // Clear the existing trajectory.
    traj.joint_names.clear();
    traj.points.clear();

    // Register all joints.
    for (const auto& cmd : commands) {
        traj.joint_names.push_back(getJointName(cmd.id));
    }

    // Generate timestamps.
    std::set<double> time_points;

    // Regularly spaced points across the entire trajectory.
    for (int i = 0; i < N_points; ++i) {
        double t = total_time * i / (N_points - 1);
        time_points.insert(t);
    }

    // Add the exact finishing time of each joint.
    for (const auto& cmd : commands) {
        double duration = use_same_time ? total_time : cmd.traj_time;
        time_points.insert(duration);
    }

    // Generate synchronized trajectory points.
    for (double t : time_points) {

        trajectory_msgs::msg::JointTrajectoryPoint point;

        for (const auto& cmd : commands) {

            double distance = cmd.final_position - cmd.start_position;

            // Determine the duration for this particular joint.
            double duration = use_same_time ? total_time : cmd.traj_time;

            double pos, vel, acs;

            if (t >= duration) {
                // This joint has already reached its target.
                pos = cmd.final_position;
                vel = 0.0;
                acs = 0.0;
            }
            else {
                // Normalized time for this joint.
                double u = t / duration;

                double u2 = u * u;
                double u3 = u2 * u;
                double u4 = u3 * u;
                double u5 = u4 * u;

                // Quintic position profile.
                pos = cmd.start_position + distance *
                    (10.0 * u3 - 15.0 * u4 + 6.0 * u5);

                // Velocity.
                vel = distance / duration *
                    (30.0 * u2 - 60.0 * u3 + 30.0 * u4);

                // Acceleration.
                acs = distance / (duration * duration) *
                    (60.0 * u - 180.0 * u2 + 120.0 * u3);
            }

            point.positions.push_back(pos);
            point.velocities.push_back(vel);
            point.accelerations.push_back(acs);
        }

        // Assign timestamp.
        point.time_from_start.sec =
            static_cast<int32_t>(std::floor(t));

        point.time_from_start.nanosec =
            static_cast<uint32_t>((t - std::floor(t)) * 1e9);

        traj.points.push_back(point);
    }
}



 void JointMovement::moveJoints(const std::vector<JointCommand>& commands, bool use_same_time){
    trajectory_msgs::msg::JointTrajectory traj;


    generateTraj(traj, commands, use_same_time);

    sendTrajectory(traj);
}



void JointMovement::calibratePlatform(double vel){
    float final_pos;
    
    std::string joint_name;
    joint_name = "platform_joint";
    final_pos = 0.6; //move up on max 
    double time_to_position =  std::fabs(final_pos)/vel;
    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {final_pos, final_pos, -final_pos}; 
    RCLCPP_INFO(node_->get_logger(), "Final position: %f",final_pos);  
    RCLCPP_INFO(node_->get_logger(), "Time to position: %f",time_to_position);
    p.time_from_start = rclcpp::Duration::from_seconds(time_to_position);
    traj.joint_names = {"platform_joint",
    "drill_joint",
    "container_joint"};
    traj.points.push_back(p);

    sendTrajectory(traj);

    return;

}


void JointMovement::calibrateJoints(double vel){
    float final_pos;
    
    std::string joint_name;
    joint_name = "platform_joint";
    final_pos = 0.6; //move up on max 
    double time_to_position =  std::fabs(final_pos)/vel;
    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {final_pos, final_pos, 0.0}; 
    RCLCPP_INFO(node_->get_logger(), "Final position: %f",final_pos);  
    RCLCPP_INFO(node_->get_logger(), "Time to position: %f",time_to_position);
    p.time_from_start = rclcpp::Duration::from_seconds(time_to_position);
    traj.joint_names = {"platform_joint",
    "drill_joint",
    "container_joint"};
    traj.points.push_back(p);

    sendTrajectory(traj);

    return;

}

std::string JointMovement::getJointName(JointsIds id)
{
    switch(id)
    {
        case JointsIds::PLATFORM:
            return "platform_joint";

        case JointsIds::DRILL:
            return "drill_joint";

        case JointsIds::CONTAINER:
            return "container_joint";

        default:
            return "";
    }
}



void JointMovement::setTrajectoryStatus(bool status){
    trajectory_finished_ = status;
}

void JointMovement::sendTrajectory(
    trajectory_msgs::msg::JointTrajectory &traj)
{
    if(!position_client_->wait_for_action_server(
        std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "JTC unavailable");

        goal_state = MissionMsg::GOAL_FAILED;
        return;
    }


    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj;


    auto options =
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();


    trajectory_finished_ = false;
    goal_state = MissionMsg::GOAL_SENT;


    options.goal_response_callback =
        [this](TJCGoalHandle::SharedPtr handle)
        {
            if(!handle)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Trajectory rejected");

                goal_state = MissionMsg::GOAL_FAILED;
                return;
            }

            RCLCPP_INFO(
                node_->get_logger(),
                "Trajectory accepted");

            goal_state = MissionMsg::GOAL_ACCEPTED;

            active_tjc_goal_ = handle;
        };


    options.result_callback =
        [this](const TJCGoalHandle::WrappedResult &result)
        {
            trajectory_finished_ = true;
            active_tjc_goal_.reset();

            switch(result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Trajectory finished");

                    goal_state = MissionMsg::GOAL_SUCCEEDED;
                    break;


                case rclcpp_action::ResultCode::CANCELED:

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Trajectory canceled");

                    goal_state = MissionMsg::GOAL_CANCELED;
                    break;


                default:

                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Trajectory failed");

                    goal_state = MissionMsg::GOAL_FAILED;
                    break;
            }
        };


    position_client_->async_send_goal(goal, options);
}

bool JointMovement::isTrajectoryFinished()
{
    return trajectory_finished_;
    // if(!result_future_.valid())
    //     return false;


    // return result_future_.wait_for(
    //     std::chrono::seconds(0))
    //     == std::future_status::ready;
}

void JointMovement::cancelMovement()
{
    if(active_tjc_goal_)
    {
        position_client_->async_cancel_goal(
            active_tjc_goal_);

        RCLCPP_INFO(
            node_->get_logger(),
            "Trajectory cancellation requested");
    }
}

void JointMovement::JointGoalStateFeedback(MissionMsg::SharedPtr &feedbackMsg){
    if(prev_goal_state == MissionMsg::GOAL_ACCEPTED){
        goal_state = MissionMsg::GOAL_EXECUTING;
    }
    feedbackMsg->goal_state = goal_state;
    prev_goal_state = goal_state;

}

}
