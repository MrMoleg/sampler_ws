#include "ros_deep_sampler/autonomy_controller.hpp"


namespace ros_deep_sampler{




AutonomyController::AutonomyStates AutonomyController::getState()const{
    return state_;
};

void AutonomyController::setState(AutonomyStates state){
    state_ = state;
}

bool AutonomyController::getCalibrationState(){
    return joints_calibrated;
}

void AutonomyController::requestCalibration(SamplerState& sampler, JointMovement& joints_,
                        const rclcpp::Logger& logger){
   if(!goal_sent){
        RCLCPP_INFO(logger, "Starting JOINTS CALIBRATION");
        joints_.stopMotors();
        joints_.calibrateJoints(PLATFORM_JOINT_CALIBRATION_SPEED);
        goal_sent = true;
    }

    if(joints_.get_velocity_controller_status()){
        joints_.send_calibration_velocities(PLATFORM_JOINT_CALIBRATION_SPEED, DRILL_JOINT_CALIBRATION_SPEED, 
            CONTAINER_JOINT_CALIBRATION_SPEED);
    }
    
    if(std::fabs(HOME_POS - joints_.get_current_position(JointMovement::JointsIds::PLATFORM)) < PRECISION_THRESHOLD 
                    && !calibrated_platform){
        PLATFORM_JOINT_CALIBRATION_SPEED = 0.0;
        joints_.send_calibration_velocities(PLATFORM_JOINT_CALIBRATION_SPEED, DRILL_JOINT_CALIBRATION_SPEED, 
                                            CONTAINER_JOINT_CALIBRATION_SPEED);
        calibrated_platform = true;
        RCLCPP_INFO(logger, "Finishing calibration of PLATFORM joint");
    }
    if(std::fabs(HOME_POS - joints_.get_current_position(JointMovement::JointsIds::DRILL)) < PRECISION_THRESHOLD
                && !calibrated_drill){
        DRILL_JOINT_CALIBRATION_SPEED = 0.0;
        joints_.send_calibration_velocities(PLATFORM_JOINT_CALIBRATION_SPEED, DRILL_JOINT_CALIBRATION_SPEED, 
                                            CONTAINER_JOINT_CALIBRATION_SPEED);
        calibrated_drill = true;
        RCLCPP_INFO(logger, "Finishing calibration of DRILL joint");
    }
    if(std::fabs(HOME_POS - joints_.get_current_position(JointMovement::JointsIds::CONTAINER)) < PRECISION_THRESHOLD
                && !calibrated_container){
        CONTAINER_JOINT_CALIBRATION_SPEED = 0.0;
        joints_.send_calibration_velocities(PLATFORM_JOINT_CALIBRATION_SPEED, DRILL_JOINT_CALIBRATION_SPEED, 
                                            CONTAINER_JOINT_CALIBRATION_SPEED);
        calibrated_container = true;
        RCLCPP_INFO(logger, "Finishing calibration of CONTAINER joint");
    }

    if (calibrated_platform && calibrated_drill && calibrated_container){
        if (joints_.get_position_controller_status()){
            joints_calibrated = true;
            joints_.send_calibration_velocities(0.0, 0.0, 0.0);
            goal_sent = false;
            joints_.setTrajectoryStatus(false);
            time_between_states = 0;
            sampler.distance_to_ground = joints_.get_distance_to_ground();
            state_ = AutonomyStates::MOVE_PLATFORM_DOWN; 
            RCLCPP_INFO(logger, "Finishing calibration of all joints");
            return;
        }
        if(joints_.get_controller_switching_tatus()){
            joints_.switchToPositionController();
        }
        

    }
}

void AutonomyController::resetStateVariables(){
    goal_sent = false;
    commands.clear();
    time_between_states = 0;
    rotation_time = 0;
    recovery_attempt = 0;
    calibrated_drill = false;
    calibrated_platform = false;
    calibrated_container = false;

    joints_calibrated = false;
}

void AutonomyController::stop(JointMovement& joints_){
  
   
    if(goal_sent){
        joints_.stopMotors();
        //move_client_->cancel_goal();
        joints_.cancelMovement();
        resetStateVariables();
    }
    
}

void AutonomyController::checkStall(
        SamplerState& sampler,
        const rclcpp::Time& now,
        const rclcpp::Logger& logger){

    // Is the drill actually rotating?
    bool drill_not_moving =
        std::fabs(sampler.rotor_velocity) < MIN_SPEED;

    if (sampler.rotor_in_action && drill_not_moving)
    {
        // Start the stall timer
        if (!sampler.drill_stall_active_)
        {
            sampler.drill_stall_active_ = true;
            sampler.stall_start_time_ = now;

            RCLCPP_WARN(
                logger,
                "Possible drill stall detected");
        }

        // Check how long it has been stalled
        double stall_duration =
            (now - sampler.stall_start_time_).seconds();

        if (stall_duration >= STALL_TIME)
        {
            if (!sampler.drill_stuck_)
            {
                sampler.drill_stuck_ = true;

                RCLCPP_INFO(
                    logger,
                    "DRILL STUCK!");
            }
        }
    }
    else
    {
        // Drill is moving normally again
        sampler.drill_stall_active_ = false;
        sampler.drill_stuck_ = false;
    }

}

}


