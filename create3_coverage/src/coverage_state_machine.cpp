// Copyright 2021 iRobot Corporation. All Rights Reserved.

#include <chrono>
#include <math.h>

#include "create3_coverage/coverage_state_machine.hpp"
#include "tf2/utils.h"

namespace create3_coverage {

CoverageStateMachine::CoverageStateMachine(
    create3_examples_msgs::action::Coverage::Goal goal,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger,
    rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
    rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher,
    bool has_reflexes)
    : m_logger(logger)
{
    m_goal = goal;

    m_clock = clock;
    m_start_time = m_clock->now();
    m_has_reflexes = has_reflexes;

    m_dock_action_client = dock_action_client;
    m_undock_action_client = undock_action_client;
    m_cmd_vel_publisher = cmd_vel_publisher;

    m_undocking = false;
    m_preparing_spiral = false;
}

CoverageStateMachine::~CoverageStateMachine()
{
    this->cancel();
}

CoverageStateMachine::CoverageOutput CoverageStateMachine::execute(const Behavior::Data& data)
{
    if (!m_current_behavior) {
        this->select_start_behavior(data);
    } else {
        this->select_next_behavior(data);
    }

    // Handle failure and success
    if (m_coverage_output.state != State::RUNNING) {
        return m_coverage_output;
    }

    m_behavior_state = m_current_behavior->execute(data);
    m_coverage_output.current_behavior = m_current_behavior->get_id();

    return m_coverage_output;
}

void CoverageStateMachine::cancel()
{
    if (m_current_behavior) {
        m_current_behavior->cleanup();
        m_current_behavior.reset();
    }
}

void CoverageStateMachine::select_start_behavior(const Behavior::Data& data)
{
    if (data.dock.is_docked) {
        this->goto_undock();
    } else {
        this->goto_spiral(SpiralBehavior::Config());
    }
}

void CoverageStateMachine::select_next_behavior(const Behavior::Data& data)
{
    // Keep going with the current behavior if it's still running
    if (m_behavior_state == State::RUNNING) {
        m_coverage_output.state = State::RUNNING;
        return;
    }

    // Check if it's time to wrap up the behavior
    bool explore_duration_elapsed = m_clock->now() - m_start_time >= m_goal.explore_duration;
    bool max_runtime_elapsed = m_clock->now() - m_start_time >= m_goal.max_runtime;
    if (max_runtime_elapsed) {
        m_coverage_output.state = State::SUCCESS;
        return;
    }
    if (m_current_behavior->get_id() != FeedbackMsg::DOCK && 
        explore_duration_elapsed && data.dock.dock_visible)
    {
        this->goto_dock();
        return;
    }

    switch (m_current_behavior->get_id())
    {
        case FeedbackMsg::DOCK:
        {
            // A dock action should indicate the termination of the behavior.
            // Do not set a new behavior, either return SUCCESS or FAILURE.
            if (m_behavior_state == State::FAILURE || !data.dock.is_docked) {
                m_coverage_output.state = State::FAILURE;
                break;
            }
            m_coverage_output.state = State::SUCCESS;
            break;
        }
        case FeedbackMsg::DRIVE_STRAIGHT:
        {
            // If we just undocked, then we want to start with a spiral motion
            if (m_undocking) {
                m_undocking = false;
                this->goto_spiral(SpiralBehavior::Config());
                break;
            }

            // If we were trying to get to spiral motion, do it only if drive straight succeeded (i.e. we moved enough from obstacle)
            if (m_preparing_spiral && m_behavior_state == State::SUCCESS) {
                m_preparing_spiral = false;
                this->goto_spiral(SpiralBehavior::Config());
                break;  
            }

            // Usually after a DRIVE_STRAIGHT go to ROTATE
            // If we failed previous drive straight, let's use a random angle 
            auto rotate_config = RotateBehavior::Config();
            if (m_behavior_state == State::FAILURE) {
                
                // Check if we failed too many times consecutively
                if (m_evade_attempts.size() > 20) {
                    m_coverage_output.state = State::FAILURE;
                    break;
                } 

                constexpr double evade_resolution = 0.175433; // 10 degrees
                rotate_config.target_rotation = compute_evade_rotation(data.pose, evade_resolution);
            } else {
                m_evade_attempts.clear();
            }
            rotate_config.robot_has_reflexes = m_has_reflexes;
            this->goto_rotate(rotate_config);
            break;
        }
        case FeedbackMsg::ROTATE:
        {
            // A rotate failure indicates that we haven't been able to clear hazards
            if (m_behavior_state == State::FAILURE) {
                m_coverage_output.state = State::FAILURE;
                break;
            }            

            auto drive_config = DriveStraightBehavior::Config();
            // Check if it's time to go back spiraling, if it's the case we will do only a short DRIVE_STRAIGHT to
            // move away from current obstacle (rather than driving forever until a new obstacle is hit).
            // Alternatively we will go into DRIVE_STRAIGHT forever.
            if (m_clock->now() - m_last_spiral_time >= rclcpp::Duration(std::chrono::seconds(60))) {
                drive_config.max_distance = 0.25;
                drive_config.min_distance = 0.25;
                m_preparing_spiral = true;
            }

            this->goto_drive_straight(drive_config);
            break;
        }
        case FeedbackMsg::SPIRAL:
        {
            if (m_behavior_state == State::SUCCESS) {
                this->goto_drive_straight(DriveStraightBehavior::Config());
            } else {
                auto rotate_config = RotateBehavior::Config();
                rotate_config.robot_has_reflexes = m_has_reflexes;
                this->goto_rotate(rotate_config);
            }
            break;
        }
        case FeedbackMsg::UNDOCK:
        {
            if (m_behavior_state == State::FAILURE || data.dock.is_docked) {
                m_coverage_output.state = State::FAILURE;
                break;
            }

            // After undocking, try to move away from the dock a little bit
            auto drive_config = DriveStraightBehavior::Config();
            drive_config.max_distance = 0.25;
            drive_config.min_distance = 0.25;
            this->goto_drive_straight(drive_config);
            break;
        }
    }
}

double CoverageStateMachine::compute_evade_rotation(const geometry_msgs::msg::Pose& pose, double resolution)
{
    tf2::Quaternion current_orientation;
    tf2::convert(pose.orientation, current_orientation);

    // Add current orientation to the list of failed attempts
    double current_yaw = tf2::getYaw(current_orientation);
    m_evade_attempts.push_back(current_yaw);

    tf2::Quaternion target_orientation;
    size_t i = 0;
    // We don't want this loop to search forever.
    // Eventually, if we failed too many times, return an orientation regardless of how different it is
    // from previous attempts.
    while (i < 100) {
        // Generate a new, random, target orientation
        double random_num = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        double random_angle = random_num * 2 * M_PI - M_PI;
        target_orientation.setRPY(0.0, 0.0, random_angle);

        // Check if the random orientation is different enough from past evade attempts
        bool valid_target = true;
        for (double angle : m_evade_attempts) {
            tf2::Quaternion attempt_orientation;
            attempt_orientation.setRPY(0.0, 0.0, angle);

            tf2::Quaternion relative_orientation = target_orientation * attempt_orientation.inverse();
            double relative_yaw = tf2::getYaw(relative_orientation);
            if (std::abs(relative_yaw) < std::abs(resolution)) {
                valid_target = false;
                break;
            }
        }

        // Exit as soon as we find a valid target orientation
        if (valid_target) {
            break;
        }
        i++;
    }

    tf2::Quaternion relative_orientation = target_orientation * current_orientation.inverse();
    double relative_yaw_rotation = tf2::getYaw(relative_orientation);
    return relative_yaw_rotation;
}

void CoverageStateMachine::goto_dock()
{
    m_current_behavior = std::make_unique<DockBehavior>(m_dock_action_client, m_logger);
    m_coverage_output.state = State::RUNNING;
}

void CoverageStateMachine::goto_drive_straight(const DriveStraightBehavior::Config& config)
{
    m_current_behavior = std::make_shared<DriveStraightBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_coverage_output.state = State::RUNNING;
}

void CoverageStateMachine::goto_rotate(const RotateBehavior::Config& config)
{
    m_current_behavior = std::make_shared<RotateBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_coverage_output.state = State::RUNNING;
}

void CoverageStateMachine::goto_spiral(const SpiralBehavior::Config& config)
{
    m_last_spiral_time = m_clock->now();
    m_current_behavior = std::make_shared<SpiralBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_coverage_output.state = State::RUNNING;
}

void CoverageStateMachine::goto_undock()
{
    m_undocking = true;
    m_current_behavior = std::make_unique<UndockBehavior>(m_undock_action_client, m_logger);
    m_coverage_output.state = State::RUNNING;
}

} // namespace create3_coverage
