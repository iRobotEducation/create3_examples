#include <chrono>
#include <math.h>

#include "create3_coverage/coverage_state_machine.hpp"
#include "create3_coverage/utils.hpp"

namespace create3_coverage {

CoverageStateMachine::CoverageStateMachine(
    create3_examples_msgs::action::Coverage::Goal goal,
    rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger,
    rclcpp_action::Client<DockAction>::SharedPtr dock_action_client,
    rclcpp_action::Client<UndockAction>::SharedPtr undock_action_client,
    rclcpp::Publisher<TwistMsg>::SharedPtr cmd_vel_publisher)
    : m_logger(logger)
{
    m_goal = goal;

    m_clock = clock;
    m_start_time = m_clock->now();

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
    if (m_output.state != State::RUNNING) {
        return m_output;
    }

    m_behavior_state = m_current_behavior->execute(data);
    m_output.current_behavior = m_current_behavior->get_id();

    return m_output;
}

void CoverageStateMachine::cancel()
{
    if (m_current_behavior) {
        m_current_behavior->cleanup();
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
        m_output.state = State::RUNNING;
        return;
    }

    // Check if it's time to wrap up the behavior
    if (m_clock->now() - m_start_time >= m_goal.max_duration) {
        if (data.dock.dock_visible) {
            this->goto_dock();
        } else {
            m_output.state = State::SUCCESS;
        }
        return;
    }

    switch (m_current_behavior->get_id())
    {
        case FeedbackMsg::DOCK:
        {
            // A dock action should indicate the termination of the behavior.
            // Do not set a new behavior, either return SUCCESS or FAILURE.
            if (m_behavior_state == State::FAILURE || !data.dock.is_docked) {
                m_output.state = State::FAILURE;
                break;
            }
            m_output.state = State::SUCCESS;
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
                double random_num = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
                double random_angle = random_num * 2 * M_PI - M_PI;
                rotate_config.target_rotation = random_angle;
            }
            this->goto_rotate(rotate_config);
            break;
        }
        case FeedbackMsg::ROTATE:
        {
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
            this->goto_rotate(RotateBehavior::Config());
            break;
        }
        case FeedbackMsg::UNDOCK:
        {
            if (m_behavior_state == State::FAILURE || data.dock.is_docked) {
                m_output.state = State::FAILURE;
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

void CoverageStateMachine::goto_dock()
{
    m_current_behavior = std::make_unique<DockBehavior>(m_dock_action_client, m_logger);
    m_output.state = State::RUNNING;
}

void CoverageStateMachine::goto_drive_straight(DriveStraightBehavior::Config config)
{
    m_current_behavior = std::make_shared<DriveStraightBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_output.state = State::RUNNING;
}

void CoverageStateMachine::goto_rotate(RotateBehavior::Config config)
{
    m_current_behavior = std::make_shared<RotateBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_output.state = State::RUNNING;
}

void CoverageStateMachine::goto_spiral(SpiralBehavior::Config config)
{
    m_last_spiral_time = m_clock->now();
    m_current_behavior = std::make_shared<SpiralBehavior>(config, m_cmd_vel_publisher, m_logger, m_clock);
    m_output.state = State::RUNNING;
}

void CoverageStateMachine::goto_undock()
{
    m_undocking = true;
    m_current_behavior = std::make_unique<UndockBehavior>(m_undock_action_client, m_logger);
    m_output.state = State::RUNNING;
}

} // namespace create3_coverage
