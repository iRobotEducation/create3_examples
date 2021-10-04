#include "create3_coverage/coverage_state_machine.hpp"
#include "create3_coverage/create3_coverage_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace create3_coverage {

Create3CoverageNode::Create3CoverageNode()
: rclcpp::Node("create3_coverage")
{
    m_coverage_action_server = rclcpp_action::create_server<CoverageAction>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "coverage",
        std::bind(&Create3CoverageNode::handle_goal, this, _1, _2),
        std::bind(&Create3CoverageNode::handle_cancel, this, _1),
        std::bind(&Create3CoverageNode::handle_accepted, this, _1));

    m_dock_action_client = rclcpp_action::create_client<DockAction>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "dock");

    m_undock_action_client = rclcpp_action::create_client<UndockAction>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "undock");
    
    m_cmd_vel_publisher = this->create_publisher<TwistMsg>("cmd_vel", 10);

    m_reflexes_param_client = std::make_shared<rclcpp::AsyncParametersClient>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_graph_interface(),
        this->get_node_services_interface(),
        "motion_control",
        rmw_qos_profile_parameters);

    m_dock_subscription = this->create_subscription<DockMsg>(
        "dock",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3CoverageNode::dock_callback, this, _1));

    m_hazards_subscription = this->create_subscription<HazardMsg>(
        "hazard_detection",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3CoverageNode::hazards_callback, this, _1));

    m_ir_opcode_subscription = this->create_subscription<OpCodeMsg>(
        "ir_opcode",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3CoverageNode::ir_opcode_callback, this, _1));

    m_odom_subscription = this->create_subscription<OdometryMsg>(
        "odom",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3CoverageNode::odom_callback, this, _1));

    m_kidnap_subscription = this->create_subscription<KidnapMsg>(
        "kidnap_status",
        rclcpp::SensorDataQoS(),
        std::bind(&Create3CoverageNode::kidnap_callback, this, _1));

    m_dock_msgs_received = false;
    m_is_running = false;
    m_last_behavior = -1;
    RCLCPP_INFO(this->get_logger(), "Node created!");
}

rclcpp_action::GoalResponse
Create3CoverageNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const CoverageAction::Goal> goal)
{
    (void)uuid;
    (void)goal;

    bool is_kidnapped = false;
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        is_kidnapped = m_last_kidnap.is_kidnapped;
    }
    if (is_kidnapped) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal request: robot is currently kidnapped");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (m_is_running.exchange(true)) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal request: can only handle 1 goal at the time");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Accepting goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Create3CoverageNode::handle_cancel(
    const std::shared_ptr<GoalHandleCoverage> goal_handle)
{
    (void)goal_handle;

    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Create3CoverageNode::handle_accepted(const std::shared_ptr<GoalHandleCoverage> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Create3CoverageNode::execute, this, _1), goal_handle}.detach();
}

void Create3CoverageNode::execute(const std::shared_ptr<GoalHandleCoverage> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto start_time = this->now();

    // We need some reflexes to be enabled on the robot in order to run this behavior
    bool reflexes_correctly_set = this->reflexes_setup();

    // Abort immediately if we couldn't setup the robot reflexes
    if (!reflexes_correctly_set) {
        RCLCPP_ERROR(this->get_logger(), "Aborting goal! Unable to set reflexes");
        auto result = std::make_shared<CoverageAction::Result>();
        result->success = false;
        result->duration = rclcpp::Duration::from_nanoseconds(0);
        goal_handle->abort(result);
    } else {
        RCLCPP_INFO(this->get_logger(), "Reflexes setup correctly! Starting behavior!");
    }

    auto state_machine = std::make_unique<CoverageStateMachine>(
        *goal,
        this->get_clock(),
        this->get_logger(),
        m_dock_action_client,
        m_undock_action_client,
        m_cmd_vel_publisher);

    CoverageStateMachine::CoverageOutput output;
    output.state = State::RUNNING;
    bool is_docked = false;
    bool is_kidnapped = false;
    do {

        Behavior::Data data;
        {
            std::lock_guard<std::mutex> guard(m_mutex);

            data.hazards = m_last_hazards;
            data.dock = m_last_dock;
            data.pose = m_last_odom.pose.pose;
            data.opcodes = m_last_opcodes;

            is_kidnapped = m_last_kidnap.is_kidnapped;

            m_last_opcodes.clear();
            is_docked = m_last_dock.is_docked;
        }

        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            m_is_running = false;
            state_machine->cancel();
            auto result = std::make_shared<CoverageAction::Result>();
            result->success = false;
            result->is_docked = is_docked;
            result->duration = this->now() - start_time;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled!");
            return;
        }

        // Check if the robot is kidnapped
        if (is_kidnapped) {
            m_is_running = false;
            state_machine->cancel();
            auto result = std::make_shared<CoverageAction::Result>();
            result->success = false;
            result->is_docked = is_docked;
            result->duration = this->now() - start_time;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Aborting goal! Robot has been kidnapped!");
            return;
        }

        if (!m_dock_msgs_received) {
            continue;
        }

        output = state_machine->execute(data);
        if (m_last_behavior != output.current_behavior) {
            auto feedback = std::make_shared<CoverageAction::Feedback>();
            feedback->current_behavior = output.current_behavior;
            goal_handle->publish_feedback(feedback);
            m_last_behavior = output.current_behavior;
        }

        loop_rate.sleep();
    } while (output.state == State::RUNNING && rclcpp::ok());

    RCLCPP_INFO(this->get_logger(), "Coverage action terminated");

    if (rclcpp::ok()) {
        m_is_running = false;
        auto result = std::make_shared<CoverageAction::Result>();
        result->success = (output.state == State::SUCCESS);
        result->is_docked = is_docked;
        result->duration = this->now() - start_time;
        if (result->success) {
            goal_handle->succeed(result);
        } else {
            goal_handle->abort(result);
        }
    }
}

bool Create3CoverageNode::reflexes_setup()
{
    // Before starting an action, we want to make sure that some reflexes are active
    auto future = m_reflexes_param_client->set_parameters(
        {
            rclcpp::Parameter("reflexes.REFLEX_BUMP", true),
            rclcpp::Parameter("reflexes.REFLEX_CLIFF", true),
            rclcpp::Parameter("reflexes.REFLEX_WHEEL_DROP", true),
            rclcpp::Parameter("reflexes_enabled", true)
        });
    
    auto results = future.get();
    bool success = true;
    for (const rcl_interfaces::msg::SetParametersResult& res : results) {
        success = success && res.successful;
    }

    return success;
}

void Create3CoverageNode::dock_callback(DockMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_dock_msgs_received = true;
    m_last_dock = *msg;
}

void Create3CoverageNode::hazards_callback(HazardMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_hazards = *msg;
}

void Create3CoverageNode::ir_opcode_callback(OpCodeMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_opcodes.push_back(*msg);
}

void Create3CoverageNode::kidnap_callback(KidnapMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_kidnap = *msg;
}

void Create3CoverageNode::odom_callback(OdometryMsg::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_last_odom = *msg;
}

} // namespace create3_coverage