from pathlib import Path


PACKAGE = Path(__file__).parents[1]
HEADER = PACKAGE / "include" / "agibot_x2_ros2_control" / "x2_system_hardware.hpp"
SOURCE = PACKAGE / "src" / "x2_system_hardware.cpp"


def test_state_streams_use_independent_latest_sample_callback_groups():
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert "MultiThreadedExecutor" in header
    assert "std::array<rclcpp::CallbackGroup::SharedPtr, 4>" in header
    assert "std::array<std::mutex, 4> state_mutexes_" in header
    assert "std::mutex data_mutex_" not in header
    assert "rclcpp::SensorDataQoS()" in source
    assert "state_qos.keep_last(1)" in source
    assert "options.callback_group = state_callback_groups_[i]" in source
    assert "state_callback(message, i)" in source
    assert "state_mutexes_[group_index]" in source
    assert "state_group_indices_[index] != group_index" in source
    assert "received on the wrong state topic" in source
    assert "executor_->spin()" in source
    assert "executor_->cancel()" in source
    assert "spin_some()" not in source


def test_initial_zero_command_is_opt_in_and_applied_on_first_arm_claim():
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")

    assert 'initial_arm_command_mode_{"measured"}' in header
    assert "initial_zero_command_pending_" in header
    assert '"initial_arm_command_mode", "measured"' in source
    assert (
        'initial_arm_command_mode_ != "measured" && '
        'initial_arm_command_mode_ != "zero"'
    ) in source
    assert "claimed && initial_zero_command_pending_" in source
    assert "initial_zero_command_pending_ = false;" in source
    assert "commands_[index] = positions_[index];" in source
    assert "add_on_set_parameters_callback" not in source
    assert "initial_zero_trajectory_pending_ = true;" in source
    assert "publish_initial_zero_trajectory();" in source
    assert "kZeroStartupHoldDurationSec" in source
    assert "hold_point.time_from_start.sec = kZeroStartupHoldDurationSec;" in source
    assert '"Published zero startup trajectory to %s"' in source
