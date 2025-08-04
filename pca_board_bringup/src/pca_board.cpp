extern "C"{
#include "i2c_pca_utils.h"
}
#include <rclcpp/rclcpp.hpp>
#include "pca_board_interfaces/msg/servo_angle_deg.hpp"
#include "pca_board_interfaces/msg/multi_servo_angle_deg.hpp"
#include "pca_board_interfaces/msg/pwm_freq_hz.hpp"
#include "pca_board_interfaces/msg/duty_cycle_percent.hpp"
#include "pca_board_interfaces/srv/servo_state.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace pca_board_interfaces::msg;
using namespace pca_board_interfaces::srv;


const char *ang_min_str[] = {
    "angle_min_0", "angle_min_1", "angle_min_2", "angle_min_3",
    "angle_min_4", "angle_min_5", "angle_min_6", "angle_min_7",
    "angle_min_8", "angle_min_9", "angle_min_10", "angle_min_11",
    "angle_min_12", "angle_min_13", "angle_min_14", "angle_min_15"
};

const char *ang_max_str[] = {
    "angle_max_0", "angle_max_1", "angle_max_2", "angle_max_3",
    "angle_max_4", "angle_max_5", "angle_max_6", "angle_max_7",
    "angle_max_8", "angle_max_9", "angle_max_10", "angle_max_11",
    "angle_max_12", "angle_max_13", "angle_max_14", "angle_max_15"
};

// PCA9685 16 channel PWM board node.
class PcaBoard : public rclcpp::Node
{
    enum {
        max_channels = 16,
        servoset_sub_queue_size = 1,
        servoadd_sub_queue_size = 1,
        multiservo_sub_queue_size = 1,
        pwmset_sub_queue_size = 1,
        dutycycle_sub_queue_size = 1
    };

    int slave_addr;      // board I2C address
    int i2c_adapter_num; // /dev/i2c-<num>
    int i2c_fd;          // file descriptor assigned to I2C connection
    int freq;            // PWM frequency [hz]
    int angle_min[max_channels];
    int angle_max[max_channels];
    int cur_angles[max_channels];

    rclcpp::Subscription<ServoAngleDeg>::SharedPtr servo_angle_set_sub_;
    rclcpp::Subscription<ServoAngleDeg>::SharedPtr servo_angle_add_sub_;
    rclcpp::Subscription<MultiServoAngleDeg>::SharedPtr multi_servo_set_sub_;
    rclcpp::Subscription<PWMFreqHz>::SharedPtr pwm_freq_set_sub_;
    rclcpp::Subscription<DutyCyclePercent>::SharedPtr duty_cycle_set_sub_;
    rclcpp::Service<ServoState>::SharedPtr servo_state_service_;

    void ConnectI2C();
    void SelectSlaveAddr();
    void WriteFreqPWM();
    void SetServoAngle(int channel, int angle_deg);
    void SetDutyCycle(int channel, float duty_cycle);

    void ServoAngleSetCallback(const ServoAngleDeg::SharedPtr msg);
    void ServoAngleAddCallback(const ServoAngleDeg::SharedPtr msg);
    void MultiServoSetCallback(const MultiServoAngleDeg::SharedPtr msg);
    void PwmFreqSetCallback(const PWMFreqHz::SharedPtr msg);
    void DutyCycleSetCallback(const DutyCyclePercent::SharedPtr msg);
    void ServoStateCallback(const ServoState::Request::SharedPtr request,
        const ServoState::Response::SharedPtr response);
public:
    PcaBoard();
    ~PcaBoard(){}
    void WakeUp();
    void Sleep();
    void Restart();
};

void PcaBoard::ConnectI2C()
{
    i2c_fd = connect_i2c(i2c_adapter_num);
    if (i2c_fd < 0){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Couldn't connect to /dev/i2c-%d",
            i2c_adapter_num
        );
    }
}

void PcaBoard::SelectSlaveAddr()
{
    int tmp = select_slave_addr(i2c_fd, slave_addr);
    if (tmp < 0){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Couldn't select slave address %d.",
            slave_addr
        );
    }
}

void PcaBoard::WriteFreqPWM()
{
    int tmp = set_pwm_freq(i2c_fd, freq, in_osc_hz);
    if (tmp < 0){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Couldn't set board PWM frequency to %d hz.",
            freq
        );
    }
}

void PcaBoard::WakeUp()
{
    int tmp = wake_up(i2c_fd);
    if (tmp < 0){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Couldn't set sleep bit to zero."
        );
    }
}

void PcaBoard::Sleep()
{
    int tmp = pca_sleep(i2c_fd);
    if (tmp < 0){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Couldn't set sleep bit to one."
        );
    }
}

void PcaBoard::Restart()
{
    int tmp = restart(i2c_fd);
    if (tmp < 0){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Couldn't restart."
        );
    }
}

void PcaBoard::ServoAngleSetCallback(
    const ServoAngleDeg::SharedPtr msg)
{
    SetServoAngle(msg->channel, msg->angle);
}

void PcaBoard::ServoAngleAddCallback(
    const ServoAngleDeg::SharedPtr msg)
{
    SetServoAngle(msg->channel, cur_angles[msg->channel] + msg->angle);
}

void PcaBoard::MultiServoSetCallback(const MultiServoAngleDeg::SharedPtr msg)
{
    int cnt = 0;
    uint16_t cb = msg->channel_bits;
    while (cb > 0){
        SetServoAngle(cnt, msg->angle[cnt]);
        cnt++;
        cb >>= 1;
    }
}

PcaBoard::PcaBoard() : Node("pca_board")
{
    declare_parameter("slave_addr", 0x40);  // default PCA slave address
    declare_parameter("i2c", 1);  // i2c bus
    declare_parameter("freq", 50);  // required PWM freq for most servos

    slave_addr = get_parameter("slave_addr").as_int();
    i2c_adapter_num = get_parameter("i2c").as_int();
    freq = get_parameter("freq").as_int();

    for (int i = 0; i < max_channels; i++){
        declare_parameter(ang_min_str[i], 0);
        declare_parameter(ang_max_str[i], 180);
        angle_min[i] = get_parameter(ang_min_str[i]).as_int();
        angle_max[i] = get_parameter(ang_max_str[i]).as_int();
    }

    ConnectI2C();
    SelectSlaveAddr();
    WriteFreqPWM();
    WakeUp();

    for (int i = 0; i < max_channels; i++){
        cur_angles[i] = get_servo_angle(i2c_fd, i, freq);
    }

    servo_angle_set_sub_ = create_subscription<ServoAngleDeg>(
        "servo_angle_set", servoset_sub_queue_size,
        std::bind(&PcaBoard::ServoAngleSetCallback, this, _1)
    );
    servo_angle_add_sub_ = create_subscription<ServoAngleDeg>(
        "servo_angle_add", servoadd_sub_queue_size,
        std::bind(&PcaBoard::ServoAngleAddCallback, this, _1)
    );
    multi_servo_set_sub_ = create_subscription<MultiServoAngleDeg>(
        "multi_servo_set", multiservo_sub_queue_size,
        std::bind(&PcaBoard::MultiServoSetCallback, this, _1)
    );
    pwm_freq_set_sub_ = create_subscription<PWMFreqHz>(
        "pwm_freq_set", pwmset_sub_queue_size,
        std::bind(&PcaBoard::PwmFreqSetCallback, this, _1)
    );
    duty_cycle_set_sub_ = create_subscription<DutyCyclePercent>(
        "duty_cycle_set", dutycycle_sub_queue_size,
        std::bind(&PcaBoard::DutyCycleSetCallback, this, _1)
    );
    servo_state_service_ = create_service<ServoState>(
        "servo_state",
        std::bind(&PcaBoard::ServoStateCallback, this, _1, _2)
    );
}

void PcaBoard::SetServoAngle(int channel, int angle_deg)
{
    int tmp;
    if (angle_deg > angle_max[channel]){
        cur_angles[channel] = angle_max[channel];
    } else if (angle_deg < angle_min[channel]){
        cur_angles[channel] = angle_min[channel];
    } else {
        cur_angles[channel] = angle_deg;
    }
    tmp = set_servo_angle(i2c_fd, channel, cur_angles[channel], freq);
    if (tmp < 0){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Couldn't set servo_%d angle to %d.",
            channel,
            angle_deg
        );
    }
}

void PcaBoard::SetDutyCycle(int channel, float duty_cycle)
{
    int tmp;
    if (channel < 0 || channel >= max_channels){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Wrong channel %d.",
            channel
        );
        return;
    }
    if (duty_cycle < 0. || duty_cycle > 100.){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Wrong duty cycle %f%%. Value must be in range [0, 100]%%.",
            duty_cycle
        );
        return;
    }
    tmp = set_duty_cycle(i2c_fd, channel, duty_cycle);
    if (tmp < 0){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Couldn't set channel %d duty cycle to %f.",
            channel,
            duty_cycle
        );
    }
}

void PcaBoard::PwmFreqSetCallback(const PWMFreqHz::SharedPtr msg)
{
    if (msg->freq < 24 || msg->freq > 1526){
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Couldn't set pwm frequency to %d Hz. Value must be in range "
            "[24, 1526] Hz.",
            msg->freq
        );
        return;
    }
    Sleep();
    freq = msg->freq;
    WriteFreqPWM();
    Restart();
}

void PcaBoard::DutyCycleSetCallback(const DutyCyclePercent::SharedPtr msg)
{
    SetDutyCycle(msg->channel, msg->duty_cycle);
}

void PcaBoard::ServoStateCallback(const ServoState::Request::SharedPtr
    request, const ServoState::Response::SharedPtr response)
{
    response->channel = request->channel;
    response->angle = cur_angles[request->channel];
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcaBoard>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
