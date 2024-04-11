#include <qpOASES.hpp>
#include <cmath>
#include <chrono> //시간 기간 및 시간 인스턴트를 나타내고 조작하는 클래스와 함수를 정의
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <vector>

USING_NAMESPACE_QPOASES

class BalanceNode : public rclcpp::Node
{
public:
    BalanceNode() : Node("BalanceNode_publisher")
    {
        // 시간 간격 (s)
        deltaTime = 0.008;
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        foward_wheel_publisher = this->create_publisher<std_msgs::msg::Float64>("/foward", qos_profile);
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", qos_profile,
                                                                   std::bind(&BalanceNode::Subscriber, this, std::placeholders::_1));
        wheel_sub = this->create_subscription<std_msgs::msg::Float64>("/wheel/data", qos_profile,
                                                                      std::bind(&BalanceNode::WheelSubscriber, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(deltaTime * 1000)), std::bind(&BalanceNode::Publisher, this));
        r = 0.06;
        bufferSize = 50;
        buffer.resize(bufferSize);
    }

private:
    void Publisher()
    {
        Calculator();
        auto msg = std_msgs::msg::Float64();
        float crtlFiltered = simpleMovingAverage(crtl);
        msg.data = crtlFiltered;
        foward_wheel_publisher->publish(msg);
    }
    void Subscriber(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double x = msg->orientation.x;
        double y = msg->orientation.y;
        double z = msg->orientation.z;
        double w = msg->orientation.w;

        // Calculating pitch in radians
        double t2 = +2.0 * (w * y - z * x);
        t2 = std::max(-1.0, std::min(+1.0, t2)); // Ensure t2 is within [-1.0, +1.0]
        pitch = std::asin(t2);                   //*180/M_PI;
        // printf("pitch = %e;\n", pitch);
        std::cout << std::fixed;
        std::cout.precision(2);
        // std::cout << "pitch = " << pitch*180/M_PI << std::endl;
        dot_pitch = msg->angular_velocity.y;
    }
    void WheelSubscriber(const std_msgs::msg::Float64::SharedPtr msg)
    {
        wheel_ang = msg->data;
    }
    void Calculator()
    {

        // H: 목적 함수의 2차 항을 정의하는 행렬 (여기서는 단순화된 예제)
        real_t H[1 * 1] = {0.01}; // 1차원 문제

        real_t g[1] = {-pitch * 10 - dot_pitch * 2}; // 가중치 조정된 예제

        // 변수의 하한과 상한
        real_t lb[1] = {-fabs(0.5 * pitch)}; // 최대 토크 제한
        real_t ub[1] = {fabs(0.5 * pitch)};  // 최대 토크 제한
        real_t A[1] = {0.};
        real_t lbA[1] = {0};
        real_t ubA[1] = {0};
        // QProblemB 객체 생성 (1차원 문제)
        QProblem example(1, 1);

        Options options;
        options.printLevel = PL_NONE; // 로그 출력 비활성화
        example.setOptions(options);

        int_t nWSR = 100;
        // QP 문제 초기화 및 해결
        example.init(H, g, A, lb, ub, lbA, ubA, nWSR);
        // 최적의 토크 계산
        real_t xOpt[1];
        example.getPrimalSolution(xOpt);

        // 최적의 토크 출력
        // printf("Optimal torque = %e;\n", xOpt[0]);
        std::cout << std::fixed;
        std::cout.precision(2);

        // std::cout << "getObjVal torque = " << example.getObjVal() << std::endl;
        crtl = xOpt[0] / r * 0.8 * 2;
        dot_x = r * wheel_ang;
        crtl = (crtl * deltaTime) / r;
        std::cout << "Optimal torque = " << xOpt[0] << " crtl velocity = " << crtl << "  pitch = " << pitch << "  dot_x = " << dot_x << std::endl;
    }
    float simpleMovingAverage(float newCrtl)
    {
        buffer[index] = newCrtl;
        index = (index + 1) % bufferSize;
        if (count < bufferSize)
        {
            count++;
        }

        float sum = 0;
        for (int i = 0; i < count; i++)
        {
            sum += buffer[i];
        }
        return sum / count;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr foward_wheel_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_sub;
    double pitch, dot_pitch, dot_x, crtl, deltaTime, wheel_ang,r;
    std::vector<float> buffer; // 동적 배열로 선언
    int bufferSize; // 버퍼의 크기
    int index = 0; // 현재 데이터를 삽입할 배열의 인덱스
    int count = 0; // 배열에 저장된 데이터의 수
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}