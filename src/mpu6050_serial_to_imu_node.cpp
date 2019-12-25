#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
//#include "tf2_ros/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include <string>
#include <serial/serial.h>

class MPU6050_IMU
{
public:
    MPU6050_IMU(rclcpp::Node::SharedPtr node) : 
        _node(node),
        _rate(200),
        _time_offset_in_seconds(0.0),
        _broadcast_tf(true),
        _linear_acceleration_stddev(0.0),
        _angular_velocity_stddev(0.0),
        _orientation_stddev(0.0),
        _zero_orientation_set(false)
    {
        _imu_pub = _node->create_publisher<sensor_msgs::msg::Imu>("imu/data", 50);

        // ros2 parameter
        _node->declare_parameter("port", "/dev/ttyUSB0");
        _node->get_parameter("port", _port);
        _node->declare_parameter("tf_parent_frame_id", "imu_base");
        _node->get_parameter("tf_parent_frame_id", _tf_parent_frame_id);
        _node->declare_parameter("tf_frame_id", "imu_link");
        _node->get_parameter("tf_frame_id", _tf_frame_id);
        _node->declare_parameter("frame_id", "imu_link");
        _node->get_parameter("frame_id", _frame_id);

    }
    void run()
    {
        sensor_msgs::msg::Imu imu;
        imu.linear_acceleration_covariance[0] = _linear_acceleration_stddev;
        imu.linear_acceleration_covariance[4] = _linear_acceleration_stddev;
        imu.linear_acceleration_covariance[8] = _linear_acceleration_stddev;

        imu.angular_velocity_covariance[0] = _angular_velocity_stddev;
        imu.angular_velocity_covariance[4] = _angular_velocity_stddev;
        imu.angular_velocity_covariance[8] = _angular_velocity_stddev;

        imu.orientation_covariance[0] = _orientation_stddev;
        imu.orientation_covariance[4] = _orientation_stddev;
        imu.orientation_covariance[8] = _orientation_stddev;
        
        tf2::Quaternion zero_orientation;
        std::string input, read;
        while(rclcpp::ok())
        {
            try
            {
                if(_ser.isOpen())
                {
                    if(_ser.available())
                    {
                        read = _ser.read(_ser.available());
                        RCLCPP_DEBUG(_node->get_logger(), "read %i new characters from serial port, adding to %i characters of old input.", (int)read.size(), (int)input.size());
                        input += read;
                        while(input.length() >= 28)
                        {
                            _data_packet_start = input.find("$\x03");
                            if (_data_packet_start != std::string::npos)
                            {
                                if ((input.length() >= _data_packet_start + 28) && (input.compare(_data_packet_start + 26, 2, "\r\n") == 0))
                                {
                                    int16_t w = (((0xff &(char)input[_data_packet_start + 2]) << 8) | 0xff &(char)input[_data_packet_start + 3]);
                                    int16_t x = (((0xff &(char)input[_data_packet_start + 4]) << 8) | 0xff &(char)input[_data_packet_start + 5]);
                                    int16_t y = (((0xff &(char)input[_data_packet_start + 6]) << 8) | 0xff &(char)input[_data_packet_start + 7]);
                                    int16_t z = (((0xff &(char)input[_data_packet_start + 8]) << 8) | 0xff &(char)input[_data_packet_start + 9]);

                                    double wf = w/16384.0;
                                    double xf = x/16384.0;
                                    double yf = y/16384.0;
                                    double zf = z/16384.0;

                                    tf2::Quaternion orientation(xf, yf, zf, wf);

                                    if(!_zero_orientation_set)
                                    {
                                        zero_orientation = orientation;
                                        _zero_orientation_set = true;
                                    }

                                    tf2::Quaternion differential_rotation = zero_orientation.inverse() * orientation;

                                    int16_t gx = (((0xff &(char)input[_data_packet_start + 10]) << 8) | 0xff &(char)input[_data_packet_start + 11]);
                                    int16_t gy = (((0xff &(char)input[_data_packet_start + 12]) << 8) | 0xff &(char)input[_data_packet_start + 13]);
                                    int16_t gz = (((0xff &(char)input[_data_packet_start + 14]) << 8) | 0xff &(char)input[_data_packet_start + 15]);

                                    double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                                    double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
                                    double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

                                    // get acelerometer values
                                    int16_t ax = (((0xff &(char)input[_data_packet_start + 16]) << 8) | 0xff &(char)input[_data_packet_start + 17]);
                                    int16_t ay = (((0xff &(char)input[_data_packet_start + 18]) << 8) | 0xff &(char)input[_data_packet_start + 19]);
                                    int16_t az = (((0xff &(char)input[_data_packet_start + 20]) << 8) | 0xff &(char)input[_data_packet_start + 21]);
                                    // calculate accelerations in m/s²
                                    double axf = ax * (8.0 / 65536.0) * 9.81;
                                    double ayf = ay * (8.0 / 65536.0) * 9.81;
                                    double azf = az * (8.0 / 65536.0) * 9.81;

                                    uint8_t received_message_number = input[_data_packet_start + 25];
                                    if(_received_message)
                                    {
                                        uint8_t message_distance = _last_received_message_number - _last_received_message_number;
                                        if(message_distance > 1)
                                        {
                                            RCLCPP_WARN(_node->get_logger(), "Missed MPU6050 data packets from arduino.");
                                        }
                                    }
                                    else
                                    {
                                        _received_message = true;
                                    }
                                    _last_received_message_number = received_message_number;

                                    rclcpp::Time measurement_time = _node->now() + rclcpp::Duration(_time_offset_in_seconds,0);

                                    imu.header.stamp = measurement_time;
                                    imu.header.frame_id = _frame_id;

                                    imu.orientation.x = differential_rotation.getX();
                                    imu.orientation.y = differential_rotation.getY();
                                    imu.orientation.z = differential_rotation.getZ();
                                    imu.orientation.w = differential_rotation.getW();

                                    imu.angular_velocity.x = gxf;
                                    imu.angular_velocity.y = gyf;
                                    imu.angular_velocity.z = gzf;

                                    imu.linear_acceleration.x = axf;
                                    imu.linear_acceleration.y = ayf;
                                    imu.linear_acceleration.z = azf;

                                    _imu_pub->publish(imu);

                                    input.erase(0, _data_packet_start + 28);
                                }
                                else
                                {
                                    if (input.length() >= _data_packet_start + 28)
                                    {
                                        input.erase(0, _data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                                    }
                                    else
                                    {
                                        // do not delete start character, maybe complete package has not arrived yet
                                        input.erase(0, _data_packet_start);
                                    }
                                }
                            }
                            else
                            {
                                input.clear();
                            }
                        }
                    }
                }
                else
                {
                    try
                    {
                        _ser.setPort(_port);
                        _ser.setBaudrate(115200);
                        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                        _ser.setTimeout(to);
                        _ser.open();
                    }
                    catch(serial::IOException& e)
                    {
                        RCLCPP_ERROR(_node->get_logger(), "Unable to open serial port");
                        rclcpp::Rate(5).sleep();
                    }

                    if(_ser.isOpen())
                    {
                        RCLCPP_DEBUG(_node->get_logger(), "Serial port initialzed and opened.");
                    }
                }
            }
            catch(serial::IOException& e)
            {
                RCLCPP_ERROR(_node->get_logger(), "Error reading from the serial port");
                _ser.close();
            }
            rclcpp::spin_some(_node);
            _rate.sleep();
        }
    }
private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;
    rclcpp::WallRate _rate;

    serial::Serial _ser;
    std::string _port;
    std::string _tf_parent_frame_id;
    std::string _tf_frame_id;
    std::string _frame_id;
    double _time_offset_in_seconds;
    double _broadcast_tf;
    double _linear_acceleration_stddev;
    double _angular_velocity_stddev;
    double _orientation_stddev;
    uint8_t _last_received_message_number;
    bool _received_message;
    int _data_packet_start;

    bool _zero_orientation_set;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mpu6050_imu_node");
    MPU6050_IMU instance(node);
    instance.run();
    rclcpp::shutdown();
    return 0;
}