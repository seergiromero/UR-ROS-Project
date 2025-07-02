#include "rclcpp/rclcpp.hpp"
#include "robotiq_85_gripper_server/srv/gripper_order.hpp"
#include "robotiq_85_gripper_server/srv/gripper_open.hpp"
#include "robotiq_85_gripper_server/srv/gripper_close.hpp"
#include "robotiq_85_msgs/msg/gripper_cmd.hpp"

class GripperServerNode : public rclcpp::Node{
    public:
        GripperServerNode():Node("gripper_server"){
            // Publisher of data
            gripper_cmd_publisher_ = create_publisher<robotiq_85_msgs::msg::GripperCmd>("gripper/cmd", 10);

            // Service to control state of the gripper
            GripperOrder_service_ = create_service<robotiq_85_gripper_server::srv::GripperOrder>(
            "GripperOrder",
            std::bind(&GripperServerNode::handle_GripperOrder, this, std::placeholders::_1, std::placeholders::_2));

            // Service to fully open
            GripperOpen_service_ = create_service<robotiq_85_gripper_server::srv::GripperOpen>(
            "GripperOpen",
            std::bind(&GripperServerNode::handle_Open, this, std::placeholders::_1, std::placeholders::_2));

            // Service to fullt close the gripper
            GripperClose_service_ = create_service<robotiq_85_gripper_server::srv::GripperClose>(
            "GripperClose",
            std::bind(&GripperServerNode::handle_Close, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(this->get_logger(), "Gripper server ready");

        }

    private:
        void handle_GripperOrder(
            const std::shared_ptr<robotiq_85_gripper_server::srv::GripperOrder::Request> request,
            std::shared_ptr<robotiq_85_gripper_server::srv::GripperOrder::Response> response){
                
                double final_speed = (0.15 - 0.02)/100 * request->speed;
                double final_force = (235 - 20)/100 * request->force;
                double final_position = 0.085/100 * request->position;

                RCLCPP_INFO(this->get_logger(), "New gripper order:");
                RCLCPP_INFO(this->get_logger(), "       Position: %f", final_position);
                RCLCPP_INFO(this->get_logger(), "       Speed: %f", final_speed);
                RCLCPP_INFO(this->get_logger(), "       Force: %f", final_force);

                pub(final_speed, final_force, final_position);
                
                response->status = true;
        }

        void handle_Open(const std::shared_ptr<robotiq_85_gripper_server::srv::GripperOpen::Request> request,
            std::shared_ptr<robotiq_85_gripper_server::srv::GripperOpen::Response> response){
                
                if(request->order == true){
                    RCLCPP_INFO(this->get_logger(), "Openning (real) gripper");
                    pub(0.15, 20, 0.085);
                    response->status = true;
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Order is not valid");
                    response->status = false;
                }
        }

        void handle_Close(const std::shared_ptr<robotiq_85_gripper_server::srv::GripperClose::Request> request,
            std::shared_ptr<robotiq_85_gripper_server::srv::GripperClose::Response> response){
                
                if(request->order == true){
                    RCLCPP_INFO(this->get_logger(), "Closing (real) gripper");
                    pub(0.15, 235, 0);
                    response->status = true;
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Order is not valid");
                    response->status = false;
                }
        }

        void pub(double final_speed, double final_force, double final_position){

            gripper_msg.emergency_release = false;
            gripper_msg.emergency_release_dir = 0;
            gripper_msg.stop = false;
            gripper_msg.speed = final_speed;
            gripper_msg.force = final_force;
            gripper_msg.position = final_position;

            gripper_cmd_publisher_->publish(gripper_msg);
        }

        rclcpp::Publisher<robotiq_85_msgs::msg::GripperCmd>::SharedPtr gripper_cmd_publisher_;
        rclcpp::Service<robotiq_85_gripper_server::srv::GripperOrder>::SharedPtr GripperOrder_service_;
        rclcpp::Service<robotiq_85_gripper_server::srv::GripperOpen>::SharedPtr GripperOpen_service_;
        rclcpp::Service<robotiq_85_gripper_server::srv::GripperClose>::SharedPtr GripperClose_service_;
        robotiq_85_msgs::msg::GripperCmd gripper_msg;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperServerNode>());
  rclcpp::shutdown();
  return 0;
}
