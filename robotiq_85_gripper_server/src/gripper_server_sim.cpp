#include "rclcpp/rclcpp.hpp"
#include "robotiq_85_gripper_server/srv/gripper_order.hpp"
#include "robotiq_85_gripper_server/srv/gripper_open.hpp"
#include "robotiq_85_gripper_server/srv/gripper_close.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"

using namespace std::chrono_literals;

//FULL OPEN
//ros2 action send_goal /robotiq_85_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 10.0}}"
//FULL CLOSE
//ros2 action send_goal /robotiq_85_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.804, max_effort: 10.0}}"


class GripperServerNode : public rclcpp::Node{
    public:
    
        using GripperAction = control_msgs::action::GripperCommand;
        using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperAction>;

        GripperServerNode():Node("gripper_server"){
            //action client
            this->client_ptr_ = rclcpp_action::create_client<GripperAction>(
            this,
            "/robotiq_85_gripper_controller/gripper_cmd");

            desired_apperture_=0.085;

            //Callback group cb_group_ defined as Reentrant so as to allow the callbacks to be 
            //see https://leanpub.com/averyinformaljourneythroughros2 section 3.3
            this->cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            //Service to control state of the gripper
            GripperOrder_service_ = create_service<robotiq_85_gripper_server::srv::GripperOrder>(
            "GripperOrder",
            std::bind(&GripperServerNode::handle_GripperOrder, this, std::placeholders::_1,    std::placeholders::_2), rmw_qos_profile_services_default, this->cb_group_);

            //Service to fully open
            GripperOpen_service_ = create_service<robotiq_85_gripper_server::srv::GripperOpen>(
            "GripperOpen",
            std::bind(&GripperServerNode::handle_Open, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->cb_group_);

            //Service to fully close the gripper
            GripperClose_service_ = create_service<robotiq_85_gripper_server::srv::GripperClose>(
            "GripperClose",
            std::bind(&GripperServerNode::handle_Close, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->cb_group_);

            RCLCPP_INFO(this->get_logger(), "Gripper server ready");
        }

    private:
        void send_goal()
        {
            using namespace std::placeholders;

            //this->timer_->cancel();

            if (!this->client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = GripperAction::Goal();

            goal_msg.command.position = desired_apperture_;
            goal_msg.command.max_effort = 10.0;
        
            RCLCPP_INFO(this->get_logger(), "Sending goal: Gripper apperture %f degrees", desired_apperture_);

            auto send_goal_options = rclcpp_action::Client<GripperAction>::SendGoalOptions();
            send_goal_options.goal_response_callback =
            std::bind(&GripperServerNode::goal_response_callback, this, _1);
            send_goal_options.feedback_callback =
            std::bind(&GripperServerNode::feedback_callback, this, _1, _2);
            send_goal_options.result_callback =
            std::bind(&GripperServerNode::result_callback, this, _1);

            auto future_goal_handle = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
            
            // Store the future goal handle for later use if needed
            this->goal_handle_future_ = std::move(future_goal_handle);

            this->goal_done_ = false;
        }

        void wait_result()
        {
            RCLCPP_INFO(this->get_logger(), " Waiting for action to complete");
            auto start_time = get_clock()->now();
            rclcpp::Duration timeout = rclcpp::Duration::from_seconds(5.0);
            bool timed_out=false;
            rclcpp::Duration elapsed = get_clock()->now() - start_time;

            while (!this->goal_done_)
            {
                rclcpp::sleep_for(500ms);
                //do something here like writing a log message before checking again
                RCLCPP_INFO(this->get_logger(), "waiting result - elapsed time: %f - timeout %f", elapsed.seconds(), timeout.seconds());
                RCLCPP_INFO(this->get_logger(), "%s", feedback_message_pos_.str().c_str());

                // check if the maximum timeout has been elapsed or not. If so, exit the waiting loop
                elapsed = get_clock()->now() - start_time;
                if ( elapsed.seconds() > timeout.seconds()) {
                            RCLCPP_INFO(this->get_logger(), " Action timed out (5s)");
                            timed_out=true;
                            break ;
                }
            }
            if (timed_out==false){//get the result
                RCLCPP_INFO(this->get_logger(), " Action completed");
                auto my_result = this->goal_handle_future_.get();
                //process result
            }
            else{ //handle failure
                RCLCPP_INFO(this->get_logger(), " Action failure");
            }
        }

        void goal_response_callback(const GoalHandleGripper::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server. Closing client.");
                //rclcpp::shutdown();
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        void feedback_callback(
            GoalHandleGripper::SharedPtr,
            const std::shared_ptr<const GripperAction::Feedback> feedback)
        {
            //float64 position  # The current gripper gap size (in meters)
            //float64 effort    # The current effort exerted (in Newtons)
            //bool stalled      # True iff the gripper is exerting max effort and not moving
            //bool reached_goal # True iff the gripper position has reached the commanded setpoint

            this->feedback_message_pos_.str("");//to start empty
            this->feedback_message_pos_ << "current position: ";
            this->feedback_message_pos_ << feedback->position <<std::endl;
        }

        void result_callback(const GoalHandleGripper::WrappedResult & result)
        {
            //float64 position  # The current gripper gap size (in meters)
            //float64 effort    # The current effort exerted (in Newtons) 
            //bool stalled      # True iff the gripper is exerting max effort and not moving
            //bool reached_goal # True iff the gripper position has reached the commanded setpoint
            
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(),"Result received: Goal SUCCEEDED");  
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was ABORTED");
                    //rclcpp::shutdown();
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was CANCELED");
                    //rclcpp::shutdown();
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code ¿?");
                    //rclcpp::shutdown();
                    break;
            }
            this->goal_done_ = true;
        }

        void handle_GripperOrder(
            const std::shared_ptr<robotiq_85_gripper_server::srv::GripperOrder::Request> request,
            std::shared_ptr<robotiq_85_gripper_server::srv::GripperOrder::Response> response){
                
                desired_apperture_ = request->position;
                RCLCPP_INFO(this->get_logger(), "* desired_apperture_: %f", desired_apperture_);
                RCLCPP_INFO(this->get_logger(), "* closed_apperture_: %f", closed_apperture_);
                RCLCPP_INFO(this->get_logger(), "* opened_apperture_: %f", opened_apperture_);
                if(desired_apperture_>closed_apperture_) desired_apperture_=closed_apperture_;
                else if(desired_apperture_<opened_apperture_) desired_apperture_=opened_apperture_;
                RCLCPP_INFO(this->get_logger(), "New gripper order:");
                RCLCPP_INFO(this->get_logger(), "       Position: %f", desired_apperture_);
                send_goal();  
                wait_result();
                response->status = true;
        }

        void handle_Open(const std::shared_ptr<robotiq_85_gripper_server::srv::GripperOpen::Request> request,
            std::shared_ptr<robotiq_85_gripper_server::srv::GripperOpen::Response> response){
                
                if(request->order == true){
                    desired_apperture_ = opened_apperture_;
                    RCLCPP_INFO(this->get_logger(), "Opening (simulated) gripper: %f", desired_apperture_);
                    send_goal();
                    wait_result();
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
                    desired_apperture_ = closed_apperture_;
                    RCLCPP_INFO(this->get_logger(), "Closing (simulated) gripper: %f", desired_apperture_);
                    send_goal();
                    wait_result();
                    response->status = true;
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Order is not valid");
                    response->status = false;
                }
        }

        rclcpp::Service<robotiq_85_gripper_server::srv::GripperOrder>::SharedPtr GripperOrder_service_;
        rclcpp::Service<robotiq_85_gripper_server::srv::GripperOpen>::SharedPtr GripperOpen_service_;
        rclcpp::Service<robotiq_85_gripper_server::srv::GripperClose>::SharedPtr GripperClose_service_;
        rclcpp_action::Client<GripperAction>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_future<GoalHandleGripper::SharedPtr> goal_handle_future_;
        rclcpp_action::ClientGoalHandle<GripperAction>::SharedPtr goal_handle_;
        rclcpp::CallbackGroup::SharedPtr cb_group_;
        double desired_apperture_;
        double opened_apperture_ = 0.085; //0.0
        double closed_apperture_ = 0.79; //0.804
        bool goal_done_;
        std::stringstream feedback_message_pos_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperServerNode>();

  //Use multithread executor to handle the callback groups
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
