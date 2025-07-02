#include <kinenikros2/kinenik_srv_server.h>

// method to handle the client request and give back a response
// the service gets the name and surname and responses with a capitalized full name
bool MyServiceNode::ComposeFullName(const std::shared_ptr<kinenikros2::srv::InverseKinematics::Request> request,
        std::shared_ptr<kinenikros2::srv::InverseKinematics::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "[kinenikros2]: IK request received.");

  //type could be: UR3, UR5, UR10, UR3e, UR5e, UR10e, UR16e
  KinenikUR myrobot(request->type);
  std::vector<JointPos> theta_sol;
  std::vector<double> values(7);

  values[0] = request->pose.position.x;
  values[1] = request->pose.position.y;
  values[2] = request->pose.position.z;
  values[3] = request->pose.orientation.x;
  values[4] = request->pose.orientation.y;
  values[5] = request->pose.orientation.z;
  values[6] = request->pose.orientation.w;

  // INFO
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[kinenikros2]: Incoming EE pose: [x:%f y:%f z:%f qx:%f qy:%f qz:%f qw:%f]", 
                                values[0], values[1], values[2], values[3],  values[4], values[5], values[6]);

  // COmpute Inverse Kinematics
  if (myrobot.solveIK(values[0], values[1], values[2], values[3],  values[4], values[5], values[6], theta_sol))
  {
    // Fill in response
    response->ik_solution.resize(theta_sol.size());
    for(unsigned int i=0; i<theta_sol.size(); i++){
        for(unsigned int j=0; j<6; j++)
            response->ik_solution[i].ik.push_back(theta_sol[i][j]);
    }
    response->status = true;
    
    // INFO
    std::stringstream sstr;
    sstr<<"[kinenikros2]: The UR-IK solution is: " <<std::endl;
    for(unsigned int i=0; i<theta_sol.size(); i++)
    {
        sstr << "[";
        for(int j=0; j<5; j++)
        {
            sstr << response->ik_solution[i].ik[j] <<", ";            
        }
        sstr << response->ik_solution[i].ik[5] << "]" << std::endl;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), sstr.str().c_str());
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[kinenikros2]: The UR-IK has not been computed.");
    response->status = false;
  }
  return response->status;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyServiceNode>("kinenik_server");
  rclcpp::spin(node);   // the service starts to wait and manage requests
  rclcpp::shutdown();
}