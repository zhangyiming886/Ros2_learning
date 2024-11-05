/* 
需求：编写动作客户端，可以发送一个整形数据到服务端，并处理服务端的连续反馈和最终响应结果
流程：
    前提：可以解析终端，动态传入的参数(一个整数)
    1.包含头文件
    2.初始化ROS2客户端
    3.自定义节点类：
      3-1.创建动作客户端
      3-2.发送请求
      3-3.处理关于目标值的服务端响应(操作对应一个专门的回调函数)
      3-1.处理连续反馈(操作对应一个专门的回调函数)
      3-2.处理最终响应(操作对应一个专门的回调函数)
    4.调用spin函数，传入自定义类对象指针，
    spin又称回旋函数，当执行到spin时，又会回去执行一次函数，而不是退出，可以理解为for语句
    5.释放资源
*/
 
// 1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using base_interfaces_demo::action::Progress;
using namespace std::chrono_literals;

// _1 _2 占位符
using std::placeholders::_1;
using std::placeholders::_2;
 
// 3.自定义节点类：
class ProgessActionClient: public rclcpp::Node{
public:
    ProgessActionClient():Node("progess_action_client_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"action客户端创建！");
        // 3-1.创建动作客户端
        /*  
          rclcpp_action::Client<ActionT>::SharedPtr 
          create_client<ActionT, 
          NodeT>(NodeT node, 
          const std::string &name, 
          rclcpp::CallbackGroup::SharedPtr group = nullptr, 
          const rcl_action_client_options_t &options = rcl_action_client_get_default_options())
        */
        client_ = rclcpp_action::create_client<Progress>(this,"get_sum");
    }
    // 3-2.发送请求
    void send_goal(int num){
      // 1.需要连接到服务端
      if (!client_->wait_for_action_server(10s)){  // 等待服务器10秒，如果超时，就取消
          RCLCPP_ERROR(this->get_logger(),"连接服务器失败！");
          return;
      }
      // 2.发送请求
      /*  
        std::shared_future<rclcpp_action::ClientGoalHandle<base_interfaces_demo::action::Progress>::SharedPtr> 
        async_send_goal(
          const base_interfaces_demo::action::Progress::Goal &goal, 
          const rclcpp_action::Client<base_interfaces_demo::action::Progress>::SendGoalOptions &options)
      */
      auto goal = Progress::Goal();
      goal.num = num;
      rclcpp_action::Client<Progress>::SendGoalOptions options;
      options.goal_response_callback = std::bind(&ProgessActionClient::goal_response_callback,this,_1);
      options.feedback_callback = std::bind(&ProgessActionClient::feedback_callback,this,_1,_2);
      options.result_callback = std::bind(&ProgessActionClient::result_callback,this,_1);
      auto future = client_->async_send_goal(goal,options);
    }
    // 3-3.处理关于目标值的服务端响应(操作对应一个专门的回调函数)
    /*  
      using GoalHandle = ClientGoalHandle<ActionT>;
      using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
    */
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle){
        // 判断 goal_handle 是不是空指针
        if (!goal_handle){
            RCLCPP_ERROR(this->get_logger(),"提交数据非法，服务器返回空指针，目标请求被服务端拒绝！");
        } else {
            RCLCPP_INFO(this->get_logger(),"目标处理中...");
        }
    }

    // 3-1.处理连续反馈(操作对应一个专门的回调函数)
    /*  
      std::function<void (
        typename ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const Feedback>)>;
    */
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle, const std::shared_ptr<const Progress::Feedback> feedback){
      (void)goal_handle;
      double progress = feedback->progress;
      int pro = (int)(progress * 100);  // 将 double 浮点数字，转换成 int 的方式显示，方便观看
      RCLCPP_INFO(this->get_logger(),"当前进度:%d%%", pro);  // 将%d 后加上%号，以 百分比 显示，方便观看

    }

    // 3-2.处理最终响应(操作对应一个专门的回调函数)
    /*  
      std::function<void (const WrappedResult & result)>
    */
    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result){
      // result.code
      // 通过状态码判断最终结果状态
      //    ABORTED: 被强行终止
      //    CANCELED: 被取消
      //    SUCCEEDED: 成功
      //    UNKNOWN: 未知异常
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
          //成功
          RCLCPP_INFO(this->get_logger(),"最终结果: %d", result.result->sum);
      } else if(result.code == rclcpp_action::ResultCode::ABORTED){ 
          // 被终止
          RCLCPP_INFO(this->get_logger(),"被终止");
      } else if(result.code == rclcpp_action::ResultCode::CANCELED){
          RCLCPP_INFO(this->get_logger(),"被取消");
      } else {
          RCLCPP_INFO(this->get_logger(),"未知异常！");
      }
      

    }

    

private:
    rclcpp_action::Client<Progress>::SharedPtr client_;
};
 
 
int main(int argc, char ** argv)
{
  // 判断参数不等于2个
  if (argc != 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"请提交一个整形数据！");
    return -1;
  }
  
  // 2.初始化ROS2客户端
  rclcpp::init(argc,argv);
 
  // 4.调用spin函数，传入自定义类对象指针
  auto node = std::make_shared<ProgessActionClient>();
  node->send_goal(atoi(argv[1]));
  rclcpp::spin(node);
 
  // 5.释放资源
  rclcpp::shutdown();
 
  return 0;
}