/* 
需求：动作服务端，需要解析客户端提交的数字，遍历该数字并累加求和，最终结果响应回
     客户端，且请求响应过程中需要生存连续反馈。
分析：
    1.创建动作服务端对象
    2.处理提交的目标值
    3.生成连续反馈
    4.响应最终结果
    5.处理取消请求
流程：
    1.包含头文件
    2.初始化ROS2客户端
    3.自定义节点类：
      3-1.创建动作服务端对象
      3-2.处理提交的目标值(操作对应一个专门的回调函数)
      3-3.处理取消请求(操作对应一个专门的回调函数)
      3-4.生成连续反馈 与 最终响应(操作对应一个专门的回调函数)
    4.调用spin函数，传入自定义类对象指针，
    spin又称回旋函数，当执行到spin时，又会回去执行一次函数，而不是退出，可以理解为for语句
    5.释放资源
*/
 
// 1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;
 
// 3.自定义节点类：
class ProgessActionServer: public rclcpp::Node{
public:
    ProgessActionServer():Node("progess_action_server_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"action服务端创建！");
      // 3-1.创建动作服务端对象
      /*  
        参数原型：
        rclcpp_action::Server<ActionT>::SharedPtr create_server<ActionT, NodeT>
        (NodeT node, 
        const std::string &name, 
        rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
        rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
        rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, 
        const rcl_action_server_options_t &options = rcl_action_server_get_default_options(), 
        rclcpp::CallbackGroup::SharedPtr group = nullptr)
      */
      server_ = rclcpp_action::create_server<Progress>(
        this,
        "get_sum",
        std::bind(&ProgessActionServer::handle_goal,this,_1,_2),
        std::bind(&ProgessActionServer::handle_cancel,this,_1),
        std::bind(&ProgessActionServer::handle_accepted,this,_1)
        );

    }
    /*  
      std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>
    */
    // 3-2.处理提交的目标值(操作对应一个专门的回调函数)
    /*  
      返回值：
        ACCEPT_AND_DEFER: 同意并延迟执行
        ACCEPT_AND_EXECUTE： 同意并执行
        REJECT： 返回拒绝
    */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Progress::Goal> goal){
      (void)uuid;  //暂时没有使用的参数，先申明void
      // 业务逻辑： 判断提交的数字是否大于1,是就接受，否就拒绝
      if(goal->num <= 1){
          RCLCPP_INFO(this->get_logger(),"提交的目标值，必须大于1！");
          return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(this->get_logger(),"提交的目标值合法！");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /*  
      std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>
    */
    // 3-3.处理取消请求(操作对应一个专门的回调函数)
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
      (void)goal_handle;  //暂时没有使用的参数，先申明void
      RCLCPP_INFO(this->get_logger(),"接收到任务取消请求！");
      // ACCEPT：无条件同意
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    /*  
      std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>
    */
    // 3-4.生成连续反馈 与 最终响应(操作对应一个专门的回调函数)
    // 主逻辑
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
        // 1.生成连续反馈，返回给客户端
        // void publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
        // goal_handle->publish_feedback();
        // 首先，获取目标值，然后遍历，遍历中累加，且每循环一次就计算进度，并作为连续反馈
        int num = goal_handle->get_goal()->num;
        int sum = 0;
        auto feedback = std::make_shared<Progress::Feedback>();

        // 设置休眠，方便观看过程; 1.0Hz(每秒输出1次)，2.0Hz(每秒输出2次)
        rclcpp::Rate rate(1.0);

        auto result = std::make_shared<Progress::Result>();

        for (int i = 1; i <= num; i++){
            sum += i;
            double progress = i / (double)num;  //计算进度
            feedback->progress = progress;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(),"连续反馈中，进度: %.2f",progress);   //输出提示

            // 判断是否有取消请求
            // goal_handle->is_canceling()   //是否有取消，返回 true: 是，有取消请求； false: 否，没有取消请求
            // void canceled(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
            // goal_handle->canceled();
            if ( goal_handle->is_canceling())
            {
              // 如果接收到取消请求，则终止程序 ,用 return 返回(用 return 不是特别合适)
              result->sum = sum;
              goal_handle->canceled(result);

              RCLCPP_INFO(this->get_logger(),"客户端取消任务！");   //输出提示
              
              return;
            }
          
            rate.sleep();
        }

        // 2.生成最终响应结果
        // void succeed(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
        // goal_handle->succeed();
        // 用 if 判断，服务端是否还在正常进行中 
        if (rclcpp::ok())
        {
          // 如果正常，就创建一个 Progress_Result 对象
          result->sum = sum;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(),"最终结果：%d",sum);   //输出提示

        }
        

    }
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
      /*
        主逻辑反馈，这是一个耗时操作，如果是一个耗时操作，为避免主逻辑阻塞
        建议新开一个线程，来处理 连续反馈 与 最终响应
      */
      // 新建子线程，处理耗时逻辑事件
      std::thread(std::bind(&ProgessActionServer::execute,this,goal_handle)).detach();
    }


private:
    rclcpp_action::Server<Progress>::SharedPtr server_;
};
 
 
int main(int argc, char ** argv)
{
  // 2.初始化ROS2客户端
  rclcpp::init(argc,argv);
 
  // 4.调用spin函数，传入自定义类对象指针
  rclcpp::spin(std::make_shared<ProgessActionServer>());
 
  // 5.释放资源
  rclcpp::shutdown();
 
  return 0;
}