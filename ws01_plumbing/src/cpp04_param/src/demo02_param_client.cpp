/* 
需求：创建客户端，查询或修改服务端参数
流程：
    1.包含头文件
    2.初始化ROS2客户端
    3.自定义节点类：
      3-1.创建参数客户端对象
      3-2.连接服务端
      3-3.参数查询
      3-4.修改参数
    // 4.调用spin函数，传入自定义类对象指针（可选）
    // spin又称回旋函数，当执行到spin时，又会回去执行一次函数，而不是退出，可以理解为for语句
    4.创建自定义节点对象，并调用其函数实现（以课程为准，不使用 spin 函数）
    5.释放资源
*/
 
// 1.包含头文件
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
 
// 3.自定义节点类：
class ParamClient: public rclcpp::Node{
public:
    ParamClient():Node("param_client_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"参数客户端启动...");
        // 3-1.创建参数客户端对象
        // 参数1：当前对象所依赖的节点
        // 参数2：远程服务端节点名称
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"param_server_node_cpp");
        /*  
          视频说明: 《2.5.3_参数服务_C++实现_03客户端_01代码框架》  18:30
          问题：服务通信不是通过服务话题关联么？为什么参数客户端时通过参数服务端的节
               点名称 param_server_node_cpp 关联？
          答：
              1.参数服务端启动，底层封装了多个服务通信的服务端
              2.每个服务端话题，都是采用 "/服务端节点名称/XXX"
              3.参数客户端创建后，也会封装多个服务通信的客户端
              4.这些客户端与服务端相呼应，也要使用相同的话题，因此客户端创建时需要使用服务端节点名称。
        */
    }
    // 3-2.连接服务端
    bool connect_server(){
      // 判断是否连接上
      // 表达式，等待1秒检查一次，成功返回true继续执行后面的,失败返回false继续while循环
      while (!param_client_->wait_for_service(1s))
      {
          // 用户是否按下 Ctrl + C 结束
          if (!rclcpp::ok())
          {
              /*  
                为什么不使用 this->get_logger() ? 因为则 this->get_logger() 是
                应用程序级别的调用，程序销毁时，就会调用不到，所以要调用框架级别的函数，
                才能正常输出提示。  

                更详细的说明：看 cpp02_service/demo02_client.cpp  116行           
              */
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Ctrl+C 程序退出...");
              return false;
          }
          RCLCPP_INFO(this->get_logger(),"服务连接中...");
      }
      
      return true;
    }
    // 3-3.参数查询
    void get_param(){
        RCLCPP_INFO(this->get_logger(),"------ 参数查询操作 --------");
        // 获取某个参数
        RCLCPP_INFO(this->get_logger(),"-------- 单个参数 --");
        // std::string car_name 返回值类型和<std::string>相同
        std::string car_name = param_client_->get_parameter<std::string>("car_name");
        double width = param_client_->get_parameter<double>("width");
        RCLCPP_INFO(this->get_logger(),"car_name = %s",car_name.c_str());
        RCLCPP_INFO(this->get_logger(),"width = %.2f",width);

        // 获取多个参数
        RCLCPP_INFO(this->get_logger(),"-------- 多个参数 --");
        auto params = param_client_->get_parameters({"car_name","width","wheels"});
        for (auto &&param : params)
        {
          RCLCPP_INFO(this->get_logger(),"%s = %s",param.get_name().c_str(),param.value_to_string().c_str());
        }
        

        // 判断是否包含某个参数
        RCLCPP_INFO(this->get_logger(),"-------- 包含某个参数 --");
        RCLCPP_INFO(this->get_logger(),"包含car_name吗？%d",param_client_->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(),"包含car_name吗？%d",param_client_->has_parameter("height"));
    }
    // 3-4.修改参数
    void update_param(){
        RCLCPP_INFO(this->get_logger(),"------ 参数修改操作 --------");
        // 可以修改单个参数
        // param_client_->set_parameters(rclcpp::Parameter("car_name","pig"));
        // 也可以同时修改多个参数，用{}花括号括起来，用 ,逗号分割多个参数
        param_client_->set_parameters({
          rclcpp::Parameter("car_name","pig"),
          rclcpp::Parameter("width",3.0),
          /*  
              当修改一个服务端没有的参数length时，如允许操作，需保证服务端构造函数的节点创建时，要设置
              allow_undeclared_parameters(true)，在 demo01_param_server.cpp 文件23行处。
          */
          rclcpp::Parameter("length",5.0)
          });
        RCLCPP_INFO(this->get_logger(),"新设置参数:length = %.2f",param_client_->get_parameter<double>("length"));

    }

private:
    // API,同步参数客户端
    rclcpp::SyncParametersClient::SharedPtr param_client_;
};
 
 
int main(int argc, char ** argv)
{
  // 2.初始化ROS2客户端
  rclcpp::init(argc,argv);
 
  // 4.调用spin函数，传入自定义类对象指针
  // rclcpp::spin(std::make_shared<ParamClient>());

  // 4.不用spin函数
  auto client = std::make_shared<ParamClient>();
  bool flag = client->connect_server();
  // 判断是否链接上
  if(!flag){
    return 0; // 程序终止
  }
  
  // 修改前查询参数
  client->get_param();
  // 修改参数
  client->update_param();
  // 修改后查询参数
  client->get_param();


  // 5.释放资源
  rclcpp::shutdown();
 
  return 0;
}