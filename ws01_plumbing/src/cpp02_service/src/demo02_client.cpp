/* 
需求：创建客户端，组织数据并提交，然后处理响应结果(需要关注业务流程)
流程：
    前提：main函数中需要判断提交的参数是否正确
    1.包含头文件
    2.初始化ROS2客户端
    3.自定义节点类：
      3-1.创建客户端
      3-2.连接服务器（对于服务通信而言，如果客户端连接不到服务器，那么不能发送请求）
      3-3.发送请求
    4.创建对象指针
      需要调用连接服务器函数，根据连接结果作下一步处理
      连接服务器后，调用请求发送函数
      再处理响应结果
    5.释放资源
*/
 
// 1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

// 简化调用
using base_interfaces_demo::srv::AddInts;

using namespace std::chrono_literals;

 
// 3.自定义节点类：
class AddIntsClient: public rclcpp::Node{
public:
    AddIntsClient():Node("add_ints_client_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"客户端节点创建！");
        // 3-1.创建客户端
        /* 
            模板：<AddInts> 服务接口
            参数："add_ints"服务话题名称
            返回值：client_ 服务对象指针
         */
        client_ = this->create_client<AddInts>("add_ints");
    }
    // 3-2.连接服务器（对于服务通信而言，如果客户端连接不到服务器，那么不能发送请求）
    /* 
        连接服务器，连接成功返回 true， 否则返回 false；
        此函数不需要参数
     */
    bool connect_server(){
    
        // 在指定超时时间内连接服务器，如果连接上了，那么返回 true ，否则返回 false
        // 等待连接，方式一：
        //client_->wait_for_service(1s);

        // 等待连接，方式二：
        // 循环以 2s 为超时时间连接服务器，直到连接到服务器才退出循环 
        while (!client_->wait_for_service(2s))  
        {
            // 对 Ctrl + C 这个操作特殊处理
            // 1.怎么判断 Ctrl+C 按下? 
            // 2.如何处理？
            // 按下 ctrl+c 是要结束 ROS2 程序，意味着要释放资源，比如：关闭 context
            if(!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"键 Ctrl+C 按下，强行终止客户端！");
                return false;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接中...");
        }
        
        return true;
    }
    // 3-3.发送请求
    // 编写发送请求函数 ---参数是两个整形数据，返回值是提交请求户服务端的返回结果
    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1, int num2){
      // 组织请求数据

      // 发送
      /* 
          rclcpp::Client<base_interfaces_demo::srv::AddInts>::FutureAndRequestId
            async_send_request(std::shared_ptr<base_interfaces_demo::srv::AddInts_Request> request)
          // AddInts_Request 相当于 AddInts::Request
       */
      auto request = std::make_shared<AddInts::Request>();
      request->num1 = num1;
      request->num2 = num2;
      return client_->async_send_request(request);
    }

private:
    rclcpp::Client<AddInts>::SharedPtr client_;
};
 
 
int main(int argc, char ** argv)
{
  //判断参数个数 
  // 第一个参数，是当前可执行程序文件名(demo02_client)
  // 第二第三个参数，是输入的计算数字值
  // 参数多或少，都会报错
  if(argc != 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"请提交两个整形数字！");
    return -1;
  }

  // 2.初始化ROS2客户端
  rclcpp::init(argc,argv);
 
  // (此客户端不需要挂起等待，执行完退出即可，所以注释掉)4.调用spin函数，传入自定义类对象指针
  //rclcpp::spin(std::make_shared<AddIntsClient>());
  auto client = std::make_shared<AddIntsClient>();
  //调用客户端对象的连接服务器功能
  bool flag = client->connect_server();
  //根据连接结果作进一步处理,如果flag 为 true 则不会执行 if 里内容
  if (!flag)
  {
    /* 
       rclcpp::get_logger("rclcpp") 创建 logger 对象不依赖余 context
     */
    //为什么不用 client->get_logger() ?
    // 参看《2.3.3_服务通信_C++实现_03客户端实现_04连接服务BUG说明》05:06
    // 
    // rclcpp::get_logger("rclcpp") 是 ROS2 框架里提供的函数
    // client->get_logger() 是本程序提供的函数
    // 所以，当程序执行过程中，被按下 ctrl+c 时，程序退出，client->get_logger()调用
    // 失败就会抛出异常，但是 rclcpp::get_logger("rclcpp") 是框架里的函数，就不会抛出异常
    // RCLCPP_ERROR(client->get_logger(),"服务器连接失败，程序退出!");
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"服务器连接失败，程序退出!");
    return 0;
  }
  // 执行后续。。。
  // 调用请求提交函数，接受并处理响应结果
  // 传入参数方式一，写成固定的
  //auto future = client->send_request(10,20);  
  // 传入参数方式二，接受输入参数
  auto future = client->send_request(atoi(argv[1]),atoi(argv[2]));
  if (rclcpp::spin_until_future_complete(client,future) == rclcpp::FutureReturnCode::SUCCESS) {  
    // 成功
    RCLCPP_INFO(client->get_logger(),"响应成功！sum = %d", future.get()->sum);
  } else {
    // 失败
    RCLCPP_INFO(client->get_logger(),"响应失败！");
  }

  // 5.释放资源
  rclcpp::shutdown();
 
  return 0;
}