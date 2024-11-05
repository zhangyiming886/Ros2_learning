/* 
需求：演示参数 API 的使用
流程：
    1.包含头文件
    2.初始化ROS2客户端
    3.自定义节点类：
      3-1.参数对象创建
      3-2.参数对象的解析（获取键、值、将值转换成字符串）
    4.调用spin函数，传入自定义类对象指针，
    spin又称回旋函数，当执行到spin时，又会回去执行一次函数，而不是退出，可以理解为for语句
    5.释放资源
*/
 
// 1.包含头文件
#include "rclcpp/rclcpp.hpp"
 
// 3.自定义节点类：
class MyParam: public rclcpp::Node{
public:
    MyParam():Node("my_param_node_cpp"){
      RCLCPP_INFO(this->get_logger(),"参数API使用");
      // 3-1.参数对象创建
      rclcpp::Parameter p1("car_name","tiger"); //车名 老虎
      rclcpp::Parameter p2("height",1.68);  //车高度 1.68
      rclcpp::Parameter p3("wheels",4);   //车轮数 4
      RCLCPP_INFO(this->get_logger(),"----");
      // 3-2.参数对象的解析（获取键、值、将值转换成字符串）
      // 解析值
      RCLCPP_INFO(this->get_logger(),"解析值:");
      RCLCPP_INFO(this->get_logger(),"car_name = %s", p1.as_string().c_str());
      RCLCPP_INFO(this->get_logger(),"height = %.2f", p2.as_double());
      RCLCPP_INFO(this->get_logger(),"wheels = %ld", p3.as_int());  //返回int64 所以用%ld 长整形
      RCLCPP_INFO(this->get_logger(),"----");


      // 获取键参数
      RCLCPP_INFO(this->get_logger(),"获取键:");
      RCLCPP_INFO(this->get_logger(),"p1_name = %s", p1.get_name().c_str());
      RCLCPP_INFO(this->get_logger(),"p1_type = %s", p1.get_type_name().c_str());
      RCLCPP_INFO(this->get_logger(),"p2_value2string = %s", p2.value_to_string().c_str());

    }
};
 
 
int main(int argc, char ** argv)
{
  // 2.初始化ROS2客户端
  rclcpp::init(argc,argv);
 
  // 4.调用spin函数，传入自定义类对象指针
  rclcpp::spin(std::make_shared<MyParam>());
 
  // 5.释放资源
  rclcpp::shutdown();
 
  return 0;
}