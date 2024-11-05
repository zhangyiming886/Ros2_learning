/* 
需求：创建参数服务端并操作数据
流程：
    1.包含头文件
    2.初始化ROS2客户端
    3.自定义节点类：
      3-1.增
      3-2.查
      3-3.改
      3-4.删
    4.调用spin函数，传入自定义类对象指针，
    spin又称回旋函数，当执行到spin时，又会回去执行一次函数，而不是退出，可以理解为for语句
    5.释放资源
*/
 
// 1.包含头文件
#include "rclcpp/rclcpp.hpp"
 
// 3.自定义节点类：
class ParamServer: public rclcpp::Node{
public:
    // ROS2中，如果允许删除参数，那么就需要 NodeOptions 声明 true
    ParamServer():Node("param_server_node_cpp", rclcpp::NodeOptions().allow_undeclared_parameters(true)){
        RCLCPP_INFO(this->get_logger(),"参数服务端启动...");
    }
    // 3-1.增
    void declare_param(){
        RCLCPP_INFO(this->get_logger(),"------------ 增 ------------");
        this->declare_parameter("car_name","tiger");
        this->declare_parameter("width",1.55);
        this->declare_parameter("wheels",5);    //车轮数量 5

        // set_parameter 也可以用于，新增参数
        /*  
            set_parameter 和 declare_parameter 使用有区别，
            使用 set_parameter 时，构造函数的参数里， 必须有 
            rclcpp::NodeOptions().allow_undeclared_parameters(true) 这个约束条件,
            否则，编译没问题，运行时就会报错，提示 height 没有定义错误。
        */
        this->set_parameter(rclcpp::Parameter("height",2.00));
    }
    // 3-2.查
    void get_param(){
        RCLCPP_INFO(this->get_logger(),"------------ 查 ------------");
        // 获取制定参数
        // this->get_parameter()
        auto car = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(),"key = %s, value = %s", car.get_name().c_str(), car.as_string().c_str());

        // 获取一些参数
        // this->get_parameters()
        auto params = this->get_parameters({"car_name","width","wheels"});
        for (auto &&param : params)
        {
            RCLCPP_INFO(this->get_logger(),"(%s=%s)",param.get_name().c_str(),param.value_to_string().c_str());
        }

        // 判断是否包含,返回 bool 值，存在1；不存在0
        // this->has_parameter()
        RCLCPP_INFO(this->get_logger(),"是否包含 car_name ? %d", this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(),"是否包含 height ？ %d", this->has_parameter("height"));
    }
    // 3-3.改
    void update_param(){
        RCLCPP_INFO(this->get_logger(),"------------ 改 ------------");
        // 修改操作。
        // set_parameter函数，除修改操作，还可以设置新参数(具体使用，看上面3-1:增部分内容)
        this->set_parameter(rclcpp::Parameter("width",1.75));
        RCLCPP_INFO(this->get_logger(),"width = %.2f", this->get_parameter("width").as_double());

    }
    // 3-4.删
    void del_param(){
        RCLCPP_INFO(this->get_logger(),"------------ 删 ------------");
        /*  
            为了使用安全，ROS2不允许随意删除键值，undeclare_parameter只能用于删除
            set_parameter 设置的键值，而不能删除用 declare_parameter 声明的键值
        */
        // this->undeclare_parameter("car_name");
        // RCLCPP_INFO(this->get_logger(),"删除后还包含 car_name 吗？%d",this->has_parameter("car_name"));

        // 打印删除前的状态，1：存在，0：不存在
        RCLCPP_INFO(this->get_logger(),"删除前还包含 height 吗？%d",this->has_parameter("height"));
        // 可以删除未声明，被设置国的参数
        this->undeclare_parameter("height");
        // 打印删除后的状态，1：存在，0：不存在
        RCLCPP_INFO(this->get_logger(),"删除后还包含 height 吗？%d",this->has_parameter("height"));


    }
};
 
 
int main(int argc, char ** argv)
{
  // 2.初始化ROS2客户端
  rclcpp::init(argc,argv);
 
  // 4.调用spin函数，传入自定义类对象指针
  auto node = std::make_shared<ParamServer>();

  node->declare_param();
  node->get_param();
  node->update_param();
  node->del_param();
  
  rclcpp::spin(node);
 
  // 5.释放资源
  rclcpp::shutdown();
 
  return 0;
}