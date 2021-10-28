/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "../include/robot_hmi/qnode.hpp"
#include <map>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_hmi {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {                              //使用本地的环境变量连接的参数
    ros::init(init_argc,init_argv,"robot_hmi");
    if ( ! ros::master::check() ) {               //判断 master 是否连接成功
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);  //创建一个发布者，在这里定义，在头文件中进行的声明
    chatter_sub = n.subscribe("/chatter",1000, &QNode::chatter_callback, this);//第一个参数是话题名称，第二个是队列大小，第三个是回调函数__add_by_csy_20-12-18
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);  //创建一个速度的发布者，话题名为 cmd_vel, 队列大小为 1000
    //订阅的话题名称（我暂时没有这个话题，可以写一个发布)，队列大小，回调函数，当前对象 __add_by_csy_20-12-19
    odom_sub = n.subscribe("raw_odom", 1000, &QNode::odom_callback, this);
    //初始化电量的订阅，话题名称为 power(我暂时没有这个话题，可以写一个发布) __add_by_csy_20-12-20
    power_sub = n.subscribe("power", 1000, &QNode::power_callback, this);
    start();  //!!这是 Qt 里的知识点，这是一个多线程
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) { //使用自己设置的 ip 地址和 master 地址进行连接
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"robot_hmi");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    chatter_sub = n.subscribe("/chatter",1000,&QNode::chatter_callback, this);//第一个参数是话题名称，第二个是队列大小，第三个是回调函数__add_by_csy_20-12-18
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);  //创建一个速度的发布者，话题名为 cmd_vel, 队列大小为 1000
    //订阅的话题名称（我暂时没有这个话题，可以写一个发布)，队列大小，回调函数，当前对象 __add_by_csy_20-12-19
    odom_sub = n.subscribe("raw_odom", 1000, &QNode::odom_callback, this);
    //初始化电量的订阅，话题名称为 power(我暂时没有这个话题，可以写一个发布) __add_by_csy_20-12-20
    power_sub = n.subscribe("power", 1000, &QNode::power_callback, this);
    start();  //!!这是 Qt 里的知识点，这是一个多线程，相当于调用下面的 QNode::run 函数，开启这个线程
	return true;
}


// 定义订阅图像数据的 接口函数__add_by_csy_20-12-20
void QNode::sub_image(QString topic_name)
{
    ros::NodeHandle n; //创建一个 ROS 的处理句柄
    image_transport::ImageTransport it_(n); //新建一个图像话题的处理句柄，需要通过 n 初始化
    image_sub = it_.subscribe(topic_name.toStdString(), 1000, &QNode::image_callback, this);
    ros::spinOnce();//modi__add_by_csy_20-12-21
}
// 定义订阅图像数据的 回调函数 在这里面发送一个自定义的信号到 MainWindow ，从而让 MainWindow 显示图像__add_by_csy_20-12-20
// 但是要先对数据进行转换，转换为 opencv 的一个数据类型
void QNode::image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding); //消息，图像消息的编码格式
    // 将 opencv 的 mat 数据类型，转换为 Qt label 支持的 pixmap 显示类型
    QImage im = Mat2QImage(cv_ptr->image); // 对 cv_ptr 中的图像数据格式转换__add_by_csy_20-12-20
    emit image_vel(im);
}

// 将 opencv 的 mat 数据类型，转换为 Qt label 支持的 pixmap 显示类型的函数
QImage QNode::Mat2QImage(cv::Mat const& src)
{
    QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
    const float scale = 255.0;
    if (src.depth() == CV_8U) {
        if (src.channels() == 1) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    int level = src.at<quint8>(i, j);
                    dest.setPixel(j, i, qRgb(level, level, level));
                }
            }
        } else if (src.channels() == 3) {
            for (int i = 0; i < src.rows; ++i) {
                for (int j = 0; j < src.cols; ++j) {
                    cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
                    dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
                }
            }
        }
        } else if (src.depth() == CV_32F) {
            if (src.channels() == 1) {
                for (int i = 0; i < src.rows; ++i) {
                    for (int j = 0; j < src.cols; ++j) {
                        int level = scale * src.at<float>(i, j);
                        dest.setPixel(j, i, qRgb(level, level, level));
                    }
                }
            } else if (src.channels() == 3) {
                for (int i = 0; i < src.rows; ++i) {
                    for (int j = 0; j < src.cols; ++j) {
                        cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
                        dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
                    }
                }
            }
        }
    return dest;
}


// 订阅电量的回调函数 __add_by_csy_20-12-20，要在回调函数里，通过自定义信号把电量发送给 ui，即 MainWindow 类
void QNode::power_callback(const std_msgs::Float32 &msg)
{
    emit power_vel(msg.data); //直接发送信号
}


/*  订阅速度的回调函数，用来把仪表盘的指针更新位置 __add_by_csy_20-12-19
*  ！注意：因为 QNode 和 MainWindow 是两个类，ui 界面要在 MainWindow 类中进行访问
*  ，所以要创建一个自定义信号（在Q_SIGNALS里），把当前x轴和y轴的线速度通过这个信号发送法到 Mainwindow 这个类中
*/
void QNode::odom_callback(const nav_msgs::Odometry &msg)
{
    emit speed_vel(msg.twist.twist.linear.x, msg.twist.twist.linear.y); // 发送信号出去,然后要在 MainWindow 中链接这个信号
}


// 回调函数__add_by_csy_20-12-18
void QNode::chatter_callback(const std_msgs::String &msg)
{
    log(Info,"I receive "+msg.data);
}

// 可以理解为 QNode 的一个接口函数，在 MainWindow 中进行调用就可以
void QNode::set_cmd_vel(char k, float linear, float angular)
{
    // Map for movement keys
    std::map<char, std::vector<float>> moveBindings //从teleop_twist_keyboard_cpp中复制来的
    {
      {'i', {1, 0, 0, 0}},
      {'o', {1, 0, 0, -1}},
      {'j', {0, 0, 0, 1}},
      {'l', {0, 0, 0, -1}},
      {'u', {1, 0, 0, 1}},
      {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}},
      {'m', {-1, 0, 0, -1}},
      {'O', {1, -1, 0, 0}},
      {'I', {1, 0, 0, 0}},
      {'J', {0, 1, 0, 0}},
      {'L', {0, -1, 0, 0}},
      {'U', {1, 1, 0, 0}},
      {'<', {-1, 0, 0, 0}},
      {'>', {-1, -1, 0, 0}},
      {'M', {-1, 1, 0, 0}},
      {'t', {0, 0, 1, 0}},
      {'b', {0, 0, -1, 0}},
      {'k', {0, 0, 0, 0}},
      {'K', {0, 0, 0, 0}}
    };
    char key = k;
    // Grab the direction data
    int x = moveBindings[key][0];
    int y = moveBindings[key][1];
    int z = moveBindings[key][2];
    int th = moveBindings[key][3];

    geometry_msgs::Twist twist; //创建这个消息
    twist.linear.x = x*linear;
    twist.linear.y = y*linear;
    twist.linear.z = z*linear;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th*angular;

    cmd_vel_pub.publish(twist); //发布这个消息

}

void QNode::run() {              //在 run 函数中实现一些耗时的功能，比如while循环，在这里写都不会影响主线程的界面
    ros::Rate loop_rate(40);
//	int count = 0;
	while ( ros::ok() ) {

//		std_msgs::String msg;
//		std::stringstream ss;
//		ss << "hello world " << count;
//		msg.data = ss.str();
//		chatter_publisher.publish(msg);
//		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
//		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace robot_hmi
