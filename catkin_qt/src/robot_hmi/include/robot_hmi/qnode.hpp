/**
 * @file /include/robot_hmi/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robot_hmi_QNODE_HPP_
#define robot_hmi_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/String.h>   //__add_by_csy_20-12-18
#include <geometry_msgs/Twist.h> //__add_by_csy_20-12-18
#include <map>
#include <nav_msgs/Odometry.h> //__add_by_csy_20-12-19
#include <std_msgs/Float32.h> //__add_by_csy_20-12-20
// 用于显示图像需要包含的头文件 __add_by_csy_20-12-20
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <QImage> //用于图像数据编码格式转换的__add_by_csy_20-12-20

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_hmi {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
    // 因为在两个类之间，所以要在 qnode 中创建一个共有的方法，可以让 MainWindow 类通过 qNode 这个对象去发布话题
    void set_cmd_vel(char k, float linear, float angular);  //__add_by_csy_20-12-18
    // 创建一个借口函数，因为是点击按钮后，订阅图像数据 __add_by_csy_20-12-20
    void sub_image(QString topic_name); //传入数据是话题的名称
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void speed_vel(float, float); //自定义的信号，用来对应发送 x，y轴的线速度 __add_by_csy_20-12-19
    void power_vel(float); //自定义信号，用来传 power __add_by_csy_20-12-20
    void image_vel(QImage); // 自定义信号，用来传图像 __add_by_csy_20-12-20

private:
	int init_argc;
	char** init_argv;
    ros::Publisher chatter_publisher;  // 声明了一个发布者
    ros::Publisher cmd_vel_pub;    // 声明一个速度指令的发布者，然后在 init 函数中初始化
    QStringListModel logging_model;
    ros::Subscriber chatter_sub; // 声明一个订阅者 __add_by_csy_20-12-18
    ros::Subscriber odom_sub;  // 声明速度的订阅者 __add_by_csy_20-12-19
    ros::Subscriber power_sub; // 声明电量的订阅者 __add_by_csy_20-12-20
    image_transport::Subscriber image_sub; // 创建一个图像数据的订阅者 __add_by_csy_20-12-20
    void chatter_callback(const std_msgs::String &msg); // __add_by_csy_20-12-18
    void odom_callback(const nav_msgs::Odometry &msg); // 订阅速度的回调函数 __add_by_csy_20-12-19
    void power_callback(const std_msgs::Float32 &msg); // 订阅电量的回调函数 __add_by_csy_20-12-20
    void image_callback(const sensor_msgs::ImageConstPtr &msg); //订阅图像数据的回调函数__add_by_csy_20-12-20

    QImage Mat2QImage(cv::Mat const& src);//用于转换图像数据编码格式__add_by_csy_20-12-20

};

}  // namespace robot_hmi

#endif /* robot_hmi_QNODE_HPP_ */
