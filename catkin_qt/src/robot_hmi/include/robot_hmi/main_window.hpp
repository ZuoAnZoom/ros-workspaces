/**
 * @file /include/robot_hmi/main_window.hpp
 *
 * @brief Qt based gui for robot_hmi.
 *
 * @date November 2010
 **/
#ifndef robot_hmi_MAIN_WINDOW_H
#define robot_hmi_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "CCtrlDashBoard.h"  //让仪表盘显示__add_by_csy_20-12-19
#include <QMutex> //modi__add_by_csy_20-12-20_
#include <QProcess>//用来终端运行命令__add_by_csy_20-12-23
#include <QComboBox>

#include "qrviz.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robot_hmi {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    //声明槽函数__add_by_csy_20-12-18
    void slot_linear_value_change(int value);
    void slot_raw_value_change(int value);
    void slot_pushbtn_click();
    void slot_update_dashboard(float x, float y);//声明槽函数 __add_by_csy_20-12-19
    void slot_update_power(float value); //声明更新电量显示的槽函数 __add_by_csy_20-12-20

    void slot_update_image(QImage);// 声明 更新图像用的槽函数 __add_by_csy_20-12-20
    void slot_sub_image(); //声明 订阅图像 用的槽函数 __add_by_csy_20-12-20

    void slot_quick_cmd_clicked();//声明 终端命令 用的槽函数__add_by_csy_20-12-23
    void slot_quick_output();

    void slot_treewidget_value_change(QString);


private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    CCtrlDashBoard* speed_x_dashBoard; //声明两个私有的控件
    CCtrlDashBoard* speed_y_dashBoard; //__add_by_csy_20-12-19

    QImage qimage_;//modi__add_by_csy_20-12-20
    mutable QMutex qimage_mutex_;//modi__add_by_csy_20-12-20

    QProcess * laser_cmd;

    qrviz* myrviz; //声明一个 qrviz 类的对象
    QComboBox* fixed_box;
};

}  // namespace robot_hmi

#endif // robot_hmi_MAIN_WINDOW_H
