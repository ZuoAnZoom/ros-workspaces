/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_hmi/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_hmi {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)//首先执行构造函数
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.初始化UI界面
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application链接槽函数

    ReadSettings();//读取配置文件
//    setWindowIcon(QIcon(":/images/icon.png"));//设置window的图标
    setWindowIcon(QIcon("://images/whu_school_icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));//链接qnode,rosShutdown的信号

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    //1. 绑定两个滑动条的值和对应label的值，分别对应线速度滑动条和角速度滑动条__add_by_csy_20-12-18
    connect(ui.horizontalSlider_linear,SIGNAL(valueChanged(int)),this,SLOT(slot_linear_value_change(int)));
    connect(ui.horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(slot_raw_value_change(int)));
    //2. 绑定几个 pushbutton 的点击事件，这里采用多个pushbutton 链接同一个槽函数 slot_pushbtn_click()
    connect(ui.pushButton_u,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_i,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_o,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_j,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_l,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_m,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_dou,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_dian,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));

    // init ui 在构造函数中对仪表盘的 ui 进行初始化 __add_by_csy_20-12-19
    speed_x_dashBoard = new CCtrlDashBoard(ui.widget_speed_x);
    speed_y_dashBoard = new CCtrlDashBoard(ui.widget_speed_y);
    speed_x_dashBoard->setGeometry(ui.widget_speed_x->rect()); // 获取 widget 的长宽高，这样 dashBoard 的大小就和 widget 一样了__add_by_csy_20-12-19
    speed_y_dashBoard->setGeometry(ui.widget_speed_y->rect());
    speed_x_dashBoard->setValue(0); // 更改指针指向的值，先初始化值为0 __add_by_csy_20-12-19
    speed_y_dashBoard->setValue(0);

    //链接发布的 速度 信号__add_by_csy_20-12-19
    connect(&qnode, SIGNAL(speed_vel(float, float)), this, SLOT(slot_update_dashboard(float, float)));

    // 给 sliderbar 设置默认初始值
    ui.horizontalSlider_linear->setValue(50);
    ui.horizontalSlider_raw->setValue(50);

    //链接 电量 的信号 __add_by_csy_20-12-20
    connect(&qnode, SIGNAL(power_vel(float)), this, SLOT(slot_update_power(float)));

    //链接 图像数据 的信号 和 ui 上按钮的点击事件 __add_by_csy_20-12-20
    connect(&qnode, SIGNAL(image_vel(QImage)), this, SLOT(slot_update_image(QImage)));
    connect(ui.pushButton_sub_image, SIGNAL(clicked()), this, SLOT(slot_sub_image()));

    //链接 终端
    connect(ui.laser_btn, SIGNAL(clicked()), this, SLOT(slot_quick_cmd_clicked()));

    //rviz
//    ui.treeWidget->setWindowTitle("Display");
//    ui.treeWidget->setWindowIcon(QIcon("://images/classes/Displays.svg"));
    //header
    ui.treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");
    ui.treeWidget->setHeaderHidden(true); //默认展开
    //Global options
    QTreeWidgetItem* Global = new QTreeWidgetItem(QStringList()<<"Global Options");
    Global->setIcon(0, QIcon("://images/options.png"));
    ui.treeWidget->addTopLevelItem(Global); //把 Global 添加到顶层
    Global->setExpanded(true);//展开状态要在添加父级别
    //FixFrame
    QTreeWidgetItem* Fix_frame = new QTreeWidgetItem(QStringList()<<"Fixed Frame"); //Fixed Frame
    fixed_box = new QComboBox();
    fixed_box->addItem("map");
    fixed_box->setMaximumWidth(150);
    fixed_box->setEditable(true);
    connect(fixed_box, SIGNAL(currentTextChanged(QString)), this, SLOT(slot_treewidget_value_change(QString))); //当 fixed_box 的文字改变的时候，就调用槽函数
    Global->addChild(Fix_frame); //把 fixed_frame 放在 Global 下面
    ui.treeWidget->setItemWidget(Fix_frame, 1, fixed_box); //给 fix_frame 设置 widget,放第 1 列, 放 fixed_box


}

void MainWindow::slot_treewidget_value_change(QString)
{
    myrviz->set_FixedFrame(fixed_box->currentText());
}



//用于终端运行命令槽函数的实现__add_by_csy_20-12-23
void MainWindow::slot_quick_cmd_clicked()
{
    laser_cmd = new QProcess;
    laser_cmd->start("bash");
    laser_cmd->write(ui.textEdit_laser_cmd->toPlainText().toLocal8Bit()+"\n"); //最后的 \n 表示命令的结束
    connect(laser_cmd, SIGNAL(readyReadStandardError()), this, SLOT(slot_quick_output())); // !!!这个信号是,有错误输出时，会跳入后面的槽函数中
    connect(laser_cmd, SIGNAL(readyReadStandardOutput()),this, SLOT(slot_quick_output())); // !!!这个信号是，有正常输出时，跳入后面的槽函数中
}
void MainWindow::slot_quick_output()
{
    ui.textEdit_quick_output->append("<font color=\"#FF0000\">"+laser_cmd->readAllStandardError()+"</font>");//错误输出，红色
    ui.textEdit_quick_output->append("<font color=\"#FFFFFF\">"+laser_cmd->readAllStandardOutput()+"</font>");//正常输出，白色
}


// 绑定滑动条两个槽函数的实现__add_by_csy_20-12-18
void MainWindow::slot_linear_value_change(int value)
{
    ui.label_linear->setText(QString::number(value));
}
void MainWindow::slot_raw_value_change(int value)
{
    ui.label_raw->setText(QString::number(value));
}

// 绑定 pushbutton 的槽函数的实现
void MainWindow::slot_pushbtn_click()
{
    // !!在槽函数中可以使用 sender()，来获取是 哪个对象 发过来的信号，因为返回是 QObject 类型的，所以要进行类型转换，类型转换的方式如下
    QPushButton* btn = qobject_cast<QPushButton*> (sender());
//    qDebug()<<btn->text(); //通过 qDebug 打印一下 按钮的文字 ，来检查是否上面是否获取成功
    char k = btn->text().toStdString()[0]; //[0]代表第0个字符
    // 检查 checkbox 是否勾选
    bool is_all = ui.checkBox_is_all->isChecked();
    // 获取线速度和角速度
    float linear = ui.label_linear->text().toFloat()*0.01; // 转换为 float 类型，同时要注意单位转换，cm转换到mm
    float angular = ui.label_raw->text().toFloat()*0.01;
    // 判断是哪个字符
    switch (k){
        case 'u':
          qnode.set_cmd_vel(is_all?'U':'u', linear, angular);
          break;
        case 'i':
          qnode.set_cmd_vel(is_all?'I':'i', linear, angular);
          break;
        case 'o':
          qnode.set_cmd_vel(is_all?'O':'o', linear, angular);
          break;
        case 'j':
          qnode.set_cmd_vel(is_all?'J':'j', linear, angular);
          break;
        case 'l':
          qnode.set_cmd_vel(is_all?'L':'l', linear, angular);
          break;
        case 'm':
          qnode.set_cmd_vel(is_all?'M':'m', linear, angular);
          break;
        case ',':
          qnode.set_cmd_vel(is_all?'<':',', linear, angular);
          break;
        case '.':
          qnode.set_cmd_vel(is_all?'>':'.', linear, angular);
          break;
    }
}

// 速度仪表盘槽函数的定义 实现__add_by_csy_20-12-19
void MainWindow::slot_update_dashboard(float x, float y)
{
    speed_x_dashBoard->setValue(abs(x)*100); // 消息是 m 为单位的，转换为 cm，乘 100
    speed_y_dashBoard->setValue(abs(y)*100);
    ui.label_dir_x->setText(x>0?"正向":"反向");
    ui.label_dir_y->setText(x>0?"正向":"反向");
}

// 定义电量显示槽函数 __add_by_csy_20-12-20
void MainWindow::slot_update_power(float value)
{
    ui.label_power_val->setText(QString::number(value).mid(0,5)+"V"); //截取其中的5位
    double n = (value-10.5)/(12.5-10.5);
    int val = n*100;
    ui.progressBar->setValue(val);
}

// 定义和图像有关的两个槽函数 __add_by_csy_20-12-20
void MainWindow::slot_update_image(QImage im)
{
//    ui.label_image->setPixmap(QPixmap::fromImage(im));
//    ui.label_image->resize(ui.label_image->pixmap()->size()); //modi__add_by_csy_20-12-20
    qimage_mutex_.lock();//modi__add_by_csy_20-12-20
    qimage_ = im.copy();//modi__add_by_csy_20-12-20
    ui.label_image->setPixmap(QPixmap::fromImage(qimage_));//modi__add_by_csy_20-12-20
    ui.label_image->resize(ui.label_image->pixmap()->size());//modi__add_by_csy_20-12-20
    qimage_mutex_.unlock();//modi__add_by_csy_20-12-20
}
void MainWindow::slot_sub_image()
{
    qnode.sub_image(ui.lineEdit_image_topic->text());
    qDebug()<<"__qDebugInfo__"<<ui.lineEdit_image_topic->text();
}



MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {           //如果连接失败就会显示一个noMasterMessage，提示没有连接成功
			showNoMasterMessage();
            ui.treeWidget->setEnabled(false);
		} else {
			ui.button_connect->setEnabled(false);
            //连接成功进行 qrviz 的初始化，传入layout_rviz的指针
            ui.treeWidget->setEnabled(true);
            myrviz = new qrviz(ui.layout_rviz);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
            ui.treeWidget->setEnabled(false);
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
            //连接成功进行 qrviz 的初始化，传入layout_rviz的指针
            ui.treeWidget->setEnabled(true);
            myrviz = new qrviz(ui.layout_rviz);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robot_hmi");
    restoreGeometry(settings.value("geometry").toByteArray());//窗口的大小
    restoreState(settings.value("windowState").toByteArray());//窗口的状态
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {  //写入配置文件
    QSettings settings("Qt-Ros Package", "robot_hmi");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace robot_hmi

