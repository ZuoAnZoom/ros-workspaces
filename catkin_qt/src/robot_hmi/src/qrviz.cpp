#include "../include/robot_hmi/qrviz.hpp"

#include <QDebug>

qrviz::qrviz(QVBoxLayout* layout)
{
    // 在构造函数中进行初始化 render_panel_
    // 因为 RenderPanel 是 Qwiddget 类型的，所以要在 MainWindow 上的 layout_rviz 上显示这个 render_panel_, 但是它在两个类中（qrviz，Mainwindow），如何让它们链接起来？
    // 可以在 qrviz 构造函数中初始化对象的时候传入一个 layout 的指针，然后就可以通过这个指针添加 Widget。
    // 所以需要在 头文件中 包含 #include <QVBoxLayout>

    //1_创建 rviz_panel
    render_panel_ = new rviz::RenderPanel();
    layout->addWidget(render_panel_); // 利用传进来的layout指针在 添加这个 render_panel_
    //2_创建 rviz 控制对象
    manager_ = new rviz::VisualizationManager(render_panel_);//需要传入render_panel_
    manager_->initialize();//初始化rviz控制对象
    manager_->startUpdate();//开始所有图层的更新
    manager_->removeAllDisplays();//清除所有的图层
    //3_初始化render_panel_
    render_panel_->initialize(manager_->getSceneManager(), manager_); //获取屏幕的控制对象
    ROS_ASSERT(manager_!=NULL);


}

void qrviz::set_FixedFrame(QString Frame_name)
{
    manager_->setFixedFrame(Frame_name);
    qDebug()<<manager_->getFixedFrame();
}
