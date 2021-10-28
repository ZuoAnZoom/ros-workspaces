#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
//ros
#include <ros/ros.h>
//rviz
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>

#include <QVBoxLayout>

class qrviz
{
public:
    qrviz(QVBoxLayout* layout);
    void set_FixedFrame(QString Frame_name);

private:
    rviz::RenderPanel* render_panel_; //声明rviz显示容器
    rviz::VisualizationManager* manager_;
};

#endif // QRVIZ_HPP
