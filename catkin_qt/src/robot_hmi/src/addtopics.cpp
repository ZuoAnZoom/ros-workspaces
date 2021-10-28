#include "../include/robot_hmi/addtopics.h"
#include "ui_addtopics.h"

addtopics::addtopics(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::addtopics)
{
    ui->setupUi(this);
//    initUi();
}

addtopics::~addtopics()
{
    delete ui;
}
