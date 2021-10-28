#ifndef ADDTOPICS_H
#define ADDTOPICS_H

#include <QWidget>
#include <QTreeWidgetItem>
#include <QCheckBox>

namespace Ui {
class addtopics;
}

class addtopics : public QWidget
{
    Q_OBJECT

public:
    explicit addtopics(QWidget *parent = 0);
    ~addtopics();

private:
    Ui::addtopics *ui;
};

#endif // ADDTOPICS_H
