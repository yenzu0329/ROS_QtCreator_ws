#ifndef GOOD_H
#define GOOD_H

#include <QTableWidgetItem>
#include <QPushButton>
#include <QSpinBox>
#include <QCheckBox>
#include <QTableWidget>
#include "buy.h"

class Good
{
public:
    Good(QString name, int max, int id);
    void setSpinBox(int max);
    int id;
    QString name;
    BuyBtn *delete_btn;
    QSpinBox *num;
    QCheckBox *done;
};

#endif // GOOD_H
