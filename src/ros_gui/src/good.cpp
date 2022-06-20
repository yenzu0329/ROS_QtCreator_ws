#include "good.h"

Good::Good(QString name, int max, int id)
{
    this->name = name;
    this->id = id;
    delete_btn = new BuyBtn("delete");
    num = new QSpinBox;
    done = new QCheckBox;
    setSpinBox(max);
}

void Good::setSpinBox(int max)
{
    num->setMaximumHeight(25);
    num->setMinimum(1);
    num->setMaximum(max);
    num->setValue(1);
}
