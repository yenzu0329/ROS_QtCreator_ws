#ifndef BUY_H
#define BUY_H

#include <QPushButton>
#include <QTableWidgetItem>

class BuyBtn : public QPushButton
{
public:
    BuyBtn();
    BuyBtn(QString name);
    QTableWidgetItem *item;
};

#endif // BUY_H
