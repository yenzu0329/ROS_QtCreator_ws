#include "buy.h"

BuyBtn::BuyBtn()
{
    setText("Buy");
    setMaximumHeight(25);
}
BuyBtn::BuyBtn(QString name)
{
    setText(name);
    setMaximumHeight(25);
}
