#include "mainwindow.h"
#include "ui_mainwindow.h"

const int good_count_max = 5;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //setup run button
    run_Btn = ui->run_btn;
    connect(run_Btn, &QPushButton::clicked, ui->view->scene, &Scene::compile);

    //setup timer
    QTimer *ros_timer = new QTimer(this);
    connect(ros_timer, &QTimer::timeout, ui->view->scene, &Scene::spinOnce);
    ros_timer->start(10);

    //setup slider
    ui->vel_slider->setValue(10);
    ui->vel_lbl->setText("vel = 0.250 m/s");
    ui->angvel_slider->setValue(10);
    ui->angvel_lbl->setText("ang_vel = 0.50 rad/s");

    //setup shopping table
    QWidget *widget = new QWidget;
    widget->setLayout(ui->mainLayout);
    setCentralWidget(widget);

    QStringList l1, l2;
    ui->lineEditSearch->setPlaceholderText("Search");
    ui->toolButtonSearch->setIcon(QIcon(":/search.png"));
    l1 << "Item Name" << "#" << "ID" << " ";
    ui->goods_list->horizontalHeader()->setVisible(true);
    ui->goods_list->setHorizontalHeaderLabels(l1);
    ui->goods_list->setColumnWidth(0, 100);
    ui->goods_list->setColumnWidth(1, 40);
    ui->goods_list->setColumnWidth(2, 40);
    ui->goods_list->setColumnWidth(3, 55);

    QFile file(":/goods.txt");
    QTextStream in(&file);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QString line = in.readLine();
    QStringList words;
    int i=0;
    while(!(line = in.readLine()).isNull())
    {
        words = line.split("_", QString::SkipEmptyParts);
        if(words.length() != 3) continue;
        QString name = words[0];
        int num  = words[1].toInt();
        int id   = words[2].toInt();

        QTableWidgetItem *item_num = new QTableWidgetItem();
        QTableWidgetItem *item_id = new QTableWidgetItem();
        BuyBtn *btn = new BuyBtn();
        item_num->setData(Qt::DisplayRole, num);
        item_id->setData(Qt::DisplayRole, id);
        connect(btn, &QPushButton::clicked, this, &MainWindow::buyBtn_clicked);

        ui->goods_list->setItem(i, 0, new QTableWidgetItem(name));
        ui->goods_list->setItem(i, 1, item_num);
        ui->goods_list->setItem(i, 2, item_id);
        ui->goods_list->setCellWidget(i, 3, btn);
        btn->item = ui->goods_list->item(i, 0);
        i++;
    }
    file.close();
    ui->goods_list->sortByColumn(2, Qt::AscendingOrder);
    ui->goods_list->setRowCount(i);

    l2 << " " << "Item Name" << "#" << " ";
    ui->shopping_list->setHorizontalHeaderLabels(l2);
    ui->shopping_list->setColumnWidth(0, 20);
    ui->shopping_list->setColumnWidth(1, 115);
    ui->shopping_list->setColumnWidth(2, 45);
    ui->shopping_list->setColumnWidth(3, 70);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_run_btn_clicked()
{
    if(ui->run_btn->isChecked())
    {
        ui->run_btn->setText("stop");
        ui->vel_slider->setEnabled(false);
        ui->angvel_slider->setEnabled(false);
        ui->shopping_list->setEnabled(false);
        ui->shopping_list->clearSelection();
        ui->goods_list->setEnabled(false);
        ui->goods_list->clearSelection();
        ui->view->scene->startAstar();
    }
    else
    {
        ui->run_btn->setText("run");
        ui->vel_slider->setEnabled(true);
        ui->angvel_slider->setEnabled(true);
        ui->shopping_list->setEnabled(true);
        ui->goods_list->setEnabled(true);
    }
}


void MainWindow::on_vel_slider_valueChanged(int value)
{
    double v = value * 0.5 / 20.;
    vel = v;
    ui->vel_lbl->setText("vel = " + QString::number(v, 'f', 3) + " m/s");
}


void MainWindow::on_angvel_slider_valueChanged(int value)
{
    double v = value / 20.;
    ang_vel = v;
    ui->angvel_lbl->setText("ang_vel = " + QString::number(v, 'f', 2) + " rad/s");
}

void MainWindow::buyBtn_clicked()
{
    BuyBtn *btn  = (BuyBtn*)QObject::sender();
    QString name = btn->item->text();

    if(goods_count > good_count_max) return;
    for(int i=0; i<goods.length(); i++)
        if(name == goods[i]->name)
        {
            goods[i]->num->setValue(goods[i]->num->value()+1);
            return;
        }

    int i = ui->shopping_list->rowCount();
    int num = ui->goods_list->item(btn->item->row(), 1)->text().toInt();
    int id  = ui->goods_list->item(btn->item->row(), 2)->text().toInt();

    Good *good = new Good(name, num, id);
    goods_count++;
    goods.push_back(good);
    ui->shopping_list->setRowCount(i+1);
    ui->shopping_list->setItem(i, 1, new QTableWidgetItem(good->name));
    ui->shopping_list->setCellWidget(i, 0, good->done);
    ui->shopping_list->setCellWidget(i, 2, good->num);
    ui->shopping_list->setCellWidget(i, 3, good->delete_btn);

    good->delete_btn->item = ui->shopping_list->item(i, 1);
    connect(good->delete_btn, &QPushButton::clicked, this, &MainWindow::deleteBtn_clicked);
    connect(good->num, SIGNAL(valueChanged(int)), this, SLOT(value_change(int)));
    ui->view->scene->showGoods();
}

void MainWindow::deleteBtn_clicked()
{
    BuyBtn *btn  = (BuyBtn*)QObject::sender();
    goods_count -= goods[btn->item->row()]->num->value();
    goods.removeAt(btn->item->row());
    ui->shopping_list->removeRow(btn->item->row());
    ui->view->scene->showGoods();
}

void MainWindow::value_change(int value)
{
    int after_change_count = 0;
    int len = goods.length();
    QSpinBox *spinbox = (QSpinBox*)QObject::sender();

    for(int i=0; i<len; i++)
        after_change_count += goods[i]->num->value();
    if(after_change_count > good_count_max)
    {
        spinbox->setValue(value-(after_change_count-good_count_max));
        goods_count = good_count_max;
    }
    else goods_count = after_change_count;
    ui->view->scene->showGoods();
}

void MainWindow::on_toolButtonSearch_clicked()
{
    QString keyword = ui->lineEditSearch->text();
    ui->goods_list->setSelectionMode(QAbstractItemView::MultiSelection);
    int len = ui->goods_list->rowCount();
    for(int i=0; i<len; i++)
    {
      if(ui->goods_list->item(i, 0)->text().contains(keyword, Qt::CaseInsensitive))
        ui->goods_list->setRowHidden(i, false);
      else
        ui->goods_list->setRowHidden(i, true);
    }
}

void MainWindow::on_lineEditSearch_returnPressed()
{
    on_toolButtonSearch_clicked();
}
