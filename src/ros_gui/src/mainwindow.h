#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMetaObject>
#include <QTimer>
#include "scene.h"
#include "global_var.h"
#include "buy.h"
#include "good.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_run_btn_clicked();
    void on_vel_slider_valueChanged(int value);
    void on_angvel_slider_valueChanged(int value);
    void buyBtn_clicked();
    void deleteBtn_clicked();
    void value_change(int value);
    void on_toolButtonSearch_clicked();
    void on_lineEditSearch_returnPressed();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
