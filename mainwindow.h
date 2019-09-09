#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "libmpsse.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void onButtonListDevicesClicked();
    void onCheckGPIO0Clicked();

private:
    Ui::MainWindow *ui;
    LibMPSSE *libMPSSE;
};
#endif // MAINWINDOW_H
