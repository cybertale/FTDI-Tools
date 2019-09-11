#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QBasicTimer>
#include <QMainWindow>

#include "mpsse.h"
#include "oled.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // QObject interface
protected:
    void timerEvent(QTimerEvent *event) override;

private:
    void onButtonListDevicesClicked();
    void onCheckGPIO0Clicked();

private:
    Ui::MainWindow *ui;
    MPSSE *mpsse;
    OLED *oled;
    QBasicTimer *timerUpdate;
};
#endif // MAINWINDOW_H
