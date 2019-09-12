#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QBasicTimer>
#include <QMainWindow>

#include "mpsse_i2c.h"
#include "mpsse_spi.h"
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

public slots:
    void gpioChanged(MPSSE::GPIO_HIGH_LOW high_low, char bitsChanged);

    // QObject interface
protected:
    void timerEvent(QTimerEvent *event) override;

private:
    void onButtonListDevicesClicked();
    void onCheckGPIO0Clicked();

private:
    Ui::MainWindow *ui;
    MPSSE_I2C *mpsse;
    OLED *oled;
    QBasicTimer *timerUpdate;
};
#endif // MAINWINDOW_H
