#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QTime>
#include <QThread>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , mpsse(new MPSSE(0x0403, 0x6014, MPSSE::modes::I2C, 400000, MPSSE::MSB, MPSSE::Interface::IFACE_A))
    , timerUpdate(new QBasicTimer())
{
    ui->setupUi(this);

    connect(ui->buttonListDevices, &QPushButton::clicked, this, &MainWindow::onButtonListDevicesClicked);
    connect(ui->checkGPIO0, &QCheckBox::clicked, this, &MainWindow::onCheckGPIO0Clicked);

    if (mpsse->open())
        QMessageBox::warning(this, "Error", "MPSSE device open failed.");

    mpsse->setTristate(0x06);

    oled = new OLED(mpsse);

    timerUpdate->start(1000, this);

//    libMPSSE->setCSIdleState(1);
}

MainWindow::~MainWindow()
{
    if (mpsse->getIsOpened())
        mpsse->close();
    delete ui;
}

void MainWindow::onButtonListDevicesClicked()
{
    QString stringTime = QTime().currentTime().toString("hh:mm:ss");
        oled->printString(0, 0, stringTime);
}

void MainWindow::onCheckGPIO0Clicked()
{
    if (ui->checkGPIO0->isChecked()) {
        mpsse->setGPIOState(MPSSE::GPIOL0, MPSSE::OUT, MPSSE::HIGH);
        mpsse->flushWrite();
    }
    else {
        mpsse->setGPIOState(MPSSE::GPIOL0, MPSSE::OUT, MPSSE::LOW);
        mpsse->flushWrite();
    }
}

void MainWindow::timerEvent(QTimerEvent *event)
{
    if (event->timerId() == timerUpdate->timerId()) {
        QString stringTime = QTime().currentTime().toString("hh:mm:ss");
        oled->printString(0, 0, stringTime);
    } else {
        QMainWindow::timerEvent(event);
    }
}

