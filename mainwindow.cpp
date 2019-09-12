#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QTime>
#include <QThread>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , mpsse(new MPSSE(0x0403, 0x6014, MPSSE::modes::SPI3, 400000, MPSSE::MSB, MPSSE::Interface::IFACE_A))
    , timerUpdate(new QBasicTimer())
{
    ui->setupUi(this);

    connect(ui->buttonListDevices, &QPushButton::clicked, this, &MainWindow::onButtonListDevicesClicked);
    connect(ui->checkGPIO0, &QCheckBox::clicked, this, &MainWindow::onCheckGPIO0Clicked);

    if (mpsse->open())
        QMessageBox::warning(this, "Error", "MPSSE device open failed.");

//    mpsse->setTristate(0x06);

//    oled = new OLED(mpsse);

//    timerUpdate->start(1000, this);

    mpsse->setGPIOState(MPSSE::GPIOH0, MPSSE::IN, MPSSE::HIGH);
    mpsse->setGPIORefresh(MPSSE::GPIO_HIGH, true);
    connect(mpsse, &MPSSE::gpioStateChanged, this, &MainWindow::gpioChanged);

//    libMPSSE->setCSIdleState(1);
}

MainWindow::~MainWindow()
{
    if (mpsse->getIsOpened())
        mpsse->close();
    delete ui;
}

void MainWindow::gpioChanged(MPSSE::GPIO_HIGH_LOW high_low, char bitsChanged)
{
    QMessageBox::information(this, "gpio state changed", QString().sprintf("%x", bitsChanged));
}

void MainWindow::onButtonListDevicesClicked()
{
//    QString stringTime = QTime().currentTime().toString("hh:mm:ss");
//        oled->printString(0, 0, stringTime);
    mpsse->start();
    mpsse->readWriteBytes(QByteArray().append(0xAA));
//    mpsse->writeBytes(QByteArray().append(0xAA));
    mpsse->stop();
    mpsse->flushWrite();
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

