#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , mpsse(new MPSSE(0x0403, 0x6014, MPSSE::modes::I2C, 400000, MPSSE::MSB, MPSSE::Interface::IFACE_A))
{
    ui->setupUi(this);

    connect(ui->buttonListDevices, &QPushButton::clicked, this, &MainWindow::onButtonListDevicesClicked);
    connect(ui->checkGPIO0, &QCheckBox::clicked, this, &MainWindow::onCheckGPIO0Clicked);

    if (mpsse->open())
        QMessageBox::warning(this, "Error", "MPSSE device open failed.");

    mpsse->setTristate(0x06);

    oled = new OLED(mpsse);

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
    oled->printString(0, 0, "hello world");
//    oled->showChar(0, 0, 'a', 16);

}

void MainWindow::onCheckGPIO0Clicked()
{
//    if (ui->checkGPIO0->isChecked())
//        libMPSSE->pinHigh(LibMPSSE::GPIO_PINS::GPIOL0);
//    else
//        libMPSSE->pinLow(LibMPSSE::GPIO_PINS::GPIOL0);
}

