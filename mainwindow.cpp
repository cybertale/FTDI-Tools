#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , libMPSSE(new LibMPSSE(0x0403, 0x6014, LibMPSSE::modes::SPI3, 100000, LibMPSSE::MSB, LibMPSSE::Interface::IFACE_A))
{
    ui->setupUi(this);

    connect(ui->buttonListDevices, &QPushButton::clicked, this, &MainWindow::onButtonListDevicesClicked);
    connect(ui->checkGPIO0, &QCheckBox::clicked, this, &MainWindow::onCheckGPIO0Clicked);

    if (!libMPSSE->open())
        QMessageBox::warning(this, "Error", "MPSSE device open failed.");

    libMPSSE->setCSIdleState(0);
}

MainWindow::~MainWindow()
{
    if (libMPSSE->getIsOpened())
        libMPSSE->close();
    delete ui;
}

void MainWindow::onButtonListDevicesClicked()
{
    libMPSSE->start();
    libMPSSE->write(QByteArray().append(0xAA));
    QByteArray buffer = libMPSSE->read(5);
    libMPSSE->stop();
    QMessageBox::warning(this, "Error", QString().sprintf("count %d %d %d %d %d %d", buffer.count(), buffer.at(0), buffer.at(1), buffer.at(2), buffer.at(3), buffer.at(4)));
}

void MainWindow::onCheckGPIO0Clicked()
{
    if (ui->checkGPIO0->isChecked())
        libMPSSE->pinHigh(LibMPSSE::GPIO_PINS::GPIOL0);
    else
        libMPSSE->pinLow(LibMPSSE::GPIO_PINS::GPIOL0);
}

