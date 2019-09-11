#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , libMPSSE(new LibMPSSE(0x0403, 0x6014, LibMPSSE::modes::I2C, 400000, LibMPSSE::MSB, LibMPSSE::Interface::IFACE_A))
{
    ui->setupUi(this);

    connect(ui->buttonListDevices, &QPushButton::clicked, this, &MainWindow::onButtonListDevicesClicked);
    connect(ui->checkGPIO0, &QCheckBox::clicked, this, &MainWindow::onCheckGPIO0Clicked);

    if (libMPSSE->open())
        QMessageBox::warning(this, "Error", "MPSSE device open failed.");

//    libMPSSE->setCSIdleState(1);
}

MainWindow::~MainWindow()
{
    if (libMPSSE->getIsOpened())
        libMPSSE->close();
    delete ui;
}

void MainWindow::onButtonListDevicesClicked()
{
//    bool acked = false;
//    for (int i = 0; i < 256; i++) {
//        libMPSSE->start();
//        libMPSSE->write(QByteArray().append(static_cast<char>(i)));
//        for (int j = 0; j < 15; j++)
//            libMPSSE->pinHigh(LibMPSSE::GPIO_PINS::GPIOL0);
//        libMPSSE->stop();
//        if (libMPSSE->getAck() == ACK)
//            QMessageBox::warning(this, "Error", "Acked.");
//    }

//libMPSSE->test();

//    libMPSSE->start();
//    libMPSSE->writeByteWithAck(0xd0);
//        QMessageBox::information(this, "test", "acked");
//    else
//        QMessageBox::information(this, "test", "not acked");
//    libMPSSE->stop();
//    libMPSSE->flushWrite();
    QByteArray buffer;
    libMPSSE->readRegs(0x68, 0x23, 1, buffer);
    QMessageBox::information(this, "test", QString().sprintf("buffer val: %x", buffer.at(0)));
    libMPSSE->writeRegs(0x68, 0x23, QByteArray().append(0xbb));
    libMPSSE->readRegs(0x68, 0x23, 1, buffer);
    QMessageBox::information(this, "test", QString().sprintf("buffer val: %x", buffer.at(0)));

//    for (int i = 0; i < 256; i++) {
//        libMPSSE->myStart();
//        libMPSSE->myWriteByte(0x68);
//        unsigned char ack = libMPSSE->myReadOneByte();
//        libMPSSE->myStop();
//        libMPSSE->myStart();
//        libMPSSE->myWriteByte(0x69);
//        ack = libMPSSE->myReadOneByte();
//        libMPSSE->myStop();
//        libMPSSE->myStart();
//        libMPSSE->myWriteByte(0xd0);
//        ack = libMPSSE->myReadOneByte();
//        libMPSSE->myStop();
//        libMPSSE->myStart();
//        libMPSSE->myWriteByte(0xd1);
//        ack = libMPSSE->myReadOneByte();
//        libMPSSE->myStop();
//    }

//    QMessageBox::information(this, "ack", QString().sprintf("%d", ack));

//    libMPSSE->start();
//    libMPSSE->write(QByteArray().append(0xAA));
//    libMPSSE->write(QByteArray().append(0x45).append(0x25));
//    QByteArray buffer = libMPSSE->read(5);
//    libMPSSE->stop();
//    QMessageBox::warning(this, "Error", QString().sprintf("count %d %d %d %d %d %d", buffer.count(), buffer.at(0), buffer.at(1), buffer.at(2), buffer.at(3), buffer.at(4)));
}

void MainWindow::onCheckGPIO0Clicked()
{
//    if (ui->checkGPIO0->isChecked())
//        libMPSSE->pinHigh(LibMPSSE::GPIO_PINS::GPIOL0);
//    else
//        libMPSSE->pinLow(LibMPSSE::GPIO_PINS::GPIOL0);
}

