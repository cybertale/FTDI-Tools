#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , libMPSSE(new LibMPSSE(0x0403, 0x6014, LibMPSSE::modes::GPIO, 0, 0, LibMPSSE::Interface::IFACE_A))
{
    ui->setupUi(this);

    connect(ui->buttonListDevices, &QPushButton::clicked, this, &MainWindow::onButtonListDevicesClicked);
    connect(ui->checkGPIO0, &QCheckBox::clicked, this, &MainWindow::onCheckGPIO0Clicked);

    if (!libMPSSE->open())
        QMessageBox::warning(this, "Error", "MPSSE device open failed.");
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onButtonListDevicesClicked()
{
}

void MainWindow::onCheckGPIO0Clicked()
{
    if (ui->checkGPIO0->isChecked())
        libMPSSE->pinHigh(LibMPSSE::GPIO_PINS::GPIOL0);
    else
        libMPSSE->pinLow(LibMPSSE::GPIO_PINS::GPIOL0);
}

