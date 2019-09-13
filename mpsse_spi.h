#ifndef MPSSE_SPI_H
#define MPSSE_SPI_H

#include "mpsse.h"

class MPSSE_SPI : public MPSSE
{
public:
    enum SPI_MODE
    {
        SPI0,
        SPI1,
        SPI2,
        SPI3,
    };

    MPSSE_SPI(int vid, int pid, SPI_MODE mode, int frequency, ENDIANESS endianess, INTERFACE interface);

    QByteArray readBytes(int length);
    void writeBytes(QByteArray buffer);
    QByteArray readWriteBytes(QByteArray buffer);

    // MPSSE interface
public:
    void start() override;
    void stop() override;
    int writeRegs(char address, char reg, QByteArray data) override;
    int readRegs(char address, char reg, char len, QByteArray &array) override;
    void setBitWrite(char value);

private:
    SPI_MODE spiMode;
    char bitWrite;
};

#endif // MPSSE_SPI_H
