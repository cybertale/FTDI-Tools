#include "mpsse_spi.h"

MPSSE_SPI::MPSSE_SPI(int vid, int pid, SPI_MODE mode, int frequency, MPSSE::ENDIANESS endianess, MPSSE::INTERFACE interface)
    : MPSSE(vid, pid, SPI, frequency, endianess, interface)
    , spiMode(mode)
{
    if (spiMode == SPI0 || spiMode == SPI1)
        setGPIOState(SK, OUT, LOW);
    else
        setGPIOState(SK, OUT, HIGH);

    if (spiMode == SPI0 || spiMode == SPI3) {
        setReadWriteInRising(true);
        setReadInRising(true);
        setWriteOutRising(false);
    } else {
        setReadWriteInRising(false);
        setReadInRising(false);
        setWriteOutRising(true);
    }
    setTristate(0x00);
}

QByteArray MPSSE_SPI::readBytes(int length)
{
    clockBytesIn(length);
    flushWrite();
    return rawRead(length);
}

void MPSSE_SPI::writeBytes(QByteArray buffer)
{
    clockBytesOut(buffer);
}

QByteArray MPSSE_SPI::readWriteBytes(QByteArray buffer)
{
    clockBytesInOut(buffer);
    flushWrite();
    return rawRead(buffer.length());
}

void MPSSE_SPI::start()
{
    MPSSE::start();

//    setGPIOState(DO, OUT, HIGH);
//    setGPIOState(DI, IN, HIGH);
}

void MPSSE_SPI::stop()
{
//    setGPIOState(DO, OUT, HIGH);
//    setGPIOState(DI, IN, HIGH);

    MPSSE::stop();
}

int MPSSE_SPI::writeRegs(char address, char reg, QByteArray data)
{

}

int MPSSE_SPI::readRegs(char address, char reg, char len, QByteArray &array)
{

}

