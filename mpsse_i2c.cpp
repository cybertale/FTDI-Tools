#include "mpsse_i2c.h"

MPSSE_I2C::MPSSE_I2C(int vid, int pid, int frequency, MPSSE::ENDIANESS endianess, MPSSE::INTERFACE interface)
    :MPSSE(vid, pid, I2C, frequency, endianess, interface)
{
    set3PhaseDataClocking(true);
    setTristate(0x06);
    setGPIOState(SK, OUT, LOW);

    setReadWriteInRising(false);
    setReadInRising(true);
    setWriteOutRising(false);
}

//-1 for no ack.
int MPSSE_I2C::writeRegs(char address, char reg, QByteArray data)
{
    int ret = 0;

    start();
    if (!writeByteWithAck(static_cast<char>(I2C_WRITE_ADDR(address)))) {
        ret = -1;
        goto stop_return;
    }
    if (!writeByteWithAck(reg)) {
        ret = -1;
        goto stop_return;
    }
    for (int i = 0; i < data.count(); i++) {
        if (!writeByteWithAck(data.at(i))) {
        ret = -1;
        goto stop_return;
        }
    }
    stop();
    flushWrite();
    mutex->unlock();
    return 0;

stop_return:
    stop();
    flushWrite();
    return ret;
}

int MPSSE_I2C::readRegs(char address, char reg, char length, QByteArray &array)
{
    int ret;
    array.clear();
    start();
    if (!writeByteWithAck(static_cast<char>(I2C_WRITE_ADDR(address)))) {
        ret = -1;
        goto stop_return;
    }
    if (!writeByteWithAck(reg)) {
        ret = -1;
        goto stop_return;
    }
    start();
    if (!writeByteWithAck(static_cast<char>(I2C_READ_ADDR(address)))) {
        ret = -1;
        goto stop_return;
    }
    for (int i = 0; i < length; i++) {
        if (i != length - 1)
            array.append(readByteWithAck(true));
        else
            array.append(readByteWithAck(false));
    }

    stop();
    return 0;
stop_return:
    stop();
    return ret;
}

void MPSSE_I2C::sendAck(bool ack)
{
    if (ack)
        writeBits(1, static_cast<char>(0x80));
    else
        writeBits(1, static_cast<char>(0x00));
}

void MPSSE_I2C::start()
{
    MPSSE::start();	//CS Operation.

    setGPIOState(DO, OUT, HIGH);
    setGPIOState(SK, OUT, HIGH);

    setGPIOState(DO, OUT, LOW);
    setGPIOState(SK, OUT, HIGH);

    setGPIOState(DO, OUT, LOW);
    setGPIOState(SK, OUT, LOW);
}

void MPSSE_I2C::stop()
{
    setGPIOState(DO, OUT, LOW);
    setGPIOState(SK, OUT, LOW);

    setGPIOState(DO, OUT, LOW);
    setGPIOState(SK, OUT, HIGH);

    setGPIOState(DO, OUT, HIGH);
    setGPIOState(SK, OUT, HIGH);

    MPSSE::stop(); //CS Operation.
}

char MPSSE_I2C::readByteWithAck(bool ack)
{
    setGPIOState(DO, IN, LOW);
    clockBytesIn(1);
    sendAck(ack);
    flushWrite();
    return rawRead(1).at(0);
}

bool MPSSE_I2C::writeByteWithAck(char data)
{
    setGPIOState(DO, OUT, LOW);
    clockBytesOut(QByteArray().append(data));
    setGPIOState(DO, IN, LOW);
    setGPIOState(DI, IN, LOW);
    readBits(1);	//Read ack.
    flushWrite();
    QByteArray ack = rawRead(1);
    if ((ack.at(0) & 0x01) == 0x01) {
        return false;
    }
    return true;
}

QList<char> MPSSE_I2C::detectDevices()
{
    QList<char> listDevices = QList<char>();
    for (int i = 0; i < 128; i++) {
        start();
        bool ack = writeByteWithAck(static_cast<char>(i << 1));
        stop();
        flushWrite();
        if (ack)
            listDevices.append(static_cast<char>(i));
    }

    return listDevices;
}
