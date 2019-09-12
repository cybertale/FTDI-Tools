#ifndef MPSSE_I2C_H
#define MPSSE_I2C_H

#include "mpsse.h"

class MPSSE_I2C : public MPSSE
{
public:
    MPSSE_I2C(int vid, int pid, int frequency, ENDIANESS endianess, INTERFACE interface);

    QList<char> detectDevices();
    char readByteWithAck(bool ack);
    bool writeByteWithAck(char data);

    // MPSSE interface
    void start() override;
    void stop() override;
    int readRegs(char address, char reg, char length, QByteArray &array) override;
    int writeRegs(char address, char reg, QByteArray data) override;
private:
    void sendAck(bool ack);
};

#endif // MPSSE_I2C_H
