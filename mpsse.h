#ifndef MPSSE_H
#define MPSSE_H

#include <stdint.h>
#include <libftdi1/ftdi.h>

#include <QBasicTimer>
#include <QByteArray>
#include <QMutex>
#include <QObject>

#define CHUNK_SIZE		65535
#define SPI_RW_SIZE		(63 * 1024)

#define LATENCY_MS		2
#define TIMEOUT_DIVISOR		1000000
#define USB_TIMEOUT		120000
#define SETUP_DELAY		25000

#define BITMODE_RESET		0
#define BITMODE_MPSSE		2

#define CMD_SIZE		3
#define MAX_SETUP_COMMANDS	10
#define SS_TX_COUNT		3

#define NUM_GPIOL_PINS		4
#define NUM_GPIO_PINS		12

enum mpsse_commands
{
    INVALID_COMMAND		= 0xAB,
    ENABLE_ADAPTIVE_CLOCK   = 0x96,
    DISABLE_ADAPTIVE_CLOCK  = 0x97,
    ENABLE_3_PHASE_CLOCK	= 0x8C,
    DISABLE_3_PHASE_CLOCK	= 0x8D,
    TCK_X5			= 0x8A,
    TCK_D5			= 0x8B,
    CLOCK_N_CYCLES		= 0x8E,
    CLOCK_N8_CYCLES		= 0x8F,
    PULSE_CLOCK_IO_HIGH	= 0x94,
    PULSE_CLOCK_IO_LOW	= 0x95,
    CLOCK_N8_CYCLES_IO_HIGH	= 0x9C,
    CLOCK_N8_CYCLES_IO_LOW	= 0x9D,
    TRISTATE_IO		= 0x9E,
};

enum i2c_ack
{
    ACK  = 0,
    NACK = 1
};
class MPSSE : public QObject
{
    Q_OBJECT

public:

    enum MPSSE_MODE
    {
        SPI,
        I2C,
    };


    /* FTDI interfaces */
    enum INTERFACE
    {
        IFACE_ANY	= INTERFACE_ANY,
        IFACE_A 	= INTERFACE_A,
        IFACE_B		= INTERFACE_B,
        IFACE_C		= INTERFACE_C,
        IFACE_D		= INTERFACE_D
    };

enum GPIO_PINS
{
    SK,
    DO,
    DI,
    CS,
    GPIOL0,
    GPIOL1,
    GPIOL2,
    GPIOL3,
    GPIOH0,
    GPIOH1,
    GPIOH2,
    GPIOH3,
    GPIOH4,
    GPIOH5,
    GPIOH6,
    GPIOH7,
};

enum GPIO_STATE {
    LOW,
    HIGH,
};

enum ENDIANESS {
    MSB = 0x00,
    LSB = 0x08,
};

enum GPIO_MODE {
    IN,
    OUT,
};

enum GPIO_HIGH_LOW {
    GPIO_LOW,
    GPIO_HIGH,
};

#define CLOCK_BYTES_OUT_POS_EDGE_MSB	0x10
#define CLOCK_BYTES_OUT_NEG_EDGE_MSB	0x11
#define CLOCK_BYTES_OUT_POS_EDGE_LSB	0x18
#define CLOCK_BYTES_OUT_NEG_EDGE_LSB	0x19

#define CLOCK_BYTES_IN_POS_EDGE_MSB		0x20
#define CLOCK_BYTES_IN_NEG_EDGE_MSB		0x24
#define CLOCK_BYTES_IN_POS_EDGE_LSB		0x28
#define CLOCK_BYTES_IN_NEG_EDGE_LSB		0x2C

#define CLOCK_BYTES_IN_POS_OUT_NEG_MSB	0x31
#define CLOCK_BYTES_IN_NEG_OUT_POS_MSB	0x34
#define CLOCK_BYTES_IN_POS_OUT_NEG_LSB	0x39
#define CLOCK_BYTES_IN_NEG_OUT_POS_LSB	0x3C

#define CLOCK_BITS_OUT_POS_EDGE_MSB		0x12
#define CLOCK_BITS_IN_POS_EDGE_MSB		0x22

#define I2C_WRITE_ADDR(addr)		(addr << 1)
#define I2C_READ_ADDR(addr)			((addr << 1) | 1)

#define SET_IO_TRISTATE					0x9e

#define READ_GPIO_LOW					0x81
#define READ_GPIO_HIGH					0x83

public:
    MPSSE(int vid, int pid, MPSSE_MODE mode, int frequency, ENDIANESS endianess, INTERFACE interface);
    ~MPSSE() {}

    bool open();
    void close();
    virtual void start();
    virtual void stop();
    virtual int writeRegs(char address, char reg, QByteArray data) = 0;
    virtual int readRegs(char address, char reg, char len, QByteArray &array) = 0;


    bool getIsOpened() const;

    char getGPIOState(GPIO_HIGH_LOW high_low);
    void setGPIOState(GPIO_PINS pin, GPIO_MODE gpioMode, GPIO_STATE state);
    void setTristate(uint16_t pins);
    void setGPIORefresh(GPIO_HIGH_LOW high_low, bool enable);

    void setCSIdleState(GPIO_STATE idle);
    void setRefreshInterval(int value);
    void flushWrite();
    void clockBytesOut(QByteArray buffer);

signals:
    void gpioStateChanged(GPIO_HIGH_LOW high_low, char pins);

    // QObject interface
protected:
    void timerEvent(QTimerEvent *event);

protected:
    void clockBytesInOut(QByteArray buffer);
    QByteArray rawRead(int size);
    void clockBytesIn(int length);
    void readBits(char length);
    void writeBits(char length, char data);
    void setReadWriteInRising(bool rising);
    void setReadInRising(bool rising);
    void setWriteOutRising(bool rising);

private:
    void setTimeouts(int timeout);
    int rawWrite(QByteArray buf);
    int setClock(uint32_t freq);
    uint16_t freq2div(uint32_t system_clock, uint32_t freq);
    uint32_t div2freq(uint32_t system_clock, uint16_t div);
    int setLoopback(int enable);
    int setMode();
    void enableCS();
    void disableCS();
    //TODO software CS.

protected:
    QMutex *mutex;

private:
    struct ftdi_context ftdi;
    int clock;
    bool isOpened;
    char *description;
    GPIO_STATE idleCS;

    int vid;
    int pid;
    MPSSE_MODE mode;
    int frequency;
    ENDIANESS endianess;
    INTERFACE interface;

    QByteArray *bufferWrite;
    QBasicTimer *timerUpdateGPIO;
    bool gpioRefreshHigh, gpioRefreshLow;
    char inputStateHighLast, inputStateLowLast;
    char inputStateHigh, inputStateLow;
    int refreshInterval;
    char directionHigh, directionLow;
    char outputHigh, outputLow;
    char cmdReadWriteBytes, cmdReadBytes, cmdWriteBytes;
};

#endif // MPSSE_H
