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
#define SPI_TRANSFER_SIZE	512
#define I2C_TRANSFER_SIZE	64

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

enum pins
{
    SK	= 1,
    DO	= 2,
    DI	= 4,
    CS	= 8 ,
    GPIO0	= 16,
    GPIO1	= 32,
    GPIO2	= 64,
    GPIO3	= 128
};
/* Common clock rates */
enum clock_rates
{
    ONE_HUNDRED_KHZ  = 100000,
    FOUR_HUNDRED_KHZ = 400000,
    ONE_MHZ 	 = 1000000,
    TWO_MHZ		 = 2000000,
    FIVE_MHZ	 = 5000000,
    SIX_MHZ 	 = 6000000,
    TEN_MHZ		 = 10000000,
    TWELVE_MHZ 	 = 12000000,
    FIFTEEN_MHZ      = 15000000,
    THIRTY_MHZ 	 = 30000000,
    SIXTY_MHZ 	 = 60000000
};
enum low_bits_status
{
    STARTED,
    STOPPED
};
#define DEFAULT_TRIS            (SK | DO | CS | GPIO0 | GPIO1 | GPIO2 | GPIO3)  /* SK/DO/CS and GPIOs are outputs, DI is an input */
#define DEFAULT_PORT            (SK | CS)       				/* SK and CS are high, all others low */

enum i2c_ack
{
    ACK  = 0,
    NACK = 1
};
class MPSSE : public QObject
{
    Q_OBJECT

public:
/* Supported MPSSE modes */
enum modes
{
    SPI0    = 1,
    SPI1    = 2,
    SPI2    = 3,
    SPI3    = 4,
    I2C     = 5,
    GPIO    = 6,
    BITBANG = 7,
};

/* FTDI interfaces */
enum Interface
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
#define CLOCK_BYTES_IN_POS_EDGE_MSB		0x20
#define CLOCK_BYTES_IN_NEG_EDGE_MSB		0x24
#define CLOCK_BITS_OUT_POS_EDGE_MSB		0x12
#define CLOCK_BITS_IN_POS_EDGE_MSB		0x22

#define I2C_WRITE_ADDR(addr)		(addr << 1)
#define I2C_READ_ADDR(addr)			((addr << 1) | 1)

#define SET_IO_TRISTATE					0x9e

#define READ_GPIO_LOW					0x81
#define READ_GPIO_HIGH					0x83

public:
    MPSSE(int vid, int pid, modes mode, int frequency, ENDIANESS endianess, Interface interface);
    ~MPSSE() {}
    bool open();
    void close();

    bool getIsOpened() const;

    void flushWrite();
    void setGPIOState(GPIO_PINS pin, GPIO_MODE gpioMode, GPIO_STATE state);
    void setCSIdleState(GPIO_STATE idle);
    int gpioWrite(int pin, int direction);
    int writeRegs(char address, char reg, QByteArray data);
    int readRegs(char address, char reg, char len, QByteArray &array);
    void setTristate(uint16_t pins);
    QList<char> detectDevices();
    char getGPIOState(GPIO_HIGH_LOW high_low);
    void setGPIORefresh(GPIO_HIGH_LOW high_low, bool enable);
    void setRefreshInterval(int value);

signals:
    void gpioStateChanged(GPIO_HIGH_LOW high_low, char pins);

    // QObject interface
protected:
    void timerEvent(QTimerEvent *event);

private:
    void start();
    void stop();
    void flushAfterRead(int tf);
    void setTimeouts(int timeout);
    QByteArray rawRead(int size);
    int rawWrite(QByteArray buf);
    int setClock(uint32_t freq);
    uint16_t freq2div(uint32_t system_clock, uint32_t freq);
    uint32_t div2freq(uint32_t system_clock, uint16_t div);
    int setLoopback(int enable);
    int setMode();
    bool writeByteWithAck(char data);
    char readByteWithAck(bool ack);
    int setBitsHigh(int port);
    void writeBytes(QByteArray buffer);
    void readBytes(int length);
    void readBits(char length);
    void clockBytesOut(QByteArray buffer);
    void clockBytesIn(int length);
    void writeBits(char length, char data);
    void sendAck(bool send);

private:
    struct ftdi_context ftdi;
    int flush_after_read;
    int clock;
    enum low_bits_status status;
    bool isOpened;
    char *description;
    int xsize;
    uint8_t tris;
    uint8_t pstart;
    uint8_t pstop;
    uint8_t pidle;
    uint8_t gpioh;
    uint8_t trish;
    uint8_t bitbang;
    uint8_t tx;
    uint8_t rx;
    uint8_t txrx;
    uint8_t tack;
    uint8_t rack;

    GPIO_STATE idleCS;

    int vid;
    int pid;
    modes mode;
    int frequency;
    ENDIANESS endianess;
    Interface interface;
    QByteArray build_block_buffer(uint8_t cmd, QByteArray data);
    QByteArray internalRead(int size);

    QByteArray *bufferWrite;
    QMutex *mutex;
    QBasicTimer *timerUpdateGPIO;
    bool gpioRefreshHigh, gpioRefreshLow;
    char inputStateHighLast, inputStateLowLast;
    char inputStateHigh, inputStateLow;
    int refreshInterval;
    char directionHigh, directionLow;
    char outputHigh, outputLow;
};

#endif // MPSSE_H
