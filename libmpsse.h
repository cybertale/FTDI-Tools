#ifndef MPSSE_H
#define MPSSE_H

#include <stdint.h>
#include <libftdi1/ftdi.h>


#define MPSSE_OK		0
#define MPSSE_FAIL		-1

#define MSB			0x00
#define LSB			0x08

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

#define LOW			0
#define HIGH			1
#define NUM_GPIOL_PINS		4
#define NUM_GPIO_PINS		12

#define NULL_CONTEXT_ERROR_MSG	"NULL MPSSE context pointer!"

struct vid_pid
{
    int vid;
    int pid;
    char *description;
};

#define DEFAULT_TRIS            (SK | DO | CS | GPIO0 | GPIO1 | GPIO2 | GPIO3)  /* SK/DO/CS and GPIOs are outputs, DI is an input */
#define DEFAULT_PORT            (SK | CS)       				/* SK and CS are high, all others low */

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
class LibMPSSE
{
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

struct mpsse_context
{
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
    GPIOL0 = 0,
    GPIOL1 = 1,
    GPIOL2 = 2,
    GPIOL3 = 3,
    GPIOH0 = 4,
    GPIOH1 = 5,
    GPIOH2 = 6,
    GPIOH3 = 7,
    GPIOH4 = 8,
    GPIOH5 = 9,
    GPIOH6 = 10,
    GPIOH7 = 11
};

public:
    LibMPSSE(int vid, int pid, modes mode, int frequency, int endianess, Interface interface);
    bool open();
    int pinHigh(GPIO_PINS pin);
    int pinLow(GPIO_PINS pin);

private:
    bool openIndex(int vid, int pid, modes mode, int freq, int endianess, Interface interface, const char *description, const char *serial, int index);
    void flushAfterRead(int tf);
    void setTimeouts(int timeout);
    int rawRead(unsigned char *buf, int size);
    int rawWrite(unsigned char *buf, int size);
    int setClock(uint32_t freq);
    uint16_t freq2div(uint32_t system_clock, uint32_t freq);
    uint32_t div2freq(uint32_t system_clock, uint16_t div);
    int setLoopback(int enable);
    int setMode(int endianess);
    int setBitsLow(int port);
    void setAck(int ack);
    void sendAcks();
    int gpioWrite(int pin, int direction);
    int setBitsHigh(int port);
    void close();

public:
    struct mpsse_context *context;

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

    int vid;
    int pid;
    modes mode;
    int frequency;
    int endianess;
    Interface interface;
};

#endif // MPSSE_H
