#include "libmpsse.h"

#include <cstdlib>
#include <cstring>

LibMPSSE::LibMPSSE(int vid, int pid, modes mode, int frequency, int endianess, Interface interface)
    : vid(vid)
    , pid(pid)
    , mode(mode)
    , frequency(frequency)
    , endianess(endianess)
    , interface(interface)
{

}

/* List of known FT2232-based devices */
//static struct vid_pid supported_devices[] = {
//            { 0x0403, 0x6010, "FT2232 Future Technology Devices International, Ltd" },
//            { 0x0403, 0x6011, "FT4232 Future Technology Devices International, Ltd" },
//            { 0x0403, 0x6014, "FT232H Future Technology Devices International, Ltd" },

//            /* These devices are based on FT2232 chips, but have not been tested. */
//            { 0x0403, 0x8878, "Bus Blaster v2 (channel A)" },
//            { 0x0403, 0x8879, "Bus Blaster v2 (channel B)" },
//            { 0x0403, 0xBDC8, "Turtelizer JTAG/RS232 Adapter A" },
//            { 0x0403, 0xCFF8, "Amontec JTAGkey" },
//            { 0x0403, 0x8A98, "TIAO Multi Protocol Adapter"},
//            { 0x15BA, 0x0003, "Olimex Ltd. OpenOCD JTAG" },
//            { 0x15BA, 0x0004, "Olimex Ltd. OpenOCD JTAG TINY" },

//            { 0, 0, NULL }
//};

/*
 * Open device by VID/PID
 *
 * @vid         - Device vendor ID.
 * @pid         - Device product ID.
 * @mode        - MPSSE mode, one of enum modes.
 * @freq        - Clock frequency to use for the specified mode.
 * @endianess   - Specifies how data is clocked in/out (MSB, LSB).
 * @interface   - FTDI interface to use (IFACE_A - IFACE_D).
 * @description - Device product description (set to NULL if not needed).
 * @serial      - Device serial number (set to NULL if not needed).
 *
 * Returns a pointer to an MPSSE context structure.
 * On success, open will be set to 1.
 * On failure, open will be set to 0.
 */
bool LibMPSSE::open()
{
    return openIndex(vid, pid, mode, frequency, endianess, interface, nullptr, nullptr, 0);
}

/*
 * Enable / disable internal loopback.
 *
 * @mpsse  - MPSSE context pointer.
 * @enable - Zero to disable loopback, 1 to enable loopback.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int LibMPSSE::setLoopback(int enable)
{
    unsigned char buf[1] = { 0 };
    int retval = MPSSE_FAIL;

    if(enable)
    {
        buf[0] = LOOPBACK_START;
    }
    else
    {
        buf[0] = LOOPBACK_END;
    }

    retval = rawWrite(buf, 1);

        return retval;
    }


/* Write data to the FTDI chip */
int LibMPSSE::rawWrite(unsigned char *buf, int size)
{
        int retval = MPSSE_FAIL;

        if(mode)
    {
        if(ftdi_write_data(&ftdi, buf, size) == size)
            {
                    retval = MPSSE_OK;
        }
        }

    return retval;
}

/*
 * Sets the transmitted ACK bit.
 *
 * @mpsse - MPSSE context pointer.
 * @ack   - 0 to send ACKs, 1 to send NACKs.
 *
 * Returns void.
 */
void LibMPSSE::setAck(int ack)
{
    if(ack == NACK)
    {
        tack = 0xFF;
    }
    else
    {
        tack = 0x00;
    }
}

/*
 * Causes libmpsse to send ACKs after each read byte in I2C mode.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns void.
 */
void LibMPSSE::sendAcks()
{
    return setAck(ACK);
}

/* Set the low bit pins high/low */
int LibMPSSE::setBitsLow(int port)
{
    char buf[CMD_SIZE] = { 0 };

    buf[0] = SET_BITS_LOW;
    buf[1] = port;
    buf[2] = tris;

    return rawWrite((unsigned char *) &buf, sizeof(buf));
}

/*
 * Sets the appropriate transmit and receive commands based on the requested mode and byte order.
 *
 * @mpsse     - MPSSE context pointer.
 * @endianess - MPSSE_MSB or MPSSE_LSB.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int LibMPSSE::setMode(int endianess)
{
    int retval = MPSSE_OK, i = 0, setup_commands_size = 0;
    unsigned char buf[CMD_SIZE] = { 0 };
    unsigned char setup_commands[CMD_SIZE*MAX_SETUP_COMMANDS] = { 0 };

    /* Do not call is_valid_context() here, as the FTDI chip may not be completely configured when SetMode is called */
        /* Read and write commands need to include endianess */
        tx   = MPSSE_DO_WRITE | endianess;
        rx   = MPSSE_DO_READ  | endianess;
        txrx = MPSSE_DO_WRITE | MPSSE_DO_READ | endianess;

        /* Clock, data out, chip select pins are outputs; all others are inputs. */
        tris = DEFAULT_TRIS;

        /* Clock and chip select pins idle high; all others are low */
        pidle = pstart = pstop = DEFAULT_PORT;

        /* During reads and writes the chip select pin is brought low */
        pstart &= ~CS;

        /* Disable FTDI internal loopback */
        setLoopback(0);

        /* Send ACKs by default */
        setAck(ACK);

        /* Ensure adaptive clock is disabled */
        setup_commands[setup_commands_size++] = DISABLE_ADAPTIVE_CLOCK;

        switch(mode)
        {
            case SPI0:
                /* SPI mode 0 clock idles low */
                pidle &= ~SK;
                pstart &= ~SK;
                pstop &= ~SK;
                /* SPI mode 0 propogates data on the falling edge and read data on the rising edge of the clock */
                tx |= MPSSE_WRITE_NEG;
                rx &= ~MPSSE_READ_NEG;
                txrx |= MPSSE_WRITE_NEG;
                txrx &= ~MPSSE_READ_NEG;
                break;
            case SPI3:
                /* SPI mode 3 clock idles high */
                pidle |= SK;
                pstart |= SK;
                /* Keep the clock low while the CS pin is brought high to ensure we don't accidentally clock out an extra bit */
                pstop &= ~SK;
                /* SPI mode 3 propogates data on the falling edge and read data on the rising edge of the clock */
                tx |= MPSSE_WRITE_NEG;
                rx &= ~MPSSE_READ_NEG;
                txrx |= MPSSE_WRITE_NEG;
                txrx &= ~MPSSE_READ_NEG;
                break;
            case SPI1:
                /* SPI mode 1 clock idles low */
                pidle &= ~SK;
                /* Since this mode idles low, the start condition should ensure that the clock is low */
                pstart &= ~SK;
                /* Even though we idle low in this mode, we need to keep the clock line high when we set the CS pin high to prevent
                 * an unintended clock cycle from being sent by the FT2232. This way, the clock goes high, but does not go low until
                 * after the CS pin goes high.
                 */
                pstop |= SK;
                /* Data read on falling clock edge */
                rx |= MPSSE_READ_NEG;
                tx &= ~MPSSE_WRITE_NEG;
                txrx |= MPSSE_READ_NEG;
                txrx &= ~MPSSE_WRITE_NEG;
                break;
            case SPI2:
                /* SPI 2 clock idles high */
                pidle |= SK;
                pstart |= SK;
                pstop |= SK;
                /* Data read on falling clock edge */
                rx |= MPSSE_READ_NEG;
                tx &= ~MPSSE_WRITE_NEG;
                txrx |= MPSSE_READ_NEG;
                txrx &= ~MPSSE_WRITE_NEG;
                break;
            case I2C:
                /* I2C propogates data on the falling clock edge and reads data on the falling (or rising) clock edge */
                tx |= MPSSE_WRITE_NEG;
                rx &= ~MPSSE_READ_NEG;
                /* In I2C, both the clock and the data lines idle high */
                pidle |= DO | DI;
                /* I2C start bit == data line goes from high to low while clock line is high */
                pstart &= ~DO & ~DI;
                /* I2C stop bit == data line goes from low to high while clock line is high - set data line low here, so the transition to the idle state triggers the stop condition. */
                pstop &= ~DO & ~DI;
                /* Enable three phase clock to ensure that I2C data is available on both the rising and falling clock edges */
                setup_commands[setup_commands_size++] = ENABLE_3_PHASE_CLOCK;
                break;
            case GPIO:
                break;
            default:
                retval = MPSSE_FAIL;
        }

        /* Send any setup commands to the chip */
        if(retval == MPSSE_OK && setup_commands_size > 0)
        {
            retval = rawWrite(setup_commands, setup_commands_size);
        }

        if(retval == MPSSE_OK)
        {
            /* Set the idle pin states */
            setBitsLow(pidle);

            /* All GPIO pins are outputs, set low */
            trish = 0xFF;
            gpioh = 0x00;

            buf[i++] = SET_BITS_HIGH;
            buf[i++] = gpioh;
            buf[i++] = trish;

            retval = rawWrite(buf, i);
        }

    return retval;
}
/* Convert a frequency to a clock divisor */
uint16_t LibMPSSE::freq2div(uint32_t system_clock, uint32_t freq)
{
    return (((system_clock / freq) / 2) - 1);
}
/* Convert a clock divisor to a frequency */
uint32_t LibMPSSE::div2freq(uint32_t system_clock, uint16_t div)
{
    return (system_clock / ((1 + div) * 2));
}

/* Read data from the FTDI chip */
int LibMPSSE::rawRead(unsigned char *buf, int size)
{
    int n = 0, r = 0;

    if(mode)
    {
        while(n < size)
        {
            r = ftdi_read_data(&ftdi, buf, size);
            if(r < 0) break;
            n += r;
        }

        if(flush_after_read)
        {
            /*
             * Make sure the buffers are cleared after a read or subsequent reads may fail.
             *
             * Is this needed anymore? It slows down repetitive read operations by ~8%.
             */
            ftdi_usb_purge_rx_buffer(&ftdi);
        }
    }

    return n;
}

/* Sets the read and write timeout periods for bulk usb data transfers. */
void LibMPSSE::setTimeouts(int timeout)
{
    if(mode)
    {
        ftdi.usb_read_timeout = timeout;
        ftdi.usb_write_timeout = timeout;
    }
}

/*
 * Sets the appropriate divisor for the desired clock frequency.
 *
 * @mpsse - MPSSE context pointer.
 * @freq  - Desired clock frequency in hertz.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int LibMPSSE::setClock(uint32_t freq)
{
    int retval = MPSSE_FAIL;
    uint32_t system_clock = 0;
    uint16_t divisor = 0;
    unsigned char buf[CMD_SIZE] = { 0 };

    /* Do not call is_valid_context() here, as the FTDI chip may not be completely configured when SetClock is called */
        if(freq > SIX_MHZ)
        {
            buf[0] = TCK_X5;
            system_clock = SIXTY_MHZ;
        }
        else
        {
            buf[0] = TCK_D5;
            system_clock = TWELVE_MHZ;
        }

        if(rawWrite(buf, 1) == MPSSE_OK)
        {
            if(freq <= 0)
            {
                divisor = 0xFFFF;
            }
            else
            {
                divisor = freq2div(system_clock, freq);
            }

            buf[0] = TCK_DIVISOR;
            buf[1] = (divisor & 0xFF);
            buf[2] = ((divisor >> 8) & 0xFF);

            if(rawWrite(buf, 3) == MPSSE_OK)
            {
                clock = div2freq(system_clock, divisor);
                retval = MPSSE_OK;
            }
        }

    return retval;
}
/*
 * Enables or disables flushing of the FTDI chip's RX buffers after each read operation.
 * Flushing is disable by default.
 *
 * @mpsse - MPSSE context pointer.
 * @tf    - Set to 1 to enable flushing, or 0 to disable flushing.
 *
 * Returns void.
 */
void LibMPSSE::flushAfterRead(int tf)
{
    flush_after_read = tf;
    return;
}

/*
 * Open device by VID/PID/index
 *
 * @vid         - Device vendor ID.
 * @pid         - Device product ID.
 * @mode        - MPSSE mode, one of enum modes.
 * @freq        - Clock frequency to use for the specified mode.
 * @endianess   - Specifies how data is clocked in/out (MSB, LSB).
 * @interface   - FTDI interface to use (IFACE_A - IFACE_D).
 * @description - Device product description (set to NULL if not needed).
 * @serial      - Device serial number (set to NULL if not needed).
 * @index       - Device index (set to 0 if not needed).
 *
 * Returns a pointer to an MPSSE context structure.
 * On success, open will be set to 1.
 * On failure, open will be set to 0.
 */
bool LibMPSSE::openIndex(int vid, int pid, modes mode, int frequency, int endianess, Interface interface, const char *description, const char *serial, int index)
{
    int ret = 0;
        /* Legacy; flushing is no longer needed, so disable it by default. */
        flushAfterRead(0);

        /* ftdilib initialization */
        if(ftdi_init(&ftdi) == 0)
        {
            /* Set the FTDI interface  */
            ftdi_set_interface(&ftdi, static_cast<enum ftdi_interface>(interface));

            /* Open the specified device */
            if(ftdi_usb_open_desc_index(&ftdi, vid, pid, description, serial, index) == 0)
            {
                status = STOPPED;

                /* Set the appropriate transfer size for the requested protocol */
                if(mode == I2C)
                    xsize = I2C_TRANSFER_SIZE;
                else
                    xsize = SPI_RW_SIZE;

                ret |= ftdi_usb_reset(&ftdi);
                ret |= ftdi_set_latency_timer(&ftdi, LATENCY_MS);
                ret |= ftdi_write_data_set_chunksize(&ftdi, CHUNK_SIZE);
                ret |= ftdi_read_data_set_chunksize(&ftdi, CHUNK_SIZE);
                ret |= ftdi_set_bitmode(&ftdi, 0, BITMODE_RESET);

                if(ret == 0)
                {
                    /* Set the read and write timeout periods */
                    setTimeouts(USB_TIMEOUT);

                    if(mode != BITBANG)
                    {
                        ftdi_set_bitmode(&ftdi, 0, BITMODE_MPSSE);

                        if(setClock(frequency) == MPSSE_OK)
                        {
                            if(setMode(endianess) == MPSSE_OK)
                            {
                                /* Give the chip a few mS to initialize */
//                                usleep(SETUP_DELAY);

                                /*
                                 * Not all FTDI chips support all the commands that SetMode may have sent.
                                 * This clears out any errors from unsupported commands that might have been sent during set up.
                                 */
                                ftdi_usb_purge_buffers(&ftdi);
                            }
                        }
                    }
                    else
                    {
                        /* Skip the setup functions if we're just operating in BITBANG mode */
                        if(ftdi_set_bitmode(&ftdi, 0xFF, BITMODE_BITBANG) == 0)
                        {
                            isOpened = true;
                        }
                    }
                }
            }
        }

    return true;
}

/*
 * Sets the specified pin high.
 *
 * @mpsse - MPSSE context pointer.
 * @pin   - Pin number to set high.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int LibMPSSE::pinHigh(GPIO_PINS pin)
{
    int retval = MPSSE_FAIL;

    retval = gpioWrite(pin, HIGH);

    return retval;
}


/*
 * Sets the specified pin low.
 *
 * @mpsse - MPSSE context pointer.
 * @pin   - Pin number to set low.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int LibMPSSE::pinLow(GPIO_PINS pin)
{
    int retval = MPSSE_FAIL;

    retval = gpioWrite(pin, LOW);

    return retval;
}


/* Set the high bit pins high/low */
int LibMPSSE::setBitsHigh(int port)
{
    char buf[CMD_SIZE] = { 0 };

    buf[0] = SET_BITS_HIGH;
    buf[1] = port;
    buf[2] = trish;

    return rawWrite((unsigned char *) &buf, sizeof(buf));
}
/* Set the GPIO pins high/low */
int LibMPSSE::gpioWrite(int pin, int direction)
{
    int retval = MPSSE_FAIL;

    if(mode == BITBANG)
    {
        if(direction == HIGH)
        {
            bitbang |= (1 << pin);
        }
        else
        {
            bitbang &= ~(1 << pin);
        }

        if(setBitsHigh(bitbang) == MPSSE_OK)
        {
                    retval = rawWrite((unsigned char *) &bitbang, 1);
        }
    }
    else
    {
        /* The first four pins can't be changed unless we are in a stopped status */
        if(pin < NUM_GPIOL_PINS && status == STOPPED)
        {
            /* Convert pin number (0-3) to the corresponding pin bit */
            pin = (GPIO0 << pin);

                if(direction == HIGH)
                {
                        pstart |= pin;
                        pidle |= pin;
                        pstop |= pin;
                }
                else
                {
                        pstart &= ~pin;
                        pidle &= ~pin;
                        pstop &= ~pin;
                }

            retval = setBitsLow(pstart);
        }
        else if(pin >= NUM_GPIOL_PINS && pin < NUM_GPIO_PINS)
        {
            /* Convert pin number (4 - 11) to the corresponding pin bit */
            pin -= NUM_GPIOL_PINS;

            if(direction == HIGH)
            {
                gpioh |= (1 << pin);
            }
            else
            {
                gpioh &= ~(1 << pin);
            }

            retval = setBitsHigh(gpioh);
        }
    }

    return retval;
}

void LibMPSSE::close()
{
    if(isOpened)
    {
        ftdi_set_bitmode(&ftdi, 0, BITMODE_RESET);
        ftdi_usb_close(&ftdi);
        ftdi_deinit(&ftdi);
        isOpened = false;
    }

    return;
}
