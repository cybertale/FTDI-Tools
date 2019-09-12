#include "mpsse.h"

#include <QList>

MPSSE::MPSSE(int vid, int pid, modes mode, int frequency, ENDIANESS endianess, Interface interface)
    : QObject ()
    , vid(vid)
    , pid(pid)
    , mode(mode)
    , frequency(frequency)
    , endianess(endianess)
    , interface(interface)
    , bufferWrite(new QByteArray())
    , mutex(new QMutex())
    , timerUpdateGPIO(new QBasicTimer())
    , gpioRefreshHigh(0)
    , gpioRefreshLow(0)
    , refreshInterval(5)
    , directionHigh(0)
    , directionLow(0)
    , outputHigh(0)
    , outputLow(0)
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

//bool MPSSE::open()
//{
//    return openIndex(vid, pid, mode, frequency, endianess, interface, nullptr, nullptr, 0);
//}

int MPSSE::setLoopback(int enable)
{
    QByteArray buf;

    if(enable)
        buf.append(static_cast<char>(LOOPBACK_START));
    else
        buf.append(static_cast<char>(LOOPBACK_END));

    return rawWrite(buf);
}

int MPSSE::rawWrite(QByteArray buf)
{
    unsigned char bufSend[buf.count()];

    for (int i = 0; i < buf.count(); i++)
        bufSend[i] = static_cast<unsigned char>(buf.at(i));

    if(mode)
        if(ftdi_write_data(&ftdi, bufSend, buf.count()) == buf.count())
            return -1;

    return 0;
}

int MPSSE::setMode()
{
    int ret = 0, setup_commands_size = 0;
    QByteArray setup_commands;
    QByteArray buf;

    /* Disable FTDI internal loopback */
    setLoopback(0);

    /* Ensure adaptive clock is disabled */
    setup_commands.append(static_cast<char>(DISABLE_ADAPTIVE_CLOCK));

    switch(mode)
    {
        case SPI0:
            break;
        case SPI3:
            break;
        case SPI1:
            break;
        case SPI2:
            break;
        case I2C:
            /* Enable three phase clock to ensure that I2C data is available on both the rising and falling clock edges */
            setup_commands.append(static_cast<char>(ENABLE_3_PHASE_CLOCK));
            break;
        case GPIO:
            break;
        default:
            ret = -1;
    }

    /* Send any setup commands to the chip */
    if(ret == 0 && setup_commands_size > 0)
        ret = rawWrite(setup_commands);

    if(ret == 0)
        ret = rawWrite(buf);

    return ret;
}
/* Convert a frequency to a clock divisor */
uint16_t MPSSE::freq2div(uint32_t system_clock, uint32_t freq)
{
    return (((system_clock / freq) / 2) - 1);
}
/* Convert a clock divisor to a frequency */
uint32_t MPSSE::div2freq(uint32_t system_clock, uint16_t div)
{
    return (system_clock / ((1 + div) * 2));
}

/* Read data from the FTDI chip */
QByteArray MPSSE::rawRead(int size)
{
    int n = 0, r = 0;
    unsigned char buf[size];

    if(mode)
    {
        while(n < size)
        {
            r = ftdi_read_data(&ftdi, buf, size);
            if(r < 0) break;
            n += r;
        }
    }

    QByteArray array;
    for (int i = 0; i < size; i++)
        array.append(static_cast<char>(buf[i]));

    return array;
}

/* Sets the read and write timeout periods for bulk usb data transfers. */
void MPSSE::setTimeouts(int timeout)
{
    if(mode)
    {
        ftdi.usb_read_timeout = timeout;
        ftdi.usb_write_timeout = timeout;
    }
}

int MPSSE::setClock(uint32_t freq)
{
    int ret = -1;
    uint32_t system_clock = 0;
    uint16_t divisor = 0;
    QByteArray buf;

    /* Do not call is_valid_context() here, as the FTDI chip may not be completely configured when SetClock is called */
    if(freq > SIX_MHZ)
    {
        buf.append(TCK_X5);
        system_clock = SIXTY_MHZ;
    }
    else
    {
        buf.append(TCK_D5);
        system_clock = TWELVE_MHZ;
    }

    if(!rawWrite(buf))
    {
        if(freq <= 0)
        {
            divisor = 0xFFFF;
        }
        else
        {
            divisor = freq2div(system_clock, freq);
        }

        buf.clear();
        buf.append(TCK_DIVISOR);
        buf.append(divisor & 0xFF);
        buf.append((divisor >> 8) & 0xFF);

        if(!rawWrite(buf))
        {
            clock = div2freq(system_clock, divisor);
            ret = 0;
        }
    }

    return ret;
}

bool MPSSE::open()
{
        if(ftdi_init(&ftdi) == 0)
        {
            ftdi_set_interface(&ftdi, static_cast<enum ftdi_interface>(interface));
            if(ftdi_usb_open_desc_index(&ftdi, vid, pid, nullptr, nullptr, 0) == 0)
            {
                int ret = 0;
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

                        if(!setClock(static_cast<uint32_t>(frequency)))
                        {
                            if(!setMode())
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
            } else
                return -1;
        } else
            return -1;

    return 0;
}

bool MPSSE::getIsOpened() const
{
    return isOpened;
}

void MPSSE::close()
{
    if(isOpened)
    {
        ftdi_set_bitmode(&ftdi, 0, BITMODE_RESET);
        ftdi_usb_close(&ftdi);
        ftdi_deinit(&ftdi);
        isOpened = false;
    }
}

void MPSSE::setCSIdleState(GPIO_STATE state)
{
    idleCS = state;
}

char MPSSE::getGPIOState(GPIO_HIGH_LOW high_low)
{
    mutex->lock();
    if (high_low == GPIO_LOW)
        bufferWrite->append(static_cast<char>(READ_GPIO_LOW));
    else if (high_low == GPIO_HIGH)
        bufferWrite->append(static_cast<char>(READ_GPIO_HIGH));

    bufferWrite->append(static_cast<char>(SEND_IMMEDIATE));
    mutex->unlock();
    flushWrite();
    QByteArray data = rawRead(1);
    return data.at(0);
}

void MPSSE::setGPIOState(GPIO_PINS pin, GPIO_MODE gpioMode, GPIO_STATE state)
{
    char *pMode = &directionLow;
    char *pData = &outputLow;
    unsigned char pinIndex = static_cast<unsigned char>(pin);
    char cmd = static_cast<char>(SET_BITS_LOW);

    if (pinIndex >= 8) {
        pinIndex -= 8;
        pMode = &directionHigh;
        pData = &outputHigh;
        cmd = static_cast<char>(SET_BITS_HIGH);
    }

    if (gpioMode == OUT)
        *pMode |= (1 << pinIndex);
    else
        *pMode &= ~(1 << pinIndex);

    if (state == HIGH)
        *pData |= (1 << pinIndex);
    else
        *pData &= ~(1 << pinIndex);

    mutex->lock();
    bufferWrite->append(cmd);
    bufferWrite->append(*pData);
    bufferWrite->append(*pMode);
    mutex->unlock();
}

void MPSSE::start()
{
    setGPIOState(CS, OUT, LOW);
    if (mode == I2C) {
        setGPIOState(DO, OUT, HIGH);
        setGPIOState(SK, OUT, HIGH);

        setGPIOState(DO, OUT, LOW);
        setGPIOState(SK, OUT, HIGH);

        setGPIOState(DO, OUT, LOW);
        setGPIOState(SK, OUT, LOW);
    }
}

void MPSSE::writeBytes(QByteArray buffer)
{
    clockBytesOut(buffer);
}

void MPSSE::readBytes(int length)
{
    clockBytesIn(length);
}

void MPSSE::clockBytesOut(QByteArray buffer)
{
    //TODO mode MSB/LSB POS/NEG
    mutex->lock();
    bufferWrite->append(CLOCK_BYTES_OUT_NEG_EDGE_MSB);
    int length = buffer.count() - 1;
    bufferWrite->append(static_cast<char>(length & 0xff));
    bufferWrite->append(static_cast<char>(length >> 8));
    for (int i = 0; i < buffer.count(); i++)
        bufferWrite->append(buffer.at(i));
    mutex->unlock();
}

void MPSSE::clockBytesIn(int length)
{
    length -= 1;
    mutex->lock();
    bufferWrite->append(CLOCK_BYTES_IN_POS_EDGE_MSB);
    bufferWrite->append(static_cast<char>(length & 0xff));
    bufferWrite->append(static_cast<char>(length >> 8));
    bufferWrite->append(static_cast<char>(SEND_IMMEDIATE));
    mutex->unlock();
}

void MPSSE::flushWrite()
{
    mutex->lock();
    rawWrite(*bufferWrite);
    bufferWrite->clear();
    mutex->unlock();
}

void MPSSE::setTristate(uint16_t pins)
{
    mutex->lock();
    bufferWrite->append(static_cast<char>(SET_IO_TRISTATE));
    bufferWrite->append(static_cast<char>(pins & 0xff));
    bufferWrite->append(static_cast<char>(pins >> 8));
    mutex->unlock();
    flushWrite();
}

//length here belongs to 0-7, corresponding to 1-8 bits.
void MPSSE::readBits(char length)
{
    length -= 1;
    mutex->lock();
    bufferWrite->append(CLOCK_BITS_IN_POS_EDGE_MSB);
    bufferWrite->append(length);
    bufferWrite->append(static_cast<char>(SEND_IMMEDIATE));	//Flush read data back to PC.
    mutex->unlock();
}

void MPSSE::writeBits(char length, char data)
{
    length -= 1;
    mutex->lock();
    bufferWrite->append(CLOCK_BITS_OUT_POS_EDGE_MSB);
    bufferWrite->append(length);
    bufferWrite->append(data);
    mutex->unlock();
}

void MPSSE::stop()
{
    if (mode == I2C) {
        setGPIOState(DO, OUT, LOW);
        setGPIOState(SK, OUT, LOW);

        setGPIOState(DO, OUT, LOW);
        setGPIOState(SK, OUT, HIGH);

        setGPIOState(DO, OUT, HIGH);
        setGPIOState(SK, OUT, HIGH);
    }
    setGPIOState(CS, OUT, HIGH);
}

bool MPSSE::writeByteWithAck(char data)
{
    setGPIOState(DO, OUT, LOW);
    writeBytes(QByteArray().append(data));
//    setGPIOState(SK, OUT, LOW);
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

void MPSSE::sendAck(bool send)
{
    if (send)
        writeBits(1, static_cast<char>(0x80));
    else
        writeBits(1, static_cast<char>(0x00));
}

void MPSSE::setRefreshInterval(int value)
{
    refreshInterval = value;
}

char MPSSE::readByteWithAck(bool ack)
{
    setGPIOState(DO, IN, LOW);
    readBytes(1);
    sendAck(ack);
    mutex->lock();
    bufferWrite->append(static_cast<char>(SEND_IMMEDIATE));	//Flush read data back to PC.
    mutex->unlock();
    flushWrite();
    return rawRead(1).at(0);
}

//-1 for no ack.
int MPSSE::writeRegs(char address, char reg, QByteArray data)
{
    int ret = 0;

    if (mode == I2C) {
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
    }
    return 0;

stop_return:
    stop();
    flushWrite();
    return ret;
}

int MPSSE::readRegs(char address, char reg, char length, QByteArray &array)
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

QList<char> MPSSE::detectDevices()
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

void MPSSE::timerEvent(QTimerEvent *event)
{
    if (event->timerId() == timerUpdateGPIO->timerId()) {
        if (gpioRefreshLow) {
            inputStateLow = getGPIOState(GPIO_LOW);
            char bitsChanged = inputStateLow ^ inputStateLowLast;
            if (bitsChanged != 0x00) {
                emit gpioStateChanged(GPIO_LOW, inputStateLow);
                inputStateLowLast = inputStateLow;
            }
        }
        if (gpioRefreshHigh) {
            inputStateHigh = getGPIOState(GPIO_HIGH);
            char bitsChanged = inputStateHigh ^ inputStateHighLast;
            if (bitsChanged != 0x00) {
                emit gpioStateChanged(GPIO_HIGH, inputStateHigh);
                inputStateHighLast = inputStateHigh;
            }
        }
    }
    QObject::timerEvent(event);
}

void MPSSE::setGPIORefresh(GPIO_HIGH_LOW high_low, bool enable)
{
    bool oldRefreshHigh = gpioRefreshHigh, oldRefreshLow = gpioRefreshLow;

    if (high_low == GPIO_LOW)
        gpioRefreshLow = enable;
    if (high_low == GPIO_HIGH)
        gpioRefreshHigh = enable;

    //If none was enabled and some one is enabled.
    if (!oldRefreshLow && !oldRefreshHigh && enable)
        timerUpdateGPIO->start(refreshInterval, this);
    //Someone was enabled while no one is enabled now.
    if ((oldRefreshLow || oldRefreshHigh) && (!gpioRefreshLow && !gpioRefreshHigh))
        timerUpdateGPIO->stop();
}
