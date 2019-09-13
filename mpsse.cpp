#include "mpsse.h"

#include <QList>

MPSSE::MPSSE(int vid, int pid, MPSSE_MODE mode, int frequency, ENDIANESS endianess, INTERFACE interface)
    : QObject ()
    , mutex(new QMutex())
    , idleCS(HIGH)
    , vid(vid)
    , pid(pid)
    , mode(mode)
    , frequency(frequency)
    , endianess(endianess)
    , interface(interface)
    , bufferWrite(new QByteArray())
    , timerUpdateGPIO(new QBasicTimer())
    , gpioRefreshHigh(0)
    , gpioRefreshLow(0)
    , refreshInterval(5)
    , directionHigh(0)
    , directionLow(0)
    , outputHigh(0)
    , outputLow(0)
    , cmdReadWriteBytes(CLOCK_BYTES_IN_NEG_OUT_POS_MSB)
    , cmdReadBytes(CLOCK_BYTES_IN_POS_EDGE_MSB)
    , cmdWriteBytes(CLOCK_BYTES_OUT_NEG_EDGE_MSB)
{
    setGPIOState(DO, OUT, LOW);
}

/* List of known FT2232-based devices */
//static struct vid_pid supported_devices[] = {
//            { 0x0403, 0x6010, "FT2232 Future Technology Devices International, Ltd" },
//            { 0x0403, 0x6011, "FT4232 Future Technology Devices International, Ltd" },
//            { 0x0403, 0x6014, "FT232H Future Technology Devices International, Ltd" },

//            { 0, 0, NULL }
//};

int MPSSE::setLoopback(bool enable)
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
    unsigned char *bufSend = new unsigned char[buf.count()];
    int ret = 0;

    for (int i = 0; i < buf.count(); i++)
        bufSend[i] = static_cast<unsigned char>(buf.at(i));

    if(ftdi_write_data(&ftdi, bufSend, buf.count()) != buf.count())
        ret = -1;
    delete []bufSend;

    return ret;
}

void MPSSE::setAdaptiveClock(bool adaptive)
{
    mutex->lock();
    if (adaptive)
        bufferWrite->append(static_cast<char>(ENABLE_ADAPTIVE_CLOCK));
    else
        bufferWrite->append(static_cast<char>(DISABLE_ADAPTIVE_CLOCK));
    mutex->unlock();
}

void MPSSE::set3PhaseDataClocking(bool allow)
{
    mutex->lock();
    if (allow)
        bufferWrite->append(static_cast<char>(ENABLE_3_PHASE_DATA_CLOCKING));
    else
        bufferWrite->append(static_cast<char>(DISABLE_3_PHASE_DATA_CLOCKING));
    mutex->unlock();
}

/* Read data from the FTDI chip */
QByteArray MPSSE::rawRead(int size)
{
    int n = 0, r = 0;
    unsigned char *buf = new unsigned char[size];

    while(n < size)
    {
        if ((r = ftdi_read_data(&ftdi, buf, size)) < 0)
            break;
        n += r;
    }
    delete []buf;

    QByteArray array;
    for (int i = 0; i < size; i++)
        array.append(static_cast<char>(buf[i]));

    return array;
}

/* Sets the read and write timeout periods for bulk usb data transfers. */
void MPSSE::setTimeouts(int timeout)
{
    ftdi.usb_read_timeout = timeout;
    ftdi.usb_write_timeout = timeout;
}

//Only used by setClock.
void MPSSE::setClockDivideBy5(bool divide)
{
    mutex->lock();
    if (divide)
        bufferWrite->append(static_cast<char>(ENABLE_CLOCK_DIVIDE_BY_5));
    else
        bufferWrite->append(static_cast<char>(DISABLE_CLOCK_DIVIDE_BY_5));
    mutex->unlock();
}

int MPSSE::setClock(uint32_t frequency)
{
    uint32_t systemClock = 0;
    uint16_t divisor = 0;

    if(frequency > 6000000)
    {
        setClockDivideBy5(false);
        systemClock = 60000000;
    }
    else
    {
        setClockDivideBy5(true);
        systemClock = 12000000;
    }

    if(frequency <= 0)
        divisor = 0xFFFF;
    else
        divisor = static_cast<uint16_t>(systemClock / frequency / 2 - 1);

    mutex->lock();
    bufferWrite->append(static_cast<char>(TCK_DIVISOR));
    bufferWrite->append(static_cast<char>(divisor & 0xFF));
    bufferWrite->append(static_cast<char>((divisor >> 8) & 0xFF));
    mutex->unlock();

    return flushWrite();
}

bool MPSSE::open()
{
        if(ftdi_init(&ftdi) == 0)
        {
            ftdi_set_interface(&ftdi, static_cast<enum ftdi_interface>(interface));
            if(ftdi_usb_open_desc_index(&ftdi, vid, pid, nullptr, nullptr, 0) == 0)
            {
                int ret = 0;

                ret |= ftdi_usb_reset(&ftdi);
                ret |= ftdi_set_latency_timer(&ftdi, LATENCY_MS);
                ret |= ftdi_write_data_set_chunksize(&ftdi, CHUNK_SIZE);
                ret |= ftdi_read_data_set_chunksize(&ftdi, CHUNK_SIZE);
                ret |= ftdi_set_bitmode(&ftdi, 0, BITMODE_RESET);

                if(ret == 0)
                {
                    /* Set the read and write timeout periods */
                    setTimeouts(USB_TIMEOUT);

                    ftdi_set_bitmode(&ftdi, 0, BITMODE_MPSSE);

                    if(!setClock(static_cast<uint32_t>(frequency)))
                    {
                        setLoopback(false);

                        setAdaptiveClock(false);

                        if (!flushWrite()) {
                            /*
                             * Not all FTDI chips support all the commands that SetMode may have sent.
                             * This clears out any errors from unsupported commands that might have been sent during set up.
                             */
                            ftdi_usb_purge_buffers(&ftdi);
                        }
                    }
                    isOpened = true;
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

void MPSSE::enableCS()
{
    if (idleCS == HIGH)
        setGPIOState(CS, OUT, LOW);
    else
        setGPIOState(CS, OUT, HIGH);
}

void MPSSE::disableCS()
{
    setGPIOState(CS, OUT, idleCS);
}

void MPSSE::start()
{
    enableCS();
}

void MPSSE::stop()
{
    disableCS();
}

void MPSSE::clockBytesInOut(QByteArray buffer)
{
    int length = buffer.count() - 1;
    mutex->lock();
    bufferWrite->append(cmdReadWriteBytes);
    bufferWrite->append(static_cast<char>(length & 0xff));
    bufferWrite->append(static_cast<char>(length >> 8));
    for (int i = 0; i < buffer.count(); i++)
        bufferWrite->append(buffer.at(i));
    bufferWrite->append(static_cast<char>(SEND_IMMEDIATE));
    mutex->unlock();
}

void MPSSE::clockBytesOut(QByteArray buffer)
{
    //TODO mode MSB/LSB POS/NEG
    mutex->lock();
    bufferWrite->append(cmdWriteBytes);
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
    bufferWrite->append(cmdReadBytes);
    bufferWrite->append(static_cast<char>(length & 0xff));
    bufferWrite->append(static_cast<char>(length >> 8));
    bufferWrite->append(static_cast<char>(SEND_IMMEDIATE));
    mutex->unlock();
}

int MPSSE::flushWrite()
{
    int ret;

    mutex->lock();
    ret = rawWrite(*bufferWrite);
    bufferWrite->clear();
    mutex->unlock();

    return ret;
}

void MPSSE::setTristate(uint16_t pins)
{
    mutex->lock();
    bufferWrite->append(static_cast<char>(SET_IO_TRISTATE));
    bufferWrite->append(static_cast<char>(pins & 0xff));
    bufferWrite->append(static_cast<char>(pins >> 8));
    mutex->unlock();
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

void MPSSE::setReadWriteInRising(bool rising)
{
    if (rising) {
        if (endianess == MSB)
            cmdReadWriteBytes = CLOCK_BYTES_IN_POS_OUT_NEG_MSB;
        else
            cmdReadWriteBytes = CLOCK_BYTES_IN_POS_OUT_NEG_LSB;
    }
    else {
        if (endianess == MSB)
            cmdReadWriteBytes = CLOCK_BYTES_IN_NEG_OUT_POS_MSB;
        else
            cmdReadWriteBytes = CLOCK_BYTES_IN_NEG_OUT_POS_LSB;
    }
}

void MPSSE::setReadInRising(bool rising)
{
    if (rising) {
        if (endianess == MSB)
            cmdReadBytes = CLOCK_BYTES_IN_POS_EDGE_MSB;
        else
            cmdReadBytes = CLOCK_BYTES_IN_POS_EDGE_LSB;
    }
    else {
        if (endianess == MSB)
            cmdReadBytes = CLOCK_BYTES_IN_NEG_EDGE_MSB;
        else
            cmdReadBytes = CLOCK_BYTES_IN_NEG_EDGE_LSB;
    }
}

void MPSSE::setWriteOutRising(bool rising)
{
    if (rising) {
        if (endianess == MSB)
            cmdWriteBytes = CLOCK_BYTES_OUT_POS_EDGE_MSB;
        else
            cmdWriteBytes = CLOCK_BYTES_OUT_POS_EDGE_LSB;
    }
    else {
        if (endianess == MSB)
            cmdWriteBytes = CLOCK_BYTES_OUT_NEG_EDGE_MSB;
        else
            cmdWriteBytes = CLOCK_BYTES_OUT_NEG_EDGE_LSB;
    }
}

void MPSSE::setRefreshInterval(int value)
{
    refreshInterval = value;
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
