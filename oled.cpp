#include "oled.h"
#include <QThread>

OLED::OLED(MPSSE *mpsse)
    : mpsse(mpsse)
    , size(FONT6x8)
{
    initilize();
    fill(0x00);
}

int OLED::initilize()
{
    writeCmd(0xAE);
    writeCmd(0x00);
    writeCmd(0x10);
    writeCmd(0x40);
    writeCmd(0xB0);
    writeCmd(0x81);
    writeCmd(0xFF);
    writeCmd(0xA1);
    writeCmd(0xA6);
    writeCmd(0xA8);
    writeCmd(0x3F);
    writeCmd(0xC8);
    writeCmd(0xD3);
    writeCmd(0x00);

    writeCmd(0xD5);
    writeCmd(0x80);

    writeCmd(0xD8);
    writeCmd(0x05);

    writeCmd(0xD9);
    writeCmd(0xF1);

    writeCmd(0xDA);
    writeCmd(0x12);

    writeCmd(0xDB);
    writeCmd(0x30);

    writeCmd(0x8D);
    writeCmd(0x14);

    writeCmd(0xAF);
    return 0;
}

void OLED::fill(char data)
{
    for(int i = 0; i < 8; i++)
    {
        writeCmd(0xb0 + i);
        writeCmd(0x00 + i);
        writeCmd(0x10 + i);
        for(int n = 0; n < 128; n++)
            writeData(data);
    }
}

void OLED::setFontSize(OLED::FONT_SIZE size)
{
    this->size = size;
}

void OLED::setPosition(int x, int y)
{
    writeCmd(0xb0 + y);
    writeCmd(((x & 0xf0) >> 4) | 0x10);
    writeCmd(x & 0x0f);
}

void OLED::printChar(int x, int y, char chr)
{
    unsigned char c = static_cast<unsigned char>(chr - ' ');//得到偏移后的值
    if(x > 128 - 1) {
        x = 0;
        y = y + 2;
    }
    if(size == FONT8x16) {
        setPosition(x, y);
        for(int i = 0; i < 8; i++)
            writeData(F8x16[c][i]);
        setPosition(x,y+1);
        for(int i = 0; i < 8; i++)
            writeData(F8x16[c][i + 8]);
    } else {
        setPosition(x,y);
        for(int i = 0; i < 6; i++)
            writeData(F6x8[c][i]);
    }
}

void OLED::printString(int x, int y, const QString &str)
{
    QByteArray data = str.toUtf8();
    unsigned char j=0;
    for (int i = 0; i < data.count(); i++) {
        printChar(x, y, data.at(i));
        x += 8;
        if(x > 120) {
            x = 0;
            y += 2;
        }
        j++;
    }
}

int OLED::writeCmd(int cmd)
{
    return mpsse->writeRegs(address, 0x00, QByteArray().append(static_cast<char>(cmd)));
}

int OLED::writeData(int data)
{
    return mpsse->writeRegs(address, 0x40, QByteArray().append(static_cast<char>(data)));
}
