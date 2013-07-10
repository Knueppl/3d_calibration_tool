#include "ThermoCamThread.h"

#include "PIImager.h"
#include "ImageBuilder.h"

#include <QByteArray>
#include <QDebug>

namespace {
unsigned char*        _bufferShow = 0;
unsigned short*       _bufferT    = 0;
optris::ImageBuilder* _iBuilder   = 0;

void callbackImager(unsigned short* image, unsigned int, unsigned int)
{
    _iBuilder->convertTemperatureToPaletteImage(image, _bufferShow);
}
}

ThermoCamThread::ThermoCamThread(const QByteArray& configFile, QObject* parent)
    : QThread(parent),
      _imager(0),
      _bufferRaw(0),
      _image(CountBanks),
      _bank(BankA)
{
    _imager = new optris::PIImager(configFile.data());
    _bufferRaw = new unsigned char[_imager->getRawBufferSize()];
    _imager->setFrameCallback(callbackImager);
    _imager->startStreaming();

    delete _iBuilder;
    _iBuilder = new optris::ImageBuilder;
    _iBuilder->setSize(_imager->getWidth(), _imager->getHeight());
    delete _bufferT;
    _bufferT = new unsigned short[_imager->getWidth(), _imager->getHeight()];

    delete [] _bufferShow;
    _bufferShow = new unsigned char[_iBuilder->getStride() * _imager->getHeight() * 3];
    _imager->setFrameCallback(callbackImager);
    _iBuilder->setPaletteScalingMethod(optris::eMinMax);

    _image[BankA].create(_imager->getHeight(), _iBuilder->getStride(), CV_8UC3);
    _image[BankB].create(_imager->getHeight(), _iBuilder->getStride(), CV_8UC3);
}

ThermoCamThread::~ThermoCamThread(void)
{
    delete [] _bufferShow;
    _bufferShow = 0;
    delete _iBuilder;
    _iBuilder = 0;

    delete _imager;
    delete [] _bufferRaw;
}

void ThermoCamThread::run(void)
{
    if (!_imager || !_bufferRaw)
    {
        qDebug() << __PRETTY_FUNCTION__;
        qDebug() << "Can't start thread!";
        return;
    }

    QTimer timer;

    this->connect(&timer, SIGNAL(timeout()), this, SLOT(grab()), Qt::DirectConnection);

    timer.start(10);
    this->exec();
    timer.stop();
}

void ThermoCamThread::grab(void)
{
    _imager->getFrame(_bufferRaw);
    _imager->process(_bufferRaw);
    _imager->releaseFrame();

    this->copyToImage();
}

void ThermoCamThread::copyToImage(void)
{
    unsigned char* dataBuffer = _bufferShow;
    BufferBank bank;

    _mutex.lock();

    if (_bank == BankA)
        bank = BankB;
    else
        bank = BankA;

    unsigned char* dataImage = _image[bank].data;
    const unsigned int size = _image[bank].cols * _image[bank].rows;

    for (unsigned int i = 0; i < size; i++, ++dataBuffer, ++dataImage)
        *dataImage = *dataBuffer;

    _mutex.unlock();
}

void ThermoCamThread::setAutoScale(const bool state)
{
    _iBuilder->setPaletteScalingMethod(state ? optris::eMinMax : optris::eManual);
}

void ThermoCamThread::setTemperatureRange(const float min, const float max)
{
    _iBuilder->setManualTemperatureRange(min, max);
}

void ThermoCamThread::switchBank(void)
{
    _mutex.lock();
    this->switching();
    _mutex.unlock();
}

void ThermoCamThread::switching(void)
{
    if (_bank == BankA)
        _bank = BankB;
    else
        _bank = BankA;
}
