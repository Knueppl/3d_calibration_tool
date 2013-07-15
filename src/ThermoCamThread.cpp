#include "ThermoCamThread.h"

#include "Imager.h"
#include "ImagerUVC.h"
#include "ImageBuilder.h"

#include <QByteArray>
#include <QDebug>

#include <cstring>

namespace {
unsigned char*        _bufferShow = 0;
unsigned short*       _bufferT    = 0;
optris::ImageBuilder* _iBuilder   = 0;

void callbackImager(unsigned short* image, unsigned int w, unsigned int h)
{
    _iBuilder->convertTemperatureToPalette(image, _bufferShow, optris::eIron);
    std::memcpy(_bufferT, image, sizeof(unsigned short) * w * h);
}
}

ThermoCamThread::ThermoCamThread(const QByteArray& configFile, QObject* parent)
    : QThread(parent),
      _imager(0),
      _imagerUVC(new ImagerUVC),
      _bufferRaw(0),
      _serial(_imagerUVC->FindFirstDevice()),
      _image(CountBanks),
      _temperature(CountBanks),
      _bank(BankA)
{
    if(!_serial || _imagerUVC->OpenDevice())
    {
        qDebug() << __PRETTY_FUNCTION__;
        qDebug() << "Error finding imager device || opening UVC device: " << _imagerUVC->GetPath();
        return;
    }

    _imager = new optris::Imager(_serial, _imagerUVC->GetWidth(), _imagerUVC->GetHeight(), _imagerUVC->GetFrequency(),
                                 const_cast<char*>(configFile.data()));
    _imager->setAutoflagControl(true);
    _imagerUVC->Start();
    delete _iBuilder;
    _iBuilder = new optris::ImageBuilder;
    _iBuilder->setSize(_imager->getWidth(), _imager->getHeight());
    delete _bufferT;
    _bufferT = new unsigned short[_imagerUVC->GetWidth() * _imagerUVC->GetHeight()];
    _bufferRaw = new unsigned char[_imagerUVC->GetWidth() * _imagerUVC->GetHeight() * 2];
    delete [] _bufferShow;
    _bufferShow = new unsigned char[_iBuilder->getStride() * _imager->getHeight() * 3];
    _imager->setFrameCallback(callbackImager);
    _iBuilder->setDynamicScaling(true);

    _image[BankA].create(_imager->getHeight(), _iBuilder->getStride(), CV_8UC3);
    _image[BankB].create(_imager->getHeight(), _iBuilder->getStride(), CV_8UC3);
    _temperature[BankA].create(_imager->getHeight(), _imager->getWidth(), CV_16UC1);
    _temperature[BankB].create(_imager->getHeight(), _imager->getWidth(), CV_16UC1);

    this->start();
}

ThermoCamThread::~ThermoCamThread(void)
{
    delete [] _bufferShow;
    _bufferShow = 0;
    delete _iBuilder;
    _iBuilder = 0;

    delete _imager;
    delete _imagerUVC;
    delete [] _bufferRaw;
}

void ThermoCamThread::run(void)
{
    if (!_imager || !_imagerUVC || !_bufferRaw)
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
    _imagerUVC->GetFrame(_bufferRaw);
    _imager->process(_bufferRaw);
    _imagerUVC->ReleaseFrame();

    this->copyToImage();
}

void ThermoCamThread::copyToImage(void)
{
    unsigned char* data = _bufferShow;
    BufferBank bank;

    _mutex.lock();

    if (_bank == BankA)
        bank = BankB;
    else
        bank = BankA;

    unsigned char* dataMat = _image[bank].data;
    const int sizeMat = _image[bank].rows * _image[bank].cols * 3;

    for (int i = 0; i < sizeMat; ++i)
        *dataMat++ = *data++;

    cv::cvtColor(_image[bank], _image[bank], CV_BGR2RGB);

    const unsigned short* dataBufferT = _bufferT;

    for (int row = 0; row < _temperature[bank].rows; row++)
    {
        uint16_t* dataTemperature = reinterpret_cast<uint16_t*>(_temperature[bank].ptr(row));

        for (int col = 0; col < _temperature[bank].cols; col++)
        {
            *dataTemperature++ = *dataBufferT++;
        }
    }

    _mutex.unlock();
}

void ThermoCamThread::setAutoScale(const bool state)
{
    _iBuilder->setDynamicScaling(state);
}

void ThermoCamThread::setTemperatureRange(const float min, const float max)
{
    _iBuilder->setTemperatureRange(min, max);
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
