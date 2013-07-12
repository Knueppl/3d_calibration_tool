#include "ThermoCamThread.h"

#include "Imager.h"
#include "ImagerUVC.h"
#include "ImageBuilder.h"

#include <QByteArray>
#include <QDebug>

namespace {
unsigned char*        _bufferShow = 0;
unsigned short*       _bufferT    = 0;
optris::ImageBuilder* _iBuilder   = 0;

void callbackImager(unsigned short* image, unsigned int, unsigned int)
{
    _iBuilder->convertTemperatureToPalette(image, _bufferShow, optris::eIron);
}
}

ThermoCamThread::ThermoCamThread(const QByteArray& configFile, QObject* parent)
    : QThread(parent),
      _imager(0),
      _imagerUVC(new ImagerUVC),
      _bufferRaw(0),
      _serial(_imagerUVC->FindFirstDevice()),
      _image(CountBanks),
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
    _bufferT = new unsigned short[_imager->getWidth(), _imager->getHeight()];

    _bufferRaw = new unsigned char[_imagerUVC->GetWidth() * _imagerUVC->GetHeight() * 2];
    delete [] _bufferShow;
    _bufferShow = new unsigned char[_iBuilder->getStride() * _imager->getHeight() * 3];
    _imager->setFrameCallback(callbackImager);
    _iBuilder->setDynamicScaling(true);

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
    delete _imagerUVC;
    delete [] _bufferRaw;
}

void ThermoCamThread::run(void)
{
//    qDebug() << __PRETTY_FUNCTION__;
//    qDebug() << "Thread = " << QObject().thread();
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
//    qDebug() << __PRETTY_FUNCTION__;
//    qDebug() << "Thread = " << this->thread();
    _imagerUVC->GetFrame(_bufferRaw);
    _imager->process(_bufferRaw);
    _imagerUVC->ReleaseFrame();

    this->copyToImage();
}

void ThermoCamThread::copyToImage(void)
{
    unsigned char* data = _bufferShow;
    BufferBank bank;

//    qDebug() << __PRETTY_FUNCTION__ << ": lock";
    _mutex.lock();
//    qDebug() << __PRETTY_FUNCTION__ << ": locked";

    if (_bank == BankA)
        bank = BankB;
    else
        bank = BankA;

    unsigned char* dataMat = _image[bank].data;
    const int size = _image[bank].rows * _image[bank].cols * 3;

    for (int i = 0; i < size; ++i)
        *dataMat++ = *data++;

    cv::cvtColor(_image[bank], _image[bank], CV_BGR2RGB);

//    _waitFor.wakeAll();
    _mutex.unlock();
//    qDebug() << __PRETTY_FUNCTION__ << ": unlocked";
//    qDebug() << __PRETTY_FUNCTION__ << ": wake other threads";
//    qDebug() << "Thread = " << this->thread();
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
//    qDebug() << __PRETTY_FUNCTION__ << ": lock";
    _mutex.lock();

//    qDebug() << __PRETTY_FUNCTION__ << ": locked";
//    qDebug() << __PRETTY_FUNCTION__ << ": waitFor";
//    qDebug() << "Thread = " << this->thread();
//    _waitFor.wait(&_mutex);
//    qDebug() << __PRETTY_FUNCTION__ << ": returned";
    this->switching();

    _mutex.unlock();
//    qDebug() << __PRETTY_FUNCTION__ << ": unlocked";
}

void ThermoCamThread::switching(void)
{
    if (_bank == BankA)
        _bank = BankB;
    else
        _bank = BankA;
}
