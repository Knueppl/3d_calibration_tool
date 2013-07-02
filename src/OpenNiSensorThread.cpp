#include "OpenNiSensorThread.h"
#include "OpenNiDevice.h"

#include <QByteArray>
#include <QDebug>
#include <QTimer>

OpenNiSensorThread::OpenNiSensorThread(QObject* parent)
    : QThread(parent),
      _device(new OpenNiDevice()),
      _coords(CountBanks),
      _images(CountBanks),
      _zs(CountBanks),
      _bank(BankA)
{
    _device->init();

    for (unsigned int i = 0; i < _coords.size(); i++)
        _coords[i].resize(_device->width() * _device->height());
}

OpenNiSensorThread::~OpenNiSensorThread(void)
{
    delete _device;
}

void OpenNiSensorThread::run(void)
{
    if (!_device)
    {
        qDebug() << __PRETTY_FUNCTION__;
        qDebug() << "Device not valid!";
        return;
    }

    QTimer timer;
    this->connect(&timer, SIGNAL(timeout()), this, SLOT(grab()), Qt::DirectConnection);
    timer.start(33);
    this->exec();
    timer.stop();
}

void OpenNiSensorThread::grab(void)
{
    if (!_device->grab())
    {
        qDebug() << __PRETTY_FUNCTION__;
        qDebug() << "Can't grab device!";
        return;
    }

    BufferBank bank;
    std::vector<float>::const_iterator c(_device->coords().begin());

    _mutex.lock();

    if (_bank == BankA)
        bank = BankB;
    else
        bank = BankA;

    for (std::vector<cv::Point3f>::iterator coord = _coords[bank].begin(); coord < _coords[bank].end(); ++coord)
    {
        coord->x = *c++;
        coord->y = *c++;
        coord->z = *c++;
    }

    _device->image().copyTo(_images[bank]);
    _zs[bank] = _device->z();
    _mutex.unlock();
}

void OpenNiSensorThread::switchBank(void)
{
    _mutex.lock();

    if (_bank == BankA)
        _bank = BankB;
    else
        _bank = BankA;

    _mutex.unlock();
}
