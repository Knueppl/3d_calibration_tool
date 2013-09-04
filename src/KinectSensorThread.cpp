#include "KinectSensorThread.h"
#include "Kinect.h"

#include <QByteArray>
#include <QDebug>
#include <QTimer>

KinectSensorThread::KinectSensorThread(const QByteArray& configFile, QObject* parent)
    : QThread(parent),
      _kinect(new Kinect(configFile.data())),
      _coords(CountBanks, std::vector<cv::Point3f>(_kinect->getRows() * _kinect->getCols())),
      _images(CountBanks),
      _zs(CountBanks),
      _bank(BankA)
{
    for (int i = 0; i < CountBanks; i++)
        _zs[i].create(_kinect->getRows(), _kinect->getCols(), CV_16UC1);
}

KinectSensorThread::~KinectSensorThread(void)
{
    delete _kinect;
}

void KinectSensorThread::run(void)
{
    if (!_kinect)
    {
        qDebug() << __PRETTY_FUNCTION__;
        qDebug() << "Kinect not valid!";
        return;
    }

    QTimer timer;
    this->connect(&timer, SIGNAL(timeout()), this, SLOT(grab()), Qt::DirectConnection);
    timer.start(20);
    this->exec();
    timer.stop();
}

void KinectSensorThread::grab(void)
{
    if (!_kinect->grab())
    {
        qDebug() << __PRETTY_FUNCTION__;
        qDebug() << "Can't grab Kinect!";
        return;
    }

    BufferBank bank;
    double* coords = _kinect->getCoords();

    _mutex.lock();

    if (_bank == BankA)
        bank = BankB;
    else
        bank = BankA;

    for (std::vector<cv::Point3f>::iterator coord = _coords[bank].begin(); coord < _coords[bank].end(); ++coord)
    {
        coord->x = *coords++;
        coord->y = *coords++;
        coord->z = *coords++;
    }

    cv::Mat(_kinect->getRows(), _kinect->getCols(), CV_8UC3, _kinect->getRGB()).copyTo(_images[bank]);

    const int size = _zs[bank].rows * _zs[bank].cols;
    const double* src = _kinect->getZ();
    uint16_t* des = reinterpret_cast<uint16_t*>(_zs[bank].data);

    for (int i = 0; i < size; i++, src++, des++)
        *des = static_cast<uint16_t>(*src * 1000);

    _mutex.unlock();
}

void KinectSensorThread::switchBank(void)
{
    _mutex.lock();

    if (_bank == BankA)
        _bank = BankB;
    else
        _bank = BankA;

    _mutex.unlock();
}
