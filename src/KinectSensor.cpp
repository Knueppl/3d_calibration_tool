#include "KinectSensor.h"
#include "KinectSensorThread.h"

KinectSensor::KinectSensor(const QByteArray& configFile)
    : _kinect(new KinectSensorThread(configFile)),
      _coords(0),
      _image(0)
{
    _kinect->start();
}

KinectSensor::~KinectSensor(void)
{
    _kinect->quit();
    _kinect->wait();
    delete _kinect;
}

void KinectSensor::grab(void)
{
    _kinect->switchBank();
    _coords = _kinect->coords();
    _image = _kinect->image();
    _z = _kinect->z();
}

