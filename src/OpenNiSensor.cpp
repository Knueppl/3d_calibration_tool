#include "OpenNiSensor.h"
#include "OpenNiSensorThread.h"

OpenNiSensor::OpenNiSensor(void)
    : _device(new OpenNiSensorThread()),
      _coords(0),
      _image(0)
{
    _device->start();
}

OpenNiSensor::~OpenNiSensor(void)
{
    _device->quit();
    _device->wait();
    delete _device;
}

void OpenNiSensor::grab(void)
{
    _device->switchBank();
    _coords = _device->coords();
    _image = _device->image();
    _z = _device->z();
}
