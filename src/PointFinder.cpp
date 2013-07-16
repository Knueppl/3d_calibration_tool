#include "PointFinder.h"
#include "ConfigDialog.h"
#include "ThermoCam.h"
#include "OpenCvWidget.h"

#include <QDebug>

PointFinder::PointFinder(QObject* parent)
    : QThread(parent),
      _cam(new ThermoCam("12100076.xml")),
      _dialog(0),
      _image(0),
      _matView(new OpenCvWidget)
{
    this->start();
}

PointFinder::~PointFinder(void)
{
    delete _cam;
    delete _matView;
}

void PointFinder::trigger(void)
{
    _mutex.lock();
    _cam->grab();
    _image = &_cam->image();
    _temperature = &_cam->temperature();
    _triggered.wakeAll();
    _mutex.unlock();
}

void PointFinder::setConfigDialog(ConfigDialog* dialog)
{
    _mutex.lock();
    _dialog = dialog;
    _mutex.unlock();
}

void PointFinder::run(void)
{
    while (1)
    {
        _mutex.lock();
        _triggered.wait(&_mutex);
        this->findPoints();
        _mutex.unlock();
    }
}

void PointFinder::findPoints(void)
{
    if (!_image || !_dialog || !_temperature)
        return;

    const unsigned short tempMin = static_cast<unsigned short>(_dialog->temperatureMin() * 10);
    const unsigned short tempMax = static_cast<unsigned short>(_dialog->temperatureMax() * 10);

    cv::Mat tempImage(_temperature->rows, _temperature->cols, CV_8UC1);

    for (int row = 0; row < _temperature->rows; row++)
    {
        const uint16_t* dataTemperature = reinterpret_cast<const uint16_t*>(_temperature->ptr(row));
        unsigned char* dataTempImage = tempImage.ptr(row);

        for (int col = 0; col < _temperature->cols; col++, dataTemperature++)
        {
            const unsigned short temp = *dataTemperature - 1000;

            if (temp > tempMax)
                *dataTempImage++ = 0xff;
            else if (temp < tempMin)
                *dataTempImage++ = 0xff;
            else
                *dataTempImage++ = 0x00;
        }
    }

    _matView->setVisible(_dialog->debugThermo());
    _matView->setMat(tempImage);

    const cv::Size patternSize(_dialog->pointsHor(), _dialog->pointsVer());
    std::vector<cv::Point2f> centers;
    const bool found = cv::findCirclesGrid(tempImage, patternSize, centers, cv::CALIB_CB_SYMMETRIC_GRID);

    cv::Mat image;
    _cam->image().copyTo(image);
    cv::drawChessboardCorners(image, patternSize, cv::Mat(centers), found);

    emit this->image(image);
}
