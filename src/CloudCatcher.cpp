#include "CloudCatcher.h"

#ifdef ___USE_KINECT___
#include "KinectSensor.h"
#else
#include "OpenNiSensor.h"
#endif

#include <QDebug>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

CloudCatcher::CloudCatcher(QObject* parent)
    : QThread(parent),
      _coords(0),
      _z(0),
      _image(0),
#ifdef ___USE_KINECT___
      _sensor(new KinectSensor("kinect.xml"))
#else
      _sensor(new OpenNiSensor)
#endif
{
    this->start();
}

CloudCatcher::~CloudCatcher(void)
{
    delete _sensor;
}

void CloudCatcher::trigger(void)
{
    _mutex.lock();
    _sensor->grab();
    _z = &_sensor->z();
    _coords = &_sensor->coords();
    _image = &_sensor->image();
    _triggered.wakeAll();
    _mutex.unlock();
}

void CloudCatcher::run(void)
{
    while (1)
    {
        _mutex.lock();
        _triggered.wait(&_mutex);
        this->generateCloud();
        _mutex.unlock();
    }
}

void CloudCatcher::setOperationRange(const float min, const float max)
{
    _mutex.lock();
    _minDistance = min;
    _maxDistance = max;
    _mutex.unlock();
}

void CloudCatcher::generateCloud(void)
{
    if (!_coords || !_z || !_image)
        return;

    cv::Mat mask;
    this->substractBackground(*_z, mask);

    const int size = mask.rows * mask.cols;
    const uchar* valid = mask.data;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices());

    for (int i = 0; i < size; i++, valid++)
        if (*valid)
            indices->indices.push_back(i);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>());
    cloud->resize(_coords->size());
    cloud->width = _image->cols;
    cloud->height = _image->rows;
    std::vector<cv::Point3f>::const_iterator coord(_coords->begin());
    pcl::PointCloud<pcl::PointXYZRGBL>::iterator point(cloud->begin());
    const unsigned char* pixel = _image->data;

    while (coord < _coords->end())
    {
        point->x = coord->x;
        point->y = coord->y;
        point->z = coord->z;
        point->r = *pixel;
        point->g = *pixel;
        point->b = *pixel;

        ++point;
        ++coord;
        ++pixel;
    }

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGBL>());

    _extract.setInputCloud(cloud);
    _extract.setIndices(indices);
    _extract.setNegative(false);
    _extract.filter(*filtered);
    cloud->swap(*filtered);

    _filter.setInputCloud(cloud);
    _filter.setFilterFieldName("z");
    _filter.setFilterLimits(_minDistance, _maxDistance);
    _filter.filter(*filtered);
    emit this->catchedCloud(filtered);

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBL>());
    _filter.setFilterLimits(0.0, _minDistance);
    _filter.filter(*tmp);
    this->paintCloud(tmp, Qt::red);
    *filtered += *tmp;

    _filter.setFilterLimits(_maxDistance, 4.0);
    _filter.filter(*tmp);
    this->paintCloud(tmp, Qt::red);
    *filtered += *tmp;

    emit this->cloud(filtered);
}

void CloudCatcher::substractBackground(const cv::Mat& image, cv::Mat& mask)
{
    if (image.type() != CV_16UC1)
        return;

    static cv::Mat background;

    mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

    if (background.rows != image.rows || background.cols != image.cols)
    {
        image.copyTo(background);
        return;
    }

    const uint16_t* src = reinterpret_cast<uint16_t*>(image.data);
    const uint16_t* back = reinterpret_cast<uint16_t*>(background.data);
    uchar* des = mask.data;
    const unsigned int size = image.cols * image.rows;

    for (unsigned int i = 0; i < size; i++, src++, des++, back++)
        *des = std::abs(static_cast<int>(*back) - static_cast<int>(*src)) > 100 ? 0xff : 0x00;
}

void CloudCatcher::paintCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, const QColor& color)
{
    for (pcl::PointCloud<pcl::PointXYZRGBL>::iterator point(cloud->begin()); point < cloud->end(); ++point)
    {
        point->r = color.red();
        point->g = color.green();
        point->b = color.blue();
    }
}
