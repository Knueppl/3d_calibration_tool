#include "CloudManipulation.h"
#include "OpenNiSensor.h"

#include <QDebug>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

CloudManipulation::CloudManipulation(QObject* parent)
    : QThread(parent),
      _cloud(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _cloudGreen(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _cloudRed(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _coords(0),
      _z(0),
      _image(0),
      _sensor(new OpenNiSensor)
{
    this->start();
}

CloudManipulation::~CloudManipulation(void)
{
    delete _sensor;
}

void CloudManipulation::trigger(void)
{
    _mutex.lock();
    _sensor->grab();
    _z = &_sensor->z();
    _coords = &_sensor->coords();
    _image = &_sensor->image();
    _triggered.wakeAll();
    _mutex.unlock();
}

void CloudManipulation::run(void)
{
    while (1)
    {
        _mutex.lock();
        _triggered.wait(&_mutex);
        this->generateCloud();
        _mutex.unlock();
    }
}

void CloudManipulation::generateCloud(void)
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
    pcl::ExtractIndices<pcl::PointXYZRGBL> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*filtered);

    pcl::PassThrough<pcl::PointXYZRGBL> filter;

    _cloudRed->clear();
    filter.setInputCloud(filtered);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(0.0, 0.5);
    filter.filter(*_cloudRed);
    filter.setFilterLimits(1.0, 3.5);
    filter.filter(*_cloudRed);

    filter.setFilterLimits(0.5, 1.0);
    filter.filter(*_cloudGreen);

    this->paintCloud(_cloudRed, Qt::red);
    this->paintCloud(_cloudGreen, Qt::green);

    _cloud->clear();
    *_cloud += *_cloudRed;
    *_cloud += *_cloudGreen;

    emit this->cloudGenerated(_cloud);
    emit this->cloudRed(_cloudRed);
    emit this->cloudGreen(_cloudGreen);
}

void CloudManipulation::substractBackground(const cv::Mat& image, cv::Mat& mask)
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

void CloudManipulation::paintCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, const QColor& color)
{
    for (pcl::PointCloud<pcl::PointXYZRGBL>::iterator point(cloud->begin()); point < cloud->end(); ++point)
    {
        point->r = color.red();
        point->g = color.green();
        point->b = color.blue();
    }
}
