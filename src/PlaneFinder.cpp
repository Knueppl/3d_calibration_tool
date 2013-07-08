#include "PlaneFinder.h"
#include "ConfigDialog.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <cmath>

#include <opencv2/opencv.hpp>

#include <QDebug>

PlaneFinder::PlaneFinder(QObject* parent)
    : QThread(parent),
      _inputCloud(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _planeCloud(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _alpha(0.0),
      _beta(0.0),
      _gamma(0.0),
      _dialog(new ConfigDialog)
{
    this->start();
    _dialog->show();
}

PlaneFinder::~PlaneFinder(void)
{
    delete _dialog;
}

void PlaneFinder::run(void)
{
    while (1)
    {
        _mutex.lock();
        _updated.wait(&_mutex);
        this->search();
        _mutex.unlock();
    }
}

void PlaneFinder::setInputCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud)
{
    if (!_mutex.tryLock())
        return;

    _inputCloud->clear();
    *_inputCloud += *cloud;
    _updated.wakeAll();
    _mutex.unlock();
}

void PlaneFinder::search(void)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    this->computeNormals(_inputCloud, normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGBL, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.01);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    _planeCloud->clear();

    /* Segment the largest planar component from the remaining cloud */
    seg.setInputCloud(_inputCloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 1000)
    {
        qDebug() << "Could not estimate a plane model for the given dataset.";
        return;
    }

    const float normalAbs = std::sqrt(std::pow(coefficients->values[0], 2) + std::pow(coefficients->values[1], 2) + std::pow(coefficients->values[2], 2));
    _alpha = std::acos(coefficients->values[0] / normalAbs) / M_PI * 180.0;
    _beta = std::acos(coefficients->values[1] / normalAbs) / M_PI * 180.0;
    _gamma = std::acos(coefficients->values[2] / normalAbs) / M_PI * 180.0;

    /* check model cooefficients */
    qDebug() << "model coefficients:";
    qDebug() << "----------------------------------";
    qDebug() << "alpha = " << _alpha;
    qDebug() << "beta  = " << _beta;
    qDebug() << "gamma = " << _gamma;

    pcl::ExtractIndices<pcl::PointXYZRGBL> extract;

    extract.setInputCloud(_inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*_planeCloud);

    const float factor = 1.0 / static_cast<float>(_planeCloud->size());
    _midPoint = pcl::PointXYZ(0.0, 0.0, 0.0);

    for (pcl::PointCloud<pcl::PointXYZRGBL>::const_iterator point(_planeCloud->begin()); point < _planeCloud->end(); ++point)
    {
        _midPoint.x += point->x * factor;
        _midPoint.y += point->y * factor;
        _midPoint.z += point->z * factor;
    }

    qDebug() << "mid point = (" << _midPoint.x << ", " << _midPoint.y << ", " << _midPoint.z << ")";

    cv::Mat coords(1, _planeCloud->size(), CV_32FC3);
    float* dataCoords = reinterpret_cast<float*>(coords.data);
    for (pcl::PointCloud<pcl::PointXYZRGBL>::const_iterator point(_planeCloud->begin()); point < _planeCloud->end(); ++point)
    {
        *dataCoords++ = point->x - _midPoint.x;
        *dataCoords++ = point->y - _midPoint.y;
        *dataCoords++ = point->z - _midPoint.z;
    }
    cv::Mat distance(1, _planeCloud->size(), CV_32FC1);
    cv::pow(coords, 2.0, coords);
    float* dataDistance = reinterpret_cast<float*>(distance.data);
    dataCoords = reinterpret_cast<float*>(coords.data);
    for (unsigned int i = 0; i < _planeCloud->size(); i++, dataDistance++)
    {
        *dataDistance  = *dataCoords++;
        *dataDistance += *dataCoords++;
        *dataDistance += *dataCoords++;
    }
    cv::sqrt(distance, distance);

//    dataDistance = reinterpret_cast<float*>(distance.data);
//    for (unsigned int i = 0; i < distance.cols; i++, dataDistance++)
//        if (*dataDistance < 

    emit this->foundPlane(_planeCloud);
}

void PlaneFinder::computeNormals(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::NormalEstimation<pcl::PointXYZRGBL, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>());

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(10);
    ne.compute(*normals);
}
