#include "PlaneFinder.h"
#include "ConfigDialog.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>

#include <cmath>
#include <iostream>

#include <QDebug>

PlaneFinder::PlaneFinder(QObject* parent)
    : QThread(parent),
      _inputCloud(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _planeCloud(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _alpha(0.0),
      _beta(0.0),
      _gamma(0.0),
      _midPoint(1, 1, CV_32FC3),
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
    _alpha = std::acos(coefficients->values[0] / normalAbs);
    _beta = std::acos(coefficients->values[1] / normalAbs);
    _gamma = std::acos(coefficients->values[2] / normalAbs);

    /* check model cooefficients */
    qDebug() << "model coefficients:";
    qDebug() << "----------------------------------";
    qDebug() << "alpha = " << _alpha / M_PI * 180.0;
    qDebug() << "beta  = " << _beta / M_PI * 180.0;
    qDebug() << "gamma = " << _gamma / M_PI * 180.0;

    pcl::ExtractIndices<pcl::PointXYZRGBL> extract;

    extract.setInputCloud(_inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*_planeCloud);



    pcl::PCA<pcl::PointXYZRGBL> pca(true);
    pca.setInputCloud(_planeCloud);
    Eigen::Matrix3f& eigenvectors = pca.getEigenVectors();
    Eigen::Vector3f& eigenvalues = pca.getEigenValues();
    Eigen::Vector4f& mean = pca.getMean();

    std::cout << "eigenvectors:" << std::endl << eigenvectors << std::endl;
    std::cout << "eigenvalues :" << std::endl << eigenvalues << std::endl;
    std::cout << "mean : " << std::endl << mean << std::endl;

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

void PlaneFinder::copyCloudToMat(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, cv::Mat& mat)
{
    mat.create(1, cloud->size(), CV_32FC3);
    float* dataMat = reinterpret_cast<float*>(mat.data);

    for (pcl::PointCloud<pcl::PointXYZRGBL>::const_iterator point(cloud->begin()); point < cloud->end(); ++point)
    {
        *dataMat++ = point->x;
        *dataMat++ = point->y;
        *dataMat++ = point->z;
    }
}
