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
    Eigen::Vector3f mean(pca.getMean()[0], pca.getMean()[1], pca.getMean()[2]);

    std::cout << "eigenvectors:" << std::endl << eigenvectors << std::endl;
    std::cout << "eigenvalues :" << std::endl << eigenvalues << std::endl;
    std::cout << "mean : " << std::endl << mean << std::endl;

    emit this->foundPlane(_planeCloud);



    pcl::PointXYZ start;
    Eigen::Vector3f point(mean + eigenvectors.col(0) * (-_dialog->boardWidth() * 0.5));
    start.x = point[0];
    start.y = point[1];
    start.z = point[2];

    pcl::PointXYZ end;
    point = mean + eigenvectors.col(0) * _dialog->boardWidth() * 0.5;
    end.x = point[0];
    end.y = point[1];
    end.z = point[2];

    emit this->foundAxis(start, end);
    this->computePoints(mean, eigenvectors);
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

void PlaneFinder::computePoints(const Eigen::Vector3f& mean, const Eigen::Matrix3f& eigenvectors)
{
    const Eigen::Vector3f x(eigenvectors.col(0));
    const Eigen::Vector3f y(eigenvectors.col(1));
    const Eigen::Vector3f dx(x * (_dialog->boardWidth() - 2 * _dialog->borderLeft()) / _dialog->pointsVer());
    const Eigen::Vector3f dy(y * (_dialog->boardHeight() - 2 * _dialog->borderTop()) / _dialog->pointsHor());
    const Eigen::Vector3f topLeft(mean + x * (-(_dialog->boardWidth() - _dialog->borderLeft()) * 0.5)
                                  + y * (-(_dialog->boardHeight() - _dialog->borderTop()) * 0.5));

    std::vector<Eigen::Vector3f> points;
    std::cout << "Found points:" << std::endl;

    for (int row = 0; row < _dialog->pointsHor(); ++row)
    {
        const Eigen::Vector3f point(topLeft - row * dy);

        for (int col = 0; col < _dialog->pointsVer(); ++col)
        {
            points.push_back(point + col * dx);
            std::cout << points.back() << ", ";
        }
        std::cout << std::endl;
    }

    std::cout << std::endl;
}
