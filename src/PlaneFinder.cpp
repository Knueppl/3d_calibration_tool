#include "PlaneFinder.h"
#include "ConfigDialog.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/registration/icp.h>

#include <cmath>
#include <iostream>

#include <QDebug>

PlaneFinder::PlaneFinder(QObject* parent)
    : QThread(parent),
      _inputCloud(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _planeCloud(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _caliBoardCloud(new pcl::PointCloud<pcl::PointXYZRGBL>()),
      _alpha(0.0),
      _beta(0.0),
      _gamma(0.0),
      _midPoint(1, 1, CV_32FC3),
      _dialog(0)
{
    this->start();
    this->generateCalibrationBoard();
}

PlaneFinder::~PlaneFinder(void)
{

}

void PlaneFinder::setConfigDialog(ConfigDialog* dialog)
{
    _mutex.lock();
    _dialog = dialog;
    _mutex.unlock();
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

    if (!_dialog)
    {
        _mutex.unlock();
        return;
    }

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
    seg.setDistanceThreshold(0.03);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    _planeCloud->clear();

    /* Segment the largest planar component from the remaining cloud */
    seg.setInputCloud(_inputCloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 1000)
    {
//        qDebug() << "Could not estimate a plane model for the given dataset.";
        return;
    }

    const float normalAbs = std::sqrt(std::pow(coefficients->values[0], 2) + std::pow(coefficients->values[1], 2) + std::pow(coefficients->values[2], 2));
    _alpha = std::acos(coefficients->values[0] / normalAbs);
    _beta = std::acos(coefficients->values[1] / normalAbs);
    _gamma = std::acos(coefficients->values[2] / normalAbs);

    /* check model cooefficients */
//    qDebug() << "model coefficients:";
//    qDebug() << "----------------------------------";
//    qDebug() << "alpha = " << _alpha / M_PI * 180.0;
//    qDebug() << "beta  = " << _beta / M_PI * 180.0;
//    qDebug() << "gamma = " << _gamma / M_PI * 180.0;

    pcl::ExtractIndices<pcl::PointXYZRGBL> extract;

    extract.setInputCloud(_inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*_planeCloud);



    pcl::PCA<pcl::PointXYZRGBL> pca(true);
    pca.setInputCloud(_planeCloud);
    Eigen::Matrix3f& eigenvectors = pca.getEigenVectors();
//    Eigen::Vector3f& eigenvalues = pca.getEigenValues();
    Eigen::Vector3f mean(pca.getMean()[0], pca.getMean()[1], pca.getMean()[2]);
    Eigen::Matrix4f T;

    for (int i = 0; i < 3; i++)
        T.row(i) = Eigen::Vector4f(eigenvectors.col(i)[0], eigenvectors.col(i)[1], eigenvectors.col(i)[2], mean[i]);

    T(3, 3) = 1.0;

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pcl::IterativeClosestPoint<pcl::PointXYZRGBL, pcl::PointXYZRGBL> icp;
    icp.setInputSource(_caliBoardCloud);
    icp.setInputTarget(_planeCloud);
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(1000);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(0.001);
    icp.align(*final, T);

    if (icp.hasConverged())
        final->swap(*_planeCloud);

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

    std::vector<cv::Point3f> points;
    emit this->foundAxis(start, end);
    this->computePoints(mean, eigenvectors, points);
    std::cout << "Mean: " << mean << std::endl;
    std::cout << "Points: " << std::endl << points << std::endl;
    std::cout << "T: " << std::endl << T << std::endl;
    emit this->foundPlane(_planeCloud, start, end, points);
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

void PlaneFinder::computePoints(const Eigen::Vector3f& mean, const Eigen::Matrix3f& eigenvectors, std::vector<cv::Point3f>& points)
{
    const Eigen::Vector3f x(eigenvectors.col(0));
    const Eigen::Vector3f y(eigenvectors.col(1));
    const Eigen::Vector3f dx(x * -_dialog->spaceHor());
    const Eigen::Vector3f dy(y * -_dialog->spaceVer());
    const Eigen::Vector3f topLeft(mean + x * ((_dialog->boardWidth() - 2.0 * _dialog->borderLeft()) * 0.5)
                                  + y * ((_dialog->boardHeight() - 2.0 * _dialog->borderTop()) * 0.5));
    std::cout << "top left: " << topLeft << std::endl;
    for (int row = 0; row < _dialog->pointsVer(); ++row)
    {
        const Eigen::Vector3f point(topLeft + row * dy);

        for (int col = 0; col < _dialog->pointsHor(); ++col)
        {
            Eigen::Vector3f tmpEigen(point + col * dx);
            cv::Point3f tmpCv(tmpEigen[0], tmpEigen[1], tmpEigen[2]);
            points.push_back(tmpCv);
        }
    }
}

void PlaneFinder::generateCalibrationBoard(void)
{
    _mutex.lock();
    _caliBoardCloud->clear();

    if (!_dialog)
    {
        _mutex.unlock();
        return;
    }

    const float solution = _dialog->sensorSolution();

    // Part 1 of calibration board
    const float xEnd1 = -_dialog->boardWidth() * 0.5 + _dialog->b();
    const float yEnd1 = _dialog->boardHeight() * 0.5 - _dialog->a();

    for (float y = _dialog->boardHeight() * 0.5; y > yEnd1; y -= solution)
    {
        pcl::PointXYZRGBL point;
        point.y = y;
        point.z = 0.0;
        point.r = 0xff;
        point.g = 0x6b;
        point.b = 0x00;

        for (float x = -_dialog->boardWidth() * 0.5; x < xEnd1; x += solution)
        {
            point.x = x;
            _caliBoardCloud->push_back(point);
        }
    }


    // Part 2 of calibration board
    const float xEnd2 = _dialog->boardWidth() * 0.5;
    const float yEnd2 = _dialog->boardHeight() * 0.5 - _dialog->a();

    for (float y = _dialog->boardHeight() * 0.5; y > yEnd2; y -= solution)
    {
        pcl::PointXYZRGBL point;
        point.y = y;
        point.z = 0.0;
        point.r = 0xff;
        point.g = 0x60;
        point.b = 0x00;

        for (float x = -_dialog->boardWidth() * 0.5 + 2.0 * _dialog->b(); x < xEnd2; x += solution)
        {
            point.x = x;
            _caliBoardCloud->push_back(point);
        }
    }


    // Part 3 of calibration board
    const float xEnd3 = _dialog->boardWidth() * 0.5;
    const float yEnd3 = -_dialog->boardHeight() * 0.5 + 2.0 * _dialog->c();

    for (float y = _dialog->boardHeight() * 0.5 - _dialog->a(); y > yEnd3; y -= solution)
    {
        pcl::PointXYZRGBL point;
        point.y = y;
        point.z = 0.0;
        point.r = 0x64;
        point.g = 0xff;
        point.b = 0x00;

        for (float x = _dialog->boardWidth() * 0.5 - _dialog->a(); x < xEnd3; x += solution)
        {
            point.x = x;
            _caliBoardCloud->push_back(point);
        }
    }


    // Part 4 of calibration board
    const float xEnd4 = _dialog->boardWidth() * 0.5;
    const float yEnd4 = -_dialog->boardHeight() * 0.5;

    for (float y = -_dialog->boardHeight() * 0.5 + _dialog->c(); y > yEnd4; y -= solution)
    {
        pcl::PointXYZRGBL point;
        point.y = y;
        point.z = 0.0;
        point.r = 0x64;
        point.g = 0xff;
        point.b = 0x00;

        for (float x = _dialog->boardWidth() * 0.5 - _dialog->a(); x < xEnd4; x += solution)
        {
            point.x = x;
            _caliBoardCloud->push_back(point);
        }
    }


    // Part 5 of calibration board
    const float xEnd5 = _dialog->boardWidth() * 0.5 - _dialog->a();
    const float yEnd5 = -_dialog->boardHeight() * 0.5;

    for (float y = _dialog->boardHeight() * 0.5 - _dialog->a(); y > yEnd5; y -= solution)
    {
        pcl::PointXYZRGBL point;
        point.y = y;
        point.z = 0.0;
        point.r = 0x00;
        point.g = 0xff;
        point.b = 0xf6;

        for (float x = -_dialog->boardWidth() * 0.5; x < xEnd5; x += solution)
        {
            point.x = x;
            _caliBoardCloud->push_back(point);
        }
    }

    _mutex.unlock();
}
