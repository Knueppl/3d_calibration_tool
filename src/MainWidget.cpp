#include "MainWidget.h"
#include "ui_MainWidget.h"

#include <QDebug>

#include <algorithm>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

MainWidget::MainWidget(void)
    : QMainWindow(0),
      _ui(new Ui::MainWidget),
      _depthWidget(new OpenCvWidget),
      _testWidget(new OpenCvWidget)
{
    _ui->setupUi(this);
    _depthWidget->show();
    _testWidget->show();

    this->connect(&_timer, SIGNAL(timeout()), this, SLOT(tick()));

    _timer.start(40);
}

MainWidget::~MainWidget(void)
{
    delete _depthWidget;
}

void MainWidget::tick(void)
{
    _sensor.grab();

    const std::vector<cv::Point3f>& coords(_sensor.coords());
    const cv::Mat& image = _sensor.image();
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>());
    cloud->resize(coords.size());

    std::vector<cv::Point3f>::const_iterator coord(coords.begin());
    pcl::PointCloud<pcl::PointXYZRGBL>::iterator point(cloud->begin());
    const unsigned char* pixel = image.data;

    while (coord < coords.end())
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

//    _ui->_cloudWidget->setCloud(cloud);
    _depthWidget->setMat(_sensor.z());

    cv::Mat z(_sensor.z().rows, _sensor.z().cols, CV_8UC1);
    std::vector<std::vector<cv::Point> > contours;
//    const uint16_t max = *std::max_element(z.begin<uint16_t>(), z.end<uint16_t>());
//    const uint16_t min = *std::min_element(z.begin<uint16_t>(), z.end<uint16_t>());
    unsigned char* des = z.data;
    const uint16_t* src = reinterpret_cast<uint16_t*>(_sensor.z().data);
    const unsigned int size = z.rows * z.cols;

    for (unsigned int i = 0; i < size; i++, des++, src++)
    {
        if (*src < 500 || *src > 1000)
            *des = 0;
        else
            *des = static_cast<unsigned char>((*src - 500 >> 2) & 0xff);
    }

//cv::FileStorage f;
//f.open("depth", cv::FileStorage::WRITE);
//f << "t" << z;
//f.release();
/*
cv::Canny(z, z, 50, 50, 3);
cv::findContours(z, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

std::vector<cv::Scalar> colors;
colors.push_back(cv::Scalar(255, 0, 0));
colors.push_back(cv::Scalar(100, 0, 0));
colors.push_back(cv::Scalar(0, 255, 0));
colors.push_back(cv::Scalar(0, 100, 0));
colors.push_back(cv::Scalar(0, 0, 255));
colors.push_back(cv::Scalar(0, 0, 100));
colors.push_back(cv::Scalar(255, 255, 255));
cv::Mat result = cv::Mat::zeros(z.rows, z.cols, CV_8UC3);
unsigned int rect = 0;


for (unsigned int i = 0; i < contours.size(); i++)
 {
     cv::Mat contour(contours[i]);
     std::vector<cv::Point> approx;
     cv::approxPolyDP(contour, approx, 5.0, true);

     if (approx.size() == 4)
     {
         for (std::vector<cv::Point>::const_iterator vertex(approx.begin()); vertex < approx.end(); ++vertex)
         {
             cv::circle(result, *vertex, 3, colors[0], 1);
         }

         rect++;
     }

    cv::drawContours(result, contours, -1, colors[6], 1);
    _testWidget->setMat(result);
}
*/

    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pcl::PassThrough<pcl::PointXYZRGBL> filter;

    filter.setInputCloud(cloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(0.5, 1.0);
    filter.filter(*output);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    this->computeNormals(output, normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGBL, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.01);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pcl::PointCloud<pcl::Normal>::Ptr tmpNormals(new pcl::PointCloud<pcl::Normal>());

    /* Segment the largest planar component from the remaining cloud */
    seg.setInputCloud(output);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 1000)
    {
        qDebug() << "Could not estimate a plane model for the given dataset.";
        return;
    }

    /* check model cooefficients */
    qDebug() << "model coefficients:";
    qDebug() << "----------------------------------";
    qDebug() << "a = " << coefficients->values[0];
    qDebug() << "b = " << coefficients->values[1];
    qDebug() << "c = " << coefficients->values[2];
    qDebug() << "d = " << coefficients->values[3];

    pcl::ExtractIndices<pcl::PointXYZRGBL> extract;
    pcl::ExtractIndices<pcl::Normal> extractNormal;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pcl::PointCloud<pcl::Normal>::Ptr planeNormals(new pcl::PointCloud<pcl::Normal>());

    extract.setInputCloud(output);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);

    extractNormal.setInputCloud(normals);
    extractNormal.setIndices(inliers);
    extractNormal.setNegative(false);
    extractNormal.filter(*planeNormals);

    std::vector<pcl::PointIndices> clusters;
    this->regionGrowing(plane, planeNormals, clusters);

    if (clusters.size())
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBL>());

        extract.setInputCloud(plane);
        extract.setIndices(pcl::PointIndices::Ptr(new pcl::PointIndices(clusters[1])));
        extract.setNegative(false);
        extract.filter(*tmp);

        _ui->_cloudWidget->setCloud(plane);
    }
}

void MainWidget::computeNormals(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    pcl::NormalEstimation<pcl::PointXYZRGBL, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>());

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(10);
    ne.compute(*normals);
}

void MainWidget::regionGrowing(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals,
                               std::vector<pcl::PointIndices>& clusters)
{
    pcl::RegionGrowing<pcl::PointXYZRGBL, pcl::Normal> reg;
    pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>());

    reg.setMinClusterSize(100);
    reg.setMaxClusterSize(10000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(7.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    reg.extract(clusters);
}
