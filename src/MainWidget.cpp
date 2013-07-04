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
      _ui(new Ui::MainWidget)
{
    _ui->setupUi(this);

    this->connect(&_timer, SIGNAL(timeout()), this, SLOT(tick()));
    this->connect(&_cloudManipulation, SIGNAL(cloudGenerated(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  _ui->_cloudWidget, SLOT(setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);
    this->connect(&_cloudManipulation, SIGNAL(cloudGenerated(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  &_planeFinder, SLOT(setInputCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);
    this->connect(&_planeFinder, SIGNAL(foundPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  _ui->_planeWidget, SLOT(setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);

    _timer.start(40);
}

MainWidget::~MainWidget(void)
{

}

void MainWidget::tick(void)
{
    _cloudManipulation.trigger();
    /*
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    this->computeNormals(cloud, normals);

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

    /* Segment the largest planar component from the remaining cloud *//*
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 1000)
    {
        qDebug() << "Could not estimate a plane model for the given dataset.";
        return;
    }

    /* check model cooefficients *//*
    qDebug() << "model coefficients:";
    qDebug() << "----------------------------------";
    qDebug() << "a = " << coefficients->values[0];
    qDebug() << "b = " << coefficients->values[1];
    qDebug() << "c = " << coefficients->values[2];
    qDebug() << "d = " << coefficients->values[3];

    pcl::ExtractIndices<pcl::Normal> extractNormal;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBL>());
    pcl::PointCloud<pcl::Normal>::Ptr planeNormals(new pcl::PointCloud<pcl::Normal>());
    pcl::ExtractIndices<pcl::PointXYZRGBL> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);

    extractNormal.setInputCloud(normals);
    extractNormal.setIndices(inliers);
    extractNormal.setNegative(false);
    extractNormal.filter(*planeNormals);

//    _ui->_cloudWidget->setCloud(plane);
    return;
    std::vector<pcl::PointIndices> clusters;
    this->regionGrowing(plane, planeNormals, clusters);

    if (clusters.size())
    {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBL>());

        extract.setInputCloud(plane);
        extract.setIndices(pcl::PointIndices::Ptr(new pcl::PointIndices(clusters[1])));
        extract.setNegative(false);
        extract.filter(*tmp);

        _ui->_cloudWidget->setCloud(tmp);
    }
*/
}

void MainWidget::regionGrowing(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals,
                               std::vector<pcl::PointIndices>& clusters)
{
    pcl::RegionGrowing<pcl::PointXYZRGBL, pcl::Normal> reg;
    pcl::search::KdTree<pcl::PointXYZRGBL>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBL>());

    reg.setMinClusterSize(1000);
    reg.setMaxClusterSize(70000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(10);
    reg.setInputCloud(cloud);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(7.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    reg.extract(clusters);
}
