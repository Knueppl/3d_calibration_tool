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
//      _thermoCam("12100076.xml")
{
    _ui->setupUi(this);
    _cloudCatcher.setOperationRange(0.5, 1.0);

    this->connect(&_timer, SIGNAL(timeout()), this, SLOT(tick()));
    this->connect(&_cloudCatcher, SIGNAL(cloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  _ui->_cloudWidget, SLOT(setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);
    this->connect(&_cloudCatcher, SIGNAL(catchedCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  &_planeFinder, SLOT(setInputCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);

    this->connect(&_planeFinder, SIGNAL(foundPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  _ui->_planeWidget, SLOT(setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);
    this->connect(&_planeFinder, SIGNAL(foundAxis(const pcl::PointXYZ&, const pcl::PointXYZ&)),
                  _ui->_planeWidget, SLOT(setLine(const pcl::PointXYZ&, const pcl::PointXYZ&)), Qt::DirectConnection);

    _timer.start(40);
    _thermoView.show();
}

MainWidget::~MainWidget(void)
{

}

void MainWidget::tick(void)
{
    _cloudCatcher.trigger();
//    _thermoCam.grab();
//    _thermoView.setMat(_thermoCam.image());
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
