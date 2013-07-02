#include "MainWidget.h"
#include "ui_MainWidget.h"

#include <QDebug>

MainWidget::MainWidget(void)
    : QMainWindow(0),
      _ui(new Ui::MainWidget)
{
    _ui->setupUi(this);

    this->connect(&_timer, SIGNAL(timeout()), this, SLOT(tick()));

    _timer.start(40);
}

void MainWidget::tick(void)
{
    _sensor.grab();
    _ui->_cvWidget->setMat(_sensor.image());

    const std::vector<cv::Point3f>& coords(_sensor.coords());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->resize(coords.size());

    std::vector<cv::Point3f>::const_iterator coord(coords.begin());
    pcl::PointCloud<pcl::PointXYZ>::iterator point(cloud->begin());

    while (coord < coords.end())
    {
        point->x = coord->x;
        point->y = coord->y;
        point->z = coord->z;

        ++point;
        ++coord;
    }

    _ui->_cloudWidget->setCloud(cloud);
}
