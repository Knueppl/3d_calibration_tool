#include "MainWidget.h"
#include "ui_MainWidget.h"

#include <QDebug>

MainWidget::MainWidget(void)
    : QMainWindow(0),
      _ui(new Ui::MainWidget),
      _depthWidget(new OpenCvWidget)
{
    _ui->setupUi(this);
    _depthWidget->show();

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

    _ui->_cloudWidget->setCloud(cloud);
    _depthWidget->setMat(_sensor.z());
}
