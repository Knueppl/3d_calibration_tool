#include "MainWidget.h"
#include "ui_MainWidget.h"

#include <QDebug>

#include <algorithm>

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

    _ui->_cloudWidget->setCloud(cloud);
    _depthWidget->setMat(_sensor.z());

    cv::Mat z(_sensor.z().rows, _sensor.z().cols, CV_8UC1);
    std::vector<std::vector<cv::Point> > contours;
//    const uint16_t max = *std::max_element(z.begin<uint16_t>(), z.end<uint16_t>());
//    const uint16_t min = *std::min_element(z.begin<uint16_t>(), z.end<uint16_t>());
    unsigned char* des = z.data;
    const uint16_t* src = reinterpret_cast<uint16_t*>(_sensor.z().data);
    const unsigned int size = z.rows * z.cols;

    for (unsigned int i = 0; i < size; i++, des++, src++)
        *des = static_cast<unsigned char>((*src >> 6) & 0xf);

    cv::Canny(z, z, 30, 90, 5);
    cv::findContours(z, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cv::Scalar color(255, 0, 0);
    cv::drawContours(z, contours, -1, color, 10);
    _testWidget->setMat(z);
}
