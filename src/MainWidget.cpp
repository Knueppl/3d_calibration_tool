#include "MainWidget.h"
#include "ui_MainWidget.h"

#include <cmath>
#include <algorithm>

#include <QDebug>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

namespace {
const int TIME_S = 2;
}

MainWidget::MainWidget(void)
    : QMainWindow(0),
      _ui(new Ui::MainWidget),
      _state(Stop),
      _thermoCam("12100076.xml")
{
    _ui->setupUi(this);
    _cloudCatcher.setOperationRange(1.5, 2.5);
    _planeFinder.setConfigDialog(&_dialog);

    QMenuBar* menuBar = this->menuBar();
    menuBar->addAction("Config Dialog", &_dialog, SLOT(show()));

    this->connect(&_timer, SIGNAL(timeout()), this, SLOT(tick()));
    this->connect(&_cloudCatcher, SIGNAL(cloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  _ui->_cloudWidget, SLOT(setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);
    this->connect(&_cloudCatcher, SIGNAL(catchedCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  &_planeFinder, SLOT(setInputCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);

    this->connect(&_planeFinder, SIGNAL(foundPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr, const pcl::PointXYZ&, const pcl::PointXYZ&)),
                  this, SLOT(addPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr, const pcl::PointXYZ&, const pcl::PointXYZ&)), Qt::DirectConnection);

    this->connect(_ui->_buttonStartStop, SIGNAL(clicked()), this, SLOT(startStop()));
    _ui->_buttonStartStop->setText("Start");

    this->connect(_ui->_listPlanes, SIGNAL(currentRowChanged(int)), this, SLOT(selectPlane(int)));

    _timer.start(40);
    _thermoView.show();
    _testView.show();
}

MainWidget::~MainWidget(void)
{

}

void MainWidget::tick(void)
{
    _cloudCatcher.trigger();
    _thermoCam.grab();
//    _thermoView.setMat(_thermoCam.image());

    const unsigned short tempMin = static_cast<unsigned short>(_dialog.temperatureMin() * 10);
    const unsigned short tempMax = static_cast<unsigned short>(_dialog.temperatureMax() * 10);
    const float factor = 255.0f / (_dialog.temperatureMax() - _dialog.temperatureMin());
    const cv::Mat& temperature = _thermoCam.temperature();
    cv::Mat tempImage(temperature.rows, temperature.cols, CV_8UC1);

    for (int row = 0; row < temperature.rows; row++)
    {
        const uint16_t* dataTemperature = reinterpret_cast<const uint16_t*>(temperature.ptr(row));
        unsigned char* dataTempImage = tempImage.ptr(row);

        for (int col = 0; col < temperature.cols; col++, dataTemperature++)
        {
            const unsigned short temp = *dataTemperature - 1000;

            if (temp > tempMax)
                *dataTempImage++ = 0x00;
            else if (temp < tempMin)
                *dataTempImage++ = 0xff;
            else
                *dataTempImage++ = 0xff - static_cast<unsigned char>(static_cast<float>((*dataTemperature - tempMin) & 0xff) * factor);
        }
    }

    _thermoView.setMat(tempImage);

    std::vector<cv::Point2f> centers;
    cv::findCirclesGrid(tempImage, cv::Size(7, 5), centers, cv::CALIB_CB_SYMMETRIC_GRID);

    cv::Mat image;
    _thermoCam.image().copyTo(image);
    cv::drawChessboardCorners(image, cv::Size(7, 5), cv::Mat(centers), true);
    _testView.setMat(image);

    _mutex.lock();

    switch (_state)
    {
    case Stop:
        break;

    case Idle:
        {
            const int diff = QTime::currentTime().msecsTo(_time);
            QTime time = QTime().addMSecs(diff);

            if (diff < 0)
            {
                _ui->_labelTimer->setText("Armed");
                _state = Armed;
            }
            else
            {
                _ui->_labelTimer->setText(time.toString("s:zzz"));
            }

            break;
        }

    default:
        break;
    }

    _mutex.unlock();
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

void MainWidget::startStop(void)
{
    _mutex.lock();

    if (_state == Stop)
    {
        _time = QTime::currentTime().addSecs(TIME_S);
        _ui->_labelTimer->setText(QTime(0, 0, TIME_S).toString("s:zzz"));
        _ui->_buttonStartStop->setText("Stop");
        _state = Idle;
    }
    else
    {
        _ui->_labelTimer->setText(QTime(0, 0, TIME_S).toString("s:zzz"));
        _ui->_buttonStartStop->setText("Start");
        _state = Stop;
    }

    _mutex.unlock();
}

void MainWidget::addPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, const pcl::PointXYZ& start, const pcl::PointXYZ& end)
{
    _mutex.lock();

    if (_state != Armed)
    {
        _mutex.unlock();
        return;
    }

    _planes.push_back(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>(*cloud)));
    _planeLineStart.push_back(start);
    _planeLineEnd.push_back(end);
    _ui->_listPlanes->addItem(QString("Plane ").append(QString::number(_ui->_listPlanes->count() + 1)));

    _state = Stop;
    _mutex.unlock();
    this->startStop();
}

void MainWidget::selectPlane(int index)
{
    if (index < 0)
        return;

    _mutex.lock();
    _ui->_planeWidget->setCloud(_planes[index]);
    _ui->_planeWidget->setLine(_planeLineStart[index], _planeLineEnd[index]);
    _mutex.unlock();
}
