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
    _cloudCatcher.setOperationRange(0.5, 1.5);
    _planeFinder.setConfigDialog(&_dialog);
//    _pointFinder.setConfigDialog(&_dialog);

    QMenuBar* menuBar = this->menuBar();
    menuBar->addAction("Config Dialog", &_dialog, SLOT(show()));

    this->connect(&_timer, SIGNAL(timeout()), this, SLOT(tick()));
    this->connect(&_cloudCatcher, SIGNAL(cloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  _ui->_cloudWidget, SLOT(setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);
    this->connect(&_cloudCatcher, SIGNAL(catchedCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)),
                  &_planeFinder, SLOT(setInputCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr)), Qt::DirectConnection);

    this->connect(&_planeFinder, SIGNAL(foundPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr, const pcl::PointXYZ&, const pcl::PointXYZ&, const std::vector<cv::Point3f>&)),
                  this, SLOT(addPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr, const pcl::PointXYZ&, const pcl::PointXYZ&, const std::vector<cv::Point3f>&)), Qt::DirectConnection);
//    this->connect(&_pointFinder, SIGNAL(image(const cv::Mat&)), &_thermoView, SLOT(setMat(const cv::Mat&)), Qt::DirectConnection);

    this->connect(_ui->_buttonStartStop, SIGNAL(clicked()), this, SLOT(startStop()));
    _ui->_buttonStartStop->setText("Start");
    this->connect(_ui->_buttonCali, SIGNAL(clicked()), this, SLOT(calibrate()));
    this->connect(_ui->_buttonAccept, SIGNAL(clicked()), this, SLOT(acceptPlane()));
    this->connect(_ui->_buttonDisclaim, SIGNAL(clicked()), this, SLOT(disclaimPlane()));

    this->connect(_ui->_listPlanes, SIGNAL(currentRowChanged(int)), this, SLOT(selectPlane(int)));
    this->connect(&_dialog, SIGNAL(generateCalibrationBoard()), &_planeFinder, SLOT(generateCalibrationBoard()));

    _timer.start(40);
}

MainWidget::~MainWidget(void)
{

}

void MainWidget::tick(void)
{
    _cloudCatcher.trigger();
//    _pointFinder.trigger();
    _thermoCam.grab();

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

void MainWidget::addPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, const pcl::PointXYZ& start, const pcl::PointXYZ& end, const std::vector<cv::Point3f>& points)
{
    _mutex.lock();
    _ui->_planeWidget->setCloud(cloud);

    if (_state != Armed)
    {
        _mutex.unlock();
        return;
    }

    /* hack */
    std::vector<cv::Point2f> centers;
    cv::Mat* image = new cv::Mat;
    this->findPoints(centers, *image);

    if (!centers.size())
    {
        qDebug() << "No points found in thermal image.";
        delete image;
        _mutex.unlock();
        return;
    }

    _points.push_back(centers);
    _thermalImages.push_back(image);

    _planes.push_back(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>(*cloud)));
    _planeLineStart.push_back(start);
    _planeLineEnd.push_back(end);
    _valid.push_back(false);
    _planePoints.push_back(points);

    QListWidgetItem* item = new QListWidgetItem(QString("Plane ").append(QString::number(_ui->_listPlanes->count() + 1)));
    item->setIcon(QIcon(":/picture/DeleteRed.png"));
    _ui->_listPlanes->addItem(item);

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
    _ui->_thermoView->setMat(*_thermalImages[index]);
    _mutex.unlock();
}

void MainWidget::findPoints(std::vector<cv::Point2f>& centers, cv::Mat& image)
{
    const cv::Mat& temperature = _thermoCam.temperature();
    const unsigned short tempMin = static_cast<unsigned short>(_dialog.temperatureMin() * 10);
    const unsigned short tempMax = static_cast<unsigned short>(_dialog.temperatureMax() * 10);

    cv::Mat tempImage(temperature.rows, temperature.cols, CV_8UC1);

    for (int row = 0; row < temperature.rows; row++)
    {
        const uint16_t* dataTemperature = reinterpret_cast<const uint16_t*>(temperature.ptr(row));
        unsigned char* dataTempImage = tempImage.ptr(row);

        for (int col = 0; col < temperature.cols; col++, dataTemperature++)
        {
            const unsigned short temp = *dataTemperature - 1000;

            if (temp > tempMax)
                *dataTempImage++ = 0xff;
            else if (temp < tempMin)
                *dataTempImage++ = 0xff;
            else
                *dataTempImage++ = 0x00;
        }
    }

//    _matView.setVisible(_dialog.debugThermo());
    _ui->_matView->setMat(tempImage);

    const cv::Size patternSize(_dialog.pointsHor(), _dialog.pointsVer());
    _thermoCam.image().copyTo(image);

    if (cv::findCirclesGrid(tempImage, patternSize, centers, cv::CALIB_CB_SYMMETRIC_GRID))
        cv::drawChessboardCorners(image, patternSize, cv::Mat(centers), true);
    else
        centers.clear();

    _ui->_thermoView->setMat(image);
}

void MainWidget::acceptPlane(void)
{
    const int row = _ui->_listPlanes->currentRow();

    if (row < 0)
        return;

    _valid[row] = true;
    _ui->_listPlanes->item(row)->setIcon(QIcon(":/picture/AcceptGreen.png"));

}

void MainWidget::disclaimPlane(void)
{
    const int row = _ui->_listPlanes->currentRow();

    if (row < 0)
        return;

    _valid[row] = false;
    _ui->_listPlanes->item(row)->setIcon(QIcon(":/picture/DeleteRed.png"));
}

void MainWidget::calibrate(void)
{
    if (!_valid.size())
        return;

    std::vector<std::vector<cv::Point3f> > corners(1);
    const float dh = (_dialog.boardWidth() - 2 * _dialog.borderLeft()) / _dialog.pointsVer();
    const float dv = (_dialog.boardHeight() - 2 * _dialog.borderTop()) / _dialog.pointsHor();

    for (int v = 0; v < _dialog.pointsVer(); v++)
        for (int h = 0; h < _dialog.pointsHor(); h++)
            corners[0].push_back(cv::Point3f(static_cast<float>(v) * dv, static_cast<float>(h) * dh, 0.0));

    std::vector<std::vector<cv::Point2f> > points;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> planes;
    std::vector<cv::Point3f> planePoints;
    std::vector<cv::Point2f> imagePoints;

    for (int i = 0; i < _planes.size(); i++)
    {
        if (!_valid[i])
            continue;

        points.push_back(_points[i]);
        planes.push_back(_planes[i]);
        planePoints.insert(planePoints.end(), _planePoints[i].begin(), _planePoints[i].end());
        imagePoints.insert(imagePoints.end(), _points[i].begin(), _points[i].end());
    }

    if (!points.size())
    {
        qDebug() << "No valid frames.";
        return;
    }

    corners.resize(points.size(), corners[0]);
    cv::Mat intrinsic(3, 3, CV_64F);
    cv::Mat distortion(1, 8, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    qDebug() << "points size = " << points.size();

    qDebug() << "rms error intrinsic: " << cv::calibrateCamera(corners, points, _thermalImages[0]->size(), intrinsic,
                                                               distortion, rvecs, tvecs);

    std::cout << "object points: " << std::endl << planePoints << std::endl;

    cv::Mat r, t;
    cv::solvePnP(planePoints, imagePoints, intrinsic, distortion, r, t);

    std::cout << "r:" << std::endl << r << std::endl;
    std::cout << "t:" << std::endl << t << std::endl;
}

void MainWidget::printCvMat(const cv::Mat& mat) const
{
    for (int row = 0; row < mat.rows; row++)
    {
        for (int col = 0; col < mat.cols; col++)
        {
            std::cout << mat.at<double>(row, col) << ", ";
        }

        std::cout << std::endl;
    }
}
