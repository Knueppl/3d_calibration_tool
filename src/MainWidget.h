#ifndef __MAIN_WIDGET__
#define __MAIN_WIDGET__

#include "CloudCatcher.h"
#include "PlaneFinder.h"
#include "OpenCvWidget.h"
#include "ConfigDialog.h"
//#include "PointFinder.h"
#include "ThermoCam.h"

#include <QTimer>
#include <QTime>
#include <QMainWindow>
#include <QColor>
#include <QMutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Ui {
class MainWidget;
}

class MainWidget : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWidget(void);
    ~MainWidget(void);

private slots:
    void tick(void);
    void startStop(void);
    void addPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, const pcl::PointXYZ& start,
                  const pcl::PointXYZ& end, const std::vector<cv::Point3f>& points);
    void selectPlane(int index);
    void acceptPlane(void);
    void disclaimPlane(void);
    void calibrate(void);

private:

    enum State {
        Stop,
        Idle,
        Armed
    };

    void regionGrowing(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals,
                       std::vector<pcl::PointIndices>& clusters);
    void findPoints(std::vector<cv::Point2f>& centers, cv::Mat& image);
    void printCvMat(const cv::Mat& mat) const;

    Ui::MainWidget* _ui;
    ConfigDialog _dialog;
    CloudCatcher _cloudCatcher;
    PlaneFinder _planeFinder;
//    PointFinder _pointFinder;

    QVector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> _planes;
    QVector<pcl::PointXYZ> _planeLineStart;
    QVector<pcl::PointXYZ> _planeLineEnd;
    QVector<std::vector<cv::Point2f> > _points;
    QVector<cv::Mat*> _thermalImages;
    QVector<bool> _valid;
    QVector<std::vector<cv::Point3f> > _planePoints;

    State _state;
    QTimer _timer;
    QTime _time;
    QMutex _mutex;

    ThermoCam _thermoCam;
};

#endif
