#ifndef __MAIN_WIDGET__
#define __MAIN_WIDGET__

#include "CloudCatcher.h"
#include "PlaneFinder.h"
#include "ThermoCam.h"
#include "OpenCvWidget.h"

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
    void addPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, const pcl::PointXYZ& start, const pcl::PointXYZ& end);
    void selectPlane(int index);

private:

    enum State {
        Stop,
        Idle,
        Armed
    };

    void regionGrowing(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals,
                       std::vector<pcl::PointIndices>& clusters);

    Ui::MainWidget* _ui;
    CloudCatcher _cloudCatcher;
    PlaneFinder _planeFinder;

    QVector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> _planes;
    QVector<pcl::PointXYZ> _planeLineStart;
    QVector<pcl::PointXYZ> _planeLineEnd;
    State _state;
    QTimer _timer;
    QTime _time;
    QMutex _mutex;

    OpenCvWidget _thermoView;
    ThermoCam _thermoCam;
};

#endif
