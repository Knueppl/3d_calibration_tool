#ifndef __MAIN_WIDGET__
#define __MAIN_WIDGET__

#include "CloudCatcher.h"
#include "PlaneFinder.h"
#include "ThermoCam.h"
#include "OpenCvWidget.h"

#include <QTimer>
#include <QMainWindow>
#include <QColor>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

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

private:
    void regionGrowing(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals,
                       std::vector<pcl::PointIndices>& clusters);

    Ui::MainWidget* _ui;
    QTimer _timer;
    CloudCatcher _cloudCatcher;
    PlaneFinder _planeFinder;

    OpenCvWidget _thermoView;
//    ThermoCam _thermoCam;
};

#endif
