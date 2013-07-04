#ifndef __MAIN_WIDGET__
#define __MAIN_WIDGET__

#include "OpenNiSensor.h"
#include "OpenCvWidget.h"

#include <QTimer>
#include <QMainWindow>
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
    void computeNormals(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals);
    void regionGrowing(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals,
                       std::vector<pcl::PointIndices>& clusters);

    Ui::MainWidget* _ui;
    QTimer _timer;
    OpenNiSensor _sensor;
    OpenCvWidget* _depthWidget;
    OpenCvWidget* _testWidget;
};

#endif
