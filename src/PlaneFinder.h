#ifndef __PLANE_FINDER__
#define __PLANE_FINDER__

#include <QThread>
#include <QMutex>
#include <QWaitCondition>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ConfigDialog;

class PlaneFinder : public QThread
{
    Q_OBJECT

public:
    PlaneFinder(QObject* parent = 0);
    virtual ~PlaneFinder(void);

public slots:
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud);

signals:
    void foundPlane(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud);

protected:
    virtual void run(void);

private:
    void search(void);
    void computeNormals(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    QMutex _mutex;
    QWaitCondition _updated;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _inputCloud;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _planeCloud;
    float _alpha;
    float _beta;
    float _gamma;
    pcl::PointXYZ _midPoint;
    ConfigDialog* _dialog;
};

#endif
