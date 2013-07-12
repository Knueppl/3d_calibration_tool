#ifndef __PLANE_FINDER__
#define __PLANE_FINDER__

#include <QThread>
#include <QMutex>
#include <QWaitCondition>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

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
    void foundAxis(const pcl::PointXYZ& start, const pcl::PointXYZ& end);

protected:
    virtual void run(void);

private:
    void search(void);
    void computeNormals(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
    void copyCloudToMat(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud, cv::Mat& mat);
    void computePoints(const Eigen::Vector3f& mean, const Eigen::Matrix3f& eigenvectors);

    QMutex _mutex;
    QWaitCondition _updated;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _inputCloud;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _planeCloud;
    float _alpha;
    float _beta;
    float _gamma;
    cv::Mat _midPoint;
    ConfigDialog* _dialog;
};

#endif
