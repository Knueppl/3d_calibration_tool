#ifndef __CLOUD_CATCHER__
#define __CLOUD_CATCHER__

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QColor>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>

#ifdef ___USE_KINECT___
class KinectSensor;
#else
class OpenNiSensor;
#endif

class CloudCatcher : public QThread
{
    Q_OBJECT

public:
    CloudCatcher(QObject* parent = 0);
    virtual ~CloudCatcher(void);

    void setOperationRange(const float min, const float max);

public slots:
    void trigger(void);

signals:
    void cloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud);
    void catchedCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud);

protected:
    virtual void run(void);

private:
    void substractBackground(const cv::Mat& image, cv::Mat& mask);
    void generateCloud(void);
    void paintCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, const QColor& color);

    QMutex _mutex;
    QWaitCondition _triggered;
    pcl::ExtractIndices<pcl::PointXYZRGBL> _extract;
    pcl::PassThrough<pcl::PointXYZRGBL> _filter;
    const std::vector<cv::Point3f>* _coords;
    const cv::Mat* _z;
    const cv::Mat* _image;

#ifdef ___USE_KINECT___
    KinectSensor* _sensor;
#else
    OpenNiSensor* _sensor;
#endif

    float _minDistance;
    float _maxDistance;
};

#endif
