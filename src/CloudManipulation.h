#ifndef __CLOUD_MANIPULATION__
#define __CLOUD_MANIPULATION__

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QColor>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

class OpenNiSensor;

class CloudManipulation : public QThread
{
    Q_OBJECT

public:
    CloudManipulation(QObject* parent = 0);
    virtual ~CloudManipulation(void);

public slots:
    void trigger(void);

signals:
    void cloudGenerated(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud);
    void cloudGreen(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud);
    void cloudRed(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud);

protected:
    virtual void run(void);

private:
    void substractBackground(const cv::Mat& image, cv::Mat& mask);
    void generateCloud(void);
    void paintCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, const QColor& color);

    QMutex _mutex;
    QWaitCondition _triggered;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloudGreen;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloudRed;
    const std::vector<cv::Point3f>* _coords;
    const cv::Mat* _z;
    const cv::Mat* _image;
    OpenNiSensor* _sensor;
};

#endif
