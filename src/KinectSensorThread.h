#ifndef __KINECT_SENSOR_THREAD__
#define __KINECT_SENSOR_THREAD__

#include <QThread>
#include <QVector>
#include <QMutex>

#include <vector>
#include <opencv2/opencv.hpp>

class Kinect;
class QByteArray;

class KinectSensorThread : public QThread
{
    Q_OBJECT

public:
    KinectSensorThread(const QByteArray& configFile, QObject* parent = 0);
    ~KinectSensorThread(void);

    const std::vector<cv::Point3f>* coords(void) const { return &_coords[_bank]; }
    const cv::Mat* image(void) const { return &_images[_bank]; }
    const cv::Mat* z(void) const { return &_zs[_bank]; }
    void switchBank(void);

protected:
    virtual void run(void);

private slots:
    void grab(void);

private:

    enum BufferBank {
        BankA = 0,
        BankB,
        CountBanks
    };

    Kinect* _kinect;
    QVector<std::vector<cv::Point3f> > _coords;
    QVector<cv::Mat> _images;
    QVector<cv::Mat> _zs;
    BufferBank _bank;
    QMutex _mutex;
};

#endif
