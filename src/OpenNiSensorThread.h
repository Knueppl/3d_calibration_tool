#ifndef __OPEN_NI_SENSOR_THREAD__
#define __OPEN_NI_SENSOR_THREAD__

#include <QThread>
#include <QVector>
#include <QMutex>
#include <QWaitCondition>

#include <vector>
#include <opencv2/opencv.hpp>

class OpenNiDevice;

class OpenNiSensorThread : public QThread
{
    Q_OBJECT

public:
    OpenNiSensorThread(QObject* parent = 0);
    virtual ~OpenNiSensorThread(void);

    const std::vector<cv::Point3f>* coords(void) const { return &_coords[_bank]; }
    const cv::Mat* image(void) const { return &_images[_bank]; }
    const std::vector<float>* z(void) const { return &_zs[_bank]; }
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

    OpenNiDevice* _device;
    QVector<std::vector<cv::Point3f> > _coords;
    QVector<cv::Mat> _images;
    QVector<std::vector<float> > _zs;
    BufferBank _bank;
    QMutex _mutex;
};

#endif
