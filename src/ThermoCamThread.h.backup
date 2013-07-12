#ifndef __THERMO_CAM_THREAD__
#define __THERMO_CAM_THREAD__

#include <QThread>
#include <QTimer>
#include <QMutex>
#include <QVector>
#include <QWaitCondition>

#include <opencv2/opencv.hpp>

namespace optris {
class PIImager;
}

class ThermoCamThread : public QThread
{
    Q_OBJECT

public:
    ThermoCamThread(const QByteArray& configFile, QObject* parent = 0);
    ~ThermoCamThread(void);

    void setAutoScale(const bool state);
    void setTemperatureRange(const float min, const float max);

    const cv::Mat* image(void) const { return &_image[_bank]; }
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

    void copyToImage(void);
    void switching(void);

    optris::PIImager*   _imager;
    unsigned char*      _bufferRaw;
    QTimer _timer;
    QVector<cv::Mat> _image;
    QMutex _mutex;
    BufferBank _bank;
    QWaitCondition _waitFor;
};

#endif
