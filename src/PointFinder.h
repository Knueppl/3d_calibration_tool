#ifndef __POINT_FINDER__
#define __POINT_FINDER__

#include <QThread>
#include <QMutex>
#include <QWaitCondition>

#include <opencv2/opencv.hpp>

#include <vector>

class ThermoCam;
class ConfigDialog;
class OpenCvWidget;

class PointFinder : public QThread
{
    Q_OBJECT

public:
    PointFinder(QObject* parent = 0);
    virtual ~PointFinder(void);

    void setConfigDialog(ConfigDialog* dialog);

public slots:
    void trigger(void);

signals:
    void foundPoints(const std::vector<cv::Point2f>& points);
    void image(const cv::Mat& image);

protected:
    void run(void);

private:
    void findPoints(void);

    ThermoCam* _cam;
    ConfigDialog* _dialog;
    QMutex _mutex;
    QWaitCondition _triggered;
    std::vector<cv::Point2f> _points;
    const cv::Mat* _image;
    const cv::Mat* _temperature;
    OpenCvWidget* _matView;
};

#endif
