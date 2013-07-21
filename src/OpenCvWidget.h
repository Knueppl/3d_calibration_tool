#ifndef __OPEN_CV_LABEL__
#define __OPEN_CV_LABEL__

#include <QWidget>
#include <QMutex>

#include <opencv2/core/core.hpp>

class QImage;

class OpenCvWidget : public QWidget
{
    Q_OBJECT

public:
    OpenCvWidget(QWidget* parent = 0) : QWidget(parent), _image(0) { }
    virtual ~OpenCvWidget(void);

public slots:
    void setMat(const cv::Mat& mat);

protected:
    void paintEvent(QPaintEvent* event);

private:
    QImage* _image;
    QMutex _mutex;
    cv::Mat _mat;
};

#endif
