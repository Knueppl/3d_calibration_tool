#ifndef __OPEN_CV_LABEL__
#define __OPEN_CV_LABEL__

#include <QLabel>

namespace cv {
class Mat;
}

class QImage;

class OpenCvWidget : public QWidget
{
    Q_OBJECT

public:
    OpenCvWidget(QWidget* parent = 0) : QWidget(parent), m_image(0) { }
    ~OpenCvWidget(void) { delete m_image; }

public slots:
    void setMat(const cv::Mat& mat);

protected:
    void paintEvent(QPaintEvent* event);

private:
    QImage* m_image;
};

#endif
