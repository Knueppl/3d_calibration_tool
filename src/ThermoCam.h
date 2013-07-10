#ifndef __THERMO_CAM__
#define __THERMO_CAM__

#include <opencv2/opencv.hpp>

class ThermoCamThread;
class QByteArray;

class ThermoCam
{
public:
    ThermoCam(const QByteArray& configFile);
    ~ThermoCam(void);

    void grab(void);
    const cv::Mat& image(void) const { return *_image; }

private:
    ThermoCamThread* _cam;
    const cv::Mat* _image;
};

#endif
