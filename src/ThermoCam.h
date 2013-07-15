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
    const cv::Mat& temperature(void) const { return *_temperature; }

private:
    ThermoCamThread* _cam;
    const cv::Mat* _image;
    const cv::Mat* _temperature;
};

#endif
