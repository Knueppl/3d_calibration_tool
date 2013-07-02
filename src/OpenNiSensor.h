#ifndef __OPEN_NI_SENSOR__
#define __OPEN_NI_SENSOR__

#include <opencv2/opencv.hpp>
#include <vector>

class OpenNiSensorThread;
class QByteArray;

class OpenNiSensor
{
public:
    OpenNiSensor(void);
    ~OpenNiSensor(void);

    void grab(void);
    const std::vector<cv::Point3f>& coords(void) const { return *_coords; }
    const cv::Mat& image(void) const { return *_image; }

private:
    OpenNiSensorThread* _device;
    const std::vector<cv::Point3f>* _coords;
    const cv::Mat* _image;
};

#endif
