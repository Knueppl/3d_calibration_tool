#ifndef __KINECT_SENSOR__
#define __KINECT_SENSOR__

#include <opencv2/opencv.hpp>
#include <vector>

class KinectSensorThread;
class QByteArray;

class KinectSensor
{
public:
    KinectSensor(const QByteArray& configFile);
    ~KinectSensor(void);

    void grab(void);
    const std::vector<cv::Point3f>& coords(void) const { return *_coords; }
    const cv::Mat& image(void) const { return *_image; }
    const cv::Mat& z(void) const { return *_z; }

private:
    KinectSensorThread* _kinect;
    const std::vector<cv::Point3f>* _coords;
    const cv::Mat* _image;
    const cv::Mat* _z;
};

#endif
