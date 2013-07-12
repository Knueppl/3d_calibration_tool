#ifndef __CLOUD_WIDGET__
#define __CLOUD_WIDGET__

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <QVTKWidget.h>
#include <vtkSmartPointer.h>

class vtkRenderer;
class vtkPolyData;
class vtkPoints;
class vtkUnsignedCharArray;
class vtkLineSource;

class CloudWidget : public QVTKWidget
{
    Q_OBJECT

public:
    CloudWidget(QWidget* parent = 0);
    ~CloudWidget(void);

public slots:
    void setCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    void setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud);
    void setLine(const pcl::PointXYZ& start, const pcl::PointXYZ& end);

private:
    vtkSmartPointer<vtkRenderer>          _renderer;
    vtkSmartPointer<vtkPolyData>          _polyData;
    vtkSmartPointer<vtkLineSource>        _lineSource;
    vtkSmartPointer<vtkPoints>            _points;
    vtkSmartPointer<vtkUnsignedCharArray> _colors;
};


#endif
