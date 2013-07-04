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

class CloudWidget : public QVTKWidget
{
    Q_OBJECT

public:
    CloudWidget(QWidget* parent = 0);
    ~CloudWidget(void);

public slots:
    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void setCloud(pcl::PointCloud<pcl::PointXYZRGBL>* cloud);
    void setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud);

private:
    vtkSmartPointer<vtkRenderer>          m_renderer;
    vtkSmartPointer<vtkPolyData>          m_polyData;
    vtkSmartPointer<vtkPoints>            m_points;
    vtkSmartPointer<vtkUnsignedCharArray> m_colors;
};


#endif
