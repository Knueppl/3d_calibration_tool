#include "CloudWidget.h"

#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkLineSource.h>
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkActor.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkRenderWindowInteractor.h"
#include <vtkCamera.h>

#include <QDebug>

CloudWidget::CloudWidget(QWidget* parent)
    : QVTKWidget(parent),
      _renderer(vtkRenderer::New()),
      _polyData(vtkSmartPointer<vtkPolyData>::New()),
      _lineSource(vtkSmartPointer<vtkLineSource>::New()),
      _points(vtkSmartPointer<vtkPoints>::New()),
      _colors(vtkSmartPointer<vtkUnsignedCharArray>::New())
{
    _renderer->SetBackground(.0, .0, .0);
    _renderer->GetActiveCamera()->Yaw(0);
    this->GetRenderWindow()->AddRenderer(_renderer);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(_polyData->GetProducerPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(1);

    _renderer->AddActor(actor);

    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(_lineSource->GetOutputPort());
    actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetLineWidth(4);

    _renderer->AddActor(actor);
    _renderer->ResetCamera();
}

CloudWidget::~CloudWidget(void)
{
    // Maybe i should delete m_polyData, m_points, ...
}

void CloudWidget::setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud)
{
    _polyData->Reset();
    _points->Reset();
    _colors->Reset();
    _colors->SetNumberOfComponents(3);

    for (pcl::PointCloud<pcl::PointXYZRGBL>::const_iterator point = cloud->begin(); point < cloud->end(); ++point)
    {
        unsigned char temp[3];

        _points->InsertNextPoint(point->x, point->y, point->z);
        temp[0] = point->r;
        temp[1] = point->g;
        temp[2] = point->b;
        _colors->InsertNextTupleValue(temp);
    }

    _polyData->GetPointData()->SetNormals(NULL);
    _polyData->SetPoints(_points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputConnection(_polyData->GetProducerPort());
    glyphFilter->Update();

    _polyData->ShallowCopy(glyphFilter->GetOutput());
    _polyData->GetPointData()->SetScalars(_colors);
    _points->Modified();
    this->update();
}

void CloudWidget::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    _polyData->Reset();
    _points->Reset();
    _colors->Reset();
    _colors->SetNumberOfComponents(3);

    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator point(cloud->begin()); point < cloud->end(); ++point)
    {
        unsigned char temp[3];

        _points->InsertNextPoint(point->x, point->y, point->z);
        temp[0] = point->r;
        temp[1] = point->g;
        temp[2] = point->b;
        _colors->InsertNextTupleValue(temp);
    }

    _polyData->GetPointData()->SetNormals(NULL);
    _polyData->SetPoints(_points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputConnection(_polyData->GetProducerPort());
    glyphFilter->Update();

    _polyData->ShallowCopy(glyphFilter->GetOutput());
    _polyData->GetPointData()->SetScalars(_colors);
    _points->Modified();
    this->update();
}

void CloudWidget::setCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    _polyData->Reset();
    _points->Reset();
    _colors->Reset();
    _colors->SetNumberOfComponents(3);

    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator point(cloud->begin()); point < cloud->end(); ++point)
    {
        unsigned char temp[3];

        _points->InsertNextPoint(point->x, point->y, point->z);
        temp[0] = 0xff;
        temp[1] = 0xff;
        temp[2] = 0xff;
        _colors->InsertNextTupleValue(temp);
    }

    _polyData->GetPointData()->SetNormals(NULL);
    _polyData->SetPoints(_points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputConnection(_polyData->GetProducerPort());
    glyphFilter->Update();

    _polyData->ShallowCopy(glyphFilter->GetOutput());
    _polyData->GetPointData()->SetScalars(_colors);
    _points->Modified();
    this->update();
}

void CloudWidget::setLine(const pcl::PointXYZ& start, const pcl::PointXYZ& end)
{
    double p0[3] = { start.x, start.y, start.z };
    double p1[3] = { end.x, end.y, end.z };

    _lineSource->SetPoint1(p0);
    _lineSource->SetPoint2(p1);
    _lineSource->Update();
}
