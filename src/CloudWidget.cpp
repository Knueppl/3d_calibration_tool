#include "CloudWidget.h"

#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
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
      m_renderer(vtkRenderer::New()),
      m_polyData(vtkSmartPointer<vtkPolyData>::New()),
      m_points(vtkSmartPointer<vtkPoints>::New()),
      m_colors(vtkSmartPointer<vtkUnsignedCharArray>::New())
{
    m_renderer->SetBackground(.0, .0, .0);
    m_renderer->GetActiveCamera()->Yaw(180);
    this->GetRenderWindow()->AddRenderer(m_renderer);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(m_polyData->GetProducerPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(1);

    m_renderer->AddActor(actor);
    m_renderer->ResetCamera();
}

CloudWidget::~CloudWidget(void)
{
    // Maybe i should delete m_polyData, m_points, ...
}

void CloudWidget::setCloud(pcl::PointCloud<pcl::PointXYZRGBL>* cloud)
{
    m_polyData->Reset();
    m_points->Reset();
    m_colors->Reset();
    m_colors->SetNumberOfComponents(3);

    for (pcl::PointCloud<pcl::PointXYZRGBL>::iterator it = cloud->begin(); it < cloud->end(); ++it)
    {
        unsigned char temp[3];

        m_points->InsertNextPoint((*it).x, (*it).y, -(*it).z);
        temp[0] = it->r;
        temp[1] = it->g;
        temp[2] = it->b;
        m_colors->InsertNextTupleValue(temp);
    }

    m_polyData->GetPointData()->SetNormals(NULL);
    m_polyData->SetPoints(m_points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputConnection(m_polyData->GetProducerPort());
    glyphFilter->Update();

    m_polyData->ShallowCopy(glyphFilter->GetOutput());
    m_polyData->GetPointData()->SetScalars(m_colors);
    m_points->Modified();
    this->update();
}

void CloudWidget::setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud)
{
    m_polyData->Reset();
    m_points->Reset();
    m_colors->Reset();
    m_colors->SetNumberOfComponents(3);

    for (pcl::PointCloud<pcl::PointXYZRGBL>::iterator it = cloud->begin(); it < cloud->end(); ++it)
    {
        unsigned char temp[3];

        m_points->InsertNextPoint((*it).x, (*it).y, -(*it).z);
        temp[0] = it->r;
        temp[1] = it->g;
        temp[2] = it->b;
        m_colors->InsertNextTupleValue(temp);
    }

    m_polyData->GetPointData()->SetNormals(NULL);
    m_polyData->SetPoints(m_points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputConnection(m_polyData->GetProducerPort());
    glyphFilter->Update();

    m_polyData->ShallowCopy(glyphFilter->GetOutput());
    m_polyData->GetPointData()->SetScalars(m_colors);
    m_points->Modified();
    this->update();
}

void CloudWidget::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    m_polyData->Reset();
    m_points->Reset();
    m_colors->Reset();
    m_colors->SetNumberOfComponents(3);

    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator point(cloud->begin()); point < cloud->end(); ++point)
    {
        unsigned char temp[3];

        m_points->InsertNextPoint(point->x, point->y, point->z);
        temp[0] = point->r;
        temp[1] = point->g;
        temp[2] = point->b;
        m_colors->InsertNextTupleValue(temp);
    }

    m_polyData->GetPointData()->SetNormals(NULL);
    m_polyData->SetPoints(m_points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputConnection(m_polyData->GetProducerPort());
    glyphFilter->Update();

    m_polyData->ShallowCopy(glyphFilter->GetOutput());
    m_polyData->GetPointData()->SetScalars(m_colors);
    m_points->Modified();
    this->update();
}

void CloudWidget::setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    m_polyData->Reset();
    m_points->Reset();
    m_colors->Reset();
    m_colors->SetNumberOfComponents(3);

    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator point(cloud->begin()); point < cloud->end(); ++point)
    {
        unsigned char temp[3];

        m_points->InsertNextPoint(point->x, point->y, point->z);
        temp[0] = 0xff;
        temp[1] = 0xff;
        temp[2] = 0xff;
        m_colors->InsertNextTupleValue(temp);
    }

    m_polyData->GetPointData()->SetNormals(NULL);
    m_polyData->SetPoints(m_points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputConnection(m_polyData->GetProducerPort());
    glyphFilter->Update();

    m_polyData->ShallowCopy(glyphFilter->GetOutput());
    m_polyData->GetPointData()->SetScalars(m_colors);
    m_points->Modified();
    this->update();
}
