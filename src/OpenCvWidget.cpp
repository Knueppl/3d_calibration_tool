#include "OpenCvWidget.h"

#include <QImage>
#include <QDebug>
#include <QPainter>

#include "opencv2/core/core.hpp"

void OpenCvWidget::setMat(const cv::Mat& mat)
{
    delete m_image;

    // 8-bits unsigned, NO. OF CHANNELS=1
    if(mat.type() == CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i = 0; i < 256; i++)
            colorTable.push_back(qRgb(i, i, i));

        // Copy input Mat
        const uchar *qImageBuffer = static_cast<const uchar*>(mat.data);

        // Create QImage with same dimensions as input Mat
        m_image = new QImage(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
        m_image->setColorTable(colorTable);
    }

    // 8-bits unsigned, NO. OF CHANNELS=3
    else if(mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const uchar *qImageBuffer = static_cast<const uchar*>(mat.data);

        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        m_image = new QImage(img.rgbSwapped());
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return;
    }

    this->setMinimumSize(m_image->size());
    this->update();
}

void OpenCvWidget::paintEvent(QPaintEvent*)
{
    if (!m_image)
        return;

    QPainter painter(this);
    painter.drawImage(this->rect(), *m_image, m_image->rect());
}
