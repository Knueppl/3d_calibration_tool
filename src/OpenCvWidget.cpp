#include "OpenCvWidget.h"

#include <QImage>
#include <QDebug>
#include <QPainter>
#include <QCoreApplication>

OpenCvWidget::~OpenCvWidget(void)
{
    _mutex.lock();
    delete _image;
    _mutex.unlock();
}

void OpenCvWidget::setMat(const cv::Mat& mat)
{
    _mutex.lock();
    delete _image;
    mat.copyTo(_mat);

    // 8-bits unsigned, NO. OF CHANNELS=1
    if(_mat.type() == CV_8UC1)
    {
        // Set the color table (used to translate colour indexes to qRgb values)
        QVector<QRgb> colorTable;
        for (int i = 0; i < 256; i++)
            colorTable.push_back(qRgb(i, i, i));

        // Copy input Mat
        const unsigned char* qImageBuffer = static_cast<const unsigned char*>(_mat.data);

        // Create QImage with same dimensions as input Mat
        _image = new QImage(qImageBuffer, _mat.cols, _mat.rows, _mat.step, QImage::Format_Indexed8);
        _image->setColorTable(colorTable);
    }

    // 8-bits unsigned, NO. OF CHANNELS=3
    else if(_mat.type() == CV_8UC3)
    {
        // Copy input Mat
        const unsigned char* qImageBuffer = static_cast<const unsigned char*>(_mat.data);

        // Create QImage with same dimensions as input Mat
        QImage img(qImageBuffer, _mat.cols, _mat.rows, _mat.step, QImage::Format_RGB888);
        _image = new QImage(img.rgbSwapped());
    }

    // 16-bits depth image
    else if (_mat.type() == CV_16UC1)
    {
        const unsigned char* qImageBuffer = static_cast<const unsigned char*>(_mat.data);
        _image = new QImage(qImageBuffer, _mat.cols, _mat.rows, _mat.step, QImage::Format_RGB16);
    }
    else
    {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        _mutex.unlock();
        return;
    }

    this->setMinimumSize(_image->size());
    QObject* sender = this->sender();

    // if this method is called from a other thread send just a event to queue.
    if (sender && sender->thread() != this->thread())
    {
        QEvent event(QEvent::UpdateRequest);
        QCoreApplication::sendEvent(this, &event);
        _mutex.unlock();
        return;
    }

    this->update();
    _mutex.unlock();
}

void OpenCvWidget::paintEvent(QPaintEvent*)
{
    _mutex.lock();

    if (!_image)
    {
        _mutex.unlock();
        return;
    }

    QPainter painter(this);
    painter.drawImage(this->rect(), *_image, _image->rect());

    _mutex.unlock();
}
