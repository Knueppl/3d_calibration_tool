#ifndef __MAIN_WIDGET__
#define __MAIN_WIDGET__

#include "OpenNiSensor.h"
#include "OpenCvWidget.h"

#include <QTimer>
#include <QMainWindow>

namespace Ui {
class MainWidget;
}

class MainWidget : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWidget(void);
    ~MainWidget(void);

private slots:
    void tick(void);

private:
    Ui::MainWidget* _ui;
    QTimer _timer;
    OpenNiSensor _sensor;
    OpenCvWidget* _depthWidget;
    OpenCvWidget* _testWidget;
};

#endif
