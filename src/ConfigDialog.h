#ifndef __CONFIG_DIALOG__
#define __CONFIG_DIALOG__

#include <QDialog>

namespace Ui {
class ConfigDialog;
}

class ConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConfigDialog(QWidget* parent = 0);

    float boardWidth(void) const;
    float boardHeight(void) const;
    int pointsVer(void) const;
    int pointsHor(void) const;
    float borderTop(void) const;
    float borderLeft(void) const;

private:
    Ui::ConfigDialog* _ui;
};

#endif
