#include "ConfigDialog.h"
#include "ui_ConfigDialog.h"

ConfigDialog::ConfigDialog(QWidget* parent)
    : QDialog(parent),
      _ui(new Ui::ConfigDialog)
{
    _ui->setupUi(this);
    this->connect(_ui->_buttonClose, SIGNAL(clicked()), this, SLOT(accept()));
    this->connect(_ui->_buttonGenerate, SIGNAL(clicked()), this, SLOT(generate()));
}

void ConfigDialog::generate(void)
{
    emit this->generateCalibrationBoard();
}

float ConfigDialog::boardHeight(void) const
{
    return _ui->_spinHeight->value();
}

float ConfigDialog::boardWidth(void) const
{
    return _ui->_spinWidth->value();
}

int ConfigDialog::pointsHor(void) const
{
    return _ui->_spinPointsWidth->value();
}

int ConfigDialog::pointsVer(void) const
{
    return _ui->_spinPointsHeight->value();
}

float ConfigDialog::borderTop(void) const
{
    return _ui->_spinBorderTop->value();
}

float ConfigDialog::borderLeft(void) const
{
    return _ui->_spinBorderLeft->value();
}

float ConfigDialog::temperatureMin(void) const
{
    return _ui->_spinTempMin->value();
}

float ConfigDialog::temperatureMax(void) const
{
    return _ui->_spinTempMax->value();
}

bool ConfigDialog::debugThermo(void) const
{
    return _ui->_checkDebugThermo->isChecked();
}

float ConfigDialog::spaceHor(void) const
{
    return _ui->_spinSpaceHor->value();
}

float ConfigDialog::spaceVer(void) const
{
    return _ui->_spinSpaceVer->value();
}

float ConfigDialog::sensorSolution(void) const
{
    return _ui->_spinSensorSolution->value();
}

float ConfigDialog::a(void) const
{
    return _ui->_spinA->value();
}

float ConfigDialog::b(void) const
{
    return _ui->_spinB->value();
}

float ConfigDialog::c(void) const
{
    return _ui->_spinC->value();
}
