#include "ConfigDialog.h"
#include "ui_ConfigDialog.h"

ConfigDialog::ConfigDialog(QWidget* parent)
    : QDialog(parent),
      _ui(new Ui::ConfigDialog)
{
    _ui->setupUi(this);
    this->connect(_ui->_buttonClose, SIGNAL(clicked()), this, SLOT(accept()));
}

float ConfigDialog::boardHeight(void) const
{
    return _ui->_spinHeight->value();
}

float ConfigDialog::boardWidth(void) const
{
    return _ui->_spinWidth->value();
}

int ConfigDialog::pointsHeight(void) const
{
    return _ui->_spinHeight->value();
}

int ConfigDialog::pointsWidth(void) const
{
    return _ui->_spinWidth->value();
}
