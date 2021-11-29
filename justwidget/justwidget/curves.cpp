#include "curves.h"
#include "ui_curves.h"

curves::curves(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::curves)
{
    ui->setupUi(this);
}

curves::~curves()
{
    delete ui;
}
