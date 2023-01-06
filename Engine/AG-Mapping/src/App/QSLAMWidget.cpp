#include "QSLAMWidget.h"
#include "ui_slamwidget.h"

QSLAMWidget::QSLAMWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::QSLAMWidget)
{
    ui->setupUi(this);
}

QSLAMWidget::~QSLAMWidget()
{
    delete ui;
}
