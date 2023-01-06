#include "QSLAMWidget.h"
#include "ui_slamwidget.h"

#include <QPixmap>
#include "AppConstants.h"

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

void QSLAMWidget::updateTraj(QImage img)
{
    CONSOLE << "Update?";
    ui->trajectory->setPixmap(QPixmap::fromImage(img));
}
