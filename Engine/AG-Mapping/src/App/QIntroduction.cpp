#include "QIntroduction.h"
#include "ui_introduction.h"

QIntroduction::QIntroduction(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::QIntroduction)
{
    ui->setupUi(this);
}

QIntroduction::~QIntroduction()
{
    delete ui;
}
