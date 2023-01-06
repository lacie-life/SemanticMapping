#include "QIntroduction.h"
#include "ui_introduction.h"

#include <QPushButton>

QIntroduction::QIntroduction(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::QIntroduction)
{
    ui->setupUi(this);

    connect(ui->run_action, &QPushButton::clicked, this, &QIntroduction::actionRunClicked);
    connect(ui->system_config, &QPushButton::clicked, this, &QIntroduction::systemConfigOpen);

}

QIntroduction::~QIntroduction()
{
    delete ui;
}

void QIntroduction::actionRunClickedSlot()
{
    emit actionRunClicked();
}

void QIntroduction::systemConfigOpenSlot()
{
    emit systemConfigOpen();
}
