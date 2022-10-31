#include <QFileDialog>

#include "parameters.h"
#include "QConfigDialog.h"
#include "ui_qconfigdialog.h"

QConfigDialog::QConfigDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::QConfigDialog)
{
    ui->setupUi(this);

    connect(ui->choose_config_file, &QPushButton::clicked, this, &QConfigDialog::readSetting);
}

QConfigDialog::~QConfigDialog()
{
    delete ui;
}

void QConfigDialog::display()
{
    this->ui->imu_Num->setText(QString::number(USE_IMU));
    this->ui->cam_Num->setText(QString::number(NUM_OF_CAM));
}

void QConfigDialog::readSetting()
{
    QString config_path = QFileDialog::getOpenFileName(this, "Open a file", "${HOME}");

    this->ui->config_path->setText(config_path);

    readParameters(config_path.toStdString());

    this->display();
}

void QConfigDialog::readSeq()
{

}

void QConfigDialog::readGT()
{

}
