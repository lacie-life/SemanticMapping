#include <QFileDialog>
#include <QDebug>

#include "parameters.h"
#include "QConfigDialog.h"
#include "ui_qconfigdialog.h"

#include "AppConstants.h"

QConfigDialog::QConfigDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::QConfigDialog)
{
    ui->setupUi(this);

    connect(ui->choose_config_file, &QPushButton::clicked, this, &QConfigDialog::readSetting);
    connect(ui->choose_dataset, &QPushButton::clicked, this, &QConfigDialog::readSeq);
    connect(ui->choose_gt, &QPushButton::clicked, this, &QConfigDialog::readGT);
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
    QString seq_path = QFileDialog::getOpenFileName(this, "Open a file", "${HOME}");
    this->ui->dataset_path->setText(seq_path);
    m_seqPath = seq_path;

    if(USE_IMU)
    {
        m_IMUPath = m_seqPath + "imu0/data.csv";
    }
    if (NUM_OF_CAM == 1)
    {
        m_ImgPath = m_seqPath + "cam0/data";
        m_AssoPath = m_seqPath + "timestamp/data/time.txt";
    }
    if(NUM_OF_CAM == 2)
    {
        m_LeftImgPath = m_seqPath + "cam0/data";
        m_RightImgPath = m_seqPath + "cam1/data";
        m_AssoPath = m_seqPath + "timestamp/data/time.txt";
    }

    if (USE_IMU == 1 && NUM_OF_CAM == 1)
    {

    }


}

void QConfigDialog::readGT()
{
    QString gt_path = QFileDialog::getOpenFileName(this, "Open a file", "${HOME}");
    this->ui->dataset_path->setText(gt_path);
    m_GTPath = gt_path;
}
