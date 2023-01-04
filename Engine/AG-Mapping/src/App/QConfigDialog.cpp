#include <QFileDialog>
#include <QDebug>

#include "parameters.h"
#include "QConfigDialog.h"
#include "ui_qconfigdialog.h"

#include "AppConstants.h"

QConfigDialog::QConfigDialog(QWidget *parent, AppModel *model) :
    QDialog(parent),
    ui(new Ui::QConfigDialog)
{
    ui->setupUi(this);

    m_model = model;

    connect(ui->choose_config_file, &QPushButton::clicked, this, &QConfigDialog::readSetting);
    connect(ui->choose_dataset, &QPushButton::clicked, this, &QConfigDialog::readSeq);
    connect(ui->choose_gt, &QPushButton::clicked, this, &QConfigDialog::readGT);
    connect(ui->cancel_config_button, &QPushButton::clicked, this, &QConfigDialog::configReset);
    connect(ui->close_config, &QPushButton::clicked, this, &QConfigDialog::configClose);
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

    m_model->m_slamSettingPath = config_path;

    readParameters(config_path.toStdString());


    if (USE_IMU == 1 && NUM_OF_CAM == 1)
    {
        m_model->set_slam_type(AppEnums::VSLAM_TYPE::MONO_IMU);
    }
    else if (USE_IMU == 0 && NUM_OF_CAM == 1)
    {
        m_model->set_slam_type(AppEnums::VSLAM_TYPE::MONO);
    }
    else if (USE_IMU == 0 && NUM_OF_CAM == 2)
    {
        m_model->set_slam_type(AppEnums::VSLAM_TYPE::STEREO);
    }
    else if (USE_IMU == 1 && NUM_OF_CAM == 2)
    {
        m_model->set_slam_type(AppEnums::VSLAM_TYPE::STEREO_IMU);
    }

    CONSOLE << m_model->get_slam_type();

    this->display();
}

void QConfigDialog::readSeq()
{
    QString seq_path = QFileDialog::getExistingDirectory(this, "Open a file", "${HOME}");
    this->ui->dataset_path->setText(seq_path);
    m_model->m_seqPath = seq_path;

    if(USE_IMU)
    {
        m_model->m_IMUPath = m_model->m_seqPath + "/imu0/data.csv";
        CONSOLE << m_model->m_IMUPath;
    }
    if (NUM_OF_CAM == 1)
    {
        m_model->m_ImgPath = m_model->m_seqPath + "/cam0/data";
        m_model->m_AssoPath = m_model->m_seqPath + "/timestamp/data/time.txt";

        CONSOLE << m_model->m_ImgPath;
        CONSOLE << m_model->m_AssoPath;
    }
    if(NUM_OF_CAM == 2)
    {
        m_model->m_LeftImgPath = m_model->m_seqPath + "/cam0/data";
        m_model->m_RightImgPath = m_model->m_seqPath + "/cam1/data";
        m_model->m_AssoPath = m_model->m_seqPath + "/timestamp/data/time.txt";

        CONSOLE << m_model->m_LeftImgPath;
        CONSOLE << m_model->m_RightImgPath;
        CONSOLE << m_model->m_AssoPath;
    }
}

void QConfigDialog::readGT()
{
    QString gt_path = QFileDialog::getOpenFileName(this, "Open a file", "${HOME}");
    this->ui->gt_path->setText(gt_path);
    m_model->m_GTPath = gt_path;
}

void QConfigDialog::configReset()
{
    m_model->m_slamSettingPath = "";
    m_model->m_seqPath = "";

    m_model->m_ImgPath = "";
    m_model->m_LeftImgPath = "";
    m_model->m_RightImgPath = "";

    m_model->m_AssoPath = "";
    m_model->m_LeftAssoPath = "";
    m_model->m_RightAssoPath = "";

    m_model->m_IMUPath = "";

    m_model->m_vocPath = "";
    m_model->m_settingPath = "";

    m_model->m_GTPath = "";

    m_model->m_classes = "";
    m_model->m_modelConfig = "";
    m_model->m_modelWeights = "";

    this->close();
}

void QConfigDialog::configClose()
{
    // TODO: Check path/file availabel
    emit configDone();
    this->close();
}
