#include "QConfigDialog.h"
#include "AppConstants.h"
#include "ui_qconfigdialog.h"

#include <QMessageBox>
#include <QCloseEvent>
#include <QFileDialog>

QConfigDialog::QConfigDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::QConfigDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("App Configure");
    this->connection();
    {
        // pre setting
        this->m_temp_seqPath = this->m_seqPath = SEQ_PATH;
        this->m_temp_colorImgPath = this->m_colorImgPath = RGB_IMAGE_PATH;
        this->m_temp_depthImgPath = this->m_depthImgPath = DEPTH_IMAGE_PATH;
        this->m_temp_assoPath_d = this->m_assoPath_d = ASSO_DEPTH_PATH;
        this->m_temp_assoPath_rgb = this->m_assoPath_rgb = ASSO_RGB_PATH;

        // -----------
        this->m_temp_settingPath = this->m_settingPath = SETTING_PATH;
        this->m_temp_vocPath = this->m_vocPath = VOCABULARY_PATH;
        this->m_temp_classes = this->m_classes = CLASSES_PATH;
        this->m_temp_modelConfig = this->m_modelConfig = MODEL_CONFIG;
        this->m_temp_modelWeights = this->m_modelWeights = MODEL_WEIGHTS;
        // display
        this->display();
    }
}

QConfigDialog::~QConfigDialog()
{
    delete ui;
}

void QConfigDialog::connection()
{
    // sequence
    connect(ui->load_seq, &QPushButton::clicked, this, [=]() {
        QString seqPath;
        if (this->m_temp_seqPath.isEmpty()) {
            seqPath = QFileDialog::getExistingDirectory(this, "Sequence Path", QDir::currentPath());
        } else {
            seqPath = QFileDialog::getExistingDirectory(this, "Sequence Path", this->m_temp_seqPath);
        }

        if (seqPath.isEmpty()) {
            return;
        }
        this->m_temp_seqPath = seqPath;

        this->m_temp_colorImgPath = seqPath + "/rgb";
        this->m_temp_depthImgPath = seqPath + "/depth";
        this->m_temp_assoPath_d = seqPath + "depth.txt";
        this->m_temp_assoPath_rgb = seqPath + "rgb.txt";

        this->display();
    });
    // vocabulary
    connect(ui->load_voc, &QPushButton::clicked, this, [=]() {
        QString vocPath;
        if (this->m_temp_vocPath.isEmpty()) {
            vocPath = QFileDialog::getOpenFileName(this, "Vocabulary Path", QDir::currentPath());
        } else {
            vocPath = QFileDialog::getOpenFileName(this, "Vocabulary Path", this->m_temp_vocPath);
        }

        if (vocPath.isEmpty()) {
            return;
        }

        this->m_temp_vocPath = vocPath;

        this->display();
    });
    // settings
    connect(ui->load_set, &QPushButton::clicked, this, [=]() {
        QString setPath;
        if (this->m_temp_settingPath.isEmpty()) {
            setPath = QFileDialog::getOpenFileName(this, "Settings Path", QDir::currentPath());
        } else {
            setPath = QFileDialog::getOpenFileName(this, "Settings Path", this->m_temp_settingPath);
        }

        if (setPath.isEmpty()) {
            return;
        }

        this->m_temp_settingPath = setPath;

        this->display();
    });

    // classes
    connect(ui->load_classes, &QPushButton::clicked, this, [=]() {
        QString classPath;
        if (this->m_temp_classes.isEmpty()) {
            classPath = QFileDialog::getOpenFileName(this, "Classes Path", QDir::currentPath());
        } else {
            classPath = QFileDialog::getOpenFileName(this, "Classes Path", this->m_temp_classes);
        }

        if (classPath.isEmpty()) {
            return;
        }

        this->m_temp_classes = classPath;

        this->display();
    });
    // modelConfig
    connect(ui->load_modelConfig, &QPushButton::clicked, this, [=]() {
        QString modelConfigPath;
        if (this->m_temp_modelConfig.isEmpty()) {
            modelConfigPath = QFileDialog::getOpenFileName(this, "Model Config Path", QDir::currentPath());
        } else {
            modelConfigPath = QFileDialog::getOpenFileName(this, "Model Config Path", this->m_temp_modelConfig);
        }

        if (modelConfigPath.isEmpty()) {
            return;
        }

        this->m_temp_modelConfig = modelConfigPath;

        this->display();
    });
    // modelweight
    connect(ui->load_modelWeight, &QPushButton::clicked, this, [=]() {
        QString modelWeightPath;
        if (this->m_temp_classes.isEmpty()) {
            modelWeightPath = QFileDialog::getOpenFileName(this, "Model Weights Path", QDir::currentPath());
        } else {
            modelWeightPath = QFileDialog::getOpenFileName(this, "Model Weights Path", this->m_temp_modelWeights);
        }

        if (modelWeightPath.isEmpty()) {
            return;
        }

        this->m_temp_modelWeights = modelWeightPath;

        this->display();
    });

    // cancel button
    connect(ui->btn_cancel, &QPushButton::clicked, this, [=]() {
        // restore
        this->m_temp_seqPath = this->m_seqPath;
        this->m_temp_colorImgPath = this->m_colorImgPath;
        this->m_temp_depthImgPath = this->m_depthImgPath;
        this->m_temp_assoPath_d = this->m_assoPath_d;
        this->m_temp_assoPath_rgb = this->m_assoPath_rgb;

        this->m_temp_settingPath = this->m_settingPath;
        this->m_temp_vocPath = this->m_vocPath;

        this->close();
    });

    // ok button
    connect(ui->btn_ok, &QPushButton::clicked, this, [=]() {
        // should get path form lineedit
        this->m_seqPath = this->m_temp_seqPath = ui->lineEdit_seq->text();
        this->m_assoPath_d = this->m_temp_assoPath_d = ui->lineEdit_asso->text();
        this->m_colorImgPath = this->m_temp_colorImgPath = ui->lineEdit_color->text();
        this->m_depthImgPath = this->m_temp_depthImgPath = ui->lineEdit_depth->text();

        this->m_settingPath = this->m_temp_settingPath = ui->lineEdit_set->text();
        this->m_vocPath = this->m_temp_vocPath = ui->lineEdit_voc->text();

        this->close();
    });
}

void QConfigDialog::display()
{
    // display the path accroding to the temp variables
    ui->lineEdit_seq->setText(this->m_temp_seqPath);
    ui->lineEdit_color->setText(this->m_temp_colorImgPath);
    ui->lineEdit_depth->setText(this->m_temp_depthImgPath);
    ui->lineEdit_asso->setText(this->m_temp_assoPath_d);

    ui->lineEdit_set->setText(this->m_temp_settingPath);
    ui->lineEdit_voc->setText(this->m_temp_vocPath);

    ui->lineEdit_class->setText(this->m_temp_classes);
    ui->lineEdit_modelConfig->setText(this->m_temp_modelConfig);
    ui->lineEdit_modelWeight->setText(this->m_temp_modelWeights);
}

bool QConfigDialog::isSetted()
{
    return !this->m_seqPath.isEmpty() &
            !this->m_assoPath_d.isEmpty() &
            !this->m_colorImgPath.isEmpty() &
            !this->m_depthImgPath.isEmpty() &
            !this->m_settingPath.isEmpty() &
            !this->m_vocPath.isEmpty() &
            !this->m_classes.isEmpty() &
            !this->m_modelConfig.isEmpty() &
            !this->m_modelWeights.isEmpty();
}

void QConfigDialog::closeEvent(QCloseEvent *e)
{
    // if the configure is changed, then emit a warning
    bool changed = (ui->lineEdit_seq->text() != this->m_seqPath) ||
            (ui->lineEdit_asso->text() != this->m_assoPath_d) ||
            (ui->lineEdit_color->text() != this->m_colorImgPath) ||
            (ui->lineEdit_depth->text() != this->m_depthImgPath) ||
            (ui->lineEdit_set->text() != this->m_settingPath) ||
            (ui->lineEdit_voc->text() != this->m_vocPath) ||
            (ui->lineEdit_class->text() != this->m_classes) ||
            (ui->lineEdit_modelConfig->text() != this->m_modelConfig) ||
            (ui->lineEdit_modelWeight->text() != this->m_temp_modelWeights);
    if (changed) {
        auto choice = QMessageBox::information(
                    this,
                    "Attention",
                    "Your configure has been changed, do you want to save it?",
                    QMessageBox::StandardButton::Yes | QMessageBox::StandardButton::No);

        if (choice == QMessageBox::StandardButton::Yes) {
            ui->btn_ok->click();
        }
    }
    return QDialog::closeEvent(e);
}
