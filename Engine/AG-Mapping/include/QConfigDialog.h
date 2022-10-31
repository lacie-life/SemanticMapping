#ifndef QCONFIGDIALOG_H
#define QCONFIGDIALOG_H

#include <QDialog>
#include <QObject>
#include <QString>

#include "AppModel.h"

namespace Ui {
class QConfigDialog;
}

class QConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QConfigDialog(QWidget *parent = nullptr, AppModel *model = nullptr);
    ~QConfigDialog();

    void display();

public slots:
    void readSetting();
    void readSeq();
    void readGT();

private:
    Ui::QConfigDialog *ui;

    AppModel* m_model;

    QString m_seqPath;

    QString m_ImgPath;
    QString m_LeftImgPath;
    QString m_RightImgPath;

    QString m_AssoPath;
    QString m_LeftAssoPath;
    QString m_RightAssoPath;

    QString m_IMUPath;

    QString m_vocPath;
    QString m_settingPath;

    QString m_GTPath;

    QString m_classes;
    QString m_modelConfig;
    QString m_modelWeights;
};

#endif // QCONFIGDIALOG_H
