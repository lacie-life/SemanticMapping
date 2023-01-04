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

signals:
    void configDone();

public slots:
    void readSetting();
    void readSeq();
    void readGT();
    void configReset();
    void configClose();

private:
    Ui::QConfigDialog *ui;

    AppModel* m_model;
};

#endif // QCONFIGDIALOG_H
