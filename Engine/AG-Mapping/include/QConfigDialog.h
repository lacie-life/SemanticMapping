#ifndef QCONFIGDIALOG_H
#define QCONFIGDIALOG_H

#include <QDialog>

namespace Ui {
class QConfigDialog;
}

class QConfigDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QConfigDialog(QWidget *parent = nullptr);
    ~QConfigDialog();

private:
    Ui::QConfigDialog *ui;
};

#endif // QCONFIGDIALOG_H
