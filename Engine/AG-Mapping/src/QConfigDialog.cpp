#include "QConfigDialog.h"
#include "ui_qconfigdialog.h"

QConfigDialog::QConfigDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::QConfigDialog)
{
    ui->setupUi(this);
}

QConfigDialog::~QConfigDialog()
{
    delete ui;
}
