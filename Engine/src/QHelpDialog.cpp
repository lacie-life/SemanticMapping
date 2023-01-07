#include "QHelpDialog.h"
#include "ui_qhelpdialog.h"

QHelpDialog::QHelpDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::QHelpDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("Help Document");
    connect(ui->btn_ok, &QPushButton::clicked, this, [=]() {
        this->close();
    });

    ui->label_Q->setPixmap(QPixmap(":/images/imgs/q.png").scaledToWidth(ui->label_Q->width()));
    ui->label_R->setPixmap(QPixmap(":/images/imgs/r.png").scaledToWidth(ui->label_R->width()));
    ui->label_G->setPixmap(QPixmap(":/images/imgs/g.png").scaledToWidth(ui->label_G->width()));
    ui->label_B->setPixmap(QPixmap(":/images/imgs/b.png").scaledToWidth(ui->label_B->width()));
    ui->label_D->setPixmap(QPixmap(":/images/imgs/d.png").scaledToWidth(ui->label_D->width()));
}

QHelpDialog::~QHelpDialog()
{
    delete ui;
}
