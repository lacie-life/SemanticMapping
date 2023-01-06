#ifndef QSLAMWIDGET_H
#define QSLAMWIDGET_H

#include <QWidget>

namespace Ui {
class QSLAMWidget;
}

class QSLAMWidget : public QWidget
{
    Q_OBJECT

public:
    explicit QSLAMWidget(QWidget *parent = nullptr);
    ~QSLAMWidget();

private:
    Ui::QSLAMWidget *ui;
};

#endif // QSLAMWIDGET_H
