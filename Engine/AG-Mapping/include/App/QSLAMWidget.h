#ifndef QSLAMWIDGET_H
#define QSLAMWIDGET_H

#include <QWidget>
#include <QObject>
#include <QImage>

namespace Ui {
class QSLAMWidget;
}

class QSLAMWidget : public QWidget
{
    Q_OBJECT

public:
    explicit QSLAMWidget(QWidget *parent = nullptr);
    ~QSLAMWidget();

public slots:
    void updateTraj(QImage img);

private:
    Ui::QSLAMWidget *ui;
};

#endif // QSLAMWIDGET_H
