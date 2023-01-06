#ifndef QINTRODUCTION_H
#define QINTRODUCTION_H

#include <QWidget>

namespace Ui {
class QIntroduction;
}

class QIntroduction : public QWidget
{
    Q_OBJECT

public:
    explicit QIntroduction(QWidget *parent = nullptr);
    ~QIntroduction();

signals:
    void systemConfigOpen();
    void actionRunClicked();

public slots:
    void actionRunClickedSlot();
    void systemConfigOpenSlot();

private:
    Ui::QIntroduction *ui;
};

#endif // QINTRODUCTION_H
