#ifndef QSLAM_H
#define QSLAM_H

#include <QObject>

#include <QDebug>
#include <QObject>
#include <QThread>
#include <memory>
#include <opencv2/core/core.hpp>

#include "estimator.h"

class QSLAM
{
public:
    explicit QSLAM(QObject *parent = nullptr);

    void shutdown();

private:

};

#endif // QSLAM_H
