/****************************************************************************
**
** Copyright (C) 2013 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
**     of its contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "logging.h"

#include <QMediaService>
#include <QCameraViewfinder>
#include <QMediaMetaData>

#include <QMessageBox>
#include <QPalette>

#include <QtWidgets>

#if (defined(Q_WS_MAEMO_6)) && QT_VERSION >= 0x040700
#define HAVE_CAMERA_BUTTONS
#endif

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    camera(0)
{
    ui->setupUi(this);

    //Camera devices:
    QByteArray cameraDevice;

    QActionGroup *videoDevicesGroup = new QActionGroup(this);
    videoDevicesGroup->setExclusive(true);
    foreach(const QByteArray &deviceName, QCamera::availableDevices()) {
        QString description = camera->deviceDescription(deviceName);
        QAction *videoDeviceAction = new QAction(description, videoDevicesGroup);
        videoDeviceAction->setCheckable(true);
        videoDeviceAction->setData(QVariant(deviceName));
        if (cameraDevice.isEmpty()) {
            cameraDevice = deviceName;
            videoDeviceAction->setChecked(true);
        }
        ui->menuDevices->addAction(videoDeviceAction);
    }

    connect(videoDevicesGroup, SIGNAL(triggered(QAction*)), SLOT(updateCameraDevice(QAction*)));
    connect(ui->stopButton, SIGNAL(clicked()), SLOT(stopCamera()));
    connect(ui->startButton, SIGNAL(clicked()), SLOT(startCamera()));

    setCamera(cameraDevice);
}

MainWindow::~MainWindow()
{
    delete camera;
}

void MainWindow::setCamera(const QByteArray &cameraDevice)
{
    delete camera;

    if (cameraDevice.isEmpty())
        camera = new QCamera;
    else
        camera = new QCamera(cameraDevice);

    camera->setViewfinder(ui->videoWidget->videoSurface());
    camera->start();
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
    if (event->isAutoRepeat())
        return;

    switch (event->key()) {
    case Qt::Key_CameraFocus:
        camera->searchAndLock();
        event->accept();
        break;
    case Qt::Key_Camera:
        if (camera->captureMode() == QCamera::CaptureStillImage) {
            // takeImage();
        }

        event->accept();
        break;
    default:
        QMainWindow::keyPressEvent(event);
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;

    switch (event->key()) {
    case Qt::Key_CameraFocus:
        camera->unlock();
        break;
    default:
        QMainWindow::keyReleaseEvent(event);
    }
}

void MainWindow::startCamera()
{
    LOG(Debug, "Started the camera");
    camera->start();
}

void MainWindow::stopCamera()
{
    LOG(Debug, "Stopped the camera");
    camera->stop();
}

void MainWindow::updateCameraDevice(QAction *action)
{
    setCamera(action->data().toByteArray());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    event->accept();
}
