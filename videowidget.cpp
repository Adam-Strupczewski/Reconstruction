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

#include "stdafx.h"

 #include "videowidget.h"

 #include <QtMultimedia>

 VideoWidget::VideoWidget(QWidget *parent)
     : QWidget(parent)
     , surface(0), receivedFrameCounter(0)
 {
	 surface = new VideoSurface(this);
 }

 void VideoWidget::initialize(SFMViewer *sfmViewer){
	 
	 sceneModel = new SceneModel();

     // Connect surface to our slot
     connect(surface, SIGNAL(frameAvailable()), this, SLOT(frameReady()));

     processor = new ProcessingThread(this);
     connect(processor, SIGNAL(frameProcessed()), this, SLOT(onFrameProcessed()));
     connect(processor, SIGNAL(queueFull()), this, SLOT(onThreadCongested()));

	 processor->initialize(sceneModel);
	 processor->setUpdateListener(sfmViewer);

     processor->start();
}

 VideoWidget::~VideoWidget()
 {
     if (processor!=NULL){
		 processor->stop();
		 processor->deleteLater();
	 }
	 if (surface != NULL){
		surface->stop();
		delete surface;
	 }

	 delete sceneModel;
 }

 void VideoWidget::setImageBuffers(QImage *im1, QImage *im2){
     imageBuffer1 = im1;
     imageBuffer2 = im2;
 }

 void VideoWidget::frameReady(){

     receivedFrameCounter++;

     QImage frame = surface->frame();

     // Add received frame to processing thread for processing
     if (processor) {
         processor->addFrameToProcessingQueue(frame);
     }

     // And take a copy for ourselves for drawing it on the screen
     currentFrame = QPixmap::fromImage(frame);

     // Update the UI
     update();
 }

 void VideoWidget::imageReady(){

     receivedFrameCounter++;

     QImage *image;

     if ((receivedFrameCounter%2)==1){
        image = imageBuffer1;
     }else{
        image = imageBuffer2;
     }

     // Add received frame to processing thread for processing
     if (processor) {
         processor->addFrameToProcessingQueue(*image);
     }

	 // Wait with updates until the processing thread finishes

     // And take a copy for ourselves for drawing it on the screen
     // currentFrame = QPixmap::fromImage(*image);
     // update();
 }

 void VideoWidget::onFrameProcessed()
 {
     processedFrameCounter++;
     //emit processedCountChanged(processedFrameCounter);

	 // The processing thread has just processed a new frame
	 // We want to take the new image from the thread and mark the computed keypoints in this image
	 currentFrame =  QPixmap::fromImage(*(processor->getCurrentFrame()));

     update();
 }

 void VideoWidget::onThreadCongested(){

 }

 QSize VideoWidget::sizeHint() const
 {
     return surface->surfaceFormat().sizeHint();
 }

 void VideoWidget::paintEvent(QPaintEvent *event)
 {
     QPainter painter(this);
     QRect rect = event->rect();

     if (currentFrame.isNull()) {
         painter.fillRect(rect, Qt::lightGray);
     } else {
         painter.drawPixmap(rect, currentFrame);
     }

 }

 void VideoWidget::resizeEvent(QResizeEvent *event)
 {
     QWidget::resizeEvent(event);
     //surface->updateVideoRect();
 }
