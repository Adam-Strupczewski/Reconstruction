#ifndef SFMVIEWER_H
#define SFMVIEWER_H

#include <QGLViewer/qglviewer.h>
#include "sfmupdatelistener.h"

#include "scenemodel.h"

class SFMViewer : public QGLViewer, public SfMUpdateListener
{
	Q_OBJECT
public:
	SFMViewer(QWidget *parent = 0):QGLViewer(QGLFormat::defaultFormat(),parent) 
	{

	}

	virtual void update(std::vector<cv::Point3d> points, std::vector<cv::Vec3b> pointsRGB);

protected :
	virtual void draw();
	virtual void init();
	//virtual QString helpString() const;

private:
	std::vector<cv::Point3d> points;
	std::vector<cv::Vec3b> pointsRGB;

};

#endif