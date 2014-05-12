#ifndef SFMVIEWER_H
#define SFMVIEWER_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <QGLViewer/qglviewer.h>
#include "sfmupdatelistener.h"

#include "scenemodel.h"

class SFMViewer : public QGLViewer, public SfMUpdateListener
{
	Q_OBJECT
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SFMViewer(QWidget *parent = 0):QGLViewer(QGLFormat::defaultFormat(),parent) 
	{
		points.clear();
		pointsRGB.clear();
		cameras.clear();
		cameraTransforms.clear();
	}

	virtual void update(std::vector<cv::Point3d> points, 
						std::vector<cv::Vec3b> pointsRGB,
						std::vector<cv::Matx34d> cameras);

protected :
	virtual void draw();
	virtual void init();

private:
	std::vector<cv::Point3d> points;
	std::vector<cv::Vec3b> pointsRGB;


	// Necessary for drawing cameras axes
	std::vector<cv::Matx34d> cameras;
	std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> cameraTransforms;
	Eigen::Affine3d globalTransform;
	float vizScale;
	double scaleCamerasDown;
};

#endif