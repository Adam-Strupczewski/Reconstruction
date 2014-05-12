#include "stdafx.h"

#include "sfmviewer.h"

using namespace std;

void SFMViewer::update(std::vector<cv::Point3d> points, 
						std::vector<cv::Vec3b> pointsRGB,
						std::vector<cv::Matx34d> cameras){
	this->points = points;
	this->pointsRGB = pointsRGB;
	this->cameras = cameras;

	// TODO Verify below code

	// Get the scale of the result cloud using PCA
	{
		cv::Mat_<double> cldm(points.size(), 3);
		for (unsigned int i = 0; i < points.size(); i++) {
			cldm.row(i)(0) = points[i].x;
			cldm.row(i)(1) = points[i].y;
			cldm.row(i)(2) = points[i].z;
		}
		cv::Mat_<double> mean;
		cv::PCA pca(cldm, mean, CV_PCA_DATA_AS_ROW);
		scaleCamerasDown = 1.0 / (3.0 * sqrt(pca.eigenvalues.at<double> (0)));
	}

	// Compute transformation to place cameras in world
	cameraTransforms.resize(cameras.size());
	Eigen::Vector3d c_sum(0,0,0);
	for (int i = 0; i < cameras.size(); ++i) {
		Eigen::Matrix<double, 3, 4> P = Eigen::Map<Eigen::Matrix<double, 3, 4,
				Eigen::RowMajor> >(cameras[i].val);
		Eigen::Matrix3d R = P.block(0, 0, 3, 3);
		Eigen::Vector3d t = P.block(0, 3, 3, 1);
		Eigen::Vector3d c = -R.transpose() * t;
		c_sum += c;
		cameraTransforms[i] =
				Eigen::Translation<double, 3>(c) *
				Eigen::Quaterniond(R) *
				Eigen::UniformScaling<double>(scaleCamerasDown)
				;
	}

	globalTransform = Eigen::Translation<double,3>(-c_sum / (double)(cameras.size()));
}

void SFMViewer::draw()
{
	// TODO Sort out scale

	glPushMatrix();
	//glScaled(vizScale,vizScale,vizScale);
	glMultMatrixd(globalTransform.data());

	// Draw reconstructed points
	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(2);
	glBegin(GL_POINTS);
	for (int i = 0; i < points.size(); ++i) {

		// Draw color points
		glColor3ub(pointsRGB[i][0],pointsRGB[i][1],pointsRGB[i][2]);
		glVertex3dv(&(points[i].x));
	}
	glEnd();

	//glScaled(scaleCamerasDown,scaleCamerasDown,scaleCamerasDown);

	// Draw cameras
	glEnable(GL_RESCALE_NORMAL);
	glEnable(GL_LIGHTING);
	for (int i = 0; i < cameraTransforms.size(); ++i) {

		glPushMatrix();
		glMultMatrixd(cameraTransforms[i].data());

	    glColor4f(1, 0, 0, 1);
	    QGLViewer::drawArrow(qglviewer::Vec(0,0,0), qglviewer::Vec(3,0,0));
	    glColor4f(0, 1, 0, 1);
	    QGLViewer::drawArrow(qglviewer::Vec(0,0,0), qglviewer::Vec(0,3,0));
	    glColor4f(0, 0, 1, 1);
	    QGLViewer::drawArrow(qglviewer::Vec(0,0,0), qglviewer::Vec(0,0,3));

	    glPopMatrix();
	}

	glPopAttrib();
	glPopMatrix();
}

void SFMViewer::init()
{
	// Restore previous viewer state.
	restoreStateFromFile();

	setFPSIsDisplayed();

	int bound = 50;
	setSceneBoundingBox(qglviewer::Vec(-bound,-bound,-bound), qglviewer::Vec(bound,bound,bound));

	showEntireScene();

	// Opens help window
	//help();
}
/*
QString SFMViewer::helpString() const
{
  QString text("<h2>S i m p l e V i e w e r</h2>");
  text += "Use the mouse to move the camera around the object. ";
  text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
  text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
  text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
  text += "Simply press the function key again to restore it. Several keyFrames define a ";
  text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
  text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
  text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
  text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
  text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
  text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
  text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
  text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
  text += "Press <b>Escape</b> to exit the viewer.";
  return text;
}*/