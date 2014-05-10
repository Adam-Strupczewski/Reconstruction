#include "stdafx.h"

#include "sfmviewer.h"

using namespace std;

void SFMViewer::update(std::vector<cv::Point3d> points, std::vector<cv::Vec3b> pointsRGB){
	this->points = points;
	this->pointsRGB = pointsRGB;
}

void SFMViewer::draw()
{

	glPushAttrib(GL_ENABLE_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(2);
	glBegin(GL_POINTS);
	for (int i = 0; i < points.size(); ++i) {

		// Draw color points!
		// Draw cameras!


        glColor3ub(255-10*i,100,10*i);
		glVertex3dv(&(points[i].x));
	}
	glEnd();

}

void SFMViewer::init()
{
	// Restore previous viewer state.
	restoreStateFromFile();

	setFPSIsDisplayed();

	int bound = 20;
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