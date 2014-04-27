#ifndef SFMUPDATELISTENER_H
#define SFMUPDATELISTENER_H

#include <opencv2/core/core.hpp>

class SfMUpdateListener
{
public:
	virtual void update(std::vector<cv::Point3d> points) = 0;
};

#endif