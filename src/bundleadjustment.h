#ifndef BUNDLEADJUSTMENT_H
#define BUNDLEADJUSTMENT_H

#include "scenemodel.h"

class BundleAdjustment {
public:
	void adjustBundle(std::vector<CloudPoint>& pointcloud, 
					  cv::Mat& cam_matrix,
					  const std::vector<std::vector<cv::KeyPoint> >& imgpts,
					  std::map<int ,cv::Matx34d>& Pmats);
private:
	int count2DMeasurements(const std::vector<CloudPoint>& pointcloud);
};

#endif