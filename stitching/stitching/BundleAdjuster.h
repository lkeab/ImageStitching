#pragma once
#ifndef BUNDLEADJUSTOR_H
#define BUNDLEADJUSTOR_H

#include <vector>
#include <memory>

namespace cv{
	namespace detail{
		struct ImageFeatures;
		struct MatchesInfo;
		struct CameraParams;
	}
}

class BundleAdjuster
{
public:
	explicit BundleAdjuster(double th) :conf_thresh(th){}

	bool operator()(const std::vector<cv::detail::ImageFeatures> &features,
					const std::vector<cv::detail::MatchesInfo> &matches,
					std::vector<cv::detail::CameraParams> &cameras);

	void initPara(const std::vector<cv::detail::CameraParams> &cameras);
	
	void obtainPara(std::vector<cv::detail::CameraParams> &cameras);

private:
	std::unique_ptr<double[]> cam_paras;
	double conf_thresh;
};

#endif	// BUNDLEADJUSTOR_H