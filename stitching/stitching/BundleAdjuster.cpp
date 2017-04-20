#include "BundleAdjuster.h"
// for std
#include <iostream>
#include <fstream>
#include <utility>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "detail/matchers.hpp"
#include "detail/camera.hpp"
#include "detail/util.hpp"
// for ceres
#include <ceres/ceres.h>
//#include <Eigen\Dense>

using std::vector;
using cv::detail::Graph;
using cv::detail::GraphEdge;
using cv::detail::MatchesInfo;
using cv::detail::ImageFeatures;
using cv::detail::CameraParams;

struct IncDistance
{
	IncDistance(vector<int> &vdists) : dists(&vdists[0]) {}
	void operator ()(const GraphEdge &edge) { dists[edge.to] = dists[edge.from] + 1; }
	int* dists;
};

void findMaxSpanningTree(int num_images, const std::vector<MatchesInfo> &pairwise_matches,
	Graph &span_tree, std::vector<int> &centers)
{
	Graph graph(num_images);
	std::vector<GraphEdge> edges;

	// Construct images graph and remember its edges
	for (int i = 0; i < num_images; ++i)
	{
		for (int j = 0; j < num_images; ++j)
		{
			if (pairwise_matches[i * num_images + j].H.empty())
				continue;
			float conf = static_cast<float>(pairwise_matches[i * num_images + j].num_inliers);
			graph.addEdge(i, j, conf);
			edges.push_back(GraphEdge(i, j, conf));
		}
	}

	cv::detail::DisjointSets comps(num_images);
	span_tree.create(num_images);
	vector<int> span_tree_powers(num_images, 0);

	// Find maximum spanning tree
	sort(edges.begin(), edges.end(), std::greater<GraphEdge>());
	for (size_t i = 0; i < edges.size(); ++i)
	{
		int comp1 = comps.findSetByElem(edges[i].from);
		int comp2 = comps.findSetByElem(edges[i].to);
		if (comp1 != comp2)
		{
			comps.mergeSets(comp1, comp2);
			span_tree.addEdge(edges[i].from, edges[i].to, edges[i].weight);
			span_tree.addEdge(edges[i].to, edges[i].from, edges[i].weight);
			span_tree_powers[edges[i].from]++;
			span_tree_powers[edges[i].to]++;
		}
	}

	// Find spanning tree leafs
	vector<int> span_tree_leafs;
	for (int i = 0; i < num_images; ++i)
		if (span_tree_powers[i] == 1)
			span_tree_leafs.push_back(i);

	// Find maximum distance from each spanning tree vertex
	vector<int> max_dists(num_images, 0);
	vector<int> cur_dists;
	for (size_t i = 0; i < span_tree_leafs.size(); ++i)
	{
		cur_dists.assign(num_images, 0);
		span_tree.walkBreadthFirst(span_tree_leafs[i], IncDistance(cur_dists));
		for (int j = 0; j < num_images; ++j)
			max_dists[j] = std::max(max_dists[j], cur_dists[j]);
	}

	// Find min-max distance
	int min_max_dist = max_dists[0];
	for (int i = 1; i < num_images; ++i)
		if (min_max_dist > max_dists[i])
			min_max_dist = max_dists[i];

	// Find spanning tree centers
	centers.clear();
	for (int i = 0; i < num_images; ++i)
		if (max_dists[i] == min_max_dist)
			centers.push_back(i);
	CV_Assert(centers.size() > 0 && centers.size() <= 2);
}



//template<typename T>
//Eigen::Matrix<T, 3, 3> Rodrigues(const T *const v)
//{
//	Eigen::Matrix<T, 3, 1> r(v);
//	T theta = r.norm();
//	if (theta < T(DBL_EPSILON))
//		return Eigen::Matrix<T, 3, 3>::Identity();
//
//	T c = cos(theta);
//	T s = sin(theta);
//	T c1 = T(1.) - c;
//	r /= theta;
//
//	Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();
//	Eigen::Matrix<T, 3, 3> rx;
//	rx << T(0.), -r(2, 0), r(1, 0), r(2, 0), 0, -r(0, 0), -r(1, 0), r(0, 0), T(0.);
//	
//	return c*I + c1*r*r.transpose() + s*rx;
//}

template<typename T>
void Rodrigues(const T *const v, T *R)
{
	T rx = v[0];
	T ry = v[1];
	T rz = v[2];

	T theta = sqrt(rx*rx + ry*ry + rz*rz);
	if (theta < T(DBL_EPSILON)) {
		R[0] = R[4] = R[8] = T(1.);
		return;
	}

	const T I[] = { T(1.), T(0.), T(0.), T(0.), T(1.), T(0.), T(0.), T(0.), T(1.) };

	T c = cos(theta);
	T s = sin(theta);
	T c1 = 1. - c;

	rx /= theta; ry /= theta; rz /= theta;

	T rrt[] = { rx*rx, rx*ry, rx*rz, rx*ry, ry*ry, ry*rz, rx*rz, ry*rz, rz*rz };
	T _r_x_[] = { T(0.), -rz, ry, rz, T(0.), -rx, -ry, rx, T(0.) };

	for (int k = 0; k < 9; ++k)
		R[k] = c*I[k] + c1*rrt[k] + s*_r_x_[k];
}

struct CostFunctor
{
	CostFunctor(double x1_, double y1_, double x2_, double y2_):
		x1(x1_), y1(y1_), x2(x2_), y2(y2_) {}

	template<typename T>
	bool operator()(const T *const paras1, const T *const paras2, T *residual) const
	{
		T _R1[9]{T(0.0)}, _R2[9]{T(0.)};
		Rodrigues(paras1 + 4, _R1);
		Rodrigues(paras2 + 4, _R2);

		Eigen::Matrix<T, 3, 3, Eigen::RowMajor> R1(_R1);
		Eigen::Matrix<T, 3, 3, Eigen::RowMajor> R2(_R2);
		
		T f1 = paras1[0], f2 = paras2[0];

		Eigen::Matrix<T, 3, 3> K1 = Eigen::Matrix<T, 3, 3>::Identity();
		K1(0, 0) = f1;				K1(0, 2) = paras1[1];
		K1(1, 1) = f1*paras1[3];	K1(1, 2) = paras1[2];

		Eigen::Matrix<T, 3, 3> K2 = Eigen::Matrix<T, 3, 3>::Identity();
		K2(0, 0) = f2;				K2(0, 2) = paras2[1];
		K2(1, 1) = f2*paras2[3];	K2(1, 2) = paras2[2];

		Eigen::Matrix<T, 3, 3> H = K2 * R2.inverse() * R1 * K1.inverse();

		T x = H(0, 0)*x1 + H(0, 1)*y1 + H(0, 2);
		T y = H(1, 0)*x1 + H(1, 1)*y1 + H(1, 2);
		T z = H(2, 0)*x1 + H(2, 1)*y1 + H(2, 2);
		
		residual[0] = T(x2) - x/z;
		residual[1] = T(y2) - y/z;

		return true;
	}

	double x1, y1;
	double x2, y2;
};

bool BundleAdjuster::operator()(const std::vector<ImageFeatures> &features,
								const std::vector<MatchesInfo> &matches,
								std::vector<CameraParams> &cameras)
{
	initPara(cameras);

	//std::ofstream _log("C:\\users\\jeff\\desktop\\rodrigues.txt");
	//if (!_log)
	//	std::cerr << "creat file failed\n";

	// 建立匹配相片间的关系
	std::vector<std::pair<int, int>> edges;
	int num_images = static_cast<int>(cameras.size());
	for (int i = 0; i < num_images - 1; ++i)	// 只取上三角部分
	{
		for (int j = i + 1; j < num_images; ++j)
		{
			const MatchesInfo &matches_info = matches[i * num_images + j];
			if (matches_info.confidence > conf_thresh)
				edges.emplace_back(i, j);
		}
	}

	ceres::Problem problem;
	// construct problem
	for (auto it = edges.cbegin(), end = edges.cend(); it != end; ++it)
	{
		int i = it->first;
		int j = it->second;

		const ImageFeatures &features1 = features[i];
		const ImageFeatures &features2 = features[j];
		const MatchesInfo &matches_info = matches[i * num_images + j];
		
		for (size_t k = 0, size = matches_info.matches.size(); k < size; ++k)
		{
			if (!matches_info.inliers_mask[k])
				continue;

			const cv::DMatch& m = matches_info.matches[k];
			cv::Point2f p1 = features1.keypoints[m.queryIdx].pt;
			cv::Point2f p2 = features2.keypoints[m.trainIdx].pt;
			ceres::AutoDiffCostFunction<CostFunctor, 2, 7, 7> *function =
				new ceres::AutoDiffCostFunction<CostFunctor, 2, 7, 7>(new CostFunctor(p1.x, p1.y, p2.x, p2.y));
			problem.AddResidualBlock(function, NULL, &cam_paras[7 * i], &cam_paras[7 * j]);
		}	// iteration of DMatches
	}	// iteration of images

	// ceres::AutoDiffCostFunction < CostFunctor, ceres::DYNAMIC, 7, 7>()
	ceres::Solver::Options op;
	op.linear_solver_type = ceres::DENSE_QR;
	op.num_threads = 3;
	ceres::Solver::Summary summary;
	ceres::Solve(op, &problem, &summary);
	std::ofstream report("solver_report.txt");
	report << summary.FullReport();
	report.close();

	obtainPara(cameras);
	cv::detail::Graph span_tree;
	std::vector<int> span_tree_centers;
	findMaxSpanningTree(num_images, matches, span_tree, span_tree_centers);
	cv::Mat R_inv = cameras[span_tree_centers[0]].R.inv();
	for (int i = 0; i < num_images; ++i)
		cameras[i].R = R_inv * cameras[i].R;

	return true;
}

void BundleAdjuster::initPara(const std::vector<cv::detail::CameraParams> &cameras)
{
	size_t num_imgs = cameras.size();
	cam_paras.reset(new double[num_imgs*7]);	// 7 paras per camera

	cv::SVD svd;
	for (size_t i = 0; i < num_imgs; ++i)
	{
		cam_paras[i * 7] = cameras[i].focal;
		cam_paras[i * 7 + 1] = cameras[i].ppx;
		cam_paras[i * 7 + 2] = cameras[i].ppy;
		cam_paras[i * 7 + 3] = cameras[i].aspect;

		svd(cameras[i].R, cv::SVD::FULL_UV);
		cv::Mat R = svd.u * svd.vt;
		if (determinant(R) < 0)
			R *= -1;

		cv::Mat rvec;
		cv::Rodrigues(R, rvec);
		CV_Assert(rvec.type() == CV_32F);
		cam_paras[i*7 + 4] = rvec.at<float>(0, 0);
		cam_paras[i*7 + 5] = rvec.at<float>(1, 0);
		cam_paras[i*7 + 6] = rvec.at<float>(2, 0);
	}
}

void BundleAdjuster::obtainPara(std::vector<cv::detail::CameraParams> &cameras)
{
	size_t num_imgs = cameras.size();
	for (size_t i = 0; i < num_imgs; ++i)
	{
		cameras[i].focal = cam_paras[i * 7];
		cameras[i].ppx = cam_paras[i * 7 + 1];
		cameras[i].ppy = cam_paras[i * 7 + 2];
		cameras[i].aspect = cam_paras[i * 7 + 3];

		cv::Mat rvec(3, 1, CV_64F);
		rvec.at<double>(0, 0) = cam_paras[i * 7 + 4];
		rvec.at<double>(1, 0) = cam_paras[i * 7 + 5];
		rvec.at<double>(2, 0) = cam_paras[i * 7 + 6];
		cv::Rodrigues(rvec, cameras[i].R);

		cv:: Mat tmp;
		cameras[i].R.convertTo(tmp, CV_32F);
		cameras[i].R = tmp;
	}
	cam_paras.reset();
}