#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "detail/autocalib.hpp"
#include "detail/blenders.hpp"
#include "detail/camera.hpp"
#include "detail/exposure_compensate.hpp"
#include "detail/matchers.hpp"
#include "detail/motion_estimators.hpp"
#include "detail/seam_finders.hpp"
#include "detail/util.hpp"
#include "detail/warpers.hpp"

#include "warpers.hpp"
#include "BundleAdjuster.h"

#undef ENABLE_LOG
#define ENABLE_LOG

using namespace cv;
using namespace cv::detail;

//static void printUsage()
//{
//	std::cout <<
//		"Rotation model images stitcher.\n\n"
//		"stitching_detailed img1 img2 [...imgN] [flags]\n\n"
//		"Flags:\n"
//		"  --preview\n"
//		"      Run stitching in the preview mode. Works faster than usual mode,\n"
//		"      but output image will have lower resolution.\n"
//		"  --try_gpu (yes|no)\n"
//		"      Try to use GPU. The default value is 'no'. All default values\n"
//		"      are for CPU mode.\n"
//		"\nMotion Estimation Flags:\n"
//		"  --work_megapix <float>\n"
//		"      Resolution for image registration step. The default is 0.6 Mpx.\n"
//		"  --features (surf|orb)\n"
//		"      Type of features used for images matching. The default is surf.\n"
//		"  --match_conf <float>\n"
//		"      Confidence for feature matching step. The default is 0.65 for surf and 0.3 for orb.\n"
//		"  --conf_thresh <float>\n"
//		"      Threshold for two images are from the same panorama confidence.\n"
//		"      The default is 1.0.\n"
//		"  --ba (reproj|ray)\n"
//		"      Bundle adjustment cost function. The default is ray.\n"
//		"  --ba_refine_mask (mask)\n"
//		"      Set refinement mask for bundle adjustment. It looks like 'x_xxx',\n"
//		"      where 'x' means refine respective parameter and '_' means don't\n"
//		"      refine one, and has the following format:\n"
//		"      <fx><skew><ppx><aspect><ppy>. The default mask is 'xxxxx'. If bundle\n"
//		"      adjustment doesn't support estimation of selected parameter then\n"
//		"      the respective flag is ignored.\n"
//		"  --wave_correct (no|horiz|vert)\n"
//		"      Perform wave effect correction. The default is 'horiz'.\n"
//		"  --save_graph <file_name>\n"
//		"      Save matches graph represented in DOT language to <file_name> file.\n"
//		"      Labels description: Nm is number of matches, Ni is number of inliers,\n"
//		"      C is confidence.\n"
//		"\nCompositing Flags:\n"
//		"  --warp (plane|cylindrical|spherical|fisheye|stereographic|compressedPlaneA2B1|compressedPlaneA1.5B1|compressedPlanePortraitA2B1|compressedPlanePortraitA1.5B1|paniniA2B1|paniniA1.5B1|paniniPortraitA2B1|paniniPortraitA1.5B1|mercator|transverseMercator)\n"
//		"      Warp surface type. The default is 'spherical'.\n"
//		"  --seam_megapix <float>\n"
//		"      Resolution for seam estimation step. The default is 0.1 Mpx.\n"
//		"  --seam (no|voronoi|gc_color|gc_colorgrad)\n"
//		"      Seam estimation method. The default is 'gc_color'.\n"
//		"  --compose_megapix <float>\n"
//		"      Resolution for compositing step. Use -1 for original resolution.\n"
//		"      The default is -1.\n"
//		"  --expos_comp (no|gain|gain_blocks)\n"
//		"      Exposure compensation method. The default is 'gain_blocks'.\n"
//		"  --blend (no|feather|multiband)\n"
//		"      Blending method. The default is 'multiband'.\n"
//		"  --blend_strength <float>\n"
//		"      Blending strength from [0,100] range. The default is 5.\n"
//		"  --output <result_img>\n"
//		"      The default is 'result.jpg'.\n";
//}

// Default command line args
vector<string> img_names;
// bool preview = false;		// preview 模式以较低的分辨率进行
bool try_gpu = false;
double work_megapix = 0.6;		// 配准时的分辨率
double seam_megapix = 0.1;		// 接缝查找时的分辨率
double compose_megapix = -1;	// 图像合成时的分辨率, -1为原始分辨率
float conf_thresh = .5f;		// 图像配准的置信度（来自同一场景）
std::string features_type = "surf";
std::string ba_cost_func = "reproj";
std::string ba_refine_mask = "xxxxx";
bool do_wave_correct = true;
WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;
bool save_graph = false;
std::string save_graph_to;
std::string warp_type = "cylindrical";	// 投影曲面
//string warp_type = "spherical";		// 投影曲面
int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
float match_conf = 0.65f;		// 特征匹配的置信度
string seam_find_type = "gc_color";
int blend_type = Blender::MULTI_BAND;
float blend_strength = 5;
string result_name = "result.jpg";
//std::ofstream _log("log.txt");

//static int parseCmdArgs(int argc, char** argv)
//{
//	if (argc == 1)
//	{
//		printUsage();
//		return -1;
//	}
//	for (int i = 1; i < argc; ++i)
//	{
//		if (string(argv[i]) == "--help" || string(argv[i]) == "/?")
//		{
//			printUsage();
//			return -1;
//		}
//		else if (string(argv[i]) == "--preview")
//		{
//			preview = true;
//		}
//		else if (string(argv[i]) == "--try_gpu")
//		{
//			if (string(argv[i + 1]) == "no")
//				try_gpu = false;
//			else if (string(argv[i + 1]) == "yes")
//				try_gpu = true;
//			else
//			{
//				std::cout << "Bad --try_gpu flag value\n";
//				return -1;
//			}
//			i++;
//		}
//		else if (string(argv[i]) == "--work_megapix")
//		{
//			work_megapix = atof(argv[i + 1]);
//			i++;
//		}
//		else if (string(argv[i]) == "--seam_megapix")
//		{
//			seam_megapix = atof(argv[i + 1]);
//			i++;
//		}
//		else if (string(argv[i]) == "--compose_megapix")
//		{
//			compose_megapix = atof(argv[i + 1]);
//			i++;
//		}
//		else if (string(argv[i]) == "--result")
//		{
//			result_name = argv[i + 1];
//			i++;
//		}
//		else if (string(argv[i]) == "--features")
//		{
//			features_type = argv[i + 1];
//			if (features_type == "orb")
//				match_conf = 0.3f;
//			i++;
//		}
//		else if (string(argv[i]) == "--match_conf")
//		{
//			match_conf = static_cast<float>(atof(argv[i + 1]));
//			i++;
//		}
//		else if (string(argv[i]) == "--conf_thresh")
//		{
//			conf_thresh = static_cast<float>(atof(argv[i + 1]));
//			i++;
//		}
//		else if (string(argv[i]) == "--ba")
//		{
//			ba_cost_func = argv[i + 1];
//			i++;
//		}
//		else if (string(argv[i]) == "--ba_refine_mask")
//		{
//			ba_refine_mask = argv[i + 1];
//			if (ba_refine_mask.size() != 5)
//			{
//				std::cout << "Incorrect refinement mask length.\n";
//				return -1;
//			}
//i++;
//		}
//		else if (string(argv[i]) == "--wave_correct")
//		{
//			if (string(argv[i + 1]) == "no")
//				do_wave_correct = false;
//			else if (string(argv[i + 1]) == "horiz")
//			{
//				do_wave_correct = true;
//				wave_correct = detail::WAVE_CORRECT_HORIZ;
//			}
//			else if (string(argv[i + 1]) == "vert")
//			{
//				do_wave_correct = true;
//				wave_correct = detail::WAVE_CORRECT_VERT;
//			}
//			else
//			{
//				std::cout << "Bad --wave_correct flag value\n";
//				return -1;
//			}
//			i++;
//		}
//		else if (string(argv[i]) == "--save_graph")
//		{
//			save_graph = true;
//			save_graph_to = argv[i + 1];
//			i++;
//		}
//		else if (string(argv[i]) == "--warp")
//		{
//			warp_type = string(argv[i + 1]);
//			i++;
//		}
//		else if (string(argv[i]) == "--expos_comp")
//		{
//			if (string(argv[i + 1]) == "no")
//				expos_comp_type = ExposureCompensator::NO;
//			else if (string(argv[i + 1]) == "gain")
//				expos_comp_type = ExposureCompensator::GAIN;
//			else if (string(argv[i + 1]) == "gain_blocks")
//				expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
//			else
//			{
//				std::cout << "Bad exposure compensation method\n";
//				return -1;
//			}
//			i++;
//		}
//		else if (string(argv[i]) == "--seam")
//		{
//			if (string(argv[i + 1]) == "no" ||
//				string(argv[i + 1]) == "voronoi" ||
//				string(argv[i + 1]) == "gc_color" ||
//				string(argv[i + 1]) == "gc_colorgrad" ||
//				string(argv[i + 1]) == "dp_color" ||
//				string(argv[i + 1]) == "dp_colorgrad")
//				seam_find_type = argv[i + 1];
//			else
//			{
//				std::cout << "Bad seam finding method\n";
//				return -1;
//			}
//			i++;
//		}
//		else if (string(argv[i]) == "--blend")
//		{
//			if (string(argv[i + 1]) == "no")
//				blend_type = Blender::NO;
//			else if (string(argv[i + 1]) == "feather")
//				blend_type = Blender::FEATHER;
//			else if (string(argv[i + 1]) == "multiband")
//				blend_type = Blender::MULTI_BAND;
//			else
//			{
//				std::cout << "Bad blending method\n";
//				return -1;
//			}
//			i++;
//		}
//		else if (string(argv[i]) == "--blend_strength")
//		{
//			blend_strength = static_cast<float>(atof(argv[i + 1]));
//			i++;
//		}
//		else if (string(argv[i]) == "--output")
//		{
//			result_name = argv[i + 1];
//			i++;
//		}
//		else
//			img_names.push_back(argv[i]);
//	}
//	if (preview)
//	{
//		compose_megapix = 0.6;
//	}
//	return 0;
//}

int main(int argc, char* argv[])
{
#ifdef ENABLE_LOG
	int64 app_start_time = getTickCount();
#endif

	{
		std::ifstream in("path.txt");
		if (!in)
			return -1;
		std::string img_name;
		while (std::getline(in, img_name))
			img_names.push_back(img_name);
	}

	// Check if have enough images
	int num_images = static_cast<int>(img_names.size());
	if (num_images < 2)
	{
		std::cerr << "Need more images\n";
		return -1;
	}

	double work_scale = 1;			// 特征检测时的尺度
	double seam_scale = 1;
	bool is_work_scale_set = false, is_seam_scale_set = false;

	std::cerr << "Finding features...\n";
#ifdef ENABLE_LOG
	int64 t = getTickCount();
#endif

	Ptr<FeaturesFinder> finder;
	if (features_type == "surf")
	{
#if defined(HAVE_OPENCV_NONFREE) && defined(HAVE_OPENCV_GPU)
		if (try_gpu && gpu::getCudaEnabledDeviceCount() > 0)
			finder = new SurfFeaturesFinderGpu();
		else
#endif
			finder = new SurfFeaturesFinder();
	}
	else if (features_type == "orb")
	{
		finder = new OrbFeaturesFinder();
	}
	else
	{
		std::cerr << "Unknown 2D features type: '" << features_type << "'.\n";
		return -1;
	}

	Mat full_img, img;
	vector<ImageFeatures> features(num_images);
	vector<Mat> images(num_images);			// 存放的为seamfinder所需的img
	vector<Size> full_img_sizes(num_images);
	double seam_work_aspect = 1;

	for (int i = 0; i < num_images; ++i)
	{
		full_img = imread(img_names[i]);
		full_img_sizes[i] = full_img.size();

		if (full_img.empty())
		{
			std::cerr << "Can't open image " << img_names[i] << '\n';
			return -1;
		}
		if (work_megapix < 0)
		{
			img = full_img;
			work_scale = 1;
			is_work_scale_set = true;
		}
		else
		{
			if (!is_work_scale_set)
			{
				work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
				is_work_scale_set = true;
			}
			cv::resize(full_img, img, Size(), work_scale, work_scale);
		}
		if (!is_seam_scale_set)
		{
			seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
			seam_work_aspect = seam_scale / work_scale;
			is_seam_scale_set = true;
		}

		// 对resize之后的图像进行特征检测
		(*finder)(img, features[i]);		// 找出features中的内容
		features[i].img_idx = i;
		std::cerr << "Features in image #" << i + 1 << ": " << features[i].keypoints.size() << '\n';

		// 将原图像resize存于 vector<Mat> images 中以待seamfinder使用
		cv::resize(full_img, img, Size(), seam_scale, seam_scale);
		images[i] = img.clone();
	}

	finder->collectGarbage();
	full_img.release();
	img.release();

	std::cerr << "Finding features time: " << ((getTickCount() - t) / getTickFrequency()) << " sec" << '\n';

	std::cerr << "\nPairwise matching\n";
#ifdef ENABLE_LOG
	t = getTickCount();
#endif
	// 看下如何实现 match
	vector<MatchesInfo> pairwise_matches;
	BestOf2NearestMatcher matcher(try_gpu, match_conf);
	matcher(features, pairwise_matches);
	matcher.collectGarbage();
	std::cerr << "Pairwise matching time: " << ((getTickCount() - t) / getTickFrequency()) << " sec\n";

	std::cerr << pairwise_matches[1].num_inliers << '/' << pairwise_matches[1].matches.size() << '\n';

	// Check if we should save matches graph
	if (save_graph)
	{
		std::cerr << "Saving matches graph...\n";
		std::ofstream f(save_graph_to.c_str());
		f << matchesGraphAsString(img_names, pairwise_matches, conf_thresh);
	}

	// 去除噪声图像，取原图像的子集
	// Leave only images we are sure are from the same panorama
	std::cerr << "\nrepick images: " << img_names.size() << " -> ";

	//vector<int> indices{ 0, 1 };
	vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
	vector<Mat> img_subset;
	vector<string> img_names_subset;
	vector<Size> full_img_sizes_subset;
	for (size_t i = 0; i < indices.size(); ++i)
	{
		img_names_subset.push_back(img_names[indices[i]]);
		img_subset.push_back(images[indices[i]]);
		full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
	}

	images = img_subset;
	img_names = img_names_subset;
	full_img_sizes = full_img_sizes_subset;
	std::cerr << img_names.size() << '\n';

	std::cerr << pairwise_matches[1].confidence << '\t' << pairwise_matches[1].matches.size() << '\n';

	// Check if we still have enough images
	num_images = static_cast<int>(img_names.size());
	if (num_images < 2)
	{
		std::cerr << "Need more images\n";
		return -1;
	}

#ifdef ENABLE_LOG
	std::cerr << "\nestimate cameras' pose\n";
	t = getTickCount();
#endif
	HomographyBasedEstimator estimator;
	vector<CameraParams> cameras;
	// 焦距如何获得，像主点的坐标？
	estimator(features, pairwise_matches, cameras); // 看下具体获得 R K 矩阵的方法

	std::cerr << "Estimate time: " << (getTickCount() - t) / getTickFrequency() << " sec\n";

	for (size_t i = 0; i < cameras.size(); ++i)
	{
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R;
		LOGLN("Initial intrinsics #" << indices[i] + 1 << ":\n" << cameras[i].K());
	}

#ifdef ENABLE_LOG
	std::cerr << "\nRefine pose...\n";
	t = getTickCount();
#endif
	//Ptr<detail::BundleAdjusterBase> adjuster;
	//if (ba_cost_func == "reproj") adjuster = new detail::BundleAdjusterReproj();
	//else if (ba_cost_func == "ray") adjuster = new detail::BundleAdjusterRay();
	//else
	//{
	//	std::cerr << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
	//	return -1;
	//}
	//adjuster->setConfThresh(conf_thresh);
	//Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
	//if (ba_refine_mask[0] == 'x') refine_mask(0, 0) = 1;
	//if (ba_refine_mask[1] == 'x') refine_mask(0, 1) = 1;
	//if (ba_refine_mask[2] == 'x') refine_mask(0, 2) = 1;
	//if (ba_refine_mask[3] == 'x') refine_mask(1, 1) = 1;
	//if (ba_refine_mask[4] == 'x') refine_mask(1, 2) = 1;
	//adjuster->setRefinementMask(refine_mask);
	//// BA 只refine了相机的参数：f，3个旋转参数
	//(*adjuster)(features, pairwise_matches, cameras);

	/********************************************************/
	// 自己写平差的类
	BundleAdjuster adjuster(conf_thresh);
	adjuster(features, pairwise_matches, cameras);
	/********************************************************/
	std::cerr << "Refine pose time: " << ((getTickCount() - t) / getTickFrequency()) << " sec\n";

	// Find median focal length

	vector<double> focals;
	for (size_t i = 0; i < cameras.size(); ++i)
	{
		LOGLN("Camera #" << indices[i] + 1 << ":\n" << cameras[i].K());
		focals.push_back(cameras[i].focal);
	}

	sort(focals.begin(), focals.end());
	float warped_image_scale;		// = 焦距的中值
	if (focals.size() % 2 == 1)
		warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
	else
		warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;



	std::cerr << "focal " << warped_image_scale;
	std::cerr << "cx " << cameras[0].ppx;
	std::cerr << "cy " << cameras[0].ppy;



#ifdef ENABLE_LOG
	std::cerr << "\nWave correct...\n";
	t = getTickCount();
#endif
	// 完成 wavecorrect，即更新 R
	if (do_wave_correct)
	{
		vector<Mat> rmats;
		for (size_t i = 0; i < cameras.size(); ++i)
			rmats.push_back(cameras[i].R);
		waveCorrect(rmats, wave_correct);  // wave_correct = method
		for (size_t i = 0; i < cameras.size(); ++i)
			cameras[i].R = rmats[i];  // update Mat R
	}
	std::cerr << "Wave correct time: " << ((getTickCount() - t) / getTickFrequency()) << " sec\n";

#ifdef ENABLE_LOG
	std::cerr << "\nWarping images (auxiliary)... \n";
	t = getTickCount();
#endif
	vector<Point> corners(num_images);
	vector<Mat> masks_warped(num_images);
	vector<Mat> images_warped(num_images);
	vector<Size> sizes(num_images);
	vector<Mat> masks(num_images);

	// Preapre images masks
	for (int i = 0; i < num_images; ++i)
	{
		masks[i].create(images[i].size(), CV_8U);
		masks[i].setTo(Scalar::all(255));
	}

	// Warp images and their masks

	Ptr<WarperCreator> warper_creator;
#ifdef defined(HAVE_OPENCV_GPU)
	if (try_gpu && gpu::getCudaEnabledDeviceCount() > 0)
	{
		if (warp_type == "plane") warper_creator = new cv::PlaneWarperGpu();
		else if (warp_type == "cylindrical") warper_creator = new cv::CylindricalWarperGpu();
		else if (warp_type == "spherical") warper_creator = new cv::SphericalWarperGpu();
	}
	else
#endif
	{
		if (warp_type == "plane") warper_creator = new cv::PlaneWarper();
		else if (warp_type == "cylindrical") warper_creator = new cv::CylindricalWarper();
		else if (warp_type == "spherical") warper_creator = new cv::SphericalWarper();
		else if (warp_type == "fisheye") warper_creator = new cv::FisheyeWarper();
		else if (warp_type == "stereographic") warper_creator = new cv::StereographicWarper();
		else if (warp_type == "compressedPlaneA2B1") warper_creator = new cv::CompressedRectilinearWarper(2, 1);
		else if (warp_type == "compressedPlaneA1.5B1") warper_creator = new cv::CompressedRectilinearWarper(1.5, 1);
		else if (warp_type == "compressedPlanePortraitA2B1") warper_creator = new cv::CompressedRectilinearPortraitWarper(2, 1);
		else if (warp_type == "compressedPlanePortraitA1.5B1") warper_creator = new cv::CompressedRectilinearPortraitWarper(1.5, 1);
		else if (warp_type == "paniniA2B1") warper_creator = new cv::PaniniWarper(2, 1);
		else if (warp_type == "paniniA1.5B1") warper_creator = new cv::PaniniWarper(1.5, 1);
		else if (warp_type == "paniniPortraitA2B1") warper_creator = new cv::PaniniPortraitWarper(2, 1);
		else if (warp_type == "paniniPortraitA1.5B1") warper_creator = new cv::PaniniPortraitWarper(1.5, 1);
		else if (warp_type == "mercator") warper_creator = new cv::MercatorWarper();
		else if (warp_type == "transverseMercator") warper_creator = new cv::TransverseMercatorWarper();
	}

	if (warper_creator.empty())
	{
		std::cerr << "Can't create the following warper '" << warp_type << "'\n";
		return 1;
	}
	// 由此可见 Warper 的代码也得自己写，加上 t
	Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));	// 参数为 scale

	for (int i = 0; i < num_images; ++i)
	{
		Mat_<float> K;		//  即内参数f cx cy
		cameras[i].K().convertTo(K, CV_32F);	// K()返回的是 64F
		float swa = (float)seam_work_aspect;
		K(0, 0) *= swa; K(0, 2) *= swa;
		K(1, 1) *= swa; K(1, 2) *= swa;
		// corners 记录的是什么单点？
		corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
		sizes[i] = images_warped[i].size();

		warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
	}

	vector<Mat> images_warped_f(num_images);
	for (int i = 0; i < num_images; ++i)
	{
		//cv::imwrite(img_names[i], images_warped[i]);
		images_warped[i].convertTo(images_warped_f[i], CV_32F);
	}

	std::cerr << "Warping images, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec\n";

	cv::imshow("warped", images_warped[0]);
	cv::waitKey();
	return 0;


#ifdef ENABLE_LOG
	std::cerr << "\nExposure Compensate...\n";
	t = getTickCount();
#endif
	Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
	compensator->feed(corners, images_warped, masks_warped);
	std::cerr << "Compensate time: " << ((getTickCount() - t) / getTickFrequency()) << " sec\n";

#ifdef ENABLE_LOG
	std::cerr << "\nSeam Finding...\n";
	t = getTickCount();
#endif
	Ptr<SeamFinder> seam_finder;
	if (seam_find_type == "no")
		seam_finder = new detail::NoSeamFinder();
	else if (seam_find_type == "voronoi")
		seam_finder = new detail::VoronoiSeamFinder();
	else if (seam_find_type == "gc_color")
	{
#if defined(HAVE_OPENCV_GPU)
		if (try_gpu && gpu::getCudaEnabledDeviceCount() > 0)
			seam_finder = new detail::GraphCutSeamFinderGpu(GraphCutSeamFinderBase::COST_COLOR);
		else
#endif
			seam_finder = new detail::GraphCutSeamFinder(GraphCutSeamFinderBase::COST_COLOR);
	}
	else if (seam_find_type == "gc_colorgrad")
	{
#if defined(HAVE_OPENCV_GPU)
		if (try_gpu && gpu::getCudaEnabledDeviceCount() > 0)
			seam_finder = new detail::GraphCutSeamFinderGpu(GraphCutSeamFinderBase::COST_COLOR_GRAD);
		else
#endif
			seam_finder = new detail::GraphCutSeamFinder(GraphCutSeamFinderBase::COST_COLOR_GRAD);
	}
	else if (seam_find_type == "dp_color")
		seam_finder = new detail::DpSeamFinder(DpSeamFinder::COLOR);
	else if (seam_find_type == "dp_colorgrad")
		seam_finder = new detail::DpSeamFinder(DpSeamFinder::COLOR_GRAD);
	if (seam_finder.empty())
	{
		std::cerr << "Can't create the following seam finder '" << seam_find_type << "'\n";
		return 1;
	}

	seam_finder->find(images_warped_f, corners, masks_warped);
	std::cerr << "Seam Finding time: " << ((getTickCount() - t) / getTickFrequency()) << " sec\n";

	// Release unused memory
	images.clear();
	images_warped.clear();
	images_warped_f.clear();
	masks.clear();

	std::cerr << "\nCompositing...\n";
#ifdef ENABLE_LOG
	t = getTickCount();
	double sum_comps_time = 0;
#endif

	Mat img_warped, img_warped_s;
	Mat dilated_mask, seam_mask, mask, mask_warped;
	Ptr<Blender> blender;
	//double compose_seam_aspect = 1;
	double compose_work_aspect = 1;
	double compose_scale = 1;
	bool is_compose_scale_set = false;

	compose_megapix = 0.2;

	for (int img_idx = 0; img_idx < num_images; ++img_idx)
	{
#ifdef ENABLE_LOG
		std::cerr << "Compositing image #" << indices[img_idx] + 1;
		t = getTickCount();
#endif
		// Read image and resize it if necessary
		full_img = imread(img_names[img_idx]);
		if (!is_compose_scale_set)
		{
			if (compose_megapix > 0)
				compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / full_img.size().area()));
			is_compose_scale_set = true;

			// Compute relative scales
			//compose_seam_aspect = compose_scale / seam_scale;
			compose_work_aspect = compose_scale / work_scale;

			// Update warped image scale
			warped_image_scale *= static_cast<float>(compose_work_aspect);
			warper = warper_creator->create(warped_image_scale);

			// Update corners and sizes
			for (int i = 0; i < num_images; ++i)
			{
				// Update intrinsics
				cameras[i].focal *= compose_work_aspect;
				cameras[i].ppx *= compose_work_aspect;
				cameras[i].ppy *= compose_work_aspect;

				// Update corner and size
				Size sz = full_img_sizes[i];
				if (std::abs(compose_scale - 1) > 1e-1)
				{
					sz.width = cvRound(full_img_sizes[i].width * compose_scale);
					sz.height = cvRound(full_img_sizes[i].height * compose_scale);
				}

				Mat K;
				cameras[i].K().convertTo(K, CV_32F);
				Rect roi = warper->warpRoi(sz, K, cameras[i].R);
				corners[i] = roi.tl();
				sizes[i] = roi.size();
			}
		}
		if (abs(compose_scale - 1) > 1e-1)
			cv::resize(full_img, img, Size(), compose_scale, compose_scale);
		else
			img = full_img;
		full_img.release();

		Mat K;
		cameras[img_idx].K().convertTo(K, CV_32F);

		// Warp the current image
		warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);

		// Warp the current image mask
		mask.create(img.size(), CV_8U);
		mask.setTo(Scalar::all(255));
		warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

		// Compensate exposure
		compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

		img_warped.convertTo(img_warped_s, CV_16S);
		img_warped.release();
		img.release();
		mask.release();

		cv::dilate(masks_warped[img_idx], dilated_mask, Mat());
		cv::resize(dilated_mask, seam_mask, mask_warped.size());
		mask_warped = seam_mask & mask_warped;

		if (blender.empty())
		{
			blender = Blender::createDefault(blend_type, try_gpu);
			Size dst_sz = resultRoi(corners, sizes).size();		// 准备最终 全景图的Mat
			float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
			if (blend_width < 1.f)
				blender = Blender::createDefault(Blender::NO, try_gpu);
			else if (blend_type == Blender::MULTI_BAND)
			{
				MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));
				mb->setNumBands(static_cast<int>(ceil(log(blend_width) / log(2.)) - 1.));
				// std::cerr << "Multi-band blender, number of bands: " << mb->numBands() << '\n';
			}
			else if (blend_type == Blender::FEATHER)
			{
				FeatherBlender* fb = dynamic_cast<FeatherBlender*>(static_cast<Blender*>(blender));
				fb->setSharpness(1.f / blend_width);
				// std::cerr << "Feather blender, sharpness: " << fb->sharpness()<< '\n';
			}
			blender->prepare(corners, sizes);
		}

		// Blend the current image
		blender->feed(img_warped_s, mask_warped, corners[img_idx]);
#ifdef ENABLE_LOG
		double tmp = (getTickCount() - t) / getTickFrequency();
		std::cerr << '\t' << tmp << " sec\n";
		sum_comps_time += tmp;
#endif
	}

	Mat result, result_mask;
	blender->blend(result, result_mask);

	std::cerr << "Compositing, time: " << sum_comps_time << " sec" << '\n';

	cv::imwrite(result_name, result);

	std::cerr << "\nFinished, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec" << '\n';

	getchar();
	return 0; LOG()
}
