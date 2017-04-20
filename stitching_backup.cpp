// Stitching.cpp : 定义控制台应用程序的入口点。

#include <QtWidgets/QMainWindow>
#include <qprogressdialog.h>
#include "ui_stitchingvsqt.h"
#include <iostream>
#include <fstream>
#include <string>
#include <Windows.h>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "stitching.h"
#include "qmessagebox.h"
#include "stitchingvsqt.h"


using namespace std;
using namespace cv;
using namespace cv::detail;

//static void printUsage()
//{
//	cout <<
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
/*
运行流程：
1.命令行调用程序，输入源图像以及程序的参数

2.特征点检测，判断是使用surf还是orb，默认是surf。

3.对图像的特征点进行匹配，使用最近邻和次近邻方法，将两个最优的匹配的置信度保存下来。

4.对图像进行排序以及将置信度高的图像保存到同一个集合中，删除置信度比较低的图像间的匹配，得到能正确匹配的图像序列。这样将置信度高于门限的所有匹配合并到一个集合中。

5.对所有图像进行相机参数粗略估计，然后求出旋转矩阵

6.使用光束平均法进一步精准的估计出旋转矩阵。

7.波形校正，水平或者垂直

8.拼接

9.融合，多频段融合，光照补偿
*/

vector<string> img_names;
bool preview = false; //以预览模式运行程序，比正常模式要快，但输出图像分辨率低，拼接的分辨率compose_megapix 设置为0.6
bool try_gpu = false;
double work_megapix = 0.6;//图像匹配的分辨率大小，图像的面积尺寸变为work_megapix*100000，默认为0.6
double seam_megapix = 0.1; //拼接缝像素的大小 默认是0.1
double compose_megapix = -1; //拼接分辨率
float conf_thresh = 1.f; //两幅图来自同一全景图的置信度，默认为1.0
string features_type = "surf";
//string features_type = "orb";
//string ba_cost_func = "ray";   //光束平均法的误差函数选择
string ba_cost_func = "reproj";

string ba_refine_mask = "xxxxx";
bool do_wave_correct = true;
WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;//波形校验 水平，垂直或者没有 默认是horiz
bool save_graph = false;
std::string save_graph_to;
string warp_type = "fisheye";
int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
float match_conf = 0.65f;//特征点检测置信等级，最近邻匹配距离与次近邻匹配距离的比值，surf默认为0.65，orb默认为0.3
string seam_find_type = "gc_color"; // 拼接缝隙估计方法
int blend_type = Blender::MULTI_BAND;// 融合方法，默认是多频段融合
float blend_strength = 5;//融合强度，0-100
string result_name = "surf_reproj_fisheye.jpg";

class StitchingVSQt;
////static int parseCmdArgs(int argc, char** argv)
////{
//	/*if (argc == 1)
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
//				cout << "Bad --try_gpu flag value\n";
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
//				cout << "Incorrect refinement mask length.\n";
//				return -1;
//			}
//			i++;
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
//				cout << "Bad --wave_correct flag value\n";
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
//				cout << "Bad exposure compensation method\n";
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
//				cout << "Bad seam finding method\n";
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
//				cout << "Bad blending method\n";
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
//		else*/
////			img_names.push_back(argv[i]);
////	
////	if (preview)
////	{
////		compose_megapix = 0.6;
////	}
////	return 0;
////}
string& replace_all_distinct(string&   str, const   string&   old_value, const   string&   new_value)
{
	for (string::size_type pos(0); pos != string::npos; pos += new_value.length())   {
		if ((pos = str.find(old_value, pos)) != string::npos)
			str.replace(pos, old_value.length(), new_value);
		else   break;
	}
	return   str;
}

void storeImageFileName(string name){
	img_names.push_back(name);
}

int startCalculation()
{
	StitchingVSQt s;
	
	QProgressDialog progress(QString::fromLocal8Bit("正在导入图片数据，请稍候..."),
		QString::fromLocal8Bit("取消"),
		0, 13, // Range
		NULL);
	progress.show();
	qApp->processEvents();
	progress.setWindowModality(Qt::WindowModal);
	progress.setWindowTitle(QString::fromLocal8Bit("正在导入图片数据，请稍候..."));

	progress.setValue(1);
	progress.setModal(true);
	progress.setLabelText(QString::fromLocal8Bit("读取图片，请稍候..."));

	// Check if have enough images
	int num_images = static_cast<int>(img_names.size());
	if (num_images < 2)
	{
		//LOGLN("Need more images");
		s.showMessage("Need more images");
		
		return -1;
	}

	double work_scale = 1, seam_scale = 1, compose_scale = 1;
	bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;

	/**
	*特征点提取
	*/
	progress.setValue(2);
	progress.setLabelText(QString::fromLocal8Bit("特征点提取...."));

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
	//	cout << "Unknown 2D features type: '" << features_type << "'.\n";
		s.showMessage("Unknown 2D features type: '" +QString::fromStdString(features_type));
		return -1;
	}

	Mat full_img, img;
	vector<ImageFeatures> features(num_images);
	vector<Mat> images(num_images);
	vector<Size> full_img_sizes(num_images);
	double seam_work_aspect = 1;

	for (int i = 0; i < num_images; ++i)
	{
		/*full_img = imread(img_names[i].replace()));*/

		full_img = imread(img_names[i]);
		full_img_sizes[i] = full_img.size();

		if (full_img.empty())
		{
			//LOGLN("Can't open image " << img_names[i]);
			s.showMessage("Can't open image " + QString::fromStdString(img_names[i]));
			return -1;
		}

		if (work_megapix < 0)//图像匹配的分辨率大小
		{
			img = full_img;
			work_scale = 1;
			is_work_scale_set = true;
		}
		else
		{
			if (!is_work_scale_set)
			{
				//work_scale参数设定图像大小（当图像尺寸大于1e6时，缩小图像，提高特征点检测速度），来寻找特征点。
				work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
				is_work_scale_set = true;
			}
			resize(full_img, img, Size(), work_scale, work_scale);
		}

		if (!is_seam_scale_set)////默认值seam_megapix=0.1
		{
			seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
			seam_work_aspect = seam_scale / work_scale;
			is_seam_scale_set = true;
		}

		(*finder)(img, features[i]);
		features[i].img_idx = i;
	//	LOGLN("Features in image #" << i + 1 << ": " << features[i].keypoints.size());
	//	s.showMessage("Features in image # " + QString::number(i + 1) + ":" + QString::number(features[i].keypoints.size()));
		//按照seam_scale调整图像大小，作为后面缝合缝寻找（seam_finder）的输入图像
		resize(full_img, img, Size(), seam_scale, seam_scale);
		images[i] = img.clone();
	}

	finder->collectGarbage();
	full_img.release();
	img.release();

//	LOGLN("Finding features, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

	progress.setValue(3);
	/*
	特征点匹配
	*/
	vector<MatchesInfo> pairwise_matches;
	BestOf2NearestMatcher matcher=new BestOf2NearestMatcher(try_gpu,match_conf);
	
	try{
		progress.setLabelText(QString::fromLocal8Bit("特征点匹配...."));
		matcher(features, pairwise_matches);
		matcher.collectGarbage();
	}
	catch(Exception){
		s.showMessage(QString::fromLocal8Bit("计算失败，请重试..."));
			return -1;
	}
	

// Check if we should save matches graph
	if (save_graph)
	{
		ofstream f(save_graph_to.c_str());
		f << matchesGraphAsString(img_names, pairwise_matches, conf_thresh);
	}

	// Leave only images we are sure are from the same panorama
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

	// Check if we still have enough images
	num_images = static_cast<int>(img_names.size());
	if (num_images < 2)
	{
		//LOGLN("Need more images");
		s.showMessage("Need more images");
		return -1;
	}

	progress.setValue(4);
	progress.setLabelText(QString::fromLocal8Bit("估计相机参数，计算变换矩阵...."));
	//CameraParams 是结构体参数为 Focal length（焦距），double aspect（ Aspect ratio纵横比），double ppx 主点x; double ppy 主点y; Mat R// Rotation，Mat t//Translation
	HomographyBasedEstimator estimator;
	vector<CameraParams> cameras;

	//投影矩阵k,旋转矩阵R
	estimator(features, pairwise_matches, cameras);
	

	for (size_t i = 0; i < cameras.size(); ++i)
	{
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		//cout << R << endl;
		cameras[i].R = R;
		//LOGLN("Initial intrinsics #" << indices[i] + 1 << ":\n" << cameras[i].K());
	}

	Ptr<detail::BundleAdjusterBase> adjuster;
	if (ba_cost_func == "reproj") adjuster = new detail::BundleAdjusterReproj();
	else if (ba_cost_func == "ray") adjuster = new detail::BundleAdjusterRay();
	else
	{
	//	cout << "Unknown bundle adjustment cost function: '" << ba_cost_func << "'.\n";
		s.showMessage("Unknown bundle adjustment cost function: '" + QString::fromStdString(ba_cost_func));
		
		return -1;
	}
	adjuster->setConfThresh(conf_thresh);
	Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
	if (ba_refine_mask[0] == 'x') refine_mask(0, 0) = 1;
	if (ba_refine_mask[1] == 'x') refine_mask(0, 1) = 1;
	if (ba_refine_mask[2] == 'x') refine_mask(0, 2) = 1;
	if (ba_refine_mask[3] == 'x') refine_mask(1, 1) = 1;
	if (ba_refine_mask[4] == 'x') refine_mask(1, 2) = 1;
	adjuster->setRefinementMask(refine_mask);
	(*adjuster)(features, pairwise_matches, cameras);

	// Find median focal length

	vector<double> focals;
	for (size_t i = 0; i < cameras.size(); ++i)
	{
	//	LOGLN("Camera #" << indices[i] + 1 << ":\n" << cameras[i].K());
		focals.push_back(cameras[i].focal);
	}

	sort(focals.begin(), focals.end());
	float warped_image_scale;   //赋值为所有焦距的中值
	if (focals.size() % 2 == 1)
		warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
	else
		warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

	progress.setValue(5);

	if (do_wave_correct)
	{
		vector<Mat> rmats;
		for (size_t i = 0; i < cameras.size(); ++i)
			rmats.push_back(cameras[i].R);
		waveCorrect(rmats, wave_correct);
		for (size_t i = 0; i < cameras.size(); ++i)
			cameras[i].R = rmats[i];
	}

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

	progress.setValue(6);
	// Warp images and their masks
	progress.setLabelText(QString::fromLocal8Bit("柱面投影...."));

	Ptr<WarperCreator> warper_creator;
#if defined(HAVE_OPENCV_GPU)
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
		cout << "Can't create the following warper '" << warp_type << "'\n";
		return 1;
	}

	// 投影变换
	Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

	for (int i = 0; i < num_images; ++i)
	{
		Mat_<float> K;
		cameras[i].K().convertTo(K, CV_32F);
		float swa = (float)seam_work_aspect;
		K(0, 0) *= swa; K(0, 2) *= swa;
		K(1, 1) *= swa; K(1, 2) *= swa;

		corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
		sizes[i] = images_warped[i].size();

		warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
	}

	vector<Mat> images_warped_f(num_images);
	for (int i = 0; i < num_images; ++i)
		images_warped[i].convertTo(images_warped_f[i], CV_32F);

	//LOGLN("Warping images, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
	
	progress.setValue(7);
	Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
	compensator->feed(corners, images_warped, masks_warped);

	progress.setValue(8);
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
		cout << "Can't create the following seam finder '" << seam_find_type << "'\n";
		return 1;
	}

	seam_finder->find(images_warped_f, corners, masks_warped);

	// Release unused memory
	images.clear();
	images_warped.clear();
	images_warped_f.clear();
	masks.clear();

	//LOGLN("Compositing...");
//#if ENABLE_LOG
//	t = getTickCount();
//#endif

	progress.setValue(9);
	progress.setLabelText(QString::fromLocal8Bit("图像融合...."));
	Mat img_warped, img_warped_s;
	Mat dilated_mask, seam_mask, mask, mask_warped;

	//图像融合
	Ptr<Blender> blender;
	//double compose_seam_aspect = 1;
	double compose_work_aspect = 1;

	for (int img_idx = 0; img_idx < num_images; ++img_idx)
	{
		//LOGLN("Compositing image #" << indices[img_idx] + 1);
		/*
		由于以前进行处理的图片都是以work_scale（3.1节有介绍）进行缩放的，
		所以图像的内参，corner（同一坐标后的顶点），mask（融合的掩码）都需要重新计算。
		以及将之前计算的光照增强的gain也要计算进去
		*/
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
			resize(full_img, img, Size(), compose_scale, compose_scale);
		else
			img = full_img;
		full_img.release();
		Size img_size = img.size();

		Mat K;
		cameras[img_idx].K().convertTo(K, CV_32F);

		progress.setValue(10);
		// Warp the current image
		warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);

		// Warp the current image mask
		mask.create(img_size, CV_8U);
		mask.setTo(Scalar::all(255));
		warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

		// Compensate exposure,光照补偿
		progress.setLabelText(QString::fromLocal8Bit("光照补偿...."));
		compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

		img_warped.convertTo(img_warped_s, CV_16S);
		img_warped.release();
		img.release();
		mask.release();

		dilate(masks_warped[img_idx], dilated_mask, Mat());
		resize(dilated_mask, seam_mask, mask_warped.size());
		mask_warped = seam_mask & mask_warped;

		/*
		进行多波段融合，首先初始化blend，确定blender的融合的方式，默认是多波段融合MULTI_BAND，
		以及根据corners顶点和图像的大小确定最终全景图的尺寸
		*/		
		
		if (blender.empty())
		{
			blender = Blender::createDefault(blend_type, try_gpu);
			Size dst_sz = resultRoi(corners, sizes).size();//计算最后图像的大小
			float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
			if (blend_width < 1.f)
				blender = Blender::createDefault(Blender::NO, try_gpu);
			else if (blend_type == Blender::MULTI_BAND)
			{
				MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));
				mb->setNumBands(static_cast<int>(ceil(log(blend_width) / log(2.)) - 1.));
				//LOGLN("Multi-band blender, number of bands: " << mb->numBands());
			}
			else if (blend_type == Blender::FEATHER)
			{
				FeatherBlender* fb = dynamic_cast<FeatherBlender*>(static_cast<Blender*>(blender));
				fb->setSharpness(1.f / blend_width);
				//LOGLN("Feather blender, sharpness: " << fb->sharpness());
			}
			blender->prepare(corners, sizes);////根据corners顶点和图像的大小确定最终全景图的尺寸 
		}

		/*
		*然后对每幅图图形构建金字塔，层数可以由输入的系数确定，默认是5层。
		先对顶点以及图像的宽和高做处理，使其能被2^num_bands除尽，
		这样才能将进行向下采样num_bands次，首先从源图像pyrDown向下采样，
		在由最底部的图像pyrUp向上采样，把对应层得上采样和下采样的相减，
		就得到了图像的高频分量，存储到每一个金字塔中。然后根据mask，
		将每幅图像的各层金字塔分别写入最终的金字塔层dst_pyr_laplace中。
		*
		*/

		// Blend the current image
		//将图像写入金字塔中
		blender->feed(img_warped_s, mask_warped, corners[img_idx]);
	}
	progress.setLabelText(QString::fromLocal8Bit("根据corners顶点和图像大小确定最终全景图尺寸...."));

	progress.setValue(11);
	Mat result, result_mask;
	blender->blend(result, result_mask);////将多层金字塔图形叠加  

	//LOGLN("Compositing, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");

	imwrite(result_name, result);

//	LOGLN("Finished, total time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec");

	//char szAppPath[MAX_PATH];
	//GetModuleFileName(NULL, szAppPath, MAX_PATH);
	progress.setValue(12);
	//s.showMessage("Finished!");
	
	QString path;
	//path.replace(QString("Win32/Debug"), QString(""));
	path.append(QString("./"));
	path.append(QString(QString::fromStdString(result_name)));
	//s.showMessage(path);
    //StitchingVSQt::transfer(path);
	progress.setValue(13);
    s.changePicture(path);

	//delete &s;
	img_names.clear();
	return 0;
}


