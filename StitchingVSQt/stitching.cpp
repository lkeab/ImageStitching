// Stitching.cpp : 定义控制台应用程序的入口点。

#include <QtWidgets/QMainWindow>
#include <QFileDialog>
#include <qprogressdialog.h>
#include "ui_stitchingvsqt.h"
#include <iostream>
#include <fstream>
#include <string>
#include <tchar.h>
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
#include <QRunnable>
#include "stitchingvsqt.h"


using namespace std;
using namespace cv;
using namespace cv::detail;

typedef void*(*STITCH)(std::vector<std::vector<std::string>> &img_names,
	const void *config,
	std::string &msg);

typedef void (*PANOIMAGE)(void *img, const std::string &name);

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

bool preview = false; //以预览模式运行程序，比正常模式要快，但输出图像分辨率低，拼接的分辨率compose_megapix 设置为0.6
bool try_gpu = false;
double work_megapix = 0.6;//图像匹配的分辨率大小，图像的面积尺寸变为work_megapix*100000，默认为0.6
double seam_megapix = 0.1; //拼接缝像素的大小 默认是0.1
double compose_megapix = -1; //拼接分辨率
float conf_thresh = 1.f; //两幅图来自同一全景图的置信度，默认为1.0
string features_type = "surf";
//string features_type = "orb";
string ba_cost_func = "ray";   //光束平均法的误差函数选择
//string ba_cost_func = "reproj";

string ba_refine_mask = "xxxxx";
bool do_wave_correct = true;
WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;//波形校验 水平，垂直或者没有 默认是horiz
bool save_graph = false;
std::string save_graph_to;
string warp_type = "cylindrical";
int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
float match_conf = 0.65f;//特征点检测置信等级，最近邻匹配距离与次近邻匹配距离的比值，surf默认为0.65，orb默认为0.3
string seam_find_type = "gc_color"; // 拼接缝隙估计方法
int blend_type = Blender::MULTI_BAND;// 融合方法，默认是多频段融合
float blend_strength = 5;//融合强度，0-100
string result_name = "result.jpg";

vector<std::vector<std::string>> imageNames;
vector<int> idx;

class StitchingVSQt;

string& replace_all_distinct(string&   str, const   string&   old_value, const   string&   new_value)
{
	for (string::size_type pos(0); pos != string::npos; pos += new_value.length())   {
		if ((pos = str.find(old_value, pos)) != string::npos)
			str.replace(pos, old_value.length(), new_value);
		else   break;
	}
	return   str;
}

int startCalculation()
{
	
	StitchingVSQt s;
	std::string msg;
	const char* dllName = "stitching.dll";
    const char* funName1 = "stitch";
	const char* funName2 = "pano_image";
	
	HINSTANCE hDll = LoadLibrary(TEXT("stitching.dll"));
	if (hDll != NULL){
		STITCH fp = STITCH(GetProcAddress(hDll, "stitch"));
		PANOIMAGE fPa = PANOIMAGE(GetProcAddress(hDll, "writeImage"));
		if (fp != NULL){
			//s.showMessage(QString::fromLocal8Bit("找到dll中的函数"));
			void* p=fp(imageNames, nullptr, msg);
			//s.showMessage(QString::fromLocal8Bit("第一个函数调用完毕"));

			QString fileName(QString::fromStdString(result_name));
			QString dir = QFileDialog::getExistingDirectory(NULL, QString::fromLocal8Bit("选择生成图片保存目录"),
				"/home",
				QFileDialog::ShowDirsOnly
				| QFileDialog::DontResolveSymlinks);
			s.changePicture(dir + "/" + fileName);

			fPa(p, (dir + "/" + fileName).toStdString());
			//s.showMessage(QString::fromLocal8Bit("第二个函数调用完毕"));
			/*img_names.clear();*/
			imageNames.clear();
			idx.clear();
			delete p;
		}
		else{
			s.showMessage(QString::fromLocal8Bit("未找到dll中的函数"));
		}
		FreeLibrary(hDll);
	}
	else {
		
		s.showMessage(QString::fromLocal8Bit("未找到dll"));
		return -1;
	}
	return 0;
}


QProgressDialog progress(QString::fromLocal8Bit("正在导入图片数据，请稍候..."),
		QString::fromLocal8Bit("取消"),
		0, 12, // Range
		NULL);

void controlProgress(int status){
	switch (status)
	{
	case 1:
		progress.show();
		qApp->processEvents();
		progress.setWindowModality(Qt::WindowModal);
		progress.setWindowTitle(QString::fromLocal8Bit("正在导入图片数据，请稍候..."));

		progress.setValue(1);
		progress.setModal(true);
		progress.setLabelText(QString::fromLocal8Bit("读取图片，请稍候..."));
		break;
	case 2:
		progress.setValue(2);
		progress.setLabelText(QString::fromLocal8Bit("特征点提取...."));
		break;
	case 3:
		progress.setValue(3);
		progress.setLabelText(QString::fromLocal8Bit("特征点匹配...."));
		break;
	case 4:
		progress.setValue(4);
		progress.setLabelText(QString::fromLocal8Bit("估计相机参数，计算变换矩阵...."));
		break;
	case 5:
		progress.setValue(5);
		progress.setLabelText(QString::fromLocal8Bit("波形矫正...."));
		break;
	case 6:
		progress.setValue(6);
		progress.setLabelText(QString::fromLocal8Bit("柱面投影...."));
		break;
	case 7:
		progress.setValue(7);
		break;
	case 8:
		progress.setValue(8);
		break;
	case 9:
		progress.setValue(9);
		progress.setLabelText(QString::fromLocal8Bit("图像融合...."));
		break;
	case 10:
		progress.setValue(10);
		progress.setLabelText(QString::fromLocal8Bit("光照补偿...."));
		break;
	case 11:
		progress.setValue(11);
		progress.setLabelText(QString::fromLocal8Bit("根据corners顶点和图像大小确定最终全景图尺寸...."));
		break;
	case 12:
		progress.setValue(12);
		break;
	default:
		break;
	}		
}