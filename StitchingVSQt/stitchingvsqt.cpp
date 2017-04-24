#include "stitchingvsqt.h"
#include "ui_stitchingvsqt.h"
#include "stitching.h"
#include <qmessagebox.h>
#include <qfiledialog.h>
#include <qprogressdialog.h>
#include <iostream>
#include <fstream>
#include <QRunnable>  
#include <QCompleter>
#include <QMainWindow>
#include <QInputDialog>

#include <QtConcurrent\QtConcurrentRun>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <qgraphicsview.h>
#include <qstringlist.h>
#include <string>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

using std::string;
using std::ifstream;
using std::ofstream;
using std::cout;
using namespace cv;
bool entered;
bool refreshed;

QGraphicsScene *scene;
QGraphicsScene *scene1;
QGraphicsView* qgv;
//Ui::StitchingVSQt *ui1;

extern string features_type;
extern string ba_cost_func;
extern string warp_type;
extern string seam_find_type;
extern string result_name;
extern float  blend_strength;
extern vector<std::vector<std::string>> imageNames;
extern vector<int> idx;

QString path="";

StitchingVSQt::StitchingVSQt(QWidget *parent)
	: QMainWindow(parent), ui(new Ui::StitchingVSQt)
{
	ui->setupUi(this);
	QStringList strings;
	strings << "surf" << "orb" ;
	QStringList strings1;
	strings1 << "ray" << "reproj";
	QStringList strings2;
	strings2 << "spherical " << "plane" << "cylindrical";
	QStringList strings3;
	strings3 << "gc_color " << "voronoi" << "gc_colorgrad"<<"no";

	cvSetErrMode(CV_ErrModeParent);
	/*cvRedirectError(errorReport);*/

	ui->comboBox1->addItems(strings);
	ui->comboBox2->addItems(strings1);
	ui->comboBox3->addItems(strings2);
	ui->comboBox4->addItems(strings3);

	ui->frame->setVisible(false);

	ui->horizontalSlider->setMaximum(100);
	ui->horizontalSlider->setValue(5);

	setWindowFlags(windowFlags()& ~Qt::WindowMaximizeButtonHint);
	setFixedSize(this->width(), this->height());

	entered = false;
	QAction *import = new QAction(QString::fromLocal8Bit("导入图片文件"), this);
	QAction *exitApp = new QAction(QString::fromLocal8Bit("退出"), this);
	QAction *setting = new QAction(QString::fromLocal8Bit("详细参数设置"), this);
	QAction *calibrate = new QAction(QString::fromLocal8Bit("标定"), this);

	ui->menu->addAction(import);
	ui->menu->addAction(exitApp);
	ui->menu_3->addAction(calibrate);
	ui->menu_2->addAction(setting);

	this->image = new QImage();
	
	 
	connect(import, SIGNAL(triggered()), this, SLOT(fileOpenActionSlot()));
	connect(exitApp, SIGNAL(triggered()), this, SLOT(fileCloseActionSlot()));
	//connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(startCal()));

	connect(setting, SIGNAL(triggered()), this, SLOT(setDetail()));
	connect(calibrate, SIGNAL(triggered()), this, SLOT(calibrate()));
	connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(finishSetting()));
	connect(ui->pushButton_3, SIGNAL(clicked()), this, SLOT(back()));

	QString path1 = QCoreApplication::applicationDirPath();

	path1.append(QString("/imgBack.jpg"));

	if (image->load(path1))
	{
		QGraphicsScene *scene = new QGraphicsScene;
		scene->addPixmap(QPixmap::fromImage(*image).scaled(680, 450));
		ui->graphicsView->setScene(scene);
		ui->graphicsView->show();
	}
}

StitchingVSQt::~StitchingVSQt()
{
	/*delete ui;
    delete image;*/
}

int StitchingVSQt::errorReport(){
	QMessageBox::warning(this, QString::fromLocal8Bit("提示"), QString::fromLocal8Bit("所选取的图片重合程度过低"));
	return -1;
}

void StitchingVSQt::startCal(){
	if (entered){
		try{	
			//QtConcurrent::run(startCalculation);
			startCalculation();
		}
		catch (cv::Exception& e){
			const char*m = e.what();
			string s(m);
			QMessageBox::warning(this, QString::fromLocal8Bit("提示"), QString::fromStdString("所选取的图片重合程度过低"+s));
		}
		
		if (image->load(path))
		{
			QGraphicsScene *scene = new QGraphicsScene;
			scene->addPixmap(QPixmap::fromImage(*image)/*.scaled(1000,800)*/);
			ui->graphicsView->setScene(scene);
			ui->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
			ui->graphicsView->resize(681, 461);
			//ui->graphicsView->addScrollBarWidget->setVisible(false);
			ui->graphicsView->show();
		}
	}
	else
	{
		QMessageBox::warning(this, QString::fromLocal8Bit("提示"), QString::fromLocal8Bit("请先导入相应需要拼接的图片"));
	}
}

void StitchingVSQt::fileOpenActionSlot()
{
//	QMessageBox::warning(this, QString::fromLocal8Bit("提示"), QString::fromLocal8Bit("你点击了~打开文件~菜单"), QMessageBox::Yes | QMessageBox::No);
	
	vector<QStringList>imagNameTemp;
	int i = 0;
    while(true){
			std::stringstream stream;
			stream << i+1; 
			QStringList fileNames;
			//fileNames;
			QStringList	filesTemp=QFileDialog::getOpenFileNames(this,
				QString::fromLocal8Bit("第") +QString::fromStdString(stream.str()) + QString::fromLocal8Bit("层"),
				"",
				"IMAGE Files(*.jpg)",
				0);
			if ((filesTemp.length() == 0) && (imagNameTemp.size() != 0)){
				break;
			}

			if ((filesTemp.length() == 0) && (imagNameTemp.size()==0)){
				return;
			}
			fileNames.append(filesTemp);
			imagNameTemp.push_back(fileNames);
			idx.push_back(0);
			i++;
			//imagNameTemp.push_back()
			/*if (imagNameTemp.size() == 0)
			{
				return;
			}*/
	}
	//QMessageBox::StandardButton rb = QMessageBox::question(NULL, QString::fromLocal8Bit("图像层数"), QString::fromLocal8Bit("是否分层导入图像？"), QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
	//if (rb == QMessageBox::Yes)
	//{
	//	bool isOK;
	//	int level = 0;
	//	QString text = QInputDialog::getText(NULL, QString::fromLocal8Bit("层数"),
	//		QString::fromLocal8Bit("请输入有效的拼接层数"),
	//		QLineEdit::Normal,
	//		"",
	//		&isOK);
	//	if (isOK) {
	//		level = text.toInt();
	//		if (level <= 0){
	//			return;
	//		}
	//	}
	//	else{
	//		return;
	//	}
	//	for (int i = 0; i <level; i++){
	//		std::stringstream stream;
	//		stream << i+1; 
	//		QStringList fileNames;
	//		//fileNames;
	//		QStringList	filesTemp=QFileDialog::getOpenFileNames(this,
	//			QString::fromLocal8Bit("第") +QString::fromStdString(stream.str()) + QString::fromLocal8Bit("层"),
	//			"",
	//			"IMAGE Files(*.jpg)",
	//			0);
	//		fileNames.append(filesTemp);
	//		imagNameTemp.push_back(fileNames);
	//		idx.push_back(0);
	//		//imagNameTemp.push_back()
	//		if (fileNames.length() == 0)
	//		{
	//			return;
	//		}
	//	}
	//}
	//else{
	//	QStringList fileNames;
	//	fileNames = QFileDialog::getOpenFileNames(this,
	//		tr("Open File"),
	//		"",
	//		"IMAGE Files(*.jpg)",
	//		0);
	//	if (fileNames.length() == 0)
	//	{
	//		return;
	//	}
	//	imagNameTemp.push_back(fileNames);
	//	idx.push_back(0);
	//}
	
	//QProgressDialog progress(tr("正在导入图片数据，请稍候..."),
	//	tr("取消"),
	//	0, fileNames.length(), // Range
	//	this);
	//progress.show();
	//qApp->processEvents();
	//progress.setWindowModality(Qt::WindowModal);

	/*for (int i = 0; i < fileNames.length(); i++)
	{
		progress.setValue(i);
		string fileName = fileNames[i].toLocal8Bit().data();

		
		storeImageFileName(fileName);
		
		if (progress.wasCanceled()) {
			break;
		}
	}*/
	for (int i = 0; i < imagNameTemp.size(); i++)
	{
		vector<string> tempF;
		for (int j = 0; j < imagNameTemp[i].length(); j++){
			tempF.push_back(imagNameTemp[i][j].toLocal8Bit().data());
		}
		//progress.setValue(i);
		//string fileName = fileNames[i].toLocal8Bit().data();
		imageNames.push_back(tempF);
		//storeImageFileName(fileName);
	}
	/*progress.setValue(fileNames.length());*/
	entered = true;
//	QMessageBox::warning(this, QString::fromLocal8Bit("提示"), QString::fromLocal8Bit("图片选取成功，请点击开始计算按钮"));
	startCal();
	//QtConcurrent::run(startCalculation);
}

void StitchingVSQt::fileCloseActionSlot()
{
	QApplication* app;
	app->exit(0);
}

void StitchingVSQt::showMessage(QString content){
	QMessageBox::warning(this, QString::fromLocal8Bit("提示"), content, QMessageBox::Yes | QMessageBox::No);
}

void StitchingVSQt::changePicture(QString path1){
	path = path1;
}

void StitchingVSQt::setDetail(){
	ui->graphicsView->setVisible(false);
//	ui->pushButton->setVisible(false);
	ui->frame->setVisible(true);
}

void StitchingVSQt::calibrate(){
	//QMessageBox::warning(this, QString::fromLocal8Bit("提示"), QString::fromLocal8Bit("开始标定,请耐心等待！"));
	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(this, QString::fromLocal8Bit("提示"), QString::fromLocal8Bit("是否开始标定，若是，请耐心等待！"), QMessageBox::Yes | QMessageBox::No);
	if (reply == QMessageBox::Yes){
		//ifstream fin("calibdata.txt"); /* 标定所用图像文件的路径 */
		QStringList fileNames = QFileDialog::getOpenFileNames(this,
			QString::fromLocal8Bit("请选择标定图片"),
			"",
			"IMAGE Files(*.jpg)",
			0);
		if (fileNames.length() == 0)
		{
			return;
		}
		ofstream fout("caliberation_result.txt");  /* 保存标定结果的文件 */
		//读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化	
		//cout << "开始提取角点………………";
		int image_count = 0;  /* 图像数量 */
		Size image_size;  /* 图像的尺寸 */
		Size board_size = Size(6, 9);    /* 标定板上每行、列的角点数 */
		vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
		vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
		//string filename;
		int count = -1;//用于存储角点个数。

		for (int i = 0; i < fileNames.length(); i++)
		{
			string fileName = fileNames[i].toLocal8Bit().data();
			image_count++;
			// 用于观察检验输出
			//cout << "image_count = " << image_count << std::endl;
			///* 输出检验*/
			//cout << "-->count = " << count;
			Mat imageInput = imread(fileName);
			if (image_count == 1)  //读入第一张图片时获取图像宽高信息
			{
				image_size.width = imageInput.cols;
				image_size.height = imageInput.rows;
				cout << "image_size.width = " << image_size.width << std::endl;
				cout << "image_size.height = " << image_size.height << std::endl;
			}

			/* 提取角点 */
			if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
			{
				cout << "can not find chessboard corners!\n"; //找不到角点
				exit(1);
			}
			else
			{
				Mat view_gray;
				cvtColor(imageInput, view_gray, CV_RGB2GRAY);
				/* 亚像素精确化 */
				find4QuadCornerSubpix(view_gray, image_points_buf, Size(11, 11)); //对粗提取的角点进行精确化
				image_points_seq.push_back(image_points_buf);  //保存亚像素角点
				/* 在图像上显示角点位置 */
				drawChessboardCorners(view_gray, board_size, image_points_buf, true); //用于在图片中标记角点	
			}
		}
		
		int total = image_points_seq.size();
		cout << "total = " << total << std::endl;
		int CornerNum = board_size.width*board_size.height;  //每张图片上总的角点数
		for (int ii = 0; ii<total; ii++)
		{
			if (0 == ii%CornerNum)// 24 是每幅图片的角点个数。此判断语句是为了输出 图片号，便于控制台观看 
			{
				int i = -1;
				i = ii / CornerNum;
				int j = i + 1;
				cout << "--> 第 " << j << "图片的数据 --> : " << std::endl;
			}
			if (0 == ii % 3)	// 此判断语句，格式化输出，便于控制台查看
			{
				cout << std::endl;
			}
			else
			{
				cout.width(10);
			}

			//输出所有的角点
			cout << " -->" << image_points_seq[ii][0].x;
			cout << " -->" << image_points_seq[ii][0].y;
		}
		cout << "角点提取完成！\n";


		//以下是摄像机标定

		cout << "开始标定………………";
		/*棋盘三维信息*/
		Size square_size = Size(90, 90);  /* 实际测量得到的标定板上每个棋盘格的大小 */
		vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
		/*内外参数*/
		Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 摄像机内参数矩阵 */
		vector<int> point_counts;  // 每幅图像中角点的数量
		Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
		vector<Mat> tvecsMat;  /* 每幅图像的旋转向量 */
		vector<Mat> rvecsMat; /* 每幅图像的平移向量 */
		/* 初始化标定板上角点的三维坐标 */
		int i, j, t;
		for (t = 0; t<image_count; t++)
		{
			vector<Point3f> tempPointSet;
			for (i = 0; i<board_size.height; i++)
			{
				for (j = 0; j<board_size.width; j++)
				{
					Point3f realPoint;
					/* 假设标定板放在世界坐标系中z=0的平面上 */
					realPoint.x = i*square_size.width;
					realPoint.y = j*square_size.height;
					realPoint.z = 0;
					tempPointSet.push_back(realPoint);
				}
			}
			object_points.push_back(tempPointSet);
		}
		/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
		for (i = 0; i<image_count; i++)
		{
			point_counts.push_back(board_size.width*board_size.height);
		}
		/* 开始标定 */
		calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
		//	cout << "标定完成！\n";

		//对标定结果进行评价
		cout << "开始评价标定结果………………\n";
		double total_err = 0.0; /* 所有图像的平均误差的总和 */
		double err = 0.0; /* 每幅图像的平均误差 */
		vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
		//cout << "\t每幅图像的标定误差：\n";
		//fout << "每幅图像的标定误差：\n";
		for (i = 0; i<image_count; i++)
		{
			vector<Point3f> tempPointSet = object_points[i];
			/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
			projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
			/* 计算新的投影点和旧的投影点之间的误差*/
			vector<Point2f> tempImagePoint = image_points_seq[i];
			Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
			Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
			for (int j = 0; j < tempImagePoint.size(); j++)
			{
				image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
				tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
			}
			err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
			total_err += err /= point_counts[i];
		}
		//保存定标结果  	
		//std::cout << "开始保存定标结果………………" << std::endl;
		Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
		fout << "相机内参数矩阵：" << std::endl;
		fout << cameraMatrix << std::endl << std::endl;
		fout << std::endl;
		QMessageBox::warning(this, QString::fromLocal8Bit("提示"), QString::fromLocal8Bit("标定结束，结果存储至文件caliberation_result.txt！"));
	}
	else
	{
		return;
	}
}

void StitchingVSQt::finishSetting(){

	features_type = ui->comboBox1->currentText().toStdString();
	ba_cost_func = QString(ui->comboBox2->currentText()).toStdString();
	warp_type = QString(ui->comboBox3->currentText()).toStdString();
	seam_find_type = QString(ui->comboBox4->currentText()).toStdString();
	blend_strength = ui->horizontalSlider->value();

	ui->graphicsView->setVisible(true);
//	ui->pushButton->setVisible(true);
	ui->frame->setVisible(false);
}

void StitchingVSQt::back(){
	ui->graphicsView->setVisible(true);
//	ui->pushButton->setVisible(true);
	ui->frame->setVisible(false);
}