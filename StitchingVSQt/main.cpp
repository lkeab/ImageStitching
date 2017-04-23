#pragma comment(lib,"opencv_highgui249d.lib")
#pragma comment(lib,"opencv_flann249d.lib")

#include "stitchingvsqt.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{	
	QApplication a(argc, argv);
	StitchingVSQt w;
	w.setWindowIcon(QIcon("./logo.png"));
	w.show();
	
	return a.exec();
}
