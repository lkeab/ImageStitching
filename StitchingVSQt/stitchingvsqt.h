#ifndef STITCHINGVSQT_H
#define STITCHINGVSQT_H

#include <QtWidgets/QMainWindow>
#include "ui_stitchingvsqt.h"

//namespace Ui {
//	class StitchingVSQt;
//}

class StitchingVSQt : public QMainWindow
{
	Q_OBJECT

public:
	 StitchingVSQt(QWidget *parent = 0);
	~StitchingVSQt();
	void showMessage(QString content);
    void changePicture(QString path);
	int errorReport();

public slots:
	void fileOpenActionSlot();
	void fileCloseActionSlot();
	void startCal();

	void setDetail();
	void finishSetting();
	void back();
	void calibrate();

public:
	 Ui::StitchingVSQt *ui;
	 QImage*image;
};

#endif // STITCHINGVSQT_H
