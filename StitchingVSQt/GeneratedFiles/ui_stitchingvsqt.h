/********************************************************************************
** Form generated from reading UI file 'stitchingvsqt.ui'
**
** Created by: Qt User Interface Compiler version 5.3.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_STITCHINGVSQT_H
#define UI_STITCHINGVSQT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_StitchingVSQtClass
{
public:
    QWidget *centralWidget;
    QGraphicsView *graphicsView;
    QFrame *frame;
    QComboBox *comboBox1;
    QLabel *label1;
    QPushButton *pushButton_2;
    QComboBox *comboBox2;
    QComboBox *comboBox3;
    QComboBox *comboBox4;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QSlider *horizontalSlider;
    QPushButton *pushButton_3;
    QMenuBar *menuBar;
    QMenu *menu;
    QMenu *menu_2;
    QMenu *menu_3;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *StitchingVSQtClass)
    {
        if (StitchingVSQtClass->objectName().isEmpty())
            StitchingVSQtClass->setObjectName(QStringLiteral("StitchingVSQtClass"));
        StitchingVSQtClass->resize(657, 539);
        StitchingVSQtClass->setMaximumSize(QSize(16777215, 16777215));
        QIcon icon;
        icon.addFile(QStringLiteral("../Win32/Debug/logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        StitchingVSQtClass->setWindowIcon(icon);
        centralWidget = new QWidget(StitchingVSQtClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        graphicsView = new QGraphicsView(centralWidget);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));
        graphicsView->setGeometry(QRect(5, 10, 681, 461));
        graphicsView->setResizeAnchor(QGraphicsView::AnchorUnderMouse);
        frame = new QFrame(centralWidget);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(10, 10, 651, 441));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        comboBox1 = new QComboBox(frame);
        comboBox1->setObjectName(QStringLiteral("comboBox1"));
        comboBox1->setGeometry(QRect(200, 20, 391, 22));
        label1 = new QLabel(frame);
        label1->setObjectName(QStringLiteral("label1"));
        label1->setGeometry(QRect(70, 20, 81, 16));
        QFont font;
        font.setPointSize(10);
        font.setBold(false);
        font.setWeight(50);
        label1->setFont(font);
        pushButton_2 = new QPushButton(frame);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(570, 410, 75, 23));
        comboBox2 = new QComboBox(frame);
        comboBox2->setObjectName(QStringLiteral("comboBox2"));
        comboBox2->setGeometry(QRect(200, 70, 391, 22));
        comboBox3 = new QComboBox(frame);
        comboBox3->setObjectName(QStringLiteral("comboBox3"));
        comboBox3->setGeometry(QRect(200, 120, 391, 22));
        comboBox4 = new QComboBox(frame);
        comboBox4->setObjectName(QStringLiteral("comboBox4"));
        comboBox4->setGeometry(QRect(200, 170, 391, 22));
        label = new QLabel(frame);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(60, 70, 111, 16));
        QFont font1;
        font1.setPointSize(10);
        label->setFont(font1);
        label_2 = new QLabel(frame);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(80, 120, 71, 16));
        label_2->setFont(font1);
        label_3 = new QLabel(frame);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(60, 170, 101, 16));
        label_3->setFont(font1);
        label_4 = new QLabel(frame);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(60, 220, 101, 20));
        label_4->setFont(font1);
        label_4->setIndent(0);
        horizontalSlider = new QSlider(frame);
        horizontalSlider->setObjectName(QStringLiteral("horizontalSlider"));
        horizontalSlider->setGeometry(QRect(200, 220, 391, 22));
        horizontalSlider->setOrientation(Qt::Horizontal);
        pushButton_3 = new QPushButton(frame);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));
        pushButton_3->setGeometry(QRect(460, 410, 75, 23));
        StitchingVSQtClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(StitchingVSQtClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 657, 23));
        menu = new QMenu(menuBar);
        menu->setObjectName(QStringLiteral("menu"));
        menu_2 = new QMenu(menuBar);
        menu_2->setObjectName(QStringLiteral("menu_2"));
        menu_3 = new QMenu(menuBar);
        menu_3->setObjectName(QStringLiteral("menu_3"));
        StitchingVSQtClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(StitchingVSQtClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        StitchingVSQtClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(StitchingVSQtClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        StitchingVSQtClass->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());
        menuBar->addAction(menu_3->menuAction());
        menuBar->addAction(menu_2->menuAction());

        retranslateUi(StitchingVSQtClass);

        QMetaObject::connectSlotsByName(StitchingVSQtClass);
    } // setupUi

    void retranslateUi(QMainWindow *StitchingVSQtClass)
    {
        StitchingVSQtClass->setWindowTitle(QApplication::translate("StitchingVSQtClass", "ImageStitcher", 0));
        label1->setText(QApplication::translate("StitchingVSQtClass", "FeatureType:", 0));
        pushButton_2->setText(QApplication::translate("StitchingVSQtClass", "\345\256\214\346\210\220\350\256\276\347\275\256", 0));
        label->setText(QApplication::translate("StitchingVSQtClass", "Ba_cost_func:", 0));
        label_2->setText(QApplication::translate("StitchingVSQtClass", "Warp_type:", 0));
        label_3->setText(QApplication::translate("StitchingVSQtClass", "Seam_find_type:", 0));
        label_4->setText(QApplication::translate("StitchingVSQtClass", "Blend_strength:", 0));
        pushButton_3->setText(QApplication::translate("StitchingVSQtClass", "\350\277\224\345\233\236", 0));
        menu->setTitle(QApplication::translate("StitchingVSQtClass", "\346\226\207\344\273\266", 0));
        menu_2->setTitle(QApplication::translate("StitchingVSQtClass", "\350\256\276\347\275\256", 0));
        menu_3->setTitle(QApplication::translate("StitchingVSQtClass", "\346\240\207\345\256\232", 0));
    } // retranslateUi

};

namespace Ui {
    class StitchingVSQt: public Ui_StitchingVSQtClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_STITCHINGVSQT_H
