/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *cameraViewer_1;
    QLabel *label_3;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QLabel *labelIP;
    QLineEdit *textIP;
    QLabel *labelPort;
    QLineEdit *textPort;
    QPushButton *buttonOpen;
    QGroupBox *groupBox;
    QLabel *label_Port;
    QLabel *label_Baudrate;
    QComboBox *ComboBox_Port;
    QComboBox *ComboBox_Baudrate;
    QPushButton *Button_Connect;
    QPushButton *Button_refresh;
    QLabel *cameraViewer_2;
    QLabel *cameraViewer_3;
    QLabel *cameraViewer_4;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1239, 720);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        cameraViewer_1 = new QLabel(centralWidget);
        cameraViewer_1->setObjectName(QStringLiteral("cameraViewer_1"));
        cameraViewer_1->setGeometry(QRect(20, 10, 431, 321));
        cameraViewer_1->setFrameShape(QFrame::Box);
        cameraViewer_1->setAlignment(Qt::AlignCenter);
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(910, 290, 211, 71));
        QFont font;
        font.setPointSize(22);
        font.setBold(true);
        font.setWeight(75);
        label_3->setFont(font);
        label_3->setAlignment(Qt::AlignCenter);
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(960, 190, 261, 91));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        labelIP = new QLabel(layoutWidget);
        labelIP->setObjectName(QStringLiteral("labelIP"));
        QFont font1;
        font1.setFamily(QStringLiteral("Arial Unicode MS"));
        font1.setPointSize(12);
        font1.setBold(true);
        font1.setWeight(75);
        labelIP->setFont(font1);

        gridLayout->addWidget(labelIP, 0, 0, 1, 1);

        textIP = new QLineEdit(layoutWidget);
        textIP->setObjectName(QStringLiteral("textIP"));
        QFont font2;
        font2.setPointSize(10);
        textIP->setFont(font2);

        gridLayout->addWidget(textIP, 0, 1, 1, 1);

        labelPort = new QLabel(layoutWidget);
        labelPort->setObjectName(QStringLiteral("labelPort"));
        labelPort->setFont(font1);

        gridLayout->addWidget(labelPort, 1, 0, 1, 1);

        textPort = new QLineEdit(layoutWidget);
        textPort->setObjectName(QStringLiteral("textPort"));
        textPort->setFont(font2);

        gridLayout->addWidget(textPort, 1, 1, 1, 1);


        verticalLayout->addLayout(gridLayout);

        buttonOpen = new QPushButton(layoutWidget);
        buttonOpen->setObjectName(QStringLiteral("buttonOpen"));
        QFont font3;
        font3.setFamily(QStringLiteral("Arial Narrow"));
        font3.setPointSize(11);
        font3.setBold(true);
        font3.setWeight(75);
        buttonOpen->setFont(font3);

        verticalLayout->addWidget(buttonOpen);

        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(940, 400, 281, 251));
        QFont font4;
        font4.setBold(true);
        font4.setWeight(75);
        groupBox->setFont(font4);
        label_Port = new QLabel(groupBox);
        label_Port->setObjectName(QStringLiteral("label_Port"));
        label_Port->setGeometry(QRect(40, 70, 56, 12));
        label_Baudrate = new QLabel(groupBox);
        label_Baudrate->setObjectName(QStringLiteral("label_Baudrate"));
        label_Baudrate->setGeometry(QRect(30, 140, 71, 16));
        ComboBox_Port = new QComboBox(groupBox);
        ComboBox_Port->setObjectName(QStringLiteral("ComboBox_Port"));
        ComboBox_Port->setGeometry(QRect(170, 70, 76, 22));
        ComboBox_Baudrate = new QComboBox(groupBox);
        ComboBox_Baudrate->setObjectName(QStringLiteral("ComboBox_Baudrate"));
        ComboBox_Baudrate->setGeometry(QRect(170, 130, 76, 22));
        Button_Connect = new QPushButton(groupBox);
        Button_Connect->setObjectName(QStringLiteral("Button_Connect"));
        Button_Connect->setGeometry(QRect(30, 190, 251, 41));
        Button_refresh = new QPushButton(groupBox);
        Button_refresh->setObjectName(QStringLiteral("Button_refresh"));
        Button_refresh->setGeometry(QRect(170, 30, 75, 23));
        cameraViewer_2 = new QLabel(centralWidget);
        cameraViewer_2->setObjectName(QStringLiteral("cameraViewer_2"));
        cameraViewer_2->setGeometry(QRect(460, 10, 431, 321));
        cameraViewer_2->setFrameShape(QFrame::Box);
        cameraViewer_2->setAlignment(Qt::AlignCenter);
        cameraViewer_3 = new QLabel(centralWidget);
        cameraViewer_3->setObjectName(QStringLiteral("cameraViewer_3"));
        cameraViewer_3->setGeometry(QRect(20, 330, 431, 321));
        cameraViewer_3->setFrameShape(QFrame::Box);
        cameraViewer_3->setAlignment(Qt::AlignCenter);
        cameraViewer_4 = new QLabel(centralWidget);
        cameraViewer_4->setObjectName(QStringLiteral("cameraViewer_4"));
        cameraViewer_4->setGeometry(QRect(460, 330, 431, 321));
        cameraViewer_4->setFrameShape(QFrame::Box);
        cameraViewer_4->setAlignment(Qt::AlignCenter);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1239, 21));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        cameraViewer_1->setText(QApplication::translate("MainWindow", "TextLabel", 0));
        label_3->setText(QApplication::translate("MainWindow", "Recv", 0));
        labelIP->setText(QApplication::translate("MainWindow", "IP", 0));
        textIP->setText(QString());
        labelPort->setText(QApplication::translate("MainWindow", "Port", 0));
        textPort->setText(QString());
        buttonOpen->setText(QApplication::translate("MainWindow", "OPEN", 0));
        groupBox->setTitle(QApplication::translate("MainWindow", "Setting", 0));
        label_Port->setText(QApplication::translate("MainWindow", "Port", 0));
        label_Baudrate->setText(QApplication::translate("MainWindow", "Baudrate", 0));
        ComboBox_Port->clear();
        ComboBox_Port->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "select", 0)
        );
        ComboBox_Baudrate->clear();
        ComboBox_Baudrate->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "select", 0)
        );
        Button_Connect->setText(QApplication::translate("MainWindow", "Connect", 0));
        Button_refresh->setText(QApplication::translate("MainWindow", "refresh", 0));
        cameraViewer_2->setText(QApplication::translate("MainWindow", "TextLabel", 0));
        cameraViewer_3->setText(QApplication::translate("MainWindow", "TextLabel", 0));
        cameraViewer_4->setText(QApplication::translate("MainWindow", "TextLabel", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
