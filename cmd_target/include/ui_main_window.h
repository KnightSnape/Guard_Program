/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QLineEdit *WriteX;
    QLabel *NavigationXLabel;
    QLabel *NavigationYLabel;
    QLineEdit *WriteY;
    QPushButton *ClearButton;
    QPushButton *SaveButton;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(656, 445);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        WriteX = new QLineEdit(centralwidget);
        WriteX->setObjectName(QString::fromUtf8("WriteX"));
        WriteX->setGeometry(QRect(360, 140, 191, 25));
        NavigationXLabel = new QLabel(centralwidget);
        NavigationXLabel->setObjectName(QString::fromUtf8("NavigationXLabel"));
        NavigationXLabel->setGeometry(QRect(130, 140, 101, 20));
        NavigationYLabel = new QLabel(centralwidget);
        NavigationYLabel->setObjectName(QString::fromUtf8("NavigationYLabel"));
        NavigationYLabel->setGeometry(QRect(130, 220, 101, 20));
        WriteY = new QLineEdit(centralwidget);
        WriteY->setObjectName(QString::fromUtf8("WriteY"));
        WriteY->setGeometry(QRect(360, 220, 191, 25));
        ClearButton = new QPushButton(centralwidget);
        ClearButton->setObjectName(QString::fromUtf8("ClearButton"));
        ClearButton->setGeometry(QRect(130, 300, 89, 25));
        SaveButton = new QPushButton(centralwidget);
        SaveButton->setObjectName(QString::fromUtf8("SaveButton"));
        SaveButton->setGeometry(QRect(410, 300, 89, 25));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 656, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        NavigationXLabel->setText(QApplication::translate("MainWindow", "NavigationX", nullptr));
        NavigationYLabel->setText(QApplication::translate("MainWindow", "NavigationY", nullptr));
        ClearButton->setText(QApplication::translate("MainWindow", "Clear", nullptr));
        SaveButton->setText(QApplication::translate("MainWindow", "Save", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
