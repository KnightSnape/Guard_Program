/********************************************************************************
** Form generated from reading UI file 'cmd_window.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CMD_WINDOW_H
#define UI_CMD_WINDOW_H

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

class Ui_Navigation_Window
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

    void setupUi(QMainWindow *Navigation_Window)
    {
        if (Navigation_Window->objectName().isEmpty())
            Navigation_Window->setObjectName(QString::fromUtf8("Navigation_Window"));
        Navigation_Window->resize(408, 200);
        centralwidget = new QWidget(Navigation_Window);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        WriteX = new QLineEdit(centralwidget);
        WriteX->setObjectName(QString::fromUtf8("WriteX"));
        WriteX->setGeometry(QRect(200, 30, 191, 25));
        NavigationXLabel = new QLabel(centralwidget);
        NavigationXLabel->setObjectName(QString::fromUtf8("NavigationXLabel"));
        NavigationXLabel->setGeometry(QRect(40, 30, 101, 20));
        NavigationYLabel = new QLabel(centralwidget);
        NavigationYLabel->setObjectName(QString::fromUtf8("NavigationYLabel"));
        NavigationYLabel->setGeometry(QRect(40, 80, 101, 20));
        WriteY = new QLineEdit(centralwidget);
        WriteY->setObjectName(QString::fromUtf8("WriteY"));
        WriteY->setGeometry(QRect(200, 80, 191, 25));
        ClearButton = new QPushButton(centralwidget);
        ClearButton->setObjectName(QString::fromUtf8("ClearButton"));
        ClearButton->setGeometry(QRect(40, 130, 89, 25));
        SaveButton = new QPushButton(centralwidget);
        SaveButton->setObjectName(QString::fromUtf8("SaveButton"));
        SaveButton->setGeometry(QRect(240, 130, 89, 25));
        Navigation_Window->setCentralWidget(centralwidget);
        menubar = new QMenuBar(Navigation_Window);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 408, 22));
        Navigation_Window->setMenuBar(menubar);
        statusbar = new QStatusBar(Navigation_Window);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        Navigation_Window->setStatusBar(statusbar);

        retranslateUi(Navigation_Window);

        QMetaObject::connectSlotsByName(Navigation_Window);
    } // setupUi

    void retranslateUi(QMainWindow *Navigation_Window)
    {
        Navigation_Window->setWindowTitle(QApplication::translate("Navigation_Window", "MainWindow", nullptr));
        NavigationXLabel->setText(QApplication::translate("Navigation_Window", "NavigationX", nullptr));
        NavigationYLabel->setText(QApplication::translate("Navigation_Window", "NavigationY", nullptr));
        ClearButton->setText(QApplication::translate("Navigation_Window", "Clear", nullptr));
        SaveButton->setText(QApplication::translate("Navigation_Window", "Save", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Navigation_Window: public Ui_Navigation_Window {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CMD_WINDOW_H
