/********************************************************************************
** Form generated from reading UI file 'Garfield_control.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GARFIELD_CONTROL_H
#define UI_GARFIELD_CONTROL_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Garfield_control
{
public:
    QAction *actionSettings;
    QAction *actionDebug;
    QWidget *centralwidget;
    QSlider *DebugSlider_Dir;
    QSlider *DebugSlider_speed;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_left;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_up;
    QPushButton *pushButton_down;
    QPushButton *pushButton_right;
    QCheckBox *checkBox_light;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_2;
    QLabel *angle_label;
    QLineEdit *angle_lineEdit;
    QLabel *speed_label;
    QLineEdit *speed_lineEdit;
    QLabel *AccGrid_label;
    QLabel *GridPoint_label;
    QLabel *acc_label;
    QPushButton *connect_pushButton;
    QLabel *connstate_label;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *Gyro_X_label;
    QLineEdit *Gyro_X_lineEdit;
    QHBoxLayout *horizontalLayout_4;
    QLabel *Gyro_Y_label;
    QLineEdit *Gyro_Y_lineEdit;
    QHBoxLayout *horizontalLayout_5;
    QLabel *Gyro_Z_label;
    QLineEdit *Gyro_Z_lineEdit;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_6;
    QLabel *temperature_label;
    QLineEdit *temperatur_lineEdit;
    QMenuBar *menubar;
    QMenu *menuConfig;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Garfield_control)
    {
        if (Garfield_control->objectName().isEmpty())
            Garfield_control->setObjectName(QStringLiteral("Garfield_control"));
        Garfield_control->resize(759, 298);
        actionSettings = new QAction(Garfield_control);
        actionSettings->setObjectName(QStringLiteral("actionSettings"));
        actionDebug = new QAction(Garfield_control);
        actionDebug->setObjectName(QStringLiteral("actionDebug"));
        actionDebug->setCheckable(true);
        centralwidget = new QWidget(Garfield_control);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        DebugSlider_Dir = new QSlider(centralwidget);
        DebugSlider_Dir->setObjectName(QStringLiteral("DebugSlider_Dir"));
        DebugSlider_Dir->setEnabled(false);
        DebugSlider_Dir->setGeometry(QRect(445, 240, 70, 16));
        DebugSlider_Dir->setMinimum(0);
        DebugSlider_Dir->setMaximum(180);
        DebugSlider_Dir->setValue(90);
        DebugSlider_Dir->setOrientation(Qt::Horizontal);
        DebugSlider_speed = new QSlider(centralwidget);
        DebugSlider_speed->setObjectName(QStringLiteral("DebugSlider_speed"));
        DebugSlider_speed->setEnabled(false);
        DebugSlider_speed->setGeometry(QRect(470, 170, 16, 70));
        DebugSlider_speed->setMinimum(-100);
        DebugSlider_speed->setMaximum(100);
        DebugSlider_speed->setValue(0);
        DebugSlider_speed->setOrientation(Qt::Vertical);
        layoutWidget = new QWidget(centralwidget);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 70, 391, 111));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_left = new QPushButton(layoutWidget);
        pushButton_left->setObjectName(QStringLiteral("pushButton_left"));

        horizontalLayout->addWidget(pushButton_left);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        pushButton_up = new QPushButton(layoutWidget);
        pushButton_up->setObjectName(QStringLiteral("pushButton_up"));

        verticalLayout->addWidget(pushButton_up);

        pushButton_down = new QPushButton(layoutWidget);
        pushButton_down->setObjectName(QStringLiteral("pushButton_down"));

        verticalLayout->addWidget(pushButton_down);


        horizontalLayout->addLayout(verticalLayout);

        pushButton_right = new QPushButton(layoutWidget);
        pushButton_right->setObjectName(QStringLiteral("pushButton_right"));

        horizontalLayout->addWidget(pushButton_right);

        checkBox_light = new QCheckBox(layoutWidget);
        checkBox_light->setObjectName(QStringLiteral("checkBox_light"));

        horizontalLayout->addWidget(checkBox_light);

        layoutWidget1 = new QWidget(centralwidget);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 220, 371, 25));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        angle_label = new QLabel(layoutWidget1);
        angle_label->setObjectName(QStringLiteral("angle_label"));

        horizontalLayout_2->addWidget(angle_label);

        angle_lineEdit = new QLineEdit(layoutWidget1);
        angle_lineEdit->setObjectName(QStringLiteral("angle_lineEdit"));
        angle_lineEdit->setEnabled(true);
        angle_lineEdit->setAcceptDrops(true);
        angle_lineEdit->setReadOnly(true);

        horizontalLayout_2->addWidget(angle_lineEdit);

        speed_label = new QLabel(layoutWidget1);
        speed_label->setObjectName(QStringLiteral("speed_label"));

        horizontalLayout_2->addWidget(speed_label);

        speed_lineEdit = new QLineEdit(layoutWidget1);
        speed_lineEdit->setObjectName(QStringLiteral("speed_lineEdit"));
        speed_lineEdit->setReadOnly(true);

        horizontalLayout_2->addWidget(speed_lineEdit);

        AccGrid_label = new QLabel(centralwidget);
        AccGrid_label->setObjectName(QStringLiteral("AccGrid_label"));
        AccGrid_label->setGeometry(QRect(430, 45, 120, 120));
        AccGrid_label->setPixmap(QPixmap(QString::fromUtf8("acceleration_grid.png")));
        GridPoint_label = new QLabel(centralwidget);
        GridPoint_label->setObjectName(QStringLiteral("GridPoint_label"));
        GridPoint_label->setGeometry(QRect(485, 100, 10, 10));
        GridPoint_label->setPixmap(QPixmap(QString::fromUtf8("acceleration_point.png")));
        acc_label = new QLabel(centralwidget);
        acc_label->setObjectName(QStringLiteral("acc_label"));
        acc_label->setGeometry(QRect(430, 20, 78, 23));
        connect_pushButton = new QPushButton(centralwidget);
        connect_pushButton->setObjectName(QStringLiteral("connect_pushButton"));
        connect_pushButton->setGeometry(QRect(10, 20, 91, 22));
        connstate_label = new QLabel(centralwidget);
        connstate_label->setObjectName(QStringLiteral("connstate_label"));
        connstate_label->setGeometry(QRect(110, 20, 131, 23));
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(560, 40, 191, 162));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        Gyro_X_label = new QLabel(verticalLayoutWidget);
        Gyro_X_label->setObjectName(QStringLiteral("Gyro_X_label"));

        horizontalLayout_3->addWidget(Gyro_X_label);

        Gyro_X_lineEdit = new QLineEdit(verticalLayoutWidget);
        Gyro_X_lineEdit->setObjectName(QStringLiteral("Gyro_X_lineEdit"));
        Gyro_X_lineEdit->setReadOnly(true);

        horizontalLayout_3->addWidget(Gyro_X_lineEdit);


        verticalLayout_2->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        Gyro_Y_label = new QLabel(verticalLayoutWidget);
        Gyro_Y_label->setObjectName(QStringLiteral("Gyro_Y_label"));

        horizontalLayout_4->addWidget(Gyro_Y_label);

        Gyro_Y_lineEdit = new QLineEdit(verticalLayoutWidget);
        Gyro_Y_lineEdit->setObjectName(QStringLiteral("Gyro_Y_lineEdit"));
        Gyro_Y_lineEdit->setReadOnly(true);

        horizontalLayout_4->addWidget(Gyro_Y_lineEdit);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        Gyro_Z_label = new QLabel(verticalLayoutWidget);
        Gyro_Z_label->setObjectName(QStringLiteral("Gyro_Z_label"));

        horizontalLayout_5->addWidget(Gyro_Z_label);

        Gyro_Z_lineEdit = new QLineEdit(verticalLayoutWidget);
        Gyro_Z_lineEdit->setObjectName(QStringLiteral("Gyro_Z_lineEdit"));
        Gyro_Z_lineEdit->setReadOnly(true);

        horizontalLayout_5->addWidget(Gyro_Z_lineEdit);


        verticalLayout_2->addLayout(horizontalLayout_5);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Fixed);

        verticalLayout_2->addItem(verticalSpacer);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        temperature_label = new QLabel(verticalLayoutWidget);
        temperature_label->setObjectName(QStringLiteral("temperature_label"));

        horizontalLayout_6->addWidget(temperature_label);

        temperatur_lineEdit = new QLineEdit(verticalLayoutWidget);
        temperatur_lineEdit->setObjectName(QStringLiteral("temperatur_lineEdit"));
        temperatur_lineEdit->setReadOnly(true);

        horizontalLayout_6->addWidget(temperatur_lineEdit);


        verticalLayout_2->addLayout(horizontalLayout_6);

        Garfield_control->setCentralWidget(centralwidget);
        menubar = new QMenuBar(Garfield_control);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 759, 19));
        menuConfig = new QMenu(menubar);
        menuConfig->setObjectName(QStringLiteral("menuConfig"));
        Garfield_control->setMenuBar(menubar);
        statusBar = new QStatusBar(Garfield_control);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Garfield_control->setStatusBar(statusBar);

        menubar->addAction(menuConfig->menuAction());
        menuConfig->addAction(actionSettings);
        menuConfig->addAction(actionDebug);

        retranslateUi(Garfield_control);

        QMetaObject::connectSlotsByName(Garfield_control);
    } // setupUi

    void retranslateUi(QMainWindow *Garfield_control)
    {
        Garfield_control->setWindowTitle(QApplication::translate("Garfield_control", "Garfield Control", Q_NULLPTR));
        actionSettings->setText(QApplication::translate("Garfield_control", "Settings", Q_NULLPTR));
        actionDebug->setText(QApplication::translate("Garfield_control", "Debug", Q_NULLPTR));
        pushButton_left->setText(QApplication::translate("Garfield_control", "\342\227\200", Q_NULLPTR));
        pushButton_up->setText(QApplication::translate("Garfield_control", "\342\226\262", Q_NULLPTR));
        pushButton_down->setText(QApplication::translate("Garfield_control", "\342\226\274", Q_NULLPTR));
        pushButton_right->setText(QApplication::translate("Garfield_control", "\342\226\266", Q_NULLPTR));
        checkBox_light->setText(QApplication::translate("Garfield_control", "light", Q_NULLPTR));
        angle_label->setText(QApplication::translate("Garfield_control", "angle", Q_NULLPTR));
        speed_label->setText(QApplication::translate("Garfield_control", "speed", Q_NULLPTR));
        AccGrid_label->setText(QString());
        GridPoint_label->setText(QString());
        acc_label->setText(QApplication::translate("Garfield_control", "acceleration", Q_NULLPTR));
        connect_pushButton->setText(QApplication::translate("Garfield_control", "connect", Q_NULLPTR));
        connstate_label->setText(QApplication::translate("Garfield_control", "TextLabel", Q_NULLPTR));
        Gyro_X_label->setText(QApplication::translate("Garfield_control", "Gyroscope X", Q_NULLPTR));
        Gyro_Y_label->setText(QApplication::translate("Garfield_control", "Gyroscope Y", Q_NULLPTR));
        Gyro_Z_label->setText(QApplication::translate("Garfield_control", "Gyroscope Z", Q_NULLPTR));
        temperature_label->setText(QApplication::translate("Garfield_control", "Temperature", Q_NULLPTR));
        menuConfig->setTitle(QApplication::translate("Garfield_control", "config", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Garfield_control: public Ui_Garfield_control {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GARFIELD_CONTROL_H
