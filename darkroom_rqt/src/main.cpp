#include <QApplication>
#include <QtQuick/QQuickView>
#include <QBoxLayout>
#include <QWidget>
#include <QTableView>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QLineEdit>

#include <iostream>
#include <fstream>
#include <cstdlib>


using namespace std;

int main(int argc, char *argv[])
{
//    QApplication a(argc, argv);
//    QWidget *mainwindow = new QWidget();
////    mainwindow->setStyleSheet("background-color:red;"); // so that we can see the problem more clearly
//
//    QVBoxLayout *layout = new QVBoxLayout(mainwindow);
////
////
////    QQuickView *view1 = new QQuickView();
////    view1->setSource(QUrl("qrc:///trackedObject"));
////    QWidget* container1 = QWidget::createWindowContainer(view1, mainwindow);
////    container1->setMinimumSize(500, 500);
////
////    QScrollArea *scrollArea = new QScrollArea;
////    scrollArea->setBackgroundRole(QPalette::Dark);
////    scrollArea->addScrollBarWidget(container0, nullptr);
//
//    QScrollArea* scrollArea = new QScrollArea(mainwindow);
//    scrollArea->setBackgroundRole(QPalette::Window);
//    scrollArea->setFrameShadow(QFrame::Plain);
//    scrollArea->setFrameShape(QFrame::NoFrame);
//    scrollArea->setWidgetResizable(true);
//
//    //vertical box that contains all the checkboxes for the filters
//    QWidget* techArea = new QWidget(mainwindow);
//    techArea->setObjectName("techarea");
//    techArea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
//    techArea->setLayout(new QVBoxLayout(techArea));
//    scrollArea->setWidget(techArea);
//
//    for(int i=0;i<5;i++) {
//        QWidget *widget = new QWidget(techArea);
//        char str[100];
//        sprintf(str, "motor%d", i);
//        widget->setObjectName(str);
//        widget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
//        widget->setLayout(new QHBoxLayout(widget));
//
//        QRadioButton *pos = new QRadioButton(widget);
//        pos->setText("pos");
//        pos->setFixedSize(60,30);
//        pos->setCheckable(true);
//        pos->setObjectName("pos");
//
//        widget->layout()->addWidget(pos);
//
//        QRadioButton *vel = new QRadioButton(widget);
//        vel->setText("vel");
//        vel->setFixedSize(60,30);
//        vel->setCheckable(true);
//        vel->setObjectName("vel");
//        widget->layout()->addWidget(vel);
//
//        QRadioButton *dis = new QRadioButton(widget);
//        dis->setText("dis");
//        dis->setFixedSize(60,30);
//        dis->setCheckable(true);
//        dis->setObjectName("dis");
//        dis->setChecked(true);
//        widget->layout()->addWidget(dis);
//
//        QRadioButton *force = new QRadioButton(widget);
//        force->setText("force");
//        force->setFixedSize(60,30);
//        force->setCheckable(true);
//        force->setObjectName("force");
//        widget->layout()->addWidget(force);
//
//        QLineEdit *line = new QLineEdit(widget);
//        line->setFixedSize(100,30);
//        widget->layout()->addWidget(line);
//
//        QSlider *slider = new QSlider(Qt::Orientation::Horizontal,widget);
//        slider->setFixedSize(100,30);
//        slider->setValue(50);
//        widget->layout()->addWidget(slider);
//
//        techArea->layout()->addWidget(widget);
//    }
//
//    layout->addWidget(scrollArea);
//    mainwindow->setLayout(layout);
//
//    mainwindow->show();
//
//    return a.exec();

    ofstream file;
    file.open("calibration.yaml");
    file << "name: calibration" << endl;
    file << "ObjectID: 0" << endl;
    file << "mesh: nothing" << endl;
    file << "sensor_relative_locations:" << endl;
    vector<vector<double>> pos;
    vector<vector<double>> angles;
    int number_points = atoi(argv[1]);
    for(int i=0;i<number_points;i++){
        vector<double> p;
        if(strcmp(argv[2],"x")==0)
                p = {(rand()/(double)RAND_MAX)-0.5,0,0};
        else if(strcmp(argv[2],"xz")==0)
                p = {(rand()/(double)RAND_MAX)-0.5,0,(rand()/(double)RAND_MAX)-0.5};
        else if(strcmp(argv[2],"xyz")==0)
                p = {(rand()/(double)RAND_MAX)-0.5,(rand()/(double)RAND_MAX),(rand()/(double)RAND_MAX)-0.5};
        pos.push_back(p);
        double distance = sqrt(pow(p[0], 2.0) + pow(1+p[1], 2.0) + pow(p[2], 2.0));

        cout << p[0] << " " << p[1] << " " << p[2] << endl;
        double elevation = M_PI - acos(p[2] / distance);
        double azimuth = atan2(1+p[1], p[0]);
        vector<double> a = {M_PI - elevation,azimuth};
        angles.push_back(a);
    }
    for(int i=0;i<number_points;i++){
        file << " - [ " << i << " , " << pos[i][0] << ", " << pos[i][1] << " ," << pos[i][2] << "]"  << endl;
    }
    file << "calibration_angles:" << endl;
    for(int i=0;i<number_points;i++){
        file << " - [ " << i << " , " << angles[i][0] << ", " << angles[i][1] << "]"  << endl;
    }
    file.close();
}