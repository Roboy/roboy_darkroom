#include <QApplication>
#include <QtQuick/QQuickView>
#include <QBoxLayout>
#include <QWidget>
#include <QTableView>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QLineEdit>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QWidget *mainwindow = new QWidget();
//    mainwindow->setStyleSheet("background-color:red;"); // so that we can see the problem more clearly

    QVBoxLayout *layout = new QVBoxLayout(mainwindow);
//
//
//    QQuickView *view1 = new QQuickView();
//    view1->setSource(QUrl("qrc:///trackedObject"));
//    QWidget* container1 = QWidget::createWindowContainer(view1, mainwindow);
//    container1->setMinimumSize(500, 500);
//
//    QScrollArea *scrollArea = new QScrollArea;
//    scrollArea->setBackgroundRole(QPalette::Dark);
//    scrollArea->addScrollBarWidget(container0, nullptr);

    QScrollArea* scrollArea = new QScrollArea(mainwindow);
    scrollArea->setBackgroundRole(QPalette::Window);
    scrollArea->setFrameShadow(QFrame::Plain);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidgetResizable(true);

    //vertical box that contains all the checkboxes for the filters
    QWidget* techArea = new QWidget(mainwindow);
    techArea->setObjectName("techarea");
    techArea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    techArea->setLayout(new QVBoxLayout(techArea));
    scrollArea->setWidget(techArea);

    for(int i=0;i<5;i++) {
        QWidget *widget = new QWidget(techArea);
        char str[100];
        sprintf(str, "motor%d", i);
        widget->setObjectName(str);
        widget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        widget->setLayout(new QHBoxLayout(widget));

        QRadioButton *pos = new QRadioButton(widget);
        pos->setText("pos");
        pos->setFixedSize(60,30);
        pos->setCheckable(true);
        pos->setObjectName("pos");

        widget->layout()->addWidget(pos);

        QRadioButton *vel = new QRadioButton(widget);
        vel->setText("vel");
        vel->setFixedSize(60,30);
        vel->setCheckable(true);
        vel->setObjectName("vel");
        widget->layout()->addWidget(vel);

        QRadioButton *dis = new QRadioButton(widget);
        dis->setText("dis");
        dis->setFixedSize(60,30);
        dis->setCheckable(true);
        dis->setObjectName("dis");
        dis->setChecked(true);
        widget->layout()->addWidget(dis);

        QRadioButton *force = new QRadioButton(widget);
        force->setText("force");
        force->setFixedSize(60,30);
        force->setCheckable(true);
        force->setObjectName("force");
        widget->layout()->addWidget(force);

        QLineEdit *line = new QLineEdit(widget);
        line->setFixedSize(100,30);
        widget->layout()->addWidget(line);

        QSlider *slider = new QSlider(Qt::Orientation::Horizontal,widget);
        slider->setFixedSize(100,30);
        slider->setValue(50);
        widget->layout()->addWidget(slider);

        techArea->layout()->addWidget(widget);
    }

    layout->addWidget(scrollArea);
    mainwindow->setLayout(layout);

    mainwindow->show();

    return a.exec();
}