#include <QApplication>
#include <QtQuick/QQuickView>
#include <QBoxLayout>
#include <QWidget>
#include <QListWidget>
#include <QListWidgetItem>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QWidget *mainwindow = new QWidget();
//    mainwindow->setStyleSheet("background-color:red;"); // so that we can see the problem more clearly

    QHBoxLayout *layout = new QHBoxLayout(mainwindow);

    QListWidget *list = new QListWidget(mainwindow);

//    QQuickView *view = new QQuickView();
//    QWidget* container = QWidget::createWindowContainer(view, mainwindow);
//     container->setMinimumSize(500, 500);
//    view->setSource(QUrl("qrc:///trackedObject"));

//    QListWidgetItem *item = new QListWidgetItem(container);
    list->addItem("test");

    layout->addWidget(list);
    mainwindow->setLayout(layout);

    mainwindow->show();

    return a.exec();
}