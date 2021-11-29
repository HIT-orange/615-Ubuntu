#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "QtCharts/QChart"
#include "QtCharts/qlineseries.h"
#include "QtCharts/QChartView"
#include <QDialog>
#include <qlabel.h>
#include <QGraphicsItem>
#include "QGraphicsScene"
#include "QGraphicsView"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
