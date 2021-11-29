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
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include <vector>
#include <iostream>
#include <string>

using namespace  std;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void calculatequintic(int len, double cp, double ct, double cv, double ca,
                                                double tp, double tt, double tv, double ta,
                                                vector<double> &op, vector<double> &ov, vector<double> &oa);
private:
    Ui::MainWindow *ui;
};



#endif // MAINWINDOW_H
