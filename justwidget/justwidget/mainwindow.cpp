#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace QtCharts;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QChart *chart = new QChart;
    chart->setTitle("position/velocity/acceleration");
    QLineSeries *lineseries = new QLineSeries();
    QLineSeries *vline = new QLineSeries();
    QLineSeries *aline = new QLineSeries();

    int cp = 1, tp = 5, ct = 0, tt = 2, cv = 0, tv = 0;


    int dis = tp - cp;
    float T = tt-ct;
    float dt = 0.1;
    int a0 = cp;
    int a1 = cv;
    int a2 = (3*dis -(2*cv+tv)*T)/(T*T);
    int a3 = (-2*dis +(cv+tv)*T)/(T*T*T);
    int len = int(T/dt);
    float p[len];
    float v[len];
    float a[len];
    float t=0;



    for(int i=0;i<len;i++)
    {
        ui->graphicsView->close();
        t=i*dt;
        p[i]=a0 + a1*t + a2*(t*t) + a3*t*t*t;
        v[i]=a1 + 2*a2*t + 3*a3*t*t;
        a[i]=2*a2 + 6*a3*t;
        lineseries->append(i,p[i]);
        vline->append(i,v[i]);
        aline->append(i,a[i]);
    }

    chart->addSeries(lineseries);
    chart->addSeries(vline);
    chart->addSeries(aline);

    chart->createDefaultAxes();
    ui->graphicsView = new QChartView(chart);
    ui->graphicsView->setGeometry(200,200,600,400);
    ui->graphicsView->show();
}

MainWindow::~MainWindow()
{
    delete ui;
}

