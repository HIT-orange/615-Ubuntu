#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "stdlib.h"
#include "stdio.h"

using namespace QtCharts;
using namespace  std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QChart *chartp = new QChart;
        QChart *chartv = new QChart;
            QChart *charta = new QChart;
    chartp->setTitle("position line");
        chartv->setTitle("velocity line");
            charta->setTitle("acceleration line");
    QLineSeries *lineseries1 = new QLineSeries();
    QLineSeries *lineseries2 = new QLineSeries();
    QLineSeries *lineseries3 = new QLineSeries();
    QLineSeries *lineseries4 = new QLineSeries();
    QLineSeries *lineseries5 = new QLineSeries();
    QLineSeries *lineseries6 = new QLineSeries();

    QLineSeries *v1 = new QLineSeries();
    QLineSeries *v2 = new QLineSeries();
    QLineSeries *v3 = new QLineSeries();
    QLineSeries *v4 = new QLineSeries();
    QLineSeries *v5 = new QLineSeries();
    QLineSeries *v6 = new QLineSeries();
    QLineSeries *a1 = new QLineSeries();
    QLineSeries *a2 = new QLineSeries();
    QLineSeries *a3 = new QLineSeries();
    QLineSeries *a4 = new QLineSeries();
    QLineSeries *a5 = new QLineSeries();
    QLineSeries *a6 = new QLineSeries();

    vector<vector<double>> jointall ={{0,0,0,0,0,0},
                                      {1,2,3,4,5,6},
                                      {3,5,9,2,1,4},
                                      {5,4,6,4,4,3},
                                      {3,2,3,2,1,2},
                                      {0,0,0,0,0,0}};

    int num = jointall.size();
    vector<vector<double>> traj_p;
    vector<vector<double>>  traj_v;
    vector<vector<double>>  traj_a;
    vector<vector<double>>  t_p(6);
    vector<vector<double>>  t_v(6);
    vector<vector<double>>  t_a(6);
    int totallen =0;

    std::cout << "quintic polynomial parts : " << num-1 << std::endl;

    vector<vector<double>> cp(6);
    vector<vector<double>> cv(6);
    vector<vector<double>> ca(6);
    vector<vector<double>> tp(6);
    vector<vector<double>> tv(6);
    vector<vector<double>> ta(6);
    vector<vector<double>> tt(6);
    vector<vector<double>> ct(6);
    double dt = 0.02;
    vector<int> T(6);
    for(int p = 0; p<num-1; p++)
    {
        std::cout << "calculate part : " << p << std::endl;

        for(int jp=0;jp<6;jp++)
        {
            std::cout << "parameters for joint : " << jp << std::endl;
            cp[jp].push_back(jointall[p][jp]);
            tp[jp].push_back(jointall[p+1][jp]);
            std::cout << cp[jp][p] << std::endl;
            std::cout << tp[jp][p] << std::endl;
            ca[jp].push_back(0);
            ta[jp].push_back(0);
            std::cout << ca[jp][p] << std::endl;
            std::cout << ta[jp][p] << std::endl;
            ct[jp].push_back(0);
            tt[jp].push_back(1);
            std::cout << ct[jp][p] << std::endl;
            std::cout << tt[jp][p] << std::endl;
        }
        T[p] = (int)((tt[0][p]-ct[0][p])/dt);
        std::cout << "part : " << p << " time is : " << T[p] << std::endl;
    }
    std::cout << "tp length is : " << tp[0].size() << std::endl;
    std::cout << "ta length is : " << ta[0].size() << std::endl;
    std::cout << "tt length is : " << tt[0].size() << std::endl;

    for(int q = 0;q<num-2;q++)
    {
        for(int jp =0;jp<6;jp++)
        {
            double dot1 = (tp[jp][q]-cp[jp][q])/(tt[jp][q] - ct[jp][q]);
            double dot2 = (tp[jp][q+1]-cp[jp][q+1])/(tt[jp][q+1] - ct[jp][q+1]);

            if((dot1<0 && dot2 < 0) || (dot1>0 && dot2> 0))
            {
                if(q==0)cv[jp].push_back(0);
                if(q>0) cv[jp].push_back(tv[jp][q-1]);
                tv[jp].push_back((dot1+dot2)/2);
                if(q==num-3)
                {
                    cv[jp].push_back(tv[jp][num-3]);
                    tv[jp].push_back(0);
                }
            }
            else if(dot1*dot2<=0)
            {
                if(q==0)cv[jp].push_back(0);
                if(q>0) cv[jp].push_back(tv[jp][q-1]);
                tv[jp].push_back(0);
                if(q==num-3)
                {
                    cv[jp].push_back(tv[jp][num-3]);
                    tv[jp].push_back(0);
                }
            }

        }
    }
    std::cout << "cv length is : " << cv[0].size() << std::endl;
    std::cout << "tv length is : " << tv[0].size() << std::endl;
    for(int p = 0; p<num-1; p++)
    {


        for(int jp=0;jp<6;jp++)
        {
            calculatequintic(T[p],cp[jp][p],ct[jp][p],cv[jp][p],ca[jp][p],tp[jp][p],tt[jp][p],tv[jp][p],ta[jp][p],t_p[jp],t_v[jp],t_a[jp]);
            std::cout << "path of joint " << jp << " . sum length is :" << t_p[jp].size() << std::endl;
        }
        std::cout << "part : " << p+1 << " , lenth is : " << T[p] << std::endl;

        totallen = totallen + T[p];
    }




    for(int i=0;i<totallen;i++)
    {

        lineseries1->append(i,t_p[0][i]);
        lineseries2->append(i,t_p[1][i]);
        lineseries3->append(i,t_p[2][i]);
        lineseries4->append(i,t_p[3][i]);
        lineseries5->append(i,t_p[4][i]);
        lineseries6->append(i,t_p[5][i]);
        v1->append(i,t_v[0][i]);
        v2->append(i,t_v[1][i]);
        v3->append(i,t_v[2][i]);
        v4->append(i,t_v[3][i]);
        v5->append(i,t_v[4][i]);
        v6->append(i,t_v[5][i]);
        a1->append(i,t_a[0][i]);
        a2->append(i,t_a[1][i]);
        a3->append(i,t_a[2][i]);
        a4->append(i,t_a[3][i]);
        a5->append(i,t_a[4][i]);
        a6->append(i,t_a[5][i]);
    }




    chartp->addSeries(lineseries1);
    chartp->addSeries(lineseries2);
    chartp->addSeries(lineseries3);
    chartp->addSeries(lineseries4);
    chartp->addSeries(lineseries5);
    chartp->addSeries(lineseries6);

    chartv->addSeries(v1);
    chartv->addSeries(v2);
    chartv->addSeries(v3);
    chartv->addSeries(v4);
    chartv->addSeries(v5);
    chartv->addSeries(v6);

    charta->addSeries(a1);
    charta->addSeries(a2);
    charta->addSeries(a3);
    charta->addSeries(a4);
    charta->addSeries(a5);
    charta->addSeries(a6);

    chartp->createDefaultAxes();
    ui->graphicsView = new QChartView(chartp);
    ui->graphicsView->setGeometry(200,200,600,400);
    ui->graphicsView->show();
    chartv->createDefaultAxes();
    ui->graphicsView_2 = new QChartView(chartv);
    ui->graphicsView_2->setGeometry(200,200,600,400);
    ui->graphicsView_2->show();
    charta->createDefaultAxes();
    ui->graphicsView_3 = new QChartView(charta);
    ui->graphicsView_3->setGeometry(200,200,600,400);
    ui->graphicsView_3->show();
}
void MainWindow::calculatequintic(int len, double cp, double ct, double cv, double ca,
                                            double tp, double tt, double tv, double ta,
                                            vector<double> &op, vector<double> &ov, vector<double> &oa)
{

        double T = tt-ct;
        double dt = T/len;
        double a0 = cp;
        double a1 = cv;
        double a2 = ca/2;
//        double a3 = (20*tp - 10*cp - (8*tv + 12*cv)*T -(3*ca - ta)*T*T)/(2*T*T*T);
//        double a4 = (30*cp - 30*tp + (14*tv + 16*cv)*T + (3*ca - 2*ta)*T*T)/(2*T*T*T*T);
//        double a5 = (12*tp - 12*cp - (6*tv + 6*cv)*T - (ca - ta)*T*T)/(2*T*T*T*T*T);
        double a3 = (20*tp - 20*cp - (8*tv + 12*cv)*T -(3*ca - ta)*T*T)/(2*T*T*T);
        double a4 = (30*cp - 30*tp + (14*tv + 16*cv)*T + (3*ca - 2*ta)*T*T)/(2*T*T*T*T);
        double a5 = (12*tp - 12*cp - (6*tv + 6*cv)*T - (ca - ta)*T*T)/(2*T*T*T*T*T);
        cout << "path equation :" << "p = " << a0 << "+ " << a1 << "x + " << a2 << "x^2 + " << a3<<"x^3 + "<<a4<<"x^4 + "<<a5<<"x^5"<<endl;
        double p[len];
        double v[len];
        double a[len];
        double t=0;
        for(int i=0;i<len;i++)
        {
            t=i*dt;
            cout << "time is (0-5s) : " << t << endl;

            p[i]=a0 + a1*t + a2*(t*t) + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
            cout << "joint position : " << p[i] << endl;
            v[i]=a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
            a[i]=2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;
            op.push_back(p[i]);
            ov.push_back(v[i]);
            oa.push_back(a[i]);
            std::cout << "traj_p 's size is : " << op.size() << std::endl;
        }
}
MainWindow::~MainWindow()
{
    delete ui;
}

