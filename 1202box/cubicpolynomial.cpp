#include "cubicpolynomial.h"
#include <math.h>

polynomial::polynomial()
{

}

void polynomial::calculatecubic(int len, double cp,double ct,double cv,double tp,double tt,double tv,vector<double> & traj_p,vector<double> & traj_v,vector<double> & traj_a)
{
    double dis = tp - cp;
    double T = tt-ct;
    double dt = 0.1;
    double a0 = cp;
    double a1 = cv;
    double a2 = (3*dis -(2*cv+tv)*T)/(T*T);
    double a3 = (-2*dis +(cv+tv)*T)/(T*T*T);
    double p[len];
    double v[len];
    double a[len];
    double t=0;
    for(int i=0;i<len;i++)
    {
        t=i*dt;
        p[i]=a0 + a1*t + a2*(t*t) + a3*t*t*t;
        v[i]=a1 + 2*a2*t + 3*a3*t*t;
        a[i]=2*a2 + 6*a3*t;
        traj_p.push_back(p[i]);
        traj_v.push_back(v[i]);
        traj_a.push_back(a[i]);
        cout << "traj_p 's size is : " << traj_p.size() << endl;
    }
}

void polynomial::calculatequintic(int len, double cp, double ct, double cv, double ca,
                                            double tp, double tt, double tv, double ta,
                                            vector<double> &op, vector<double> &ov, vector<double> &oa)
{

        double T = tt-ct;
        double dt = T/len;
        double a0 = cp;
        double a1 = cv;
        double a2 = ca/2;
        double a3 = (20*tp - 20*cp - (8*tv + 12*cv)*T -(3*ca - ta)*T*T)/(2*T*T*T);
        double a4 = (30*cp - 30*tp + (14*tv + 16*cv)*T + (3*ca - 2*ta)*T*T)/(2*T*T*T*T);
        double a5 = (12*tp - 12*cp - (6*tv + 6*cv)*T - (ca - ta)*T*T)/(2*T*T*T*T*T);

        double p[len];
        double v[len];
        double a[len];
        double t=0;
        for(int i=0;i<len;i++)
        {
            t=i*dt;
            p[i]=a0 + a1*t + a2*(t*t) + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
            v[i]=a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
            a[i]=2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;
            op.push_back(p[i]);
            ov.push_back(v[i]);
            oa.push_back(a[i]);
            cout << "traj_p 's size is : " << op.size() << endl;
        }
}
