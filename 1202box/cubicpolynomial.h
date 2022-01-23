#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <stdlib.h>
namespace rbdlmath = RigidBodyDynamics::Math;
using namespace  std;


class polynomial
{
public:
    polynomial();
    void calculatecubic(int len, double cp,double ct,double cv,
                        double tp,double tt,double tv,
                        vector<double> & p,vector<double> &v , vector<double> & a);
    void calculatequintic(int len, double cp,double ct,double cv,double ca,
                          double tp,double tt,double tv, double ta,
                          vector<double> & p,vector<double> &v , vector<double> & a);



};

#endif // POLYNOMIAL_H
