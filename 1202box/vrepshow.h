#ifndef VREPSHOW_H
#define VREPSHOW_H

#include <math.h>
#include <iostream>
#include <vector>
#include"extApi.h"
#include"simConst.h"
#include"extApiPlatform.h"
#include <time.h>
#include "stdlib.h"



namespace vrepspace
{
    struct pos3d{
        double posx=0;
        double posy=0;
        double posz=0;

    };
    class VrepShow
    {
    public:
        void vrep_connect(const char* ip);
        void vrep_start();
        void vrep_stop();
        void vrep_setpose3d(int handle, int refhandle, pos3d pose, float *vreprpy);
        void vrep_gethandle(char *objname, int * handle);

        bool vrep_setjoints(double* joints);
        bool vrep_setjoints(float* joints);
        int client_id;
        int dof;
        int joint_handle[6];
        int link_handle[6];
        int obstacle_handle[10];
        int goal_handle;
        std::vector<float> jointbuffer;
    };

}


#endif // VREPSHOW_H
