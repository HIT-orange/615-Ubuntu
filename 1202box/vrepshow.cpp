#include "vrepshow.h"

namespace vrepspace
{

    using namespace std;
    void VrepShow::vrep_connect(const char* ip)
    {


        int Port = 19997;

        client_id = simxStart((simxChar*)ip, Port, 1, 1, 1000, 5);
        vrep_stop();
        extApi_sleepMs(50);

        if (client_id != -1)
        {
            cout << "V-rep connected."<<endl;
            char jname[64];
            char lname[64];
            for (int i = 0; i < 6; i++)
            {
                sprintf(jname, "joint_%d", i + 1);
                sprintf(lname, "link_%d_visual",i+1);
                int ret = simxGetObjectHandle(client_id, jname, &joint_handle[i], simx_opmode_blocking);
                int res = simxGetObjectHandle(client_id, lname, &link_handle[i], simx_opmode_blocking);
                if (ret != simx_return_ok || res!=simx_return_ok)
                {
                    cout << "Remote API error with simxGetObjectHandle:" << ret << endl;
                }
                else
                {
                    cout << "joint_handle[" << i << "] is : " << joint_handle[i] << endl;
                    cout << "link_handle[" << i << "] is : " << link_handle[i] << endl;
                }

            }
            simxGetObjectHandle(client_id,"obstacle",&obstacle_handle[0],simx_opmode_blocking);
            simxGetObjectHandle(client_id,"Goal",&goal_handle,simx_opmode_blocking);
            simxGetObjectHandle(client_id,"1202box",&obstacle_handle[1],simx_opmode_blocking);
            cout << " get new obstacle handle : " << obstacle_handle[1] <<endl;
        }
        else
        {
            cout << "V-rep can't be connected.";
        }
    }
    void VrepShow::vrep_start()
    {
        int ret = simxStartSimulation(client_id, simx_opmode_oneshot);
        extApi_sleepMs(50);
        if (ret !=0 && ret !=1)
        {
            cout << "Remote API error with simxStartSimulation:" << ret << endl << endl;

        }
        else
        {
            cout << " Simulation Start !" << endl << endl;
        }
    }
    void VrepShow::vrep_stop()
    {
        simxStopSimulation(client_id, simx_opmode_oneshot);
    }
    //void vrepshow::vrep_setpose2d(int handle, pose2d pose)
    //{
    //    float position[3] = { pose.posx, pose.posy, 0 };
    //    float orientation[3] = { 0,0,pose.oriz };
    //    simxSetObjectPosition(client_id, (simxInt)handle, -1, position,simx_opmode_oneshot);
    //    simxSetObjectOrientation(client_id, (simxInt)handle, -1, orientation,simx_opmode_oneshot);
    //}

    bool VrepShow::vrep_setjoints(double* joints)
    {
        int ret = 0;
        for (int a = 0; a < 6; a++)
        {
            ret = simxSetJointPosition(client_id, joint_handle[a], (simxFloat)(joints[a]) , simx_opmode_oneshot);

            if(ret != simx_return_ok) return false;
        }
        extApi_sleepMs(10);
        return true;
    }

    bool VrepShow::vrep_setjoints(float* joints)
    {
        int ret = 0;
        for (int a = 0; a < 6; a++)
        {
            ret = simxSetJointPosition(client_id, joint_handle[a], (simxFloat)(joints[a]) , simx_opmode_oneshot);

            if(ret != simx_return_ok) return false;
        }
        extApi_sleepMs(20);
        return true;
    }

    void VrepShow::vrep_setpose3d(int handle, int refhandle, pos3d pose, float * vreprpy)
    {
        float position[3] = { (float)pose.posx, (float)pose.posy, (float)pose.posz };

        simxSetObjectPosition(client_id, (simxInt)handle, -1, position, simx_opmode_oneshot);
        simxSetObjectOrientation(client_id, (simxInt)handle, -1, vreprpy, simx_opmode_oneshot);
        extApi_sleepMs(10);
    }
    void VrepShow::vrep_gethandle(char *objname, int * handle)
    {

        int ret = simxGetObjectHandle(client_id,objname,handle,simx_opmode_blocking);
        if (ret != simx_return_ok )
        {
            cout << "Remote API error with simxGetObjectHandle:" << ret << endl;
        }
        else
        {
            cout << "obstacle_handle is : " << *handle << endl;

        }
    }
}
