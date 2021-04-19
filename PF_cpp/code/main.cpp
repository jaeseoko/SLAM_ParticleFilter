#include <iostream>
#include "load_log.h"
#include "load_map.h"
#include "particleFilter.h"
#include <cstdlib>
#include <fssimplewindow.h>
#include <vector>
#include <random>
#include <cmath>
#include <visualize.h>
#include <chrono>

#define PI 3.141592



class ApplicationMain
{
private:
    const char* map_str ="../data/map/wean.dat";
    bool odomFirst = true;
    bool LidarFirst = true;
    bool term = false;
    
    
    
    std::vector<particleFilter::Particle> particles;

public:
    int numParticles;
    DrawMap drawMap;
    particleFilter PF;
    particleFilter::MotionModel motionModel;
    particleFilter::SensorModel sensorModel;
    particleFilter::Resampling resample;
    // particleFilter::SensorModel sensorModel;
    map_type* myMap;
    std::vector<log_data> logData;
    double u_t0[3]={0};
    double u_t1[3]={0};
    ApplicationMain(int argc,char *argv[])
    {
        
        numParticles = atoi(argv[1]);
        
        LoadMap map(map_str);
        myMap = map.GetMap();
        sensorModel.myMap = myMap;
        sensorModel.mapRows = myMap->size_y;
        sensorModel.mapCols = myMap->size_x;
        sensorModel.numParticles = numParticles;

        

        const char* log_str =argv[2];
        LoadLog logs(log_str);
        // logs.ShowLogData();
        logData = logs.GetLog();
    }
    
    void init_particles_freespace(void)
    {
        if (numParticles==1)
        {
            particleFilter::Particle p;
            p.x = 4155; p.y = 4005; p.theta = PI; p.w = 1;
            particles.push_back(p);
        }
        else
        {
            float w = 1;
            int i = 0;
            std::default_random_engine generator;
            std::uniform_real_distribution<float> distY(0.0,7500.0);
            std::uniform_real_distribution<float> distX(3000.0 ,8000.0);
            std::uniform_real_distribution<float> distTheta(-PI,PI);
            while( i < numParticles)
            {
                
                
                auto y = distY(generator);
                auto x = distX(generator);
                auto theta = distTheta(generator);
                auto yMap = (int)std::floor(y/10);
                auto xMap = (int)std::floor(x/10);
                if(myMap->prob[yMap][xMap] == 0)
                // if(myMap->prob[xMap][yMap] == 0)
                {
                    particleFilter::Particle p;
                    p.x = x; p.y = y; p.theta = theta; p.w = w/numParticles;
                    particles.push_back(p);
                    ++i;
                }
            }
        }
std::cout <<__FUNCTION__ <<__LINE__<<std::endl;
std::cout << "checking probaadsfdsa " <<myMap->prob[745][251] <<std::endl;

        
    }

    bool MustTerminate(void) const
    {
        return term;
    }
	void RunOdom(log_data odomLog)
    {
        auto key=FsInkey();
        if(FSKEY_ESC==key)
        {
            term=true;
        }

        if(odomFirst==true)
        {
            odomFirst = false;
            u_t0[0] = odomLog.x_robot;
            u_t0[1] = odomLog.y_robot;
            u_t0[2] = odomLog.theta_robot;
            
        }
        
        u_t1[0] = odomLog.x_robot;
        u_t1[1] = odomLog.y_robot;
        u_t1[2] = odomLog.theta_robot;

        
        for(auto &p: particles)
        {
// std::cout << "particle x,y before update: " << p.x << ", "<< p.y <<std::endl;
            motionModel.update(u_t0,u_t1,p);
// std::cout << "particle x,y  after update: " << p.x << ", "<< p.y <<std::endl;
        }
        u_t0[0] = u_t1[0];u_t0[1] = u_t1[1];u_t0[2] = u_t1[2];
// std::cout << __FUNCTION__ << __LINE__ << std::endl;
    }
    void RunLiDar(log_data laserLog)
    {
        auto key=FsInkey();
        if(FSKEY_ESC==key)
        {
            term=true;
        }
        sensorModel.laserHits.clear();
        for(auto &p: particles)
        {
            sensorModel.computeParticleLikelihood(p,laserLog.readings);
        }

        
    }
    void Resample(void)
    {
        resample.low_variance_sampler(particles);
    }

	void Draw(void) const
    {
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        drawMap.Draw();
        int wid,hei;
        FsGetWindowSize(wid,hei);
        for(auto &p: particles)
        {   
            // glEnable(0x8642);
            // glEnable(GL_POINT_SMOOTH);

            glColor3d(1,0,0);
            glPointSize(5.0f);
            glBegin(GL_POINTS);
            
            glVertex2d(p.x/10,p.y/10 + 100);
            glEnd();
            // Draw Lasers

            if(numParticles==1)
            {
                for(int i =0; i<sensorModel.laserHits.size()-1;)
                {
                    glColor3d(0,0,1);
                    glBegin(GL_LINES);
                    glVertex2d(p.x/10,p.y/10 +100);
                    glVertex2d(sensorModel.laserHits[i],sensorModel.laserHits[i+1] + 100);
                    i+=2;
                }
                glEnd();
            }


// std::cout << "particles x and y " <<p.x/10 << "," <<p.y/10<<std::endl;
        }
        
        FsSwapBuffers();
        FsSleep(25);
    }

};


int main(int argc,char *argv[])
{
	FsOpenWindow(0,0,900,1000,1);
	ApplicationMain app(argc,argv);
    app.init_particles_freespace();
    auto logs = app.logData;
	
    for(auto log_indv: logs)
	{
        auto t0 = std::chrono::high_resolution_clock::now();
        if(true== app.MustTerminate()) break;
		FsPollDevice();
		if(log_indv.data_type==0)
        {
            app.RunOdom(log_indv);
        }
        else if(log_indv.data_type==1)
        {
            app.RunLiDar(log_indv);
            app.Resample();
        }
        // if(app.numParticles==1)
        // {
        //     app.DrawLaser();
        // }
        
		app.Draw();
        auto passed = std::chrono::high_resolution_clock::now() - t0;
        auto passedMS=std::chrono::duration_cast <std::chrono::milliseconds>(passed).count();
	    std::cout << passedMS << " milli seconds" << std::endl;
        std::cout << "processes " <<(float) 1/passedMS * 1000 << " log lines per second." <<std::endl;
	}
	return 0;
}
