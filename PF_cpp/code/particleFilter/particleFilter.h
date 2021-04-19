#include <iostream>
#include <cmath>
#include <random>
#include <random>
#include "load_map.h"
// #include <chrono>

#define PI 3.141592

class particleFilter
{
public:
    class Particle
    {
    public:
        float x,y,theta,w;    
        Particle()
        {
            x = 0;y = 0;theta = 0;w = 0;
        }
        ~Particle()
        {
            x = 0;y = 0;theta = 0;w = 0;
        }
    };
    class MotionModel 
    {
    /* References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics.
    MIT press, 2005. [Chapter 5] */
    public:
        double _alpha1, _alpha2, _alpha3, _alpha4;
        // const float PI = 3.141592;

        // Attributes of motion model class
        // MotionModel(const float alpha1, const float alpha2, const float alpha3, const float alpha4) 
        MotionModel(void) 
        {
            
            // _alpha1 = 0.0005;
            // _alpha2 = 0.0005;
            // _alpha3 = 0.001;
            // _alpha4 = 0.001;

            _alpha1 = 0;
            _alpha2 = 0;
            _alpha3 = 0;
            _alpha4 = 0;
            
            
        }
        // Class function definition
        // void update(float *u_t0, float *u_t1, float *x_t0)
        double WrapToPi(double angle)
        {
            double angWrap;
            angWrap = angle - 2*PI * std::floor((angle + PI) / (2*PI));
            return angWrap;     
        }
        
        
        void update(double u_t0[], double u_t1[], Particle &p) 
        {
            auto x0 = p.x; auto y0 = p.y; auto theta0 = p.theta;

            double x_diff, y_diff, th_diff, drot1, dtrans, drot2, drh1, dth, drh2;
            x_diff = u_t1[0] - u_t0[0];
            y_diff = u_t1[1] - u_t0[1];
            th_diff = u_t1[2] - u_t0[2];
            
            drot1 = atan2(y_diff, x_diff) - u_t0[2];
            drot1 = WrapToPi(drot1);
            dtrans = sqrt(pow(x_diff, 2.0) + pow(y_diff, 2.0));
            drot2 = u_t1[2] - u_t0[2] - drot1;
            drot2 = WrapToPi(drot2);

            std::default_random_engine generator;
            std::normal_distribution<double> sample1(0, sqrt((_alpha1 * drot1 * drot1) + (_alpha2 * dtrans * dtrans)));
            std::normal_distribution<double> sample2(0, sqrt((_alpha3 * dtrans * dtrans) + (_alpha4 * drot1 * drot1)
                                            + (_alpha4 * drot2 * drot2)));
            std::normal_distribution<double> sample3(0, sqrt((_alpha1 * drot2 * drot2) + (_alpha2 * dtrans * dtrans)));
            
            drh1 = drot1 - sample1(generator);
            dth = dtrans - sample2(generator);
            drh2 = drot2 - sample3(generator);

            x0 += dth * cos(WrapToPi(theta0 + drh1));
            // y0 += dth * sin(WrapToPi(theta0 + drh1));
            y0 += dth * sin(WrapToPi(theta0 + drh1));
            // theta0 += drh1 + drh2;
            theta0 -= drh1 + drh2;
            theta0 = WrapToPi(theta0);
            p.x = x0;p.y = y0;p.theta = theta0;
        }  
    };
    

    class SensorModel 
    {
    private:
        double sigmaHit = 85;
        double lambdaShort = 0.01;
        
        
        // probability distribution weights
        double zHit = 70;
        double zShort = 5;
        double zMax = 0.01;
        double zRand = 1000;

        // double sigmaHit = 100;
        // double lambdaShort = 5;

        // double zHit = 150;
        // double zShort = 17.5;
        // double zMax = 15;
        // double zRand = 100;

        // self._z_hit = 150
        // self._z_short = 17.5
        // self._z_max = 15
        // self._z_rand = 100
        // self._sigma_hit = 100
        // self._lambda_short = 5


        double maxRange = 8000;
        double minProbability = 0.35;
        double laserOffset = 2.5;

        int thetaIncr = 3;
        int rayCastStepIncr = 1;
        // double **occupancyMap;
        float WrapToPi(float angle)
        {
            float angWrap;
            angWrap = angle - 2*PI * std::floor((angle + PI) / (2*PI));
            return angWrap;     
        }

        // member functions
        
        /*
        * Check whether occupancyMap[y][x] is a valid index
        */
        bool withinBounds(int x, int y) 
        {
// std::cout << __FUNCTION__ << __LINE__ <<std::endl;
// std::cout << mapCols << "," <<mapRows <<std::endl;
            return (x < mapCols) && (y < mapRows) && (x >= 0) && (y >= 0);
        }

        /*
        * Translate particle location to sensor location
        */
        std::vector<int> getSensorLocation(Particle &particle) 
        {

            double dx = laserOffset * cos(particle.theta);
            double dy = laserOffset * sin(particle.theta);

            std::vector<int> res;
            res.push_back((int) (particle.x + dx)/10);
            res.push_back((int) (particle.y + dy)/10);
            return res;
        }
    
        /*
        * Perform a ray cast operation at the indicated location and angle
        * @return distance to nearest obstacle scaled according to map scale
        */
        double rayCast(int x, int y, double theta) 
        {
            int currX = x, currY = y, currStep = 0;
// std::cout << __FUNCTION__ << __LINE__ <<std::endl;
            
            while (withinBounds(currX, currY)) 
            {
// std::cout << __FUNCTION__ << __LINE__ <<std::endl;
                double distance = euclid(x, y, currX, currY) * mapScale;
// std::cout << "check map prob iter in ray cast :" << myMap->prob[currX][currY] <<std::endl;
                if ((myMap->prob[currY][currX]) >= minProbability || ((myMap->prob[currY][currX]) < 0) || (distance >= maxRange)) 
                {
                    if(numParticles==1)
                    {
                        laserHits.push_back(currX);
                        laserHits.push_back(currY);
                    }
                    
                    return std::min(distance, maxRange);
                }


                currStep += rayCastStepIncr;
                double dx = currStep * cos(WrapToPi(theta));
                double dy = currStep * sin(WrapToPi(theta));
                currX = (int) currX + dx;
                currY = (int) currY + dy;
            }

            return std::min(euclid(x, y, currX, currY) * mapScale, maxRange);
        }
        
 
        double euclid(int x1, int y1, int x2, int y2) 
        {
            return pow(pow(x2 - x1, 2) + pow(y2 - y1, 2), 0.5);
        }
        /*
        * Compute p(z\trueDist)
        */
        double getLikelihood(double z, double trueDist) 
        {
// std::cout <<__FUNCTION__<<__LINE__<<std::endl;
// std::cout <<" z = "<< z <<std::endl;
            if (z < 0) return 0;

            double pHit = normalPdf(z, trueDist, sigmaHit);
            double pShort = getPShort(z, trueDist);
            double pFailure = getPFailure(z, trueDist);
            double pRand = 1/maxRange;
// std::cout <<"pHit = " << pHit <<std::endl;
// std::cout <<"pShort = " << pShort <<std::endl;
// std::cout <<"pFail = " << pFailure <<std::endl;
// std::cout <<"pRand = " << pRand <<std::endl;
            return (zHit * pHit) + (zShort * pShort) + (zMax * pFailure) + (zRand * pRand);
        }

        /*
        * Normal distributionn pdf of x. 
        * @param m mean of normal distribution
        * @param s standard deviation of normal distribution
        */
        double normalPdf(double reading, double trueDist, double sig) 
        {
            // static const double inv_sqrt_2pi = 0.3989422804014327;
            // double a = (reading - trueDist)/sig;

            double pHit = exp(-1/2 * (reading - trueDist)*(reading - trueDist) / (sig * sig));
            pHit = pHit/(sqrt(2*PI*sig*sig));

            return pHit;

            // return inv_sqrt_2pi / s * std::exp(-(0.5) * a * a);
        }
        /*
        * Calculate measurement likelihood component pShort
        */
        double getPShort(double z, double trueDist) 
        {
            if (z >= trueDist) return 0;
            return (lambdaShort * std::exp(-lambdaShort * z));
        }
        
        /*
        * Calculate measurement likelihood component pFailure
        */
        double getPFailure(double z, double trueDist) 
        {
            if (z >= maxRange) return 1;
            return 0;
        }
    public:
        int numParticles = 0;
        std::vector <int> laserHits;
        map_type* myMap;
        double mapScale;
        int mapRows;
        int mapCols;
        SensorModel()
        {
            myMap = nullptr;
            mapRows = 0;
            mapCols = 0;            
            mapScale = 10;
        }


        /*
        * Compute particle likelihood and update particle object
        * @param z_arr array of beam range finder measurements
        */
        void computeParticleLikelihood(Particle &particle, float* readings) 
        {
            
            std::vector<double> zArr;
// std::cout << "size of incoming float reading " << sizeof(readings)/sizeof(readings[0]) <<std::endl;
            for(int i=0; i< 180; ++i) 
            {
// std::cout << "range reading check: " << readings[i] <<std::endl;
                zArr.push_back((double)readings[i]);
            }
// std::cout << "size of range reading : " << zArr.size() <<std::endl;

            double totalProb = 1;
            // double totalProbLog = 0;
            std::vector<int> sensorLoc = getSensorLocation(particle);

            int sx = sensorLoc.at(0), sy = sensorLoc.at(1);
            int numAngles = (int) 180/thetaIncr; // subsample size
            double startAngle = particle.theta - PI/2;

            for (int i = 0; i < numAngles; i++) 
            {
                double angle = startAngle + i * (thetaIncr * PI/180);
                double trueDist = rayCast(sx, sy, angle);
// std::cout << " dist from raycast: " << trueDist <<"\n";
                double prob = getLikelihood(zArr[(int)i * thetaIncr], trueDist);
// std::cout <<__FUNCTION__<<__LINE__<<std::endl;
// std::cout << "prob "<< prob <<std::endl;
                totalProb *= prob;

                // totalProbLog += log(prob);
            }
// std::cout << totalProb <<std::endl;
            particle.w = totalProb;
            // particle.w = std::exp(totalProbLog);
            
// std::cout <<"checking if assigned weights correct: " << particle.w <<std::endl;
        }
    };

    class Resampling
    {
    public:    

        void low_variance_sampler(std::vector<Particle> &particles)
        {
            std::vector<Particle> resampled;
            int numParticles = particles.size();
            float sum = 0;
            for(auto &p:particles)
            {
                sum+=p.w;
            }
            for(auto &p:particles)
            {
                p.w /= sum;
            }     
            static std::default_random_engine generator;
            std::uniform_real_distribution<double> distR(0.0, 1.0/numParticles);
            auto r = distR(generator);      
            float c = particles[0].w;
            int i = 0;
            for(int m = 1; m < numParticles+1; ++m)
            {
                auto u = (r + ( (float)m - 1)/numParticles);

                while (u > c)
                {
                    i+=1;
                    c += particles[i].w;
                }
                resampled.push_back(particles[i]);
            }
            for(int i =0; i<resampled.size(); ++i)
            {
                particles[i] = resampled[i];
            }
            resampled.clear();
        }
    };

    int numParticles;

};