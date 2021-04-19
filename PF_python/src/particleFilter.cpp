#include <vector>
#include <numeric>
#include <math.h>
#include <stdlib.h>
#include <string>
#include "opencv2/<module>.hpp"

using namespace std;
vector<string>& split(const string &s, char delim, vector<string> &elems) 
{
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

vector<string> split(const string &s, char delim) 
{
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

class ParticleFilter
{
public:
    ParticleFilter()
    {
        string weanMap = "../data/map/wean.dat";
        weanMap = MyMap();
        image = Mat(800,800,CV_32FC3);
        frame = Mat(800,800,CV_32FC3);

        // File names and video writer
        logName   = "../data/log/robotdata4.log";
        videoName = "../data/videos/robotdata4.mpg";
        saveVideo = false;
        outputVideo.open(videoName, CV_FOURCC('M', 'P', 'E', 'G'), 30, image.size(), true);

        // Particles
        nParticles = 10000;

        // Distribution
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        xy_normal = std::normal_distribution<double>(0.0,2.0);
        theta_normal = std::normal_distribution<double>(0.0,1.0*M_PI/180);
    }

    int readMap()
    {
        int x, y, count;
        float temp;
        char line[256];
        FILE *fp;


        if((fp = fopen(weanMapName, "rt")) == NULL) {
            fprintf(stderr, "# Could not open file %s\n", weanMapName);
            return -1;
        }
        while((fgets(line, 256, fp) != NULL)
            && (strncmp("global_map[0]", line , 13) != 0)) {
            if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
            if(sscanf(&line[35], "%d", &(weanMap.offset_x)) != 0) {
            weanMap.offset_x = weanMap.offset_x;
            }
            if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) {
            if (sscanf(&line[35], "%d", &(weanMap.offset_y)) != 0) {
            weanMap.offset_y = weanMap.offset_y;
            }
            }
        }

        if(sscanf(line,"global_map[0]: %d %d", &weanMap.size_y, &weanMap.size_x) != 2) {
            fprintf(stderr, "ERROR: corrupted file %s\n", weanMapName);
            fclose(fp);
            return -1;
        }

        weanMap.min_x = weanMap.size_x;
        weanMap.max_x = 0;
        weanMap.min_y = weanMap.size_y;
        weanMap.max_y = 0;
        count = 0;
        for(x = 0; x < weanMap.size_x; x++)
        {
            for(y = 0; y < weanMap.size_y; y++, count++) 
            {
            
            fscanf(fp,"%e", &temp);
            if(temp < 0.0)
                weanMap.prob[x][y] = -1;
            else 
            {
                if(x < weanMap.min_x)
                weanMap.min_x = x;
                else if(x > weanMap.max_x)
                weanMap.max_x = x;
                if(y < weanMap.min_y)
                weanMap.min_y = y;
                else if(y > weanMap.max_y)
                weanMap.max_y = y;
                
                weanMap.prob[x][y] = 1 - temp;	   
                if (temp > 0.8) 
                {
                Particle location(x, y, 0.0);
                potentialParticles.push_back(location);
                }
            }
            }
        }
        fclose(fp);
        return 0;
        }
    }
    void readLog()
    {
        string line;
        ifstream myfile (logName);
        if (myfile.is_open()) {
            while ( getline (myfile,line) ) 
            {
            vector<string> elements = split(line, ' ');
            if (elements[0] == "O") 
            {
                OdometryData odom;
                odom.x 		   = stof(elements[1]);
                odom.y       = stof(elements[2]);
                odom.theta   = stof(elements[3]);
                odom.ts 		 = stof(elements[4]);
                logOdometryData.push_back(odom);
            }
            else if (elements[0] == "L"){
                LaserData laser;
                laser.x      = stof(elements[1]);
                laser.y      = stof(elements[2]);
                laser.theta  = stof(elements[3]);
                laser.xl     = stof(elements[4]);
                laser.yl     = stof(elements[5]);
                laser.thetal = stof(elements[6]);
                laser.ts     = stof(elements[187]);
                for (int i = 0; i < 180; i ++){
                    laser.r[i] = stoi(elements[i+7]);
                }
                logLaserData.push_back(laser);
                timestamps.push_back(laser.ts);
            }
            }
            myfile.close();
    }
private:

    char *weanMapName;
    MyMap weanMap;
    cv::Mat image;
    cv::Mat frame;

    // Particle data
    int numParticles;
    int numTestParticles;
    std::vector<Particle> potentialParticles;
    std::vector<Particle> particles;
    std::vector<float> weights;
    std::vector<float> weightedDistribution;
    std::vector<int> intervals;
    Particle estPosition = Particle(395.0,400.0,M_PI/2);


    std::vector<LaserData> logLaserData;
    std::vector<OdometryData> logOdometryData;

}