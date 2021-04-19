#include "load_log.h"



LoadLog::LoadLog(const char* log_str)
{
	ReadFromData(log_str,this->log);
}

LoadLog::~LoadLog()
{
	
}

int LoadLog::ReadFromData(const char* logfile, vector<log_data>& logfile_data)
{
	ifstream file (logfile);
	string logline;
	log_data logdata_indv;

	if(file.is_open())
	{
		while(getline(file,logline))
		{
			istringstream str(logline);
			char c;
			str >> c;
			int j = 0;			
			switch(c)
			{
				case 'L':
					logdata_indv.data_type = 1;		
					str >> logdata_indv.x_robot >> logdata_indv.y_robot >> logdata_indv.theta_robot;	
					str >> logdata_indv.x_lidar >> logdata_indv.y_lidar >> logdata_indv.theta_lidar;	
					logdata_indv.readings = (float*) malloc(sizeof(float) * total_readings);
					while (j < total_readings)
					{
						str >> logdata_indv.readings[j];
						// logdata_indv.readings[j] /= 10.0;
						
						j++;
					
					}
					str >> logdata_indv.time_stamp;	   
					break;
				case 'O':	
					logdata_indv.data_type = 0;
					str >> logdata_indv.x_robot >> logdata_indv.y_robot >> logdata_indv.theta_robot;
					str >> logdata_indv.time_stamp;
					break;
				default:
					break;
			}
			logfile_data.push_back(logdata_indv);	
		}
	}
	else
	{
		fprintf(stderr, "ERROR: Could not open file %s\n", logfile);
		return -1;
	}
	file.close();	
}


vector<log_data> LoadLog::GetLog()
{
	return this->log;
}


void LoadLog::ShowLogData()
{
	log_data indv;
	for (int i = 0;i < this->log.size();i++)
	{
		indv = this->log[i];
		cout << indv.data_type << endl;
	}
}