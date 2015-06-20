#include <string>
#include "Controller.h"
#include "Logger.h"
#include "ControllerEvent.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <map>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include "quatanion.h"

#define PI 3.141592
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

using namespace std;

typedef map<string, double> JMap;

class ActionLogger : public Controller
{
public:
	double onAction(ActionEvent &evt);
	void onRecvMsg(RecvMsgEvent &evt);
	void onInit(InitEvent &evt);
	void writeActionLog(std::string msg);
	void readActionLog();
	void playActionLog(double time);
	void strSplit(string msg, string separator);
	double gettimeofday_sec();
	void moveByKinect();
	void moveByOrs();
	std::string get_time();
	//string FloatToString(float x);


private:
	//úÊu                                                                    
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;
	//f[^iÖßjÅål                                                    
	int m_maxsize;

	BaseService* m_kinect;
	BaseService* m_ors;
	BaseService* m_view;
	
	ViewService* view;

	Vector3d m_pos;
	Rotation m_rotation;
	string body_str;
	int count;
	ofstream ofs;
	ofstream ofs_relpos;
	ofstream ofs_pose;
	stringstream ss;
	stringstream ss_relpos;
	stringstream ss_pose;

	bool play;
	bool write;
	bool playLog;
	bool writeInit;
	bool pickUp;
	
	double time_init;
	double time_start;
	double time_current;
	double time_target;
	int logIndex;
	int logSize;
	vector<string> actionLog;
	string logName;
	string logName2;

	int ors_count;
	int xtion_count;
	int dataCount;
	double time_ors;
	double time_xtion;
	double time_pre_target;
	double time_diff;

	// OñMµœyaw, pitch roll
	double pyaw, ppitch, proll;
	// ÌSÌÌpx
	double m_qw, m_qy, m_qx, m_qz;

	string headStr;
	string bodyStr;

	std::string name_man;
	std::string rp_name;
	std::string obj_name;
	std::string property;
	std::string Pick_up;

	ofstream waist;
	ofstream arm;
	ofstream log_ors;
	ofstream log_xtion;
	ofstream log_hand;
	ofstream log_head;
	ofstream log_elbow;
	double logTime;
	
	//Folder name
	std::string FolderName;
	std::string rp_FolderName;
	std::string TimeNum;
	std::string Elbow;
	std::string Hand;
	std::string Head;
	std::string Ors;
	std::string Xtion;
	std::string Waist;
	std::string Arm;
};

double ActionLogger::gettimeofday_sec(){
	struct timeval t;
	gettimeofday(&t, NULL);
	return (double)t.tv_sec + (double)t.tv_usec * 1e-6;
}

std::string ActionLogger::get_time(void){
	struct tm *date;
	time_t now;
	
	now = time(NULL);
	date = localtime(&now);
	int y, mon, d, h, min, s;
	std::string Day;
	std::string Time;
	std::stringstream year, month, day, hour, minute, second;

	y = date->tm_year + 1900;
	mon = date->tm_mon + 1;
	d = date->tm_mday;
	h = date->tm_hour;
	min = date->tm_min;
	s = date->tm_sec;

	year << std::setw(4) << std::setfill('0') << y;
	month << std::setw(2) << std::setfill('0') << mon;
	day << std::setw(2) << std::setfill('0') << d;
	hour << std::setw(2) << std::setfill('0') << h;
	minute << std::setw(2) << std::setfill('0') << min;
	second << std::setw(2) << std::setfill('0') << s;

	Day = year.str() + month.str() + day.str();
	Time = hour.str() + minute.str() + second.str();

	return Day + "_" + Time;
}


void ActionLogger::strSplit(string msg, string separator)
{
	int strPos1 = 0;
	int strPos2;
	std::string head;
	std::string body;

	strPos2 = msg.find_first_of(separator, strPos1);
	head.assign(msg, strPos1, strPos2 - strPos1);
	body.assign(msg, strPos2 + 1, msg.length() - strPos2);
	headStr = head;
	bodyStr = body;
}

void ActionLogger::onInit(InitEvent &evt)
{
	view = (ViewService*)connectToService("SIGViewer");
	
	play = false;
	write = false;
	writeInit = false;
	pickUp = false;
	
	time_start = 0;
	time_current = 0;
	time_target = 0;
	logIndex = 0;
	logSize = 0;
	dataCount = 0;

	name_man = "man_0";
	rp_name = "RelativePosition";
	Pick_up = "PickUp";

	body_str = "";

	SimObj *my = getObj(name_man.c_str());

	// úÊuæŸ                                                            
	Vector3d pos;
	my->getPosition(pos);
	m_posx = pos.x();
	m_posy = pos.y();
	m_posz = pos.z();

	// úpšiñ]jæŸ                                                                                           
	Rotation rot;
	my->getRotation(rot);
	double qw = rot.qw();
	double qy = rot.qy();

	m_yrot = acos(fabs(qw)) * 2;
	if (qw*qy > 0)
		m_yrot = -1 * m_yrot;

	m_range = 0.1;
	m_maxsize = 15;

	// ÌSÌÌü«
	m_qw = 1.0;
	m_qx = 0.0;
	m_qy = 0.0;
	m_qz = 0.0;

	pyaw = ppitch = proll = 0.0;

	time_init = gettimeofday_sec();
	
	TimeNum = get_time();
	FolderName = name_man + "/" + TimeNum;
	rp_FolderName = rp_name + "/" + TimeNum;
	
	ors_count = 0;
	xtion_count = 0;
	time_ors = 0;
	time_xtion = 0;
	time_pre_target = 0;
	time_diff = 0;
	
	//logName = "man_0_actionLog_20140929214957.txt";
	
//	Elbow = FolderName + "/" + "elbow" + ".csv";
//	Hand = FolderName + "/" + "hand" + ".csv";
//	Head = FolderName + "/" + "head" + ".csv";
//	Ors = FolderName + "/" + "ors" + ".csv";
//	Xtion = FolderName + "/" + "xtion" + ".csv";
//	Waist = FolderName + "/" + "waist" + ".csv";
//	Arm = FolderName + "/" + "arm" + ".csv";

//	ofstream clear(Waist.c_str(), ios::trunc);
//	waist.open(Waist.c_str(), ios::app);
//	ofstream clear2(Arm.c_str(), ios::trunc);
//	arm.open(Arm.c_str(), ios::app);
//	ofstream clear3(Ors.c_str(), ios::trunc);
//	log_ors.open(Ors.c_str(), ios::app);
//	ofstream clear4(Xtion.c_str(), ios::trunc);
//	log_xtion.open(Xtion.c_str(), ios::app);
//	ofstream clear5(Hand.c_str(), ios::trunc);
//	log_hand.open(Hand.c_str(), ios::app);
//	ofstream clear6(Head.c_str(), ios::trunc);
//	log_head.open(Head.c_str(), ios::app);
//	ofstream clear7(Elbow.c_str(), ios::trunc);
//	log_elbow.open(Elbow.c_str(), ios::app);
}

void ActionLogger::writeActionLog(std::string msg)
{
	SimObj *my = getObj(name_man.c_str());
	SimObj *obj = getObj(obj_name.c_str());	
	if (writeInit == true){
		writeInit = false;
		TimeNum = get_time();
		logName = name_man + "/" + TimeNum + ".txt";
		logName2 = rp_name + "/" + TimeNum + ".txt";
		
		ofstream clear(logName.c_str(), ios::trunc);
		ofs.open(logName.c_str(), ios::app);
		ofstream clear2(logName2.c_str(), ios::trunc);
		ofs_relpos.open(logName2.c_str(), ios::app);
	}

	//OÆµÄL^·éîñ
	//Ô
	time_current = gettimeofday_sec() - time_start;
	ss << "TIME:" << time_current << " ";
	ss << msg;
	body_str += ss.str();
	ofs << body_str << std::endl;
	
	stringstream msg_head_ors;
	ss_relpos << "TIME:" << time_current << " ";
	strSplit(msg, " ");
	//人の向いている方向
	if (headStr == "ORS_DATA"){
		msg_head_ors << bodyStr;
		ss_relpos << headStr << " " << msg_head_ors.str();
	}
	//人の頭の位置、物体の向き、物体の位置
	//getjoint頭（xyz）、物体（xyz、クオータニオン）
	Vector3d head_pos;
	my->getJointPosition(head_pos, "HEAD_JOINT1");
	ss_relpos << " HumanHeadPosition" << " " << head_pos.x() << "," << head_pos.y() << "," << head_pos.z();
	
	Rotation obj_rot;
	obj->getRotation(obj_rot);
	ss_relpos << " ObjectRotation" << " " << obj_rot.qw() << "," << obj_rot.qx() << "," << obj_rot.qy() << "," << obj_rot.qz();
	
	Vector3d obj_pos;
	obj->getPosition(obj_pos);
	ss_relpos << " ObjectPosition" << " " << obj_pos.x() << "," << obj_pos.y() << "," << obj_pos.z();
	
	
	ofs_relpos << ss_relpos.str() << std::endl;

	body_str = "";
	
	ss.str("");
	ss.clear(stringstream::goodbit);
	ss_relpos.str("");
	ss_relpos.clear(stringstream::goodbit);
	
}

void ActionLogger::readActionLog()
{
	ifstream ifs(logName.c_str());
	while (ifs){
		string str;
		getline(ifs, str);
		actionLog.push_back(str);
	}
	logSize = actionLog.size() - 1;
}

void ActionLogger::moveByKinect()
{
	//©ª©gÌæŸ                                                              
	SimObj *my = getObj(name_man.c_str());

	int i = 0;
	while (true)
	{
		i++;
		if (i == m_maxsize + 1) break;
		strSplit(bodyStr, ":");

		//ÌÌÊu                                                            
		if (headStr == "POSITION")
		{
			strSplit(bodyStr, ",");
			double x = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double y = atof(headStr.c_str());
			strSplit(bodyStr, " ");
			double z = atof(headStr.c_str());
			//LlNgÀW©çSIGVerseÀWÖÌÏ·                            
			double gx = cos(m_yrot)*x - sin(m_yrot)*z;
			double gz = sin(m_yrot)*x + cos(m_yrot)*z;
			my->setPosition(m_posx + gx, m_posy + y, m_posz + gz);
			continue;
		}

		//ÌSÌÌñ]                                                        
		else if (headStr == "WAIST")
		{
			strSplit(bodyStr, ",");
			double w = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double x = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double y = atof(headStr.c_str());
			strSplit(bodyStr, " ");
			double z = atof(headStr.c_str());
			my->setJointQuaternion("ROOT_JOINT0", w, x, y, z);

			double t_yrot = acos(fabs(w)) * 2;
			if (w*y > 0)
				t_yrot = -1 * t_yrot;
			waist << logTime << "," << t_yrot << "," << pyaw << "," << t_yrot + pyaw << endl;
			continue;
		}

		else if (headStr == "END")
		{
			break;
		}



		//ÖßÌñ]                                                          
		else
		{
			string type = headStr.c_str();

			strSplit(bodyStr, ",");
			double w = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double x = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double y = atof(headStr.c_str());
			strSplit(bodyStr, " ");
			double z = atof(headStr.c_str());

			if (type == "RARM_JOINT2"){
				double q[4];
				q[0] = w;
				q[1] = x;
				q[2] = y;
				q[3] = z;
				double aroll = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
				arm << logTime << "," << aroll << ",";
			}
			else if (type == "RARM_JOINT3"){
				double q[4];
				q[0] = w;
				q[1] = x;
				q[2] = y;
				q[3] = z;
				double aroll = atan2(2 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
				arm << aroll << std::endl;
			}

			double angle = acos(w) * 2;
			double tmp = sin(angle / 2);
			double vx = x / tmp;
			double vy = y / tmp;
			double vz = z / tmp;
			double len = sqrt(vx*vx + vy*vy + vz*vz);
			if (len < (1 - m_range) || (1 + m_range) < len) continue;
			if (type != "HEAD_JOINT1"){
				my->setJointQuaternion(type.c_str(), w, x, y, z);
			}
			continue;
		}
	}

}

void ActionLogger::moveByOrs()
{
	//©ª©gÌæŸ                                                              
	SimObj *my = getObj(name_man.c_str());

	double yaw, pitch, roll;

	strSplit(bodyStr, ",");
	yaw = atof(headStr.c_str());

	strSplit(bodyStr, ",");
	pitch = atof(headStr.c_str());

	strSplit(bodyStr, ",");
	roll = atof(headStr.c_str());

	if (yaw == pyaw && pitch == ppitch && roll == proll)  return;
	else {
		pyaw = yaw;
		ppitch = pitch;
		proll = roll;
	}

	dQuaternion qyaw;
	dQuaternion qpitch;
	dQuaternion qroll;

	qyaw[0] = cos(-yaw / 2.0);
	qyaw[1] = 0.0;
	qyaw[2] = sin(-yaw / 2.0);
	qyaw[3] = 0.0;

	qpitch[0] = cos(-pitch / 2.0);
	qpitch[1] = sin(-pitch / 2.0);
	qpitch[2] = 0.0;
	qpitch[3] = 0.0;

	qroll[0] = cos(-roll / 2.0);
	qroll[1] = 0.0;
	qroll[2] = 0.0;
	qroll[3] = sin(-roll / 2.0);
	dQuaternion tmpQ1;
	dQuaternion tmpQ2;

	dQMultiply0(tmpQ1, qyaw, qpitch);
	dQMultiply0(tmpQ2, tmpQ1, qroll);

	//dQuaternion bodyQ;
	//bodyQ[0] = m_qw;
	//bodyQ[1] = m_qx;
	//bodyQ[2] = m_qy;
	//bodyQ[3] = m_qz;

	//dQuaternion tmpQ3;
	//dQMultiply1(tmpQ3, bodyQ, tmpQ2);

	//my->setJointQuaternion("HEAD_JOINT0", tmpQ3[0], tmpQ3[1], -tmpQ3[2], tmpQ3[3]);
	my->setJointQuaternion("HEAD_JOINT0", tmpQ2[0], tmpQ2[1] - m_qx, -tmpQ2[2] - m_qy, tmpQ2[3] - m_qz);
}

void ActionLogger::playActionLog(double time)
{
	//bZ[WæŸ
	if (logSize > logIndex){
		string all_msg = actionLog[logIndex];

		strSplit(all_msg, ":");
		if (headStr == "TIME")
		{
			strSplit(bodyStr, " ");
			logTime = atof(headStr.c_str());
			time_target =  logTime + time_start - time_init;

			if (time >= time_target)
			{
				time_diff = time_target - time_pre_target;
				
				//ÎÛÌæŸ
				SimObj *my = getObj(name_man.c_str());
				strSplit(bodyStr, " ");
				if (headStr == "KINECT_DATA"){
					log_xtion << all_msg << std::endl;
					moveByKinect();

					//ªÌÊuÆCèÌÊuÌoÍ
					Vector3d man_0_Rhand;
					Vector3d man_0_Lhand;
					Vector3d man_0_head;
					Vector3d man_0_elbow;
					SimObj *man = getObj("man_0");
					man->getPartsPosition(man_0_Rhand, "RARM_LINK7");
					man->getPartsPosition(man_0_Lhand, "LARM_LINK7");
					man->getPartsPosition(man_0_head, "HEAD_LINK");
					double elbow_angle = man->getJointAngle("RARM_JOINT3");
					//LOG_MSG(("man_hand  :%4f", man_0.x()));
					log_hand << man_0_Rhand.x() << "," << man_0_Rhand.y() << "," << man_0_Rhand.z() << "," << " " << "," << man_0_Lhand.x() << "," << man_0_Lhand.y() << "," << man_0_Lhand.z() << "," << std::endl;
					log_head << man_0_head.x() << "," << man_0_head.y() << "," << man_0_head.z() << "," << std::endl;
					log_elbow << elbow_angle << std::endl;

					time_xtion += time_diff;
					xtion_count++;
				}
				else if (headStr == "ORS_DATA"){
					log_ors << all_msg << std::endl;
					moveByOrs();
					time_ors += time_diff;
					ors_count++;
				}
				logIndex++;
				time_pre_target = time_target;
			}
		}
	}
	else
	{
		LOG_MSG(("Play End  :%4f", time));
		LOG_MSG(("Play Time :%4f", time - (time_start - time_init)));
		LOG_MSG(("XTION TIME:%4f", time_xtion / xtion_count));
		LOG_MSG(("ORS TIME  :%4f\n", time_ors / ors_count));
		sendMsg("SIGViewer", "Play End\n");
		sendMsg(name_man, "Play End");
		play = false;
		logIndex = 0;
		time_pre_target = 0;
		time_xtion = 0;
		xtion_count = 0;
		time_ors = 0;
		ors_count = 0;
		actionLog.clear();
	}
}

double ActionLogger::onAction(ActionEvent &evt)
{
		if (write == true){
			play = false;
		}
		else if (play == true){
			write = false;
			time_current = gettimeofday_sec() - time_init;
			playActionLog(time_current);
		}


		bool available_Capture = checkService("AvatarView");
		bool Connect_Capture = false;
		if (available_Capture && m_view == NULL){
			m_view = connectToService("AvatarView");
			Connect_Capture = true;
		}
		else if (!available_Capture && m_view != NULL){
			m_view = NULL;
		}

		return 0.001;
}

void ActionLogger::onRecvMsg(RecvMsgEvent &evt)
{
	std::string sender = evt.getSender();
	//bZ[WæŸ                                                              
	string all_msg = evt.getMsg();
	strSplit(all_msg, " ");
	
	if (all_msg == "rec"){
		if (write != true){
			sendMsg("man_0" , "rec");
			sendMsg("SIGViewer", "Rec Start\n");
					
			TimeNum = get_time();
			std::string msg = "Time " + TimeNum;
			sendMsg("man_0", msg);
			FolderName = name_man + "/" + TimeNum;
			rp_FolderName = rp_name + "/" + TimeNum;
			Pick_up += "/" + TimeNum;
			if(mkdir(name_man.c_str(),S_IEXEC|S_IWRITE|S_IREAD) != 0);
			if(mkdir("RelativePosition",S_IEXEC|S_IWRITE|S_IREAD) != 0);
			if(mkdir(Pick_up.c_str(),S_IEXEC|S_IWRITE|S_IREAD) != 0);
		
			write = true;
			writeInit = true;
			time_start = gettimeofday_sec();
			LOG_MSG(("Rec Start :%4f", time_start - time_init));
		}
	}
	else if (all_msg == "stop"){
		broadcastMsg("stop");
		double time = gettimeofday_sec() - time_init;
		sendMsg("SIGViewer", "Rec Stop\n");
		LOG_MSG(("Rec Stop  :%4f", time));
		LOG_MSG(("Rec Time  :%4f\n", time - (time_start - time_init)));
		write = false;
	}
	else if (all_msg == "play"){
		if (play != true){
			//sendMsg("voiceLog1", "play");
			sendMsg(name_man, "play");
			sendMsg("SIGViewer", "Play Start\n");
					
			play = true;
			write = false;
			time_start = gettimeofday_sec();
			LOG_MSG(("logFile:%s", logName.c_str()));
			LOG_MSG(("Play Start:%4f", time_start - time_init));
			readActionLog();
		}
	}
	else if (all_msg == "getTime"){
		std::ostringstream oss;
		oss << time_current;
		//cout << oss.str() << endl;
		//LOG_MSG(("logger1 : %s", all_msg.c_str()));
		m_view->sendMsgToSrv(oss.str());
	}
	
	if (write == true){
		if(all_msg != "getTime"){
			writeActionLog(all_msg);
		}
	}

	//if(sender == "man_0"){
	strSplit(all_msg, " ");
	if(headStr == "ObjName"){
		obj_name = bodyStr;
		LOG_MSG(("Object Name is %s",obj_name.c_str()));
		std::string objNameMsg = all_msg;
		m_view->sendMsgToSrv(objNameMsg);
		LOG_MSG(("man_0 send Msg to view service : %s",objNameMsg.c_str()));
	}	

}

extern "C"  Controller * createController()
{
	return new ActionLogger;
}
