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
	//初期位置                                                                    
	double m_posx, m_posy, m_posz;
	double m_yrot;
	double m_range;
	//データ数（関節数）最大値                                                    
	int m_maxsize;

	BaseService* m_kinect;
	BaseService* m_ors;
	BaseService* m_view;

	Vector3d m_pos;
	Rotation m_rotation;
	string body_str;
	int count;
	ofstream ofs;
	stringstream ss;

	bool play;
	bool write;
	bool playLog;
	bool writeInit;
	double time_init;
	double time_start;
	double time_current;
	double time_target;
	int logIndex;
	int logSize;
	vector<string> actionLog;
	string logName;

	int ors_count;
	int xtion_count;
	double time_ors;
	double time_xtion;
	double time_pre_target;
	double time_diff;

	// 前回送信したyaw, pitch roll
	double pyaw, ppitch, proll;
	// 体全体の角度
	double m_qw, m_qy, m_qx, m_qz;

	string headStr;
	string bodyStr;

	std::string name_man;

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
	play = false;
	write = false;
	writeInit = false;
	
	time_start = 0;
	time_current = 0;
	time_target = 0;
	logIndex = 0;
	logSize = 0;

	name_man = "man_0";

	body_str = "";

	SimObj *my = getObj(name_man.c_str());

	// 初期位置取得                                                            
	Vector3d pos;
	my->getPosition(pos);
	m_posx = pos.x();
	m_posy = pos.y();
	m_posz = pos.z();

	// 初期姿勢（回転）取得                                                                                           
	Rotation rot;
	my->getRotation(rot);
	double qw = rot.qw();
	double qy = rot.qy();

	m_yrot = acos(fabs(qw)) * 2;
	if (qw*qy > 0)
		m_yrot = -1 * m_yrot;

	m_range = 0.1;
	m_maxsize = 15;

	// 体全体の向き
	m_qw = 1.0;
	m_qx = 0.0;
	m_qy = 0.0;
	m_qz = 0.0;

	pyaw = ppitch = proll = 0.0;

	time_init = gettimeofday_sec();
	
	TimeNum = get_time();
	FolderName = name_man + "/" + TimeNum;
	
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
	if (writeInit == true){
		writeInit = false;
		TimeNum = get_time();
		logName = name_man + "/" + TimeNum + ".txt";
		
		ofstream clear(logName.c_str(), ios::trunc);
		ofs.open(logName.c_str(), ios::app);
	}

	//ログとして記録する情報
	//時間
	time_current = gettimeofday_sec() - time_start;
	ss << "TIME:" << time_current << " ";
	ss << msg;

	body_str += ss.str();

	ofs << body_str << std::endl;

	body_str = "";
	ss.str("");
	ss.clear(stringstream::goodbit);

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
	//自分自身の取得                                                              
	SimObj *my = getObj(name_man.c_str());

	int i = 0;
	while (true)
	{
		i++;
		if (i == m_maxsize + 1) break;
		strSplit(bodyStr, ":");

		//体の位置                                                            
		if (headStr == "POSITION")
		{
			strSplit(bodyStr, ",");
			double x = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double y = atof(headStr.c_str());
			strSplit(bodyStr, " ");
			double z = atof(headStr.c_str());
			//キネクト座標からSIGVerse座標への変換                            
			double gx = cos(m_yrot)*x - sin(m_yrot)*z;
			double gz = sin(m_yrot)*x + cos(m_yrot)*z;
			my->setPosition(m_posx + gx, m_posy + y, m_posz + gz);
			continue;
		}

		//体全体の回転                                                        
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



		//関節の回転                                                          
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
	//自分自身の取得                                                              
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
	//メッセージ取得
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
				
				//対象の取得
				SimObj *my = getObj(name_man.c_str());
				strSplit(bodyStr, " ");
				if (headStr == "KINECT_DATA"){
					log_xtion << all_msg << std::endl;
					moveByKinect();

					//頭の位置と，手の位置の出力
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

	//メッセージ取得                                                              
	string all_msg = evt.getMsg();
	
	if (all_msg == "rec"){
		if (write != true){
			sendMsg("man_0" , "rec");
			sendMsg("SIGViewer", "Rec Start\n");
						
			TimeNum = get_time();
			FolderName = name_man + "/" + TimeNum;
			if(mkdir(name_man.c_str(),S_IEXEC|S_IWRITE|S_IREAD) != 0);
			
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
	else if (write == true){
		writeActionLog(all_msg);
	}
	
}

extern "C"  Controller * createController()
{
	return new ActionLogger;
}
