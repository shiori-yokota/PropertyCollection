#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"
#include "Rotation.h"
#include <algorithm>
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <string>

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )

using namespace std;

class MyController : public Controller {
	public:
		void onInit(InitEvent &evt);
		double onAction(ActionEvent&);
		void onRecvMsg(RecvMsgEvent &evt);
		void onCollision(CollisionEvent &evt);
		
		void QuestionGeneration();
		void strSplit(string msg, string separator);
		void moveBodyByKINECT();
		void moveBodyByOrs();
		void writeActionLog(std::string msg);
		bool recognizeObj(string &name);
		void string2double(const string &str);
	
	private:
		ViewService* m_view;
		BaseService* m_srv_Message;
		BaseService* m_srv_Voice;
		BaseService* m_srv_DB;
		BaseService* m_capture;
		BaseService* m_kinect;
		BaseService* m_ors;
		
		//Position of Human
		SimObj *Human;
		int m_state;
		//Initial position
		double m_posx, m_posy, m_posz;
		double m_yrot;
		double m_range;
		//The maximum number of data
		int m_maxsize;
		//
		double pyaw, ppitch, proll;
		//Angle of the entire body
		double m_qw, m_qx, m_qy, m_qz;
		
		//Position of Object
		SimObj *object;
		//Initial position of Object
		Vector3d ini_pos;
		//Initial rotation of Object
		Rotation ini_rot;
		//Position of Object
		Vector3d obPos;		
		
		int count;
		bool start;
		bool m_grasp;
		bool play;
		bool pickUpOrs;
		bool pickUpKinect;
		
		std::string folderName;
		std::string property;
		
		//vector<string> Voice_msg;
		vector<string> Attributes;
		string Question;
		//int v_index;
		int a_index;
		
		ofstream ofs;
		stringstream ss_ors;
		stringstream ss_kinect;
		stringstream ss_relpos;
		stringstream msg_head_ors;
		std::string fileName;
		std::string dir_prop;
		
		vector<string> MSG;
		string obName;
		vector<string> obj_name;
		vector<string> pos;
		double pos_x;
		double pos_y;
		double pos_z;
		
		//Variable that stores the string is divided at the strSplit
		string headStr;
		string bodyStr;
};

void MyController::onInit(InitEvent &evt) {
	m_kinect = NULL;
	m_ors = NULL;
	m_srv_Message = NULL;
	m_srv_Voice = NULL;
	m_srv_DB = NULL;
	m_capture = NULL;
	
	Human = getObj(myname());
	
	//Initial position acquisition
	Vector3d pos;
	Human->getPosition(pos);
	m_posx = pos.x();
	m_posy = pos.y();
	m_posz = pos.z();
	
	//
	m_qw = 1.0;
	m_qx = 0.0;
	m_qy = 0.0;
	m_qz = 0.0;
	
	pyaw = ppitch = proll = 0.0;
	
	//Whole body of orientation
	Rotation rot;
	Human->getRotation(rot);
	double qw = rot.qw();
	double qy = rot.qy();
	m_yrot = acos(fabs(qw)) * 2;
	if (qw*qy > 0) m_yrot = -1 * m_yrot;
	m_range = 0.1;
	m_maxsize = 15;
	pos_x = -204.736;
	pos_z = -95.0;
	count = 0;
	start = true;
	m_grasp = false;
	pickUpOrs = false;
	pickUpKinect = false;

	m_view = (ViewService*)connectToService("SIGViewer");
	//Check services
	m_state = 1000;
	//v_index = 0;
	a_index = 0;

	//ARM Down
	Human->setJointAngle("LARM_JOINT2", DEG2RAD(-90));
	Human->setJointAngle("RARM_JOINT2", DEG2RAD(90));
	
	//Properties
	Attributes.push_back("色");
	Attributes.push_back("形");
	Attributes.push_back("付属物");
	
	Attributes.push_back("名称");
	Attributes.push_back("用途");
}

double MyController::onAction(ActionEvent &evt) {
	switch (m_state) {
	//ERROR
	case 0: {
		LOG_MSG(("ERROR"));
		//State of stay
		m_state = 100;
		break;
	}
	//Start
	case 1: {
		LOG_MSG(("Start"));
		m_srv_Message->sendMsgToSrv("Start");
		ostringstream os;
		os << "Question:" << Attributes.size();
		string msg = os.str();
		LOG_MSG(("%s",msg.c_str()));
		m_srv_Message->sendMsgToSrv(msg);
		//state of stay
		m_state = 100;
		break;
	}
	
	//Switched to the next object
	case 5: {
		LOG_MSG(("Next"));
		//Get the parts infomation
		CParts *parts = Human->getParts("RARM_LINK7");
		//Release grasping
		parts->releaseObj();
		//Set the grasping flag to neutral
		m_grasp = false;
		
		object->setPosition(ini_pos);
		count++;
		LOG_MSG(("Count %d", count));
		m_state = 6;
		break;
	}
	//Stores the object sent to the array only once
	case 6: {
		if(start) {
			//LOG_MSG(("State 6."));
			LOG_MSG(("MSG.size = %d", MSG.size()));
			for(int i = 0; i < MSG.size(); i++) {
				if(i == 0 || i%2 == 0) {
					obj_name.push_back(MSG[i].c_str());
				}
				else {
					pos.push_back(MSG[i].c_str());
				}
			}
			//Only once output
			/*for(int j = 0; j < obj_name.size(); j++) {
				LOG_MSG(("Name %s", obj_name[j].c_str()));
				LOG_MSG(("Pos %s", pos[j].c_str()));
			}*/
		}
		start = false;
		m_state = 7;
		break;
	}
	//Whether there is a question to object
	case 7: {
		if(!this->recognizeObj(obName)){
			//State of stay
			m_state = 100;
			//LOG_MSG(("7 to 100: FINISH"));
			break;
		}
		else {
			m_state = 10;
			//LOG_MSG(("7 to 10"));
			break;
		}
		break;
	}
	
	//Place on top of the desk
	case 10: {
		object->setPosition(obPos);
		if(count > 0) m_srv_Message->sendMsgToSrv("NEXT:");
		//Check services
		m_state = 1000;
		break;
	}
	
	//Ask the questions
	case 20: {
		//m_srv_Voice->sendMsgToSrv(Voice_msg[v_index].c_str());
		QuestionGeneration();
		m_srv_Voice->sendMsgToSrv(Question.c_str());
		m_state = 100;
		break;
	}
	
	//Back to the position based on the object
	case 30: {
		//Get the part information
		CParts *parts = Human->getParts("RARM_LINK7");
		//Release grasping
		parts->releaseObj();
		//Set the grasping flag to neutral
		m_grasp = false;

		object->setPosition(obPos);
		object->setRotation(ini_rot);
		m_state = 100;
		break;
	}
		
	//State of stay
	case 100:
		m_state = 110;
		break;
	case 110:
		break;
		
	//Check services
	case 1000: {
		//Use Change the object
		if(start) {
			bool available_Message = checkService("ChangeObj");
			bool available_Voice = checkService("VoiceReco_Service");
			bool available_DB = checkService("sigverse_DB");
			
			bool Connect_Msg = false;
			bool Connect_Voice = false;
			bool Connect_DB = false;
			
			if (available_Message && m_srv_Message == NULL) {
				m_srv_Message = connectToService("ChangeObj");
				Connect_Msg = true;
			}
			else if(!available_Message && m_srv_Message != NULL) {
				m_srv_Message = NULL;
			}
			if (available_Voice && m_srv_Voice == NULL) {
				m_srv_Voice = connectToService("VoiceReco_Service");
				Connect_Voice = true;
			}
			else if(!available_Voice && m_srv_Voice != NULL) {
				m_srv_Voice = NULL;
			}
			if (available_DB && m_srv_DB == NULL) {
				m_srv_DB = connectToService("sigverse_DB");
				Connect_DB = true;
			}
			else if(!available_DB && m_srv_DB != NULL) {
				m_srv_DB = NULL;
			}
			if (Connect_Msg && Connect_Voice && Connect_DB){
				LOG_MSG(("Msg, Voice and DB are Connect OK"));
				m_state = 1;
				break;
			}
			else m_state = 0;
		}
		//Use take pictures
		else {
			bool available_Kinect = checkService("SIGKINECT");
			bool available_Ors = checkService("SIGORS");
			bool available_Capture = checkService("AvatarView");
			
			bool Connect_Kinect = false;
			bool Connect_Ors = false;
			bool Connect_Capture = false;
			
			if (available_Kinect && m_kinect == NULL){
				m_kinect = connectToService("SIGKINECT");
				Connect_Kinect = true;
			}
			else if (!available_Kinect && m_kinect != NULL){
				m_kinect = NULL;
			}
			if (available_Ors && m_ors == NULL){
				m_ors = connectToService("SIGORS");
				Connect_Ors = true;
			}
			else if (!available_Ors && m_ors != NULL){
				m_ors = NULL;
			}
			if (available_Capture && m_capture == NULL){
				m_capture = connectToService("AvatarView");
				Connect_Capture = true;
			}
			else if (!available_Capture && m_capture != NULL){
				m_capture = NULL;
			}
						
			if(Connect_Kinect && Connect_Ors && Connect_Capture){
				LOG_MSG(("Kinect, Oculus and Capture are Connected"));
				m_state = 100;
				break;
			}
		}
		//else m_state = 1000;
		break;
	}
	
	}
	return 0.5;
}
	
void MyController::onRecvMsg(RecvMsgEvent &evt) {
	string sender = evt.getSender();
	
	char *all_msg = (char*)evt.getMsg();
	string ss = all_msg;
	
	strSplit(all_msg, " ");
	//After coming array of object is sent
	if(start) {
		//LOG_MSG(("%s", all_msg));
		if (ss == "end"){
			//LOG_MSG(("%s", all_msg));
			m_state =6;
		}
		else MSG.push_back(all_msg);
	}
	else if(ss == "play"){
		if(sender == "logger1"){
			play = true;
		}
	}
	else if(ss == "Play End"){
		if(sender == "logger1"){
			play = false;
		}
	}
	
	if (play != true){
		if (headStr == "ORS_DATA"){
			moveBodyByOrs();
			sendMsg("logger1", ss);
			if(pickUpOrs == true){
				writeActionLog(ss);
			}
		}
		else if (headStr == "KINECT_DATA"){
			moveBodyByKINECT();
			sendMsg("logger1", ss);
			if(pickUpKinect == true){
				writeActionLog(ss);
			}
		}
	}
	
	if(headStr == "Time"){
		folderName = "PickUp/" + bodyStr;
		LOG_MSG(("Folder name is %s",folderName.c_str()));
		dir_prop = "DirProp:" + folderName + "/";
		sendMsg("sigverse_DB", dir_prop);
	}
	else if(headStr == "PickUp"){
		property = bodyStr;
		LOG_MSG(("property is %s",property.c_str()));
		pickUpOrs = true;
		pickUpKinect = true;
	}

	if (ss == "fin"){
		a_index = 0;
		m_state = 5;
	}
	//Ask the same questions
	else if (ss == "roop"){
		m_state = 20;
	}
	//Asked the following questions
	else if(ss == "inc") {
		//v_index++;
		a_index++;
		m_state = 20;
	}
	//New Object
	else if (ss == "go") {
		//m_srv_DB->sendMsgToSrv("Start");
		m_state = 20;
	}
	
	//Take picture
	if (ss == "rec"){
		//m_voice_rec->sendMsgToSrv("rec");
		m_capture->sendMsgToSrv("capture");
	}
	
	//Back to the position based on the object
	if (ss == "put"){
		m_state = 30;
	}
}

void MyController::onCollision(CollisionEvent &evt) {
	if(m_grasp == false) {
		typedef CollisionEvent::WithC C;
		//Get the name of the touched entity
		const vector<string> & with = evt.getWith();
		//Get the collision was their parts
		const vector<string> & mparts = evt.getMyParts();
		//The loop in the collision entity
		for (int i = 0; i < with.size(); i++){
			//printf("obname:%s", with[i].c_str());
			if (obName == with[i]){
				//When collided with the right hand
				if (mparts[i] == "RARM_LINK7"){
					//Get the parts of their own hands
					CParts * parts = Human->getParts("RARM_LINK7");
					if (parts->graspObj(with[i])) m_grasp = true;
				}
				//When collided with the left hand
				else if (mparts[i] == "LARM_LINK7"){
					//Get the parts of their own hands
					CParts * parts = Human->getParts("LARM_LINK7");
					if (parts->graspObj(with[i])) m_grasp = true;
				}
			}
		}
	}
}

void MyController::QuestionGeneration() {
	string id;
	id = "id:" + obj_name[count];
	LOG_MSG(("%s",id.c_str()));
	m_srv_DB->sendMsgToSrv(id);
	
	string Attribute;
	Attribute = Attributes[a_index].c_str();
	Question = "これの" + Attribute + "は なんですか";
	LOG_MSG(("question : %s", Question.c_str()));
	
	if(Attribute == "色"){
		m_srv_DB->sendMsgToSrv("color");
	}
	else if(Attribute == "名称"){
		m_srv_DB->sendMsgToSrv("name");
	}
	else if(Attribute == "形"){
		m_srv_DB->sendMsgToSrv("shape");
	}
	else if(Attribute == "付属物"){
		m_srv_DB->sendMsgToSrv("attachment");
	}
	else if(Attribute == "用途"){
		m_srv_DB->sendMsgToSrv("how_to_use");
	}
/*
	Voice_msg.push_back("これの名前はなんですか");
	Voice_msg.push_back("ほかに、呼び方はありますか");
	Voice_msg.push_back("これは、どういう色ですか");
	Voice_msg.push_back("ほかに、色の特徴はありますか");
	Voice_msg.push_back("これは、どんな形ですか");
	Voice_msg.push_back("ほかに、形の特徴はありますか");
	Voice_msg.push_back("これの使い方はなんですか");
	Voice_msg.push_back("ほかに、使い方はありますか");
	Voice_msg.push_back("これは、どういう状態ですか");
	Voice_msg.push_back("ほかに、わかることはありますか");
*/	
}

/* 
	 strSplit
	 Delimit a string in the separator
	 the headStr earlier than separator
	 the bodyStr later than separator
	 store each
*/
void MyController::strSplit(string msg, string separator) {
	int strPos1 = 0;
	int strPos2;
	string head;
	string body;

	strPos2 = msg.find_first_of(separator, strPos1);
	head.assign(msg, strPos1, strPos2 - strPos1);
	body.assign(msg, strPos2 + 1, msg.length() - strPos2);
	headStr = head;
	bodyStr = body;
}

void MyController::moveBodyByKINECT(){
	int i = 0;
	while (true){
		i++;
		if (i == m_maxsize + 1) break;
		strSplit(bodyStr, ":");

		//Position of the body
		if (headStr == "POSITION")
		{
			strSplit(bodyStr, ",");
			double x = atof(headStr.c_str());
			strSplit(bodyStr, ",");
			double y = atof(headStr.c_str());
			strSplit(bodyStr, " ");
			double z = atof(headStr.c_str());
			//Conversion from Kinect coordinates to SIGVerse coordinate
			double gx = cos(m_yrot)*x - sin(m_yrot)*z;
			double gz = sin(m_yrot)*x + cos(m_yrot)*z;
			Human->setPosition(m_posx + gx, m_posy + y, m_posz + gz);
			continue;
		}
		//Rotation of the body
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
			Human->setJointQuaternion("ROOT_JOINT0", w, x, y, z);
			m_qw = w;
			m_qx = x;
			m_qy = y;
			m_qz = z;
			continue;
		}
		else if (headStr == "END")
		{
			break;
		}
		//Rotation of the joint
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
			double angle = acos(w) * 2;
			double tmp = sin(angle / 2);
			double vx = x / tmp;
			double vy = y / tmp;
			double vz = z / tmp;
			double len = sqrt(vx*vx + vy*vy + vz*vz);
			if (len < (1 - m_range) || (1 + m_range) < len) continue;
			if (type != "HEAD_JOINT1"){
				Human->setJointQuaternion(type.c_str(), w, x, y, z);
			}
			continue;
		}
	}
}

//Manipulate the avatar based on data from the SIG_ORS (neck joint)
void MyController::moveBodyByOrs() {
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

	Human->setJointQuaternion("HEAD_JOINT0", tmpQ2[0], tmpQ2[1]-m_qx, -tmpQ2[2]-m_qy, tmpQ2[3]-m_qz);
}

void MyController::writeActionLog(std::string msg)
{

	if(pickUpOrs == true && pickUpKinect == true){
		fileName = folderName + "/" + obName + "_" + property + ".txt";
		
		ofstream clear(fileName.c_str(), ios::trunc);
	}
	
	ofs.open(fileName.c_str(), ios::app);
	
	if(!ofs){
		std::cout << "don't open" <<std::endl;
	}
	
	strSplit(msg, " ");
	if(headStr == "ORS_DATA"){
		LOG_MSG(("write action log %s",headStr.c_str()));
		ss_ors << msg;
		ofs << ss_ors.str() << std::endl;
		pickUpOrs = false;
		LOG_MSG(("ORS is %d",pickUpOrs));
	}
	else if(headStr == "KINECT_DATA"){
		LOG_MSG(("write action log %s",headStr.c_str()));
		ss_kinect << msg;
		ofs << ss_kinect.str() << std::endl;
		pickUpKinect = false;
		LOG_MSG(("KINECT is %d",pickUpKinect));
	}
	
	//人の頭の位置、物体の向き、物体の位置
	//getjoint頭（xyz）、物体（xyz、クオータニオン）
	if(pickUpOrs != true && pickUpKinect != true){
		ofs << std::endl;
		ofs << "relative position" << std::endl;
		strSplit(msg, " ");
		if (headStr == "ORS_DATA"){
			msg_head_ors << bodyStr;
			ss_relpos << headStr << " " << msg_head_ors.str();
		}
		Vector3d head_pos;
		Human->getJointPosition(head_pos, "HEAD_JOINT1");
		ss_relpos << " HumanHeadPosition" << " " << head_pos.x() << "," << head_pos.y() << "," << head_pos.z();
	
		Rotation obj_rot;
		object->getRotation(obj_rot);
		ss_relpos << " ObjectRotation" << " " << obj_rot.qw() << "," << obj_rot.qx() << "," << obj_rot.qy() << "," << obj_rot.qz();
	
		Vector3d obj_pos;
		object->getPosition(obj_pos);
		ss_relpos << " ObjectPosition" << " " << obj_pos.x() << "," << obj_pos.y() << "," << obj_pos.z();
	}
	
	ofs << ss_relpos.str() << std::endl;
	
	ofs.close();
	ss_ors.str("");
	ss_ors.clear(stringstream::goodbit);
	ss_kinect.str("");
	ss_kinect.clear(stringstream::goodbit);
	ss_relpos.str("");
	ss_relpos.clear(stringstream::goodbit);
}

void MyController::string2double(const std::string &str) {
	std::stringstream ss;
	ss << str;
	ss >> pos_y;
}

bool MyController::recognizeObj(std::string &name) {
	//LOG_MSG(("%d %d", count, obj_name.size()));
	if (count > obj_name.size()){
		return false;
	}
	else{
		//Get the name of the object
		name = obj_name[count];
		string objNameMsg = "ObjName " + name;
		sendMsg("logger1",objNameMsg);
		object = getObj(name.c_str());
		object->getPosition(ini_pos);
		object->getRotation(ini_rot);
		LOG_MSG(("Object name : %s", name.c_str()));
		this->string2double(pos[count].c_str());
		//Position of the object
		obPos.x(pos_x);
		obPos.y(pos_y);
		obPos.z(pos_z);
		return true;
	}
}

extern "C" Controller * createController() {
	return new MyController;
}
