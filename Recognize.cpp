#include "ControllerEvent.h"
#include "Controller.h"
#include "Logger.h"
#include <unistd.h>
#include <algorithm>
#define PI 3.1415926535
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )
class MyController : public Controller {
  public:
    void onInit(InitEvent &evt);
    double onAction(ActionEvent&);
    void onRecvMsg(RecvMsgEvent &evt);
  private:
    std::vector<std::string> m_trashes;
    int m_state;
    std::string message;
};

void MyController::onInit(InitEvent &evt)
{
  m_trashes.push_back("can_0");
  m_trashes.push_back("can_1");
  m_trashes.push_back("petbottle_0");
  srand((unsigned)time( NULL ));
  m_state = 0;
}

double MyController::onAction(ActionEvent &evt)
{
  switch(m_state){
    case 0:
      break;
    case 10:
      sendMsg("sigverse_DB",message);
			m_state = 0;
      break;
  }
  return 0.5;
}

void MyController::onRecvMsg(RecvMsgEvent &evt)
{
  std::string sender = evt.getSender();
  std::string msg = evt.getMsg();
  message = msg;
  m_state = 10;

}

extern "C" Controller * createController()
{
  return new MyController;
}
