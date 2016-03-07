
#include "project.h"

#include "DoitArm.h"
DoitArm arm;

static char strFloat[32];
char* printFloat(float a)
{
	dtostrf(a, 4, 3, strFloat);
	return strFloat;
}
void printHelp(void)
{
  print_log("\r\n----------Doit ARM demo V1.0----------\r\n");
  print_log("'h' : print this help\r\n");
  print_log("'z' : goto zero position\r\n");
  print_log("'p' : print arm status\r\n");
  print_log("'d' : set/get current speed, set: d:1,1  1~10  get: d:2\r\n");
  print_log("'c' : movetoCart, c:x,y,z\r\n");
  print_log("'t' : movetoTool, t:x,y,z\r\n");
  print_log("'s' : sigle axis move, s:1,10\r\n");
  print_log("      1: [-73,82]\r\n");
  print_log("      2: [-18,22]\r\n");
  print_log("      3: [-20,40]\r\n");
  print_log("      4: [-20,80]\r\n");
  print_log("      5: [-55,80]\r\n");
  print_log("      6: [-20,65]\r\n");
  print_log("'w' : set/goto waitPosition, w:0 set1 w:1 set2 w:2 goto w:3 get\r\n");
  print_log("----------------------------------------------------\r\n");
}
void doPrintStatus(void)
{
  float a[6];
  arm.getCurrentPosition(a, 6);
  float b[6];
  arm.getTargetPosition(b, 6);
  uint8_t v = arm.getCurrentSpeed();
  char t[256];
  sprintf(t, " Status:%d(0:SERVO_OFF 1:STAND_BY 2:MOVING)\r\n Speed:%d\r\n",
          arm.getArmStatus(),v);
  print_log("-------Arm Status-------\r\n");
  print_log(t);
  print_log(" Current pos:(");
  print_log(printFloat(a[0]));print_log(",");
  print_log(printFloat(a[1]));print_log(",");
  print_log(printFloat(a[2]));print_log(",");
  print_log(printFloat(a[3]));print_log(",");
  print_log(printFloat(a[4]));print_log(",");
  print_log(printFloat(a[5]));print_log(")\r\n");
  print_log(" Target  pos:(");
  print_log(printFloat(b[0]));print_log(",");
  print_log(printFloat(b[1]));print_log(",");
  print_log(printFloat(b[2]));print_log(",");
  print_log(printFloat(b[3]));print_log(",");
  print_log(printFloat(b[4]));print_log(",");
  print_log(printFloat(b[5]));print_log(")\r\n"); 
  
  DoitArm::stMat mat;
  arm.getToolCurrentPosture(&mat);
  char temp[256];
  char m[12];
  print_log("Current Posture:\r\n");
  dtostrf(mat.nx, 4, 3, m); sprintf(temp, "%s", m);
  dtostrf(mat.ox, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(mat.ax, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(mat.px, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(mat.ny, 4, 3, m); strcat(temp, "\r\n"); strcat(temp, m);
  dtostrf(mat.oy, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(mat.ay, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(mat.py, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(mat.nz, 4, 3, m); strcat(temp, "\r\n"); strcat(temp, m);
  dtostrf(mat.oz, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(mat.az, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(mat.pz, 4, 3, m); strcat(temp, "\t"); strcat(temp, m); strcat(temp, "\r\n0\t0\t0\t1\t\r\n");
  print_log(temp);
  
  print_log("------------------------\r\n");
}
void doSigleAxisMove(char *p)
{
  if (strlen(p) < 5 ||
      *p != 's' ||
      !(*(p + 2) >= '1' && *(p + 2) <= '6'))
  {
    print_log("Invalid parameters!\r\n");
    return;
  }
  for (int i = 0; i < strlen(p + 4); i++)
  {
    if (!((*(p + 4 + i) >= '0' && *(p + 4 + i) <= '9') || 
          (*(p + 4 + i) == '-' && i == 0) || 
          (*(p + 4 + i) == '.')))
    {
      print_log("Invalid parameters!\r\n");
      return;
    }
  }
  int axisNum = *(p + 2) - '0';
  float angle = atof(p + 4);// 0 ~180  -> SERVO_LIMIT_MIN~ SERVO_LIMIT_MAX
  if (angle < -90 || angle > 90)
  {
    print_log("angle should between:-90~90\r\n");
    return;
  }
  char t[64];
  sprintf(t, "SigleAxisMove axis:%d,angle:%s\t\r\n", axisNum, printFloat(angle));
  print_log(t);
  //arm.moveAxis(axisNum, angle);
  arm.setTargetPosition(axisNum, angle);
  arm.beginMove();
}
void doWaitPosition(char* p)
{
  if (strlen(p) < 3 )
  {
    print_log("Invalid parameters!  w:1 or w:2 w:3\r\n");
    return;
  }
  if (*(p + 2) == '0')
  {
    float pos[6] = {0,-15, 10, 0, 80, 0};
    arm.setWaitPosition(pos, 6);
    arm.gotoWaitPosition();
    print_log("gotoWaitPosition ok!\r\n");
  }
  else if (*(p + 2) == '1')
  {
    float a[6] = {70,-15, 10, 0, 80, 0};
    arm.setWaitPosition(a, 6);
    arm.gotoWaitPosition();
    print_log("gotoWaitPosition ok!\r\n");
  }
  else if (*(p + 2) == '2')
  {
    float a[6] = { 20,20,20,-10,30,30};
    arm.setWaitPosition(a, 6);
    arm.gotoWaitPosition();
    print_log("gotoWaitPosition ok!\r\n");
  }
  else if (*(p + 2) == '3')
  {
    float a[6] = {10,-5,20,-10,30,30};
    arm.setWaitPosition(a, 6);
    arm.gotoWaitPosition();
    print_log("gotoWaitPosition ok!\r\n");
  }
  else if (*(p + 2) == '4')
  {
    float a[6] = {-10,5,-10,50,30,30};
    arm.setWaitPosition(a, 6);
    arm.gotoWaitPosition();
    print_log("gotoWaitPosition ok!\r\n");
  }
  else
  {
    print_log("Invalid parameters! w:1 or w:2 w:3\r\n");
  }
}

void doSetCurrentSpeed(char* p)
{ //set: d:1,1  1~10  get: d:2
  if (strlen(p) < 3 )
  {
    print_log("Invalid parameters!  d:1,1 or d:2\r\n");
    return;
  }
  if (*(p + 2) == '1' && (strlen(p) == 5 || strlen(p) == 6))
  {
    int a = atoi(p + 4);
    if (arm.setCurrentSpeed(a) == true)
    {
      char t[64];
      sprintf(t, "Set current speed to : %d ok\r\n", a);
      print_log(t);
    }
    else
      print_log("Set current speed failed\r\n");
  }
  else
  {
    uint8_t v = arm.getCurrentSpeed();
    char t[64];
    sprintf(t, "Current speed is : %d\r\n", v);
    print_log(t);
  }
}
void moveFinished_cb(void)
{
  print_log("moveFinished_cb\r\n");
  doPrintStatus();
}
/*
  初始化
*/
void initParseData()
{
  /*s:1,-17    -90~65
s:2,-2     -20~20        反的
s:3,-45    -60~-5
s:4,-70    -90~10
s:5,-30    -85~50
s:6,-45    -65~25*/
  Serial.println("[initParseData]");
  printHelp();
  
  //各轴舵机中位偏移
  float bias[6] = { -17, -2, -45, -70, -30, -45};
  arm.setServoBias(bias,6);
  //各轴舵机移动范围
  arm.setServoRange(-73,82,-18,22,-20,40,-20,80,-55,80,-20,65);
  //各轴舵机在驱动板上的位置
  arm.setChannel(0, 4, 7, 8, 5, 2);
  arm.attachServo();
  arm.setCurrentSpeed(2);
  //设置零点
  arm.gotoZeroPosition();
  
  //int a[6] = {0,0,0,0,0};
  //arm.setWaitPosition(a, 6);
  //arm.gotoWaitPosition();
  arm.registMoveFinishCB(&moveFinished_cb);
}
void split(char *src, const char *separator, char **dest, int *num)
{
  char *pNext;
  int count = 0;
  if (src == NULL || strlen(src) == 0) return;
  if (separator == NULL || strlen(separator) == 0) return;
  pNext = strtok(src, separator);
  while (pNext != NULL)
  {
    *dest++ = pNext; ++count;
    pNext = strtok(NULL, separator);
  }
  *num = count;
}
void doMoveToCart(char *p)
{
  //movetoCart, c:x,y,z
  char *numStr[128];
  int num = 0;
  split(p+2, ",", numStr, &num);
  if (num != 3)
  {
    print_log("Invalid parameters!  c:10,20,30\r\n");
    return;
  }
  float x = atof(numStr[0]);
  float y = atof(numStr[1]);
  float z = atof(numStr[2]);
  if(!arm.movetoCart(x,y,z))
  {
    print_log("Can not move!\r\n");
  }
}
void doMoveToTool(char *p)
{
  //doMoveToTool, t:x,y,z
  char *numStr[128];
  int num = 0;
  split(p+2, ",", numStr, &num);
  if (num != 3)
  {
    print_log("Invalid parameters!  t:10,20,30\r\n");
    return;
  }
  float x = atof(numStr[0]);
  float y = atof(numStr[1]);
  float z = atof(numStr[2]);
  if(!arm.movetoTool(x,y,z))
  {
    print_log("Can not move!\r\n");
  }
}
void doGotoZeroPosition(void)
{
  arm.gotoZeroPosition();
  print_log("goto ZeroPosition\r\n");
}
/*
  UDP接收包处理
*/
void parseUDPPackage(char *p)
{
  switch (*p)
  {
    case 'h': printHelp(); break;
    case 'z': doGotoZeroPosition(); break;
    case 'p': doPrintStatus(); break;
    case 'c': doMoveToCart(p); break;
    case 't': doMoveToTool(p); break;
    case 'd': doSetCurrentSpeed(p); break;
    case 's': doSigleAxisMove(p); break;
    case 'w': doWaitPosition(p); break;
    default:
      print_log("[Invalid CMD]"); delay(10);
      print_log(p); delay(10); break;
  }
}
/*
  TCP接收包处理
*/
void parseTCPPackage(char *p)
{
  print_log("[TCP parseData:]");
  print_log(p);
}
/*
  串口接收包处理
*/
void parseUartPackage(char *p)
{
  print_log("[Uart parseData]");
  print_log(p);
  parseUDPPackage(p);
}

//static u32 pretick = 0;
void doArmTick(void)
{
  arm.handleServo();
  //急停
  //arm.emeryStop();

  /*if (millis() - pretick > 1000)
    {
    pretick = millis();
    print_log(".");
    }*/
}

