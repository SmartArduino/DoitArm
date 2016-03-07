/******************************************************************************
  File Name          : DoitArm.cpp
  Author             : Doit.am
  Updated            : Doit.am
  Version            : V0.0.2
  Created Date       : 2016-01-17
  Modified Date      : 2016-01-18
  Description        :
  License            :
  Copyright(C) 2016 Doctors of Intelligence & Technolog Company.
  All right reserved.
*******************************************************************************/

#include "DoitArm.h"
#include <Wire.h>
#define WIRE Wire
#define LED0_ON_L 0x6
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#include "math.h"
#define PI 3.1415926
#define MINUS 0.0001

#define SERVO_LIMIT_MIN 102
#define SERVO_LIMIT_MAX 512

//#define SIMULATE_DOIT_ARM
//#define DEBUG_DOIT_ARM
extern void print_log(char*);

#ifdef DEBUG_DOIT_ARM
/*
  #define print_debug(format,...) do{\
                    char buff[256];char buff2[32];\
                    sprintf(buff2,"[%d]",millis());\
                    sprintf(buff,format,##__VA_ARGS__);\
                    strcat(buff,buff2);\
                    print_log(buff);\
                    }while(0)*/
#define print_debug(format,...) do{\
    char buff[256];\
    sprintf(buff,format,##__VA_ARGS__);\
    print_log(buff);\
  }while(0)
#else
#define print_debug(...)
#endif

/*
  构造函数
*/
DoitArm::DoitArm()
{
  bServoOn = false;
  armStatus = SERVO_OFF;
  currentSpeed = 5;//默认当前速度为5
  for (int i = 0; i < 6; i++)
  {
    axisRg[i].minus = -90;
    axisRg[i].bias = 0;
    axisRg[i].maxium = 90;
    axisRg[i].current = 0;
    axisRg[i].target = axisRg[i].current;
    axisRg[i].wait = axisRg[i].current;
  }
  axisRg[0].i2c_addr = 1;
  axisRg[1].i2c_addr = 1;
  axisRg[2].i2c_addr = 1;
  axisRg[3].i2c_addr = 1;
  axisRg[4].i2c_addr = 1;
  axisRg[5].i2c_addr = 1;
  setChannel(0, 1, 2, 3, 4, 5);
  pMoveFinish_cb_fun = NULL;
}

/*
  矩阵初始化
*/
void DoitArm::matrix_init(_Matrix *m)
{
  m->arr = (float *)malloc( 4 * 4 * sizeof(float));
  memset(m->arr, 0x00, 4 * 4 * sizeof(float));
}
/*
  释放资源
*/
void DoitArm::matrix_free(_Matrix *m)
{
  free(m->arr);
}
/*
  读矩阵元素
*/
float DoitArm::matrix_read(_Matrix *m, int i, int j)
{
  if (i >= 4 || j >= 4)
  {
    return -31415;
  }
  return *(m->arr + i * 4 + j);
}
/*
  写矩阵元素
*/
int DoitArm::matrix_write(_Matrix *m, int i, int j, float val)
{
  if (i >= 4 || j >= 4)
  {
    return -1;
  }

  *(m->arr + i * 4 + j) = val;
  return 1;
}
/*
  矩阵相乘
  C = A * B
  成功返回1,失败返回-1
*/
int DoitArm::matrix_multiply(_Matrix *A, _Matrix *B, _Matrix *C)
{
  int i = 0;
  int j = 0;
  int k = 0;
  float temp = 0;

  //运算
  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 4; j++)
    {
      temp = 0;
      for (k = 0; k < 4; k++)
      {
        temp += matrix_read(A, i, k) * matrix_read(B, k, j);
      }
      matrix_write(C, i, j, temp);
    }
  }
  return 1;
}

/*
  判断向量是否正交
*/
bool DoitArm::isVectorOrthogonal(stMat *t)
{
  //t->nx,t->ny, t->nz的平方和为1
  //t->n t->o t->a  n*o n*a o*a 内积均为0则正交
  if (fabs((t->nx) * (t->nx) + (t->ny) * (t->ny) + (t->nz) * (t->nz) - 1) > MINUS) return false;
  if (fabs((t->ox) * (t->ox) + (t->oy) * (t->oy) + (t->oz) * (t->oz) - 1) > MINUS) return false;
  if (fabs((t->ax) * (t->ax) + (t->ay) * (t->ay) + (t->az) * (t->az) - 1) > MINUS) return false;
  if (fabs((t->nx) * (t->ox) + (t->ny) * (t->oy) + (t->nz) * (t->oz)) > MINUS) return false;
  if (fabs((t->nx) * (t->ax) + (t->ny) * (t->ay) + (t->nz) * (t->az)) > MINUS) return false;
  if (fabs((t->ax) * (t->ox) + (t->ay) * (t->oy) + (t->az) * (t->oz)) > MINUS) return false;
  return true;
}
/*
  是否超过允许移动范围
*/
bool DoitArm::isUnderRange(stMat *t)
{
  //t->px, t->py, t->pz
  return true;
}
/*
  是否相等
*/
bool DoitArm::isEqual(float x1, float x2)
{
  if (fabs(x1 - x2) <= MINUS)
    return true;
  else
    return false;
}
/*
  根据当前角度计算弧度值
*/
void DoitArm::calcCurrentPosition(void)
{
  currentTheta.t1 = axisRg[0].current * 0.017453;//PI / 180.0;
  currentTheta.t2 = axisRg[1].current * 0.017453;
  currentTheta.t3 = axisRg[2].current * 0.017453;
  currentTheta.t4 = axisRg[3].current * 0.017453;
  currentTheta.t5 = axisRg[4].current * 0.017453;
  currentTheta.t6 = axisRg[5].current * 0.017453;
}
/*
  根据弧度值计算角度
*/
bool DoitArm::calcCurrentAngle(svTheta*v, float *angle, int num)
{
  if (num != 6) return false;
  angle[0] = v->t1 * 57.295780;//180.0 / PI
  angle[1] = v->t2 * 57.295780;
  angle[2] = v->t3 * 57.295780;
  angle[3] = v->t4 * 57.295780;
  angle[4] = v->t5 * 57.295780;
  angle[5] = v->t6 * 57.295780;
}
/*
  机器人逆向运动学计算 ReverseKinematics calculation
  intput : stMat *t
  output : svTheta *v
  输入：笛卡尔坐标系中表示的末端期望姿态
  输出：六轴旋转角度，若计算失败返回false，角度全为0
  当输入值超过移动范围、计算值超过允许角度时，计算异常时，返回false
*/
bool DoitArm::calcReverseKinematics(stMat *t, svTheta*v)
{
  int i, k;
  v->t1 = v->t2 = v->t3 = v->t4 = v->t5 = v->t6 = 0;
  //更新当前值
  calcCurrentPosition();
  //检查输入是否合法
  //判断t中旋转向量是否正交
  if (!isVectorOrthogonal(t))
  {
    print_debug("isVectorOrthogonal Failed!\r\n");
    return false;
  }
  //是否超出移动范围
  if (!isUnderRange(t))
  {
    print_debug("isUnderRange Failed!\r\n");
    return false;
  }

  float Px = t->px - t->ax * ARM_D6;
  float Py = t->py - t->ay * ARM_D6;
  float Pz = t->pz - t->az * ARM_D6;
  //theta1  只考虑前臂的情况
  float theta1 = atan2(Py, Px);
  char m[12];
  dtostrf(Py, 4, 6, m); print_debug("Py:%s ", m);
  dtostrf(Px, 4, 6, m); print_debug("Px:%s\r\n", m);

  //theta2 theta3共4组
  //float r = 1 / invSqrt(Px * Px + Py * Py);
  float r = sqrt(Px * Px + Py * Py);
  float D1 = r - ARM_A1;
  float D2 = Pz - ARM_D1;
  float mB1 = 2 * (D2 * ARM_D3 - D1 * ARM_A3);
  float B2 = 2 * (D1 * ARM_D3 + D2 * ARM_A3);
  float beta0 = atan2(B2, mB1);
  //float gamma0 = asin((ARM_A2 * ARM_A2 - D1 * D1 - D2 * D2 - ARM_A3 * ARM_A3 - ARM_D3 * ARM_D3) * invSqrt(mB1 * mB1 + B2 * B2) );
  float gamma0 = asin((ARM_A2 * ARM_A2 - D1 * D1 - D2 * D2 - ARM_A3 * ARM_A3 - ARM_D3 * ARM_D3) / sqrt(mB1 * mB1 + B2 * B2) );
  float alpha[4] = {0.0, 0.0, 0.0, 0.0};
  alpha[0] = beta0 + gamma0;    //4种可能解
  alpha[1] = PI + beta0 + gamma0;
  alpha[2] = PI + beta0 - gamma0;
  alpha[3] = beta0 - gamma0;
  float theta2[4] = {0.0, 0.0, 0.0, 0.0};
  float theta3[4] = {0.0, 0.0, 0.0, 0.0};

  /*char m[12];
    dtostrf(D1, 4, 3, m); print_debug("D1:%s ", m);
    dtostrf(D2, 4, 3, m); print_debug("D2:%s ", m);
    dtostrf(mB1, 4, 3, m); print_debug("mB1:%s ", m);
    dtostrf(B2, 4, 3, m); print_debug("B2:%s ", m);
    dtostrf(beta0, 4, 3, m); print_debug("beta0:%s ", m);
    dtostrf(gamma0, 4, 3, m); print_debug("gamma0:%s ", m);
    dtostrf(r, 4, 3, m); print_debug("r:%s\r\n", m);*/

  for (i = 0; i < 4; i++)
  {
    theta2[i] = atan2( D1 - ARM_A3 * sin(alpha[i]) - ARM_D3 * cos(alpha[i]), D2 - ARM_A3 * cos(alpha[i]) + ARM_D3 * sin(alpha[i]));
    theta3[i] = alpha[i] - theta2[i];
  }

  //求解theta6 theta4 theta5
  float theta4[4][2] = {0, 0, 0, 0, 0, 0, 0, 0};
  float theta5[4][2] = {0, 0, 0, 0, 0, 0, 0, 0};
  float theta6[4][2] = {0, 0, 0, 0, 0, 0, 0, 0};

  float st1, ct1, st2, ct2, st3, ct3, st4, ct4, st5, ct5, st6, ct6;
  float b11, b12, b13, b21, b22, b31, b32, b33;
  float temp1, temp2, c12_temp1, c12_temp2;
  float c12, c22, c31, c33;
  float b14, b34, c11, c13, c14, c21, c23, c24, c34;
  float f11, f13, f14, f21, f23, f24, f34;

  float minEnergy = 10000.0;
  float tempEnergy = 0.0;
  int outI = -1, outK = -1;
  st1 = sin(theta1); ct1 = cos(theta1);
  for (i = 0 ; i < 4; i++)
  {
    st2 = sin(theta2[i]); ct2 = cos(theta2[i]);
    st3 = sin(theta3[i]); ct3 = cos(theta3[i]);
    b11 = ct1 * st2 * ct3 + ct1 * ct2 * st3;
    b12 = st1 * st2 * ct3 + st1 * ct2 * st3;
    b13 = ct2 * ct3 - st2 * st3;
    b21 = st1;
    b22 = -ct1;
    b31 = -ct1 * st2 * st3 + ct1 * ct2 * ct3;
    b32 = -st1 * st2 * st3 + st1 * ct2 * ct3;
    b33 = -ct2 * st3 - st2 * ct3;
    temp1 = b31 * t->ox + b32 * t->oy + b33 * t->oz;
    temp2 = b31 * t->nx + b32 * t->ny + b33 * t->nz;
    if (fabs(temp2) <= MINUS && fabs(temp1) > MINUS)//if(temp2==0 && temp1 ~=0)
    {
      theta6[i][0] = PI / 2;
      theta6[i][1] = PI / 2;
    }
    else if (fabs(temp2) > MINUS && fabs(temp1) <= MINUS) //if(temp2 ~=0 && temp1==0)
    {
      theta6[i][0] = 0;
      theta6[i][1] = 0;
    }
    else if (fabs(temp2) <= MINUS && fabs(temp1) <= MINUS) //if(temp2 ==0 && temp1==0)
    {
      theta6[i][0] = currentTheta.t6;//等于当前值
      theta6[i][1] = currentTheta.t6;
    }
    else
    {
      theta6[i][0] = atan2( -temp1, temp2);
      theta6[i][1] = atan2( temp1, -temp2);
    }
    /*char m[12];
      dtostrf(temp1, 4, 6, m); print_debug("\r\n\t temp1:%s\r\n", m);
      dtostrf(temp2, 4, 6, m); print_debug("\t temp2:%s\r\n", m);*/

    for (k = 0; k < 2; k++)
    {
      st6 = sin(theta6[i][k]); ct6 = cos(theta6[i][k]);
      c12_temp1 = b11 * t->nx + b12 * t->ny + b13 * t->nz;
      c12_temp2 = b11 * t->ox + b12 * t->oy + b13 * t->oz;
      c12 = (c12_temp1) * st6 + (c12_temp2) * ct6; //c12 = (b11*nx+b12*ny+b13*nz)*st6+(b11*ox+b12*oy+b13*oz)*ct6;
      c22 = (b21 * t->nx + b22 * t->ny) * st6 + (b21 * t->ox + b22 * t->oy) * ct6;
      c31 = (b31 * t->nx + b32 * t->ny + b33 * t->nz) * ct6 - (b31 * t->ox + b32 * t->oy + b33 * t->oz) * st6;
      c33 = b31 * t->ax + b32 * t->ay + b33 * t->az;
      theta4[i][k] = atan2( -c12, c22);
      theta5[i][k] = atan2( c31, c33);

#ifdef DEBUG_DOIT_ARM
      char m[12];
      dtostrf(theta1, 4, 6, m); print_debug("Calc(%d,%d) (%s", i, k, m);
      dtostrf(theta2[i], 4, 6, m); print_debug(",%s", m);
      dtostrf(theta3[i], 4, 6, m); print_debug(",%s", m);
      dtostrf(theta4[i][k], 4, 6, m); print_debug(",%s", m);
      dtostrf(theta5[i][k], 4, 6, m); print_debug(",%s", m);
      dtostrf(theta6[i][k], 4, 6, m); print_debug(",%s)\r\n", m);
#endif

      st4 = sin(theta4[i][k]); ct4 = cos(theta4[i][k]);
      st5 = sin(theta5[i][k]); ct5 = cos(theta5[i][k]);

      b14 = -ARM_A1 * st2 * ct3 - ARM_A1 * ct2 * st3 - ARM_D1 * ct2 * ct3 + ARM_D1 * st2 * st3 - ARM_A2 * ct3 - ARM_A3;
      b34 = ARM_A1 * st2 * st3 - ARM_A1 * ct2 * ct3 + ARM_D1 * ct2 * st3 + ARM_D1 * st2 * ct3 + ARM_A2 * st3 - ARM_D3;
      c11 = (c12_temp1) * ct6 - (c12_temp2) * st6; //c11 = (b11*nx+b12*ny+b13*nz)*ct6 - (b11*ox+b12*oy+b13*oz)*st6;
      f11 = ct4 * ct5;

      /*dtostrf(b14, 4, 6, m); print_debug("\t b14:%s\r\n", m);
        dtostrf(b34, 4, 6, m); print_debug("\t b34:%s\r\n", m);
        dtostrf(c11, 4, 6, m); print_debug("\t c11:%s\r\n", m);
        dtostrf(f11, 4, 6, m); print_debug("\t f11:%s\r\n", m);*/
      if (!isEqual(c11, f11)) continue;

      c13 = b11 * t->ax + b12 * t->ay + b13 * t->az;
      f13 = -ct4 * st5;
      /*dtostrf(c13, 4, 6, m); print_debug("\t c13:%s\r\n", m);
        dtostrf(f13, 4, 6, m); print_debug("\t f13:%s\r\n", m);*/
      if (!isEqual(c13, f13)) continue;

      c14 = -(c13) * ARM_D6 + (b11 * t->px + b12 * t->py + b13 * t->pz) + b14; //c14 = -(b11*ax + b12*ay + b13*az)*d6 + (b11*px + b12*py + b13*pz) + b14;
      f14 = 0;
      /*dtostrf(b11, 4, 6, m); print_debug("\t b11:%s\r\n", m);
        dtostrf(b12, 4, 6, m); print_debug("\t b12:%s\r\n", m);
        dtostrf(b13, 4, 6, m); print_debug("\t b13:%s\r\n", m);
        dtostrf(b14, 4, 6, m); print_debug("\t b14:%s\r\n", m);
        dtostrf(c14, 4, 6, m); print_debug("\t c14:%s\r\n", m);
        dtostrf(f14, 4, 6, m); print_debug("\t f14:%s\r\n", m);*/
      if (!isEqual(c14, f14)) continue;

      c21 = (b21 * t->nx + b22 * t->ny) * ct6 - (b21 * t->ox + b22 * t->oy) * st6;
      f21 = st4 * ct5;

      /*dtostrf(c21, 4, 6, m); print_debug("\t c21:%s\r\n", m);
        dtostrf(f21, 4, 6, m); print_debug("\t f21:%s\r\n", m);*/
      if (!isEqual(c21, f21)) continue;

      c23 = b21 * t->ax + b22 * t->ay;
      f23 = -st4 * st5;
      /*dtostrf(c23, 4, 6, m); print_debug("\t c23:%s\r\n", m);
        dtostrf(f23, 4, 6, m); print_debug("\t f23:%s\r\n", m);*/
      if (!isEqual(c23, f23)) continue;

      c24 = -(c23) * ARM_D6 + (b21 * t->px + b22 * t->py); //c24 = -(b21*ax + b22*ay)*d6 + (b21*px + b22*py);
      f24 = 0;
      /*dtostrf(c24, 4, 6, m); print_debug("\t c24:%s\r\n", m);
        dtostrf(f24, 4, 6, m); print_debug("\t f24:%s\r\n", m);   */
      if (!isEqual(c24, f24)) continue;
      c34 = -( c33) * ARM_D6 + ( b31 * t->px + b32 * t->py + b33 * t->pz) + b34; //c34 = -( b31*ax + b32*ay + b33*az)*d6 + ( b31*px + b32*py + b33*pz) + b34;
      f34 = 0;
      /*dtostrf(c34, 4, 6, m); print_debug("\t c34:%s\r\n", m);
        dtostrf(f34, 4, 6, m); print_debug("\t f34:%s\r\n", m);  */
      if (!isEqual(c34, f34)) continue;

      //计算完成
      //是否超过移动范围
      //选取移动量最小的值返回
      tempEnergy = fabs(currentTheta.t1 - theta1) + fabs(currentTheta.t2 - theta2[i]) + fabs(currentTheta.t3 - theta3[i]) + fabs(currentTheta.t4 - theta4[i][k]);
      if (tempEnergy < minEnergy)
      {
        minEnergy = tempEnergy;
        outI = i;
        outK = k;
      }
#ifdef DEBUG_DOIT_ARM
      print_debug("Calc Success!(%d,%d),", i, k);
      dtostrf(tempEnergy, 4, 6, m); print_debug("tempEnergy:%s ", m);
      dtostrf(minEnergy, 4, 6, m); print_debug("minEnergy:%s\r\n", m);
#endif
    }//for(k=0;k<2;k++)
  }//end for(i = 0 ; i < 4; i++)
  //判断计算是否成功
  if (outI == -1)
  {
    print_debug("outI == -1");
    return false; //计算失败
  }

  v->t1 = theta1;
  v->t2 = theta2[outI];
  v->t3 = theta3[outI];
  v->t4 = theta4[outI][outK];
  v->t5 = theta5[outI][outK];
  v->t6 = theta6[outI][outK];
  print_debug("Calc Success!\r\nUse theta:(%d,%d)(%s", outI, outK, printFloat(v->t1));
  print_debug(",%s", printFloat(v->t2));
  print_debug(",%s", printFloat(v->t3));
  print_debug(",%s", printFloat(v->t4));
  print_debug(",%s", printFloat(v->t5));
  print_debug(",%s)\r\n", printFloat(v->t6));
  return true;
}

void DoitArm::printMatrix(_Matrix *A)
{
  int i = 0;
  int j = 0;
  char m[12];
  char temp[128];
  //运算
  for (i = 0; i < 4; i++)
  {
    temp[0] = 0x00;
    for (j = 0; j < 4; j++)
    {
      dtostrf(matrix_read(A, i, j), 4, 3, m);
      strcat(temp, "\t");
      strcat(temp, m);
    }
    print_debug("%s\r\n", temp);
  }
}
void DoitArm::printSTMat(stMat *t)
{
  char temp[256];
  char m[12];
  dtostrf(t->nx, 4, 3, m); sprintf(temp, "%s", m);
  dtostrf(t->ox, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(t->ax, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(t->px, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);

  dtostrf(t->ny, 4, 3, m); strcat(temp, "\r\n"); strcat(temp, m);
  dtostrf(t->oy, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(t->ay, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(t->py, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);

  dtostrf(t->nz, 4, 3, m); strcat(temp, "\r\n"); strcat(temp, m);
  dtostrf(t->oz, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(t->az, 4, 3, m); strcat(temp, "\t"); strcat(temp, m);
  dtostrf(t->pz, 4, 3, m); strcat(temp, "\t"); strcat(temp, m); strcat(temp, "\r\n0\t0\t0\t1\t\r\n");
  print_debug(temp);
}
/*
  获取当前工具坐标系在笛卡尔坐标系中的位姿
*/
void DoitArm::getToolCurrentPosture(stMat* t)
{
  struct _Matrix A1;
  struct _Matrix A2;
  struct _Matrix A;
  matrix_init(&A1);
  matrix_init(&A2);
  matrix_init(&A);
  calcCurrentPosition();
  //A1
  float st = sin(currentTheta.t1);
  float ct = cos(currentTheta.t1);
  matrix_write(&A1, 0, 0, ct); matrix_write(&A1, 0, 2, -st); matrix_write(&A1, 0, 3, ARM_A1 * ct);
  matrix_write(&A1, 1, 0, st); matrix_write(&A1, 1, 2, ct); matrix_write(&A1, 1, 3, ARM_A1 * st);
  matrix_write(&A1, 2, 1, -1); matrix_write(&A1, 2, 3, ARM_D1);
  matrix_write(&A1, 3, 3, 1);

  //print_debug("A1\r\n");
  //printMatrix(&A1);

  st = sin(currentTheta.t2);
  ct = cos(currentTheta.t2);
  matrix_write(&A2, 0, 0, st); matrix_write(&A2, 0, 1, ct); matrix_write(&A2, 0, 3, ARM_A2 * st);
  matrix_write(&A2, 1, 0, -ct); matrix_write(&A2, 1, 1, st); matrix_write(&A2, 1, 3, -ARM_A2 * ct);
  matrix_write(&A2, 2, 2, 1);
  matrix_write(&A2, 3, 3, 1);
  matrix_multiply(&A1, &A2, &A);//A=A1*A2

  //print_debug("A1*A2\r\n");
  //printMatrix(&A);

  st = sin(currentTheta.t3);
  ct = cos(currentTheta.t3);
  matrix_init(&A1);
  matrix_write(&A1, 0, 0, ct); matrix_write(&A1, 0, 2, -st); matrix_write(&A1, 0, 3, ARM_A3 * ct - ARM_D3 * st);
  matrix_write(&A1, 1, 0, st); matrix_write(&A1, 1, 2, ct); matrix_write(&A1, 1, 3, ARM_A3 * st + ARM_D3 * ct);
  matrix_write(&A1, 2, 1, -1);
  matrix_write(&A1, 3, 3, 1);
  matrix_multiply(&A, &A1, &A2);

  //print_debug("A1*A2*A3\r\n");
  //printMatrix(&A2);

  st = sin(currentTheta.t4);
  ct = cos(currentTheta.t4);
  matrix_init(&A1);
  matrix_write(&A1, 0, 0, ct); matrix_write(&A1, 0, 2, st);
  matrix_write(&A1, 1, 0, st); matrix_write(&A1, 1, 2, -ct);
  matrix_write(&A1, 2, 1, 1);
  matrix_write(&A1, 3, 3, 1);
  matrix_multiply(&A2, &A1, &A);

  //print_debug("A1*A2*A3*A4\r\n");
  //printMatrix(&A);

  st = sin(currentTheta.t5);
  ct = cos(currentTheta.t5);
  matrix_init(&A1);
  matrix_write(&A1, 0, 0, ct); matrix_write(&A1, 0, 2, -st);
  matrix_write(&A1, 1, 0, st); matrix_write(&A1, 1, 2, ct);
  matrix_write(&A1, 2, 1, -1);
  matrix_write(&A1, 3, 3, 1);
  matrix_multiply(&A, &A1, &A2);//A=A*A5

  //print_debug("A1*A2*A3*A4*A5\r\n");
  //printMatrix(&A2);

  st = sin(currentTheta.t6);
  ct = cos(currentTheta.t6);
  matrix_init(&A1);
  matrix_write(&A1, 0, 0, ct); matrix_write(&A1, 0, 1, -st);
  matrix_write(&A1, 1, 0, st); matrix_write(&A1, 1, 1, ct);
  matrix_write(&A1, 2, 2, 1); matrix_write(&A1, 2, 3, ARM_D6);
  matrix_write(&A1, 3, 3, 1);
  matrix_multiply(&A2, &A1, &A);//A=A*A6

  //print_debug("A1*A2*A3*A4*A5*A6\r\n");
  //printMatrix(&A);

  t->nx = matrix_read(&A, 0, 0);
  t->ny = matrix_read(&A, 1, 0);
  t->nz = matrix_read(&A, 2, 0);
  t->ox = matrix_read(&A, 0, 1);
  t->oy = matrix_read(&A, 1, 1);
  t->oz = matrix_read(&A, 2, 1);
  t->ax = matrix_read(&A, 0, 2);
  t->ay = matrix_read(&A, 1, 2);
  t->az = matrix_read(&A, 2, 2);
  t->px = matrix_read(&A, 0, 3);
  t->py = matrix_read(&A, 1, 3);
  t->pz = matrix_read(&A, 2, 3);

  matrix_free(&A1);
  matrix_free(&A2);
  matrix_free(&A);
}
/*
  末端沿着工具坐标系的方向，从当前位置移动到指定位置;
  增量移动
*/
bool DoitArm::movetoTool(float x, float y, float z)
{
  if (fabs(x) > 100 || fabs(y) > 100 || fabs(z) > 100)
  {
    print_debug("Input out of range!");
    return false;
  }

  stMat t;
  svTheta v;
  getToolCurrentPosture(&t);
  print_debug("Current posture\r\n");
  printSTMat(&t);
  //A=A1*A2
  struct _Matrix A1;
  struct _Matrix A2;
  struct _Matrix A;
  matrix_init(&A1);
  matrix_init(&A2);
  matrix_init(&A);
  matrix_write(&A1, 0, 0, t.nx); matrix_write(&A1, 0, 1, t.ox); matrix_write(&A1, 0, 2, t.ax); matrix_write(&A1, 0, 3, t.px);
  matrix_write(&A1, 1, 0, t.ny); matrix_write(&A1, 1, 1, t.oy); matrix_write(&A1, 1, 2, t.ay); matrix_write(&A1, 1, 3, t.py);
  matrix_write(&A1, 2, 0, t.nz); matrix_write(&A1, 2, 1, t.oz); matrix_write(&A1, 2, 2, t.az); matrix_write(&A1, 2, 3, t.pz);
  matrix_write(&A1, 3, 3, 1);
  matrix_write(&A2, 0, 3, x); matrix_write(&A2, 1, 3, y); matrix_write(&A2, 2, 3, z);
  matrix_write(&A2, 0, 0, 1); matrix_write(&A2, 1, 1, 1); matrix_write(&A2, 2, 2, 1); matrix_write(&A2, 3, 3, 1);
  matrix_multiply(&A1, &A2, &A);//A=A1*A2
  t.nx = matrix_read(&A, 0, 0); t.ny = matrix_read(&A, 1, 0); t.nz = matrix_read(&A, 2, 0);
  t.ox = matrix_read(&A, 0, 1); t.oy = matrix_read(&A, 1, 1); t.oz = matrix_read(&A, 2, 1);
  t.ax = matrix_read(&A, 0, 2); t.ay = matrix_read(&A, 1, 2); t.az = matrix_read(&A, 2, 2);
  t.px = matrix_read(&A, 0, 3); t.py = matrix_read(&A, 1, 3); t.pz = matrix_read(&A, 2, 3);
  matrix_free(&A1); matrix_free(&A2); matrix_free(&A);

  print_debug("Ready to posture\r\n");
  printSTMat(&t);
  if (calcReverseKinematics(&t, &v) == false)
  {
    print_debug("calcReverseKinematics failed!");
    return false;
  }
  float a[6] = {0, 0, 0, 0, 0, 0};
  calcCurrentAngle(&v, a, 6);
  print_debug("Target angle:(%s", printFloat(a[0]));
  print_debug(",%s", printFloat(a[1]));
  print_debug(",%s", printFloat(a[2]));
  print_debug(",%s", printFloat(a[3]));
  print_debug(",%s", printFloat(a[41]));
  print_debug(",%s)\r\n", printFloat(a[5]));
  if (setTargetPosition(a, 6))
  {
    print_debug("movetoTool calc ok!\r\n");
    beginMove();
    return true;
  }
  else
  {
    print_debug("movetoTool calc not ok!\r\n");
    return false;
  }
}
/*
  末端沿着工具坐标系的方向，从当前姿态移动到指定姿态(工具坐标系下的姿态)；
  绝对位姿移动
*/
bool DoitArm::moveTool(stMat *t)
{
  //将t转化为笛卡尔空间的姿态矩阵
  //calcReverseKinematics计算各轴目标角度
  //开始运动
  print_debug("moveTool2:%d\r\n", t->nx);
  return true;
}
/*
  沿着工具坐标系的z轴旋转指定角度（单位为弧度）
  若有错误，返回false
*/
bool DoitArm::rotateToolZ(float angle)
{
  return true;
}

void DoitArm::printPosture(void)
{
  stMat t;
  getToolCurrentPosture(&t);
  print_debug("Current posture\r\n");
  printSTMat(&t);
}
/*
  末端沿着笛卡尔坐标系的方向，从当前位置移动到指定位置
  增量移动
*/
bool DoitArm::movetoCart(float x, float y, float z)
{
  //获取当前姿态getToolCurrentPosture，确定目标姿态
  //calcReverseKinematics计算各轴目标角度
  //开始运动
  if (fabs(x) > 100 || fabs(y) > 100 || fabs(z) > 100)
  {
    print_debug("Input out of range!");
    return false;
  }
  stMat t;
  svTheta v;
  getToolCurrentPosture(&t);
  print_debug("Current posture\r\n");
  printSTMat(&t);

  t.px = t.px + x;
  t.py = t.py + y;
  t.pz = t.pz + z;
  print_debug("Ready to posture\r\n");
  printSTMat(&t);

  if (calcReverseKinematics(&t, &v) == false)
  {
    print_debug("calcReverseKinematics failed!");
    return false;
  }
  float a[6] = {0, 0, 0, 0, 0, 0};
  calcCurrentAngle(&v, a, 6);
  print_debug("Target angle:(%s", printFloat(a[0]));
  print_debug(",%s", printFloat(a[1]));
  print_debug(",%s", printFloat(a[2]));
  print_debug(",%s", printFloat(a[3]));
  print_debug(",%s", printFloat(a[41]));
  print_debug(",%s)\r\n", printFloat(a[5]));
  if (setTargetPosition(a, 6))
  {
    print_debug("movetoCart calc ok!\r\n");
    beginMove();
    return true;
  }
  else
  {
    print_debug("movetoCart calc not ok!\r\n");
    return false;
  }
}
/*
  沿着笛卡尔坐标系的方向，从当前姿态移动到指定姿态；
  绝对位姿移动
*/
bool DoitArm::moveCart(stMat* t)
{
  //calcReverseKinematics计算各轴目标角度
  //开始运动
  return true;
}
/*
  设置各轴目标角度
*/
bool DoitArm::setTargetPosition(float * v, int num)
{
  if (num != 6)
  {
    print_debug("Input arrary is wrong!");
    return false;
  }
  int i = 0;
  for (i = 0; i < 6; i++)
  {
    if (axisRg[i].minus > v[i] || axisRg[i].maxium < v[i])
    {
      print_debug("Axis:%d, angle:%s outrange!\r\n", i + 1, printFloat(v[i]));
      return false;
    }
  }
  for (i = 0; i < 6; i++)
  {
    axisRg[i].target = v[i];
  }
  return true;
}
/*
  设置当前各轴角度-慎用
*/
bool DoitArm::setCurrentPosition(float * v, int num)
{
  if (num != 6)
  {
    print_debug("Input arrary is wrong!");
    return false;
  }
  int i = 0;
  for (i = 0; i < 6; i++)
  {
    if (axisRg[i].minus > v[i] || axisRg[i].maxium < v[i])
    {
      print_debug("Axis:%d, angle:%s outrange!\r\n", i + 1, printFloat(v[i]));
      return false;
    }
  }
  for (i = 0; i < 6; i++)
  {
    axisRg[i].current = v[i];
  }
  return true;
}
/*
  设置指定轴角度（非弧度）
*/
bool DoitArm::setTargetPosition(int axisNum, float v)
{
  if (axisNum > AXIS_6 || axisNum < AXIS_1 ) return false;
  if (axisRg[axisNum - 1].minus > v || axisRg[axisNum - 1].maxium < v)
  {
    print_debug("Axis:%d, angle:%s outrange!\r\n", axisNum, printFloat(v));
    return false;
  }
  axisRg[axisNum - 1].target = v;
  return true;
}
/*
  返回当前各轴目标角度(非弧度值）
*/
bool DoitArm::getTargetPosition(float * v, int num)
{
  if (num != 6)
  {
    print_debug("Input arrary is wrong!");
    return false;
  }
  for (int i = 0; i < 6; i++)
    v[i] = axisRg[i].target;
  return true;
}
/*
  返回指定轴目标角度（非弧度）
*/
bool DoitArm::getTargetPosition(int axisNum, float * p)
{
  if (axisNum > AXIS_6 || axisNum < AXIS_1 ) return false;
  *p = axisRg[axisNum - 1].target;
  return true;
}
/*
  返回当前各轴角度(非弧度值）
*/
bool DoitArm::getCurrentPosition(float * v, int num)
{
  if (num != 6)
  {
    print_debug("Input arrary is wrong!");
    return false;
  }
  for (int i = 0; i < 6; i++)
    v[i] = axisRg[i].current;
  return true;
}
/*
  返回指定轴角度（非弧度）
*/
bool DoitArm::getCurrentPosition(int num, float * v)
{
  if (num > AXIS_6 || num < AXIS_1 ) return false;
  *v = axisRg[num - 1].current;
  return true;
}
void DoitArm::ServoDriverBegin(uint8_t addr)
{
#ifdef SIMULATE_DOIT_ARM
  return;
#endif
  _i2caddr = addr;
  WIRE.begin();
  ServoDriverReset();
}

void DoitArm::ServoDriverReset(void)
{
  write8(PCA9685_MODE1, 0x0);
}

void DoitArm::setPWMFreq(float freq)
{
  freq *= 0.9;
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;

  uint8_t prescale = floor(prescaleval + 0.5);

  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & 0x7F) | 0x10;
  write8(PCA9685_MODE1, newmode);
  write8(PCA9685_PRESCALE, prescale);
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);
}

void DoitArm::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
#ifdef SIMULATE_DOIT_ARM
  return;
#endif
  num = channel[num];
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(LED0_ON_L + 4 * num);
  WIRE.write(on);
  WIRE.write(on >> 8);
  WIRE.write(off);
  WIRE.write(off >> 8);
  WIRE.endTransmission();
}
bool DoitArm::setChannel(uint8_t axis1, uint8_t axis2, uint8_t axis3, uint8_t axis4, uint8_t axis5, uint8_t axis6)
{
  if (axis1 > 15 || axis2 > 15 || axis3 > 15 || axis4 > 15 || axis5 > 15 || axis6 > 15) return false;

  channel[0] = axis1;
  channel[1] = axis2;
  channel[2] = axis3;
  channel[3] = axis4;
  channel[4] = axis5;
  channel[5] = axis6;
  return true;
}
uint8_t DoitArm::read8(uint8_t addr)
{
#ifdef SIMULATE_DOIT_ARM
  return 0;
#endif
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.endTransmission();

  WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return WIRE.read();
}

void DoitArm::write8(uint8_t addr, uint8_t d)
{
#ifdef SIMULATE_DOIT_ARM
  return;
#endif
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();
}

/*
  是否可运动
*/
bool DoitArm::isMovebable(void)
{
  if (!bServoOn) return false;
  return true;
}
/*
  从当前各轴位置移动到指定位置
  指定移动时间ms
*/
bool DoitArm::beginMove(void)
{
  if (!isMovebable()) return false;
  armStatus = MOVING;
  return true;
}

/*
  设置当前速度
*/
bool DoitArm::setCurrentSpeed(uint8_t spd)
{
  if (spd < 1 || spd > 10) return false;
  currentSpeed = spd;
  return true;
}

/*
  获取当前速度
*/
uint8_t DoitArm::getCurrentSpeed(void)
{
  return currentSpeed;
}

/*
  单轴转动, angle:角度值，非弧度值
*/
bool DoitArm::moveAxis(int num, float angle)
{
  if (!isMovebable()) return false;
  if (num > AXIS_6 || num < AXIS_1 ) return false;
  num = num - 1;
  if (axisRg[num].minus > angle || axisRg[num].maxium < angle)
  {
    print_debug("Axis:%d, angle:%s outrange!\r\n", num + 1, printFloat(angle));
    return false;
  }
  float angleTemp = angle + 90 + axisRg[num].bias;//转化为实际舵机角度
  if (num == 1) angleTemp = -1 * angle + 90 + axisRg[num].bias;; //第二轴反了
  int pulselen = (int)(angleTemp * (SERVO_LIMIT_MAX - SERVO_LIMIT_MIN) / 180.0 + SERVO_LIMIT_MIN);

  /*char strFloat[32];
    dtostrf(angleTemp, 4, 3, strFloat);
    print_debug("angle:%s\tPulse:%d\r\n", strFloat, pulselen);*/

  setPWM(num, 0, pulselen);
  armStatus = MOVING;
  axisRg[num].current = angle;

  /*dtostrf(axisRg[num].current, 4, 3, strFloat);
    print_debug("axisRg[%d].current:%s\r\n", num+1,strFloat);*/

  return true;
}
/*
  设置舵机中位偏移
*/
bool DoitArm::setServoBias(float *a, int num)
{
  if (num != 6)
  {
    print_debug("Input arrary is wrong!");
    return false;
  }
  int i = 0;
  for (i = 0; i < 6; i++)
  {
    if (axisRg[i].minus > a[i] || axisRg[i].maxium < a[i])
    {
      print_debug("Axis:%d, angle:%s outrange!\r\n", i + 1, printFloat(a[i]));
      return false;
    }
  }
  for (i = 0; i < 6; i++)
  {
    axisRg[i].bias = a[i];
  }

  return true;
}
/*
  设置舵机移动范围
*/
bool DoitArm::setServoRange(float a1_min, float a1_max, float a2_min, float a2_max, float a3_min, float a3_max, float a4_min, float a4_max, float a5_min, float a5_max, float a6_min, float a6_max)
{
  float a_min[6], a_max[6];
  a_min[0] = a1_min; a_max[0] = a1_max;
  a_min[1] = a2_min; a_max[1] = a2_max;
  a_min[2] = a3_min; a_max[2] = a3_max;
  a_min[3] = a4_min; a_max[3] = a4_max;
  a_min[4] = a5_min; a_max[4] = a5_max;
  a_min[5] = a6_min; a_max[5] = a6_max;
  int i = 0;
  for (i = 0; i < 6; i++)
  {
    if (a_min[i] >= a_max[i])
    {
      print_debug("Input arrary is wrong!");
      return false;
    }
  }
  for (i = 0; i < 6; i++)
  {
    axisRg[i].minus = a_min[i];
    axisRg[i].maxium = a_max[i];
  }

  return true;
}
/*
  设置初始位置
*/
bool DoitArm::setWaitPosition(float *a, int num)
{
  if (num != 6)
  {
    print_debug("Input arrary is wrong!\r\n");
    return false;
  }
  int i = 0;
  for (i = 0; i < 6; i++)
  {
    if (axisRg[i].minus > a[i] || axisRg[i].maxium < a[i])
    {
      print_debug("Axis:%d, angle:%s outrange!\r\n", i + 1, printFloat(a[i]));
      return false;
    }
  }
  for (i = 0; i < 6; i++)
  {
    axisRg[i].wait = a[i];
  }

  return true;
}
/*
  运动到零点
*/
void DoitArm::gotoZeroPosition(void)
{
  for (int i = 0; i < 6; i++)
  {
    float angleTemp = 0 + 90 + axisRg[i].bias;
    int pulselen = (int) (angleTemp * (SERVO_LIMIT_MAX - SERVO_LIMIT_MIN) / 180 + SERVO_LIMIT_MIN);
    print_debug("angle:%s\tPulse:%d\r\n", printFloat(angleTemp), pulselen);
    setPWM(i, 0, pulselen);
    axisRg[i].current = 0;
    delay(100);
  }
}
/*
  运动到初始位置
*/
bool DoitArm::gotoWaitPosition(void)
{
  if (!isMovebable()) return false;
  int angle = 0;
  int pulselen = 0;
  for (int i = 0; i < 6; i++)
  {
    //moveAxis(i + 1, axisRg[i].wait);
    axisRg[i].target = axisRg[i].wait;
  }
  beginMove();
  return true;
}
/*
  获取初始位置
*/
bool DoitArm::getWaitPosition(float* v, int num)
{
  if (num != 6)
  {
    print_debug("Input arrary is wrong!");
    return false;
  }
  for (int i = 0; i < 6; i++)
    v[i] = axisRg[i].wait;
  return true;
}
/*
  伺服开
*/
bool DoitArm::attachServo(void)
{
  ServoDriverBegin(0x40);
  setPWMFreq(50);

  bServoOn = true;
  armStatus = STAND_BY;
  return true;
}
/*
  伺服关
*/
bool DoitArm::detachServo(void)
{
  bServoOn = false;
  armStatus = SERVO_OFF;
  return true;
}
/*
  急停
*/
void DoitArm::emergencyStop(void)
{
  ServoDriverReset();
  detachServo();
}
/*
  获取当前状态
*/
uint8_t DoitArm::getArmStatus(void)
{
  return armStatus;
}
/*
   回调
*/
void DoitArm::registMoveFinishCB(moveFinish_cb_fun p)
{
  //(*)p(1)-带参数
  pMoveFinish_cb_fun = p;
}

static char strFloat[32];
char* DoitArm::printFloat(float a)
{
  dtostrf(a, 4, 3, strFloat);
  return strFloat;
}

/*
  运动控制tick
*/
void DoitArm::motionControlTick(void)
{
  static unsigned long lastTick = 0;

  if (armStatus != MOVING) return;

  if (millis() - lastTick < 5 * (10 - currentSpeed)) return;
  lastTick = millis();

  float temp = 0;
  const int step = 1;
  bool flag_moving = false;
  for (int i = 0; i < 6; i++)
  {
    if (axisRg[i].current == axisRg[i].target) continue;
    flag_moving = true;
    if (axisRg[i].target > axisRg[i].current)
    {
      temp = axisRg[i].current + step * currentSpeed;
      //temp = axisRg[i].current + step;
      if (temp > axisRg[i].target) temp = axisRg[i].target;
    }
    else
    {
      temp = axisRg[i].current - step * currentSpeed;
      //temp = axisRg[i].current - step;
      if (temp < axisRg[i].target) temp = axisRg[i].target;
    }
    if (temp > axisRg[i].maxium) {
      temp = axisRg[i].maxium;
      axisRg[i].target = temp;
    }
    else if (temp < axisRg[i].minus) {
      temp = axisRg[i].minus;
      axisRg[i].target = temp;
    }
    if (moveAxis(i + 1, temp) == false) print_debug("Not movable!");
    //print_debug("%d: moving to %s\r\n", i + 1, printFloat(temp));
  }
  if (flag_moving == false)
  {
    print_debug("Reach target!\r\n");
    armStatus = STAND_BY;
    if (pMoveFinish_cb_fun)
      (pMoveFinish_cb_fun)();
  }
}
/*
  主循环中调用
*/
void DoitArm::handleServo(void)
{
  motionControlTick();
}
