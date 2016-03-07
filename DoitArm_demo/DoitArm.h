/******************************************************************************
  File Name          : DoitArm.h
  Author             : Doit.am
  Updated            : Doit.am
  Version            : V0.0.1
  Created Date       : 2016-01-17
  Modified Date      : 2016-01-18
  Description        :
  License            :
  Copyright(C) 2016 Doctors of Intelligence & Technolog Company.
  All right reserved.
*******************************************************************************/

#ifndef DOIT_ARM_H
#define DOIT_ARM_H
//Homogeneous coordinates matrix

#include <ESP8266WiFi.h>

#define ARM_A1 45
#define ARM_D1 75
#define ARM_A2 115
#define ARM_A3 20
#define ARM_D3 130
#define ARM_D6 50

class DoitArm
{
  public:
    struct stMat {
      public: float nx; float ny; float nz; float ox; float oy; float oz; float ax; float ay; float az; float px; float py; float pz;
        stMat(void) {
          nx = 1;
          ny = 0;
          nz = 0;
          ox = 0;
          oy = 1;
          oz = 0;
          ax = 0;
          ay = 0;
          az = 1;
          px = 0;
          py = 0;
          pz = 0;
        }
        stMat(float x, float y, float z) {
          nx = 1;
          ny = 0;
          nz = 0;
          ox = 0;
          oy = 1;
          oz = 0;
          ax = 0;
          ay = 0;
          az = 1;
          px = x;
          py = y;
          pz = z;
        }
    };
    struct svTheta {
      public: float t1; float t2; float t3; float t4; float t5; float t6;
        svTheta(void) {
          t1 = t2 = t3 = t4 = t5 = t6 = 0;
        }
    };

    enum AXIS_TYPE {AXIS_X = 0, AXIS_Y, AXIS_Z,};
    enum AXIS_NUM {AXIS_1 = 1, AXIS_2, AXIS_3, AXIS_4, AXIS_5, AXIS_6,};
    enum STATUS { SERVO_OFF = 0, STAND_BY, MOVING,};
  private://矩阵相关
    struct _Matrix {
      float *arr;
    };
    //当前轴 最小值、中点偏差、最大值、i2c编号、当前位置
    struct _sAxisRange {
      float minus;
      float bias;
      float maxium;
      int i2c_addr;
      float current;
      float target;
      float wait;
    } axisRg[6];
    bool bServoOn;//伺服on/off
    uint8_t armStatus;//系统状态
    uint8_t currentSpeed;//当前速度 1~10

    void matrix_init(_Matrix *m);
    void matrix_free(_Matrix *m);
    float matrix_read(_Matrix *m, int i, int j);
    int matrix_write(_Matrix *m, int i, int j, float val);
    int matrix_multiply(_Matrix *A, _Matrix *B, _Matrix *C);
    void printMatrix(_Matrix *A);
    void printSTMat(stMat*t);
    char* printFloat(float a);
  public:
    svTheta currentTheta;//当前各轴角度-弧度值
    svTheta targetTheta;//各轴目标角度-弧度值
    DoitArm();
    //运动学相关
    bool calcReverseKinematics(stMat* t, svTheta* v);//根据笛卡尔坐标系下的位姿和当前位姿计算各轴拟旋转角度
    void getToolCurrentPosture(stMat* t);//获取当前工具坐标系在笛卡尔坐标系中的位姿
    bool movetoTool(float, float, float);//末端沿着工具坐标系的方向，从当前位置移动到指定位置;增量移动
    bool moveTool(stMat*);//末端沿着工具坐标系的方向，从当前姿态移动到指定姿态(工具坐标系下的姿态)；绝对位姿移动
    bool rotateToolZ(float angle);//沿着工具坐标系的z轴旋转指定角度（单位为弧度）

    bool movetoCart(float, float, float);//末端沿着笛卡尔坐标系的方向，从当前位置移动到指定位置;增量移动，最多100mm
    bool moveCart(stMat*);//沿着笛卡尔坐标系的方向，从当前姿态移动到指定姿态；绝对位姿移动
    //手臂各轴相关
    bool setTargetPosition(float*p, int num);//设置当前各轴角度（非弧度）
    bool setTargetPosition(int axisNum, float p);//设置指定轴角度(非弧度)
    bool getTargetPosition(float*p,int num);//获取各轴目标位置
    bool getTargetPosition(int axisNum, float*p);//获取指定轴的目标位置
    bool setCurrentPosition(float*p, int num);//设置当前各轴角度（非弧度） 慎用
    bool getCurrentPosition(float*p,int num);//获取当前各轴角度（非弧度）
    bool getCurrentPosition(int axisNum, float *p);//获取指定轴角度(非弧度)
    bool setCurrentSpeed(uint8_t spd);//设置运行速度 1~10 1最小，10最大
    uint8_t getCurrentSpeed(void);//获取运行速度
    uint8_t getArmStatus(void);//获取当前状态
    void printPosture(void);//打印当前姿态
    //运动控制
    bool attachServo();//伺服开
    bool detachServo();//伺服关
    bool beginMove(); //六轴联动 以setCurrentSpeed设置的速度移动到各轴目标
    void handleServo(void);//在主循环中调用
    void emergencyStop(void);//立刻停止运动
    //初始位置(等待位置)
    void gotoZeroPosition(void);//初始化时，在setServoBias之后，必须调用
    bool setWaitPosition(float *a, int num); //设置各轴初始位置
    bool getWaitPosition(float*p,int num);//获取各轴等待位置的值
    bool gotoWaitPosition(void);//运动到初始位置
    bool setChannel(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t); //设置六轴对应的通道号
    bool setServoBias(float *a, int num);//设置舵机的中位偏移
    bool setServoRange(float a1_min, float a1_max,float a2_min, float a2_max,float a3_min, float a3_max,float a4_min, float a4_max,float a5_min, float a5_max,float a6_min, float a6_max);//设置舵机移动范围
    //回调函数
    typedef void (*moveFinish_cb_fun)(void);
    void registMoveFinishCB(moveFinish_cb_fun);//移动结束回调函数
  private:
    bool isVectorOrthogonal(stMat*);
    bool isUnderRange(stMat*);
    bool isEqual(float, float);
    void calcCurrentPosition(void);
    bool calcCurrentAngle(svTheta*v, float *angle,int num);//根据弧度值计算角度
    bool isMovebable(void);
    //驱动相关
    uint8_t _i2caddr;
    uint8_t channel[6];
    void write8(uint8_t, uint8_t);
    uint8_t read8(uint8_t);
    void ServoDriverBegin(uint8_t);
    void ServoDriverReset(void);
    void setPWMFreq(float);
    void setPWM(uint8_t, uint16_t, uint16_t);
    bool moveAxis(int num, float angle); //单轴转动 指定角度(非弧度)
    void motionControlTick(void);//运动控制tick函数
    moveFinish_cb_fun pMoveFinish_cb_fun;//回调函数
};

#endif /* DOIT_ARM_H */
