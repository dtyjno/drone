#ifndef AUTO_TUNE
#define AUTO_TUNE
#include <cstdint>


/*-----------------------------------------------------------------------------
                          共享宏定义
-----------------------------------------------------------------------------*/
#define AUTO_TUNE_OBJ_NUM       4   /*定义同时需要自整定对象资源的最大个数*/

typedef  int32_t   TUNE_ID_t;       /*自整定类型定义，正常Id为>=0,若小于零，则返回的Id错误*/

/*-----------------------------------------------------------------------------
                          数据类型定义
-----------------------------------------------------------------------------*/
typedef enum                 /*PID控制器类型*/
{  
  CONTROLER_TYPE_PI,       /*PI控制器*/
  CONTROLER_TYPE_PID ,     /*PID控制器*/
}TUNE_CONTROLER_TYPE_t;

typedef enum                 /*PID状态*/
{
  TUNE_INIT = 0,          /*PID自整定初始化中*/
  TUNE_START_POINT,       /*寻找起始点*/
  TUNE_RUNNING,           /*PID自整定中*/
  TUNE_FAIL,              /*整定失败*/
  TUNE_SUCESS,            /*整定成功*/
  
}TUNE_STAT_t;

typedef enum                /*驱动器作用*/
{
  POSITIVE_ACTION,        /*设定值大于测量值时，执行单元执行高输出*/
  NEGATIVE_NATION,        /*设定值大于测量值时，执行单元执行低输出*/
}DRIVER_ACTION_TYPE_t;

typedef struct TUNE_CFG_PARAM_tag
{
  TUNE_CONTROLER_TYPE_t cTrlType;         /*控制器类型，默认PD控制器*/
  DRIVER_ACTION_TYPE_t  acterType;        /*驱动器作用类型，默认正向作用*/
  float maxOutputStep;                    /*最大输出阶跃值，默认值为50*/
  float minOutputStep;                    /*最小输出阶跃值，默认值为0*/
  uint32_t hysteresisNum;                 /*反馈值在设定值处的迟滞相应个数，默认为5*/
  float setpoint;                         /*整定设定值,默认值为为50*/
  float ampStdDeviation;                 /*幅值标准差预期值，用来计算自整定波形是否稳定*/
  float cycleStdDeviation;                /*周期标准差预期值，用来计算自整定波形是否稳定*/
}TUNE_CFG_PARAM_t, *pTUNE_CFG_PARAM_t;      /*pid自整定对象配置参数*/

typedef struct TUNE_OGJ_tag *pTUNE_OBJ_t;   /*PID自整定参数*/
/*-----------------------------------------------------------------------------------
函数原型:  void TUNE_Init(void)
功    能:  初始化自整定相关参数,使用默认的TUNE_CFG_PARAM_t参数初始化自整定参数
          default cTrlType = CONTROLER_TYPE_PI,
          default outputStep = 50,
          default hysteresisNum = 5
输入参数:	NA
输出参数:	NA
返 回 值:	true:pram is protected can't be modifid; false: writable
-----------------------------------------------------------------------------------*/
void TUNE_Init(void);

/*-----------------------------------------------------------------------------------
函数原型:  TUNE_ID_t TUNE_New(pTUNE_CFG_PARAM_t pParam)
功    能:  新建一个PID自整定对象
输入参数:	pParam：自整定对象配置参数
输出参数:	NA
返 回 值:	<0，则新建自整定对象失败，可能对象资源已经用完，需要通过更改AUTO_TUNE_OBJ_NUM宏定义
          增加自整定对象资源，>=0,则为分配的自整定id，后续函数调用均通过此Id
-----------------------------------------------------------------------------------*/
TUNE_ID_t TUNE_New(pTUNE_CFG_PARAM_t pParam);

/*-----------------------------------------------------------------------------------
函数原型:  TUNE_ID_t TUNE_New(pTUNE_CFG_PARAM_t pParam)
功    能:  释放ID所示自整定对象资源
输入参数:	id:自整定ID
输出参数:	NA
返 回 值:	false:资源释放失败，true:资源释放成功
注意事项：  只有在tuneStat为TUNE_FAIL或者TUNE_SUCESS状态下，才允许释放资源
-----------------------------------------------------------------------------------*/
bool TUNE_Release(TUNE_ID_t id);
/*-----------------------------------------------------------------------------------
函数原型:  bool TUNE_Work(TUNE_ID_t id, float feedbackVal, float*outputVal)
功    能:  自整定任务
输入参数:	id:自整定ID
          delayMsec:调用的时间间隔
输出参数:	outputVal:输出值
返 回 值:	true:自整定完成，false:正在自整定中
注意事项：  该函数需要以固定的时间间隔调用
-----------------------------------------------------------------------------------*/
TUNE_STAT_t TUNE_Work(TUNE_ID_t id, float feedbackVal, float*outputVal, uint32_t delayMsec);
/*-----------------------------------------------------------------------------------
函数原型:  bool TUNE_SetActerType(TUNE_ID_t id, float maxStep,DRIVER_ACTION_TYPE_t type)
功    能:  设置驱动器类型
输入参数:	id:自整定ID
          type:驱动器类型
输出参数:	NA
返 回 值:	true:设置成功，false:设置失败
-----------------------------------------------------------------------------------*/
bool TUNE_SetActerType(TUNE_ID_t id, DRIVER_ACTION_TYPE_t type);
/*-----------------------------------------------------------------------------------
函数原型:  bool TUNE_Setsetpoint(TUNE_ID_t id, float setpoint)
功    能:  设置自整定设定值
输入参数:	id:自整定ID
          setpoint:自整定设置值
          
输出参数:	NA
返 回 值:	true:设置成功，false:设置失败
-----------------------------------------------------------------------------------*/
bool TUNE_Setsetpoint(TUNE_ID_t id, float setpoint);

/*-----------------------------------------------------------------------------------
函数原型:   bool TUNE_SetOutStep(TUNE_ID_t id, float maxStep,float minStep)
功    能:  设置输出阶跃值
输入参数:	id:自整定ID
          maxStep:最大输出阶跃值
          minStep:最大输出阶跃值
输出参数:	NA
返 回 值:	true:设置成功，false:设置失败
------------------------------------------------------------------------------------*/
bool TUNE_SetOutStep(TUNE_ID_t id, float maxStep,float minStep);

/*-----------------------------------------------------------------------------------
函数原型:   bool TUNE_SetCtrlType(TUNE_ID_t id, TUNE_CONTROLER_TYPE_t type)
功    能:  设置控制器类型
输入参数:	id:自整定ID
          type:
              CONTROLER_TYPE_PI，PI控制器,积分项不使用
              CONTROLER_TYPE_PID，PID控制器
输出参数:	NA
返 回 值:	true:设置成功，false:设置失败
-----------------------------------------------------------------------------------*/
bool TUNE_SetCtrlType(TUNE_ID_t id, TUNE_CONTROLER_TYPE_t type);

/*-----------------------------------------------------------------------------------
函数原型:   float TUNE_GetKp(TUNE_ID_t id, float *pfactorP)
功    能:   获取整定后的P参数
输入参数:	id:自整定ID
输出参数:	pfactorP:整定后的P参数
返 回 值:	true:获取成功，否则失败
-----------------------------------------------------------------------------------*/
float TUNE_GetKp(TUNE_ID_t id, float *pfactorP);

/*-----------------------------------------------------------------------------------
函数原型:   float TUNE_GetKp(TUNE_ID_t id)
功    能:   获取整定后的I参数
输入参数:	id:自整定ID
输出参数:	pfactorI:整定后的I参数
返 回 值:	true:获取成功，否则失败
-----------------------------------------------------------------------------------*/
float TUNE_GetKi(TUNE_ID_t id,float *pfactorI);

/*-----------------------------------------------------------------------------------
函数原型:   float TUNE_GetKp(TUNE_ID_t id)
功    能:   获取整定后的D参数
输入参数:	id:自整定ID
输出参数:	pfactorD:整定后的D参数
返 回 值:	true:获取成功，否则失败
---------------------------------------------------------------------------------------*/
float TUNE_GetKd(TUNE_ID_t id,float *pfactorD);

/*-----------------------------------------------------------------------------------
函数原型:   float TUNE_GetKp(TUNE_ID_t id)
功    能:  获取整定后的PID参数
输入参数:	id:自整定ID
输出参数:	NA
返 回 值:	true:获取成功，否则失败
-----------------------------------------------------------------------------------*/
bool TUNE_GedPID(TUNE_ID_t id, float*paramP, float*paramI, float*paramD);
/*-----------------------------------------------------------------------------------
函数原型:   float TUNE_GetStat(TUNE_ID_t id, TUNE_STAT_t *pStat)
功    能:  获取PID自整定状态
输入参数:	id:自整定ID
输出参数:	stat，自整定状态
返 回 值:	true:获取成功，否则失败
-----------------------------------------------------------------------------------*/
bool TUNE_GetStat(TUNE_ID_t id, TUNE_STAT_t *pStat);

#endif/* __TUNE__H*/