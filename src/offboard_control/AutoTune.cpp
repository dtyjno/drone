#include "AutoTune.h"
#include <cmath>
// https://blog.csdn.net/yingzhefengyuzou/article/details/138544439

/*-----------------------------------------------------------------------------
                            自有宏定义
-----------------------------------------------------------------------------*/
#define LAST_PEAK_NUM 20     /*存储的最新峰个数*/
#define MAX_CYCLE 200       /*自整定最大震荡周期数，震荡周期数超过此值，则整定失败*/
#define MAX_TIME_MS 3600000 /*自整定最大震荡毫秒数，震荡时间数超过此值，则整定失败*/
/*-----------------------------------------------------------------------------
                            数据类型定义
-----------------------------------------------------------------------------*/
typedef enum
{
    FEEDBACK_BELOW_INPUT = 0, /*反馈值小于于设定值*/
    FEEDBACK_ABOVE_INPUT,     /*反馈值大于设定值*/
} TUNE_OFFSET_STAT_t;         /*偏差状态*/

typedef struct
{
    float feedabackVal; /*反馈值*/
    uint32_t milSecond; /*反馈值对应的相对时间*/
} PEAK_VAL_t;           /*峰值对应的反馈值和对应的时间*/

typedef struct /*一个周期的峰值*/
{
    PEAK_VAL_t maxPeak; /*一个周期内的最大值*/
    PEAK_VAL_t minPeak; /*一个周期内的最小值*/
} PEAK_VAL_IN_PERIOD_t;

typedef struct TUNE_OGJ_tag /*PID自整定参数*/
{
    TUNE_STAT_t tuneStat;           /*整定状态位*/
    TUNE_CONTROLER_TYPE_t cTrlType; /*控制器类型，PI或者PID*/
    DRIVER_ACTION_TYPE_t acterType; /*驱动器作用类型*/
    TUNE_ID_t tuneId;               /*自整定ID号，用于内部访问相应自整定对象使用*/
    TUNE_OFFSET_STAT_t offetStat;   /*反馈值相对于设定值的偏移状态*/
    float feedbackVal;              /*pid反馈值*/
    float outputVal;                /*pid输出值*/
    float setpoint;                 /*pid设定值*/
    float maxOutputStep;            /*最大输出阶跃值，默认值为1*/
    float minOutputStep;            /*最小输出阶跃值，默认值为0*/

    uint32_t tuneCounter;                            /*整定计时器*/
    uint32_t cycleCounter;                           /*周期计数器*/
    uint32_t hysteresisNum;                          /*反馈值在设定值处的迟滞相应个数*/
    uint32_t riseHysteresisCounter;                  /*上升迟滞计数器*/
    uint32_t fallHysteresisCounter;                  /*下降迟滞计数器*/
    uint32_t fullCycleFlag;                          /*运行一个完整周期的标志*/
    PEAK_VAL_IN_PERIOD_t lastPeakVal[LAST_PEAK_NUM]; /*存储找到的最新的峰值*/
    int32_t peakWriPos;                              /*下一个需要写峰的位置*/
    float Ku;                                        /*整定结果幅值*/
    float Tu;                                        /*整定结果周期*/
    float Kp;                                        /*整定结果比例值*/
    float Ki;                                        /*整定结果积分值*/
    float Kd;                                        /*整定结果微分值*/

    float ampStdDeviation;   /*幅值标准差预期值，用来计算自整定波形是否稳定*/
    float cycleStdDeviation; /*周期标准差预期值，用来计算自整定波形是否稳定*/
    float CurAmpStdDeviation;
    float CurCycleStdDeviation;
} TUNE_OBJ_t, *pTUNE_OBJ_t;

/*-----------------------------------------------------------------------------
                            数据结构定义
-----------------------------------------------------------------------------*/
static TUNE_OBJ_t s_tuneObject[AUTO_TUNE_OBJ_NUM]; /*PID自整定对象*/

/*-----------------------------------------------------------------------------
                            内部函数声明
-----------------------------------------------------------------------------*/
static bool TUNE_StructInitToDefaultVal(TUNE_ID_t id);
static bool TUNE_FsmReset(TUNE_ID_t id);
static float TUNE_CalStdDeviation(float *fdata, uint32_t len);
static void TUNE_PeakValReset(TUNE_ID_t id, int32_t channel, float setpoint);
static bool TUNE_CalPID(TUNE_ID_t id);

/*---------------------------------------------------------------------------------------
 函数原型:  void TUNE_Init(void)
 功    能:  初始化自整定相关参数,使用默认的TUNE_CFG_PARAM_t参数初始化自整定参数
            default cTrlType = CONTROLER_TYPE_PI,
            default outputStep = 50,
            default hysteresisNum = 5
            default acterType = POSITIVE_ACTION
 输入参数:	NA
 输出参数:	NA
 返 回 值:	NA
---------------------------------------------------------------------------------------*/
void TUNE_Init(void)
{
    for (int16_t i = 0; i < AUTO_TUNE_OBJ_NUM; i++)
    {
        TUNE_StructInitToDefaultVal(i);
    }
}

/*---------------------------------------------------------------------------------------
 函数原型:  static void TUNE_StructInitToDefaultVal(TUNE_ID_t id)
 功    能:  自整定对象初始化为默认值
 输入参数:	NA
 输出参数:	NA
 返 回 值:	true:成功，false:失败
---------------------------------------------------------------------------------------*/
static bool TUNE_StructInitToDefaultVal(TUNE_ID_t id)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;

    s_tuneObject[id].tuneId = -1;
    s_tuneObject[id].cTrlType = CONTROLER_TYPE_PID;
    s_tuneObject[id].maxOutputStep = 50;
    s_tuneObject[id].hysteresisNum = 5;
    s_tuneObject[id].acterType = POSITIVE_ACTION;
    if (!TUNE_FsmReset(id))
    {
        return false;
    }
    return true;
}

/*---------------------------------------------------------------------------------------
 函数原型:  static void TUNE_FsmReset(TUNE_ID_t id)
 功    能:  状态机复位
 输入参数:	NA
 输出参数:	NA
 返 回 值:	true:成功，false:失败
---------------------------------------------------------------------------------------*/
static bool TUNE_FsmReset(TUNE_ID_t id)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;

    s_tuneObject[id].tuneStat = TUNE_INIT;
    s_tuneObject[id].tuneCounter = 0;
    s_tuneObject[id].cycleCounter = 0;
    s_tuneObject[id].riseHysteresisCounter = 0;
    s_tuneObject[id].fallHysteresisCounter = 0;
    s_tuneObject[id].offetStat = FEEDBACK_ABOVE_INPUT;
    s_tuneObject[id].peakWriPos = 0;

    return true;
}
/*---------------------------------------------------------------------------------------
 函数原型:  TUNE_ID_t TUNE_New(pTUNE_CFG_PARAM_t pParam)
 功    能:  新建一个PID自整定对象
 输入参数:	pParam：自整定对象配置参数
 输出参数:	NA
 返 回 值:	<0，则新建自整定对象失败，可能对象资源已经用完，需要通过更改AUTO_TUNE_OBJ_NUM宏定义
            增加自整定对象资源，>=0,则为分配的自整定id，后续函数调用均通过此Id
 ---------------------------------------------------------------------------------------*/
TUNE_ID_t TUNE_New(pTUNE_CFG_PARAM_t pParam)
{
    for (int16_t i = 0; i < AUTO_TUNE_OBJ_NUM; i++)
    {
        if (s_tuneObject[i].tuneId < 0)
        {
            s_tuneObject[i].tuneId = i;
            if (pParam == NULL)
                return i;
            s_tuneObject[i].cTrlType = pParam->cTrlType;
            s_tuneObject[i].maxOutputStep = pParam->maxOutputStep;
            s_tuneObject[i].minOutputStep = pParam->minOutputStep;
            s_tuneObject[i].hysteresisNum = pParam->hysteresisNum;
            s_tuneObject[i].setpoint = pParam->setpoint;
            s_tuneObject[i].acterType = pParam->acterType;
            s_tuneObject[i].ampStdDeviation = pParam->ampStdDeviation;
            s_tuneObject[i].cycleStdDeviation = pParam->cycleStdDeviation;
            return i;
        }
    }
    return -1;
}
/*---------------------------------------------------------------------------------------
 函数原型:  TUNE_ID_t TUNE_New(pTUNE_CFG_PARAM_t pParam)
 功    能:  释放ID所示自整定对象资源
 输入参数:	id:自整定ID
 输出参数:	NA
 返 回 值:	false:资源释放失败，true:资源释放成功
 注意事项：  只有在tuneStat为TUNE_FAIL或者TUNE_SUCESS状态下，才允许释放资源
 ---------------------------------------------------------------------------------------*/
bool TUNE_Release(TUNE_ID_t id)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;

    if (s_tuneObject[id].tuneStat == TUNE_FAIL || s_tuneObject[id].tuneStat == TUNE_SUCESS || s_tuneObject[id].tuneStat == TUNE_INIT)
    {
        TUNE_StructInitToDefaultVal(id);
        return true;
    }
    return false;
}

/*---------------------------------------------------------------------------------------
 函数原型:  bool TUNE_Work(TUNE_ID_t id, float feedbackVal, float*outputVal)
 功    能:  自整定任务
 输入参数:	id:自整定ID
            delayMsec:调用的时间间隔
 输出参数:	outputVal:输出值
 返 回 值:	true:自整定完成，false:正在自整定中
 注意事项：  该函数需要以固定的时间间隔调用
---------------------------------------------------------------------------------------*/
#define DEBUG
#include <stdio.h>
TUNE_STAT_t TUNE_Work(TUNE_ID_t id, float feedbackVal, float *outputVal, uint32_t delayMsec)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return TUNE_FAIL;
    if (s_tuneObject[id].tuneId < 0)
        return TUNE_FAIL;

    static float outputInSetVGreaterFBV; // 设定值大于反馈值时，使用的输出值
    static float outputInSetVLessFBV;    // 设定值小于反馈值时，使用的输出值

    static TUNE_OBJ_t *this_o;
    this_o = &s_tuneObject[id];

    this_o->feedbackVal = feedbackVal;

    this_o->tuneCounter += delayMsec;

    /*获取驱动器是正向作用时，高输出为阶跃值，低输出为0*/
    if (this_o->acterType == POSITIVE_ACTION)
    {
        outputInSetVGreaterFBV = this_o->maxOutputStep;
        outputInSetVLessFBV = this_o->minOutputStep;
    }
    else /*获取驱动器是反向作用时，高输出为0，低输出为阶跃值*/
    {
        outputInSetVGreaterFBV = this_o->minOutputStep;
        outputInSetVLessFBV = this_o->maxOutputStep;
    }
    #ifdef DEBUG
    printf("this_o->tuneStat: %d\n", this_o->tuneStat);
    #endif
    /*状态机*/
    switch (this_o->tuneStat)
    {
    case TUNE_INIT: /*初始化状态*/
        TUNE_FsmReset(id);
        if (feedbackVal <= this_o->setpoint)
        {
            *outputVal = outputInSetVGreaterFBV; // 高输出
        }
        else
        {
            *outputVal = outputInSetVLessFBV; // 低输出
        }
        this_o->tuneStat = TUNE_START_POINT;
        return TUNE_INIT;
    case TUNE_START_POINT: /*寻找起始点，反馈值在设定值之上，则认为是起始点*/
        // AUTO_TUNE_OBJ_NUM > TRUENUM -> this_o->fall/riseHysteresisCounter = 0.0
        // printf("this_o->fallHysteresisCounter: %d\n", this_o->fallHysteresisCounter);
        // printf("this_o->riseHysteresisCounter: %d\n", this_o->riseHysteresisCounter);
        if (feedbackVal < this_o->setpoint)
        {
            this_o->riseHysteresisCounter = 0;
            // printf("IF 1 %d %d \n",this_o->fallHysteresisCounter ,this_o->hysteresisNum);
            if (++this_o->fallHysteresisCounter >= this_o->hysteresisNum)
            {
                *outputVal = outputInSetVGreaterFBV; // 高输出
            }
        }
        else
        {
            this_o->fallHysteresisCounter = 0;
            // printf("IF 0 %d %d \n",this_o->riseHysteresisCounter ,this_o->hysteresisNum);
            if (++this_o->riseHysteresisCounter >= this_o->hysteresisNum)
            {
                TUNE_PeakValReset(id, 0, this_o->setpoint);
                TUNE_PeakValReset(id, 1, this_o->setpoint);
                TUNE_PeakValReset(id, 2, this_o->setpoint);
                this_o->tuneStat = TUNE_RUNNING;
            }
        }
        return TUNE_INIT;
    case TUNE_RUNNING:

        if (feedbackVal > this_o->setpoint)
        {
            this_o->fallHysteresisCounter = 0;
            /*反馈值大于设置值的次数大于回滞量，则认为反馈值进入高于设置值的上半轴象限了*/
            if (++this_o->riseHysteresisCounter >= this_o->hysteresisNum)
            {
                *outputVal = outputInSetVLessFBV;
                if (this_o->offetStat == FEEDBACK_BELOW_INPUT)
                {
                    this_o->offetStat = FEEDBACK_ABOVE_INPUT;

                    this_o->fullCycleFlag = 0;

                    if (this_o->peakWriPos >= LAST_PEAK_NUM - 1)
                    {
                        this_o->peakWriPos = 0;
                    }
                    else
                    {
                        this_o->peakWriPos++;
                    }
                    TUNE_PeakValReset(id, this_o->peakWriPos, this_o->setpoint);
                }
                /*反馈值进入高于设置值的上半轴象限时，有最大值,寻找最大值*/
                if (feedbackVal >= this_o->lastPeakVal[this_o->peakWriPos].maxPeak.feedabackVal)
                {
                    this_o->lastPeakVal[this_o->peakWriPos].maxPeak.feedabackVal = feedbackVal;
                    this_o->lastPeakVal[this_o->peakWriPos].maxPeak.milSecond = this_o->tuneCounter;
                }
            }
        }
        else
        {
            this_o->riseHysteresisCounter = 0;
            /*反馈值小于设置值的次数大于回滞量，则认为反馈值进入低于设置值的下半轴象限了*/
            if (++this_o->fallHysteresisCounter >= this_o->hysteresisNum)
            {
                *outputVal = outputInSetVGreaterFBV;
                if (this_o->offetStat == FEEDBACK_ABOVE_INPUT)
                {
                    this_o->offetStat = FEEDBACK_BELOW_INPUT;
                    this_o->cycleCounter++;
                }
                /*反馈值进入低于设置值的下半轴象限时，有最小值,存储最小值*/
                if (feedbackVal <= this_o->lastPeakVal[this_o->peakWriPos].minPeak.feedabackVal)
                {
                    this_o->lastPeakVal[this_o->peakWriPos].minPeak.feedabackVal = feedbackVal;
                    this_o->lastPeakVal[this_o->peakWriPos].minPeak.milSecond = this_o->tuneCounter;
                    this_o->fullCycleFlag = 1;
                }
                else
                {
                    if (this_o->fullCycleFlag > 0)
                    {
                        this_o->fullCycleFlag++;
                    }
                }

                /*反馈值穿越设置值的次数大于LAST_PEAK_NUM，则认为至少运行了LAST_PEAK_NUM个周期*/
                if (this_o->cycleCounter >= LAST_PEAK_NUM && this_o->fullCycleFlag >= this_o->hysteresisNum)
                {
                    float ftemp1, ftemp2;
                    float peak[LAST_PEAK_NUM];
                    float peakTime[LAST_PEAK_NUM];
                    // 计算每个周期的峰高和周期时间
                    for (int i = 0; i < LAST_PEAK_NUM; i++)
                    {
                        peak[i] = this_o->lastPeakVal[i].maxPeak.feedabackVal - this_o->lastPeakVal[i].minPeak.feedabackVal;
                        peakTime[i] = (float)(this_o->lastPeakVal[i].minPeak.milSecond - this_o->lastPeakVal[i].maxPeak.milSecond);
                    }
                    // 计算峰高和周期时间的方差
                    ftemp1 = TUNE_CalStdDeviation(peak, LAST_PEAK_NUM);
                    ftemp2 = TUNE_CalStdDeviation(peakTime, LAST_PEAK_NUM);
                    this_o->CurAmpStdDeviation = ftemp1;
                    this_o->CurCycleStdDeviation = ftemp2;
#ifdef DEBUG
                    printf("峰高和周期时间的方差: %f %f\n", ftemp1, ftemp2);
                    TUNE_CalPID(id);
                    // printf(this_o->acterType == POSITIVE_ACTION ? "正向作用\n" : "反向作用\n");
                    printf("Ku:%f Tu:%f Kp:%f Ki:%f Kd:%f\n", this_o->Ku, this_o->Tu, this_o->Kp, this_o->Ki, this_o->Kd);
#endif

                    // 方差满足预期要求，则认为PID自整定成功
                    if (ftemp1 < this_o->ampStdDeviation && ftemp2 < this_o->cycleStdDeviation)
                    {
                        this_o->tuneStat = TUNE_SUCESS;
                    }
                }
            }
        }

        // 如果100个周期或者1小时没成功，则自整定失败
        if (this_o->cycleCounter > MAX_CYCLE || this_o->tuneCounter > MAX_TIME_MS)
        {
            this_o->tuneStat = TUNE_FAIL;
        }
#ifdef DEBUG
        printf("%f,%f\n", feedbackVal, *outputVal);
        printf("cycleCounter:%d fullCycleFlag:%d hysteresisNum设定迟滞个数: %d \n", this_o->cycleCounter, this_o->fullCycleFlag, this_o->hysteresisNum);
        
#endif
        return TUNE_RUNNING;
    case TUNE_FAIL:
#ifdef DEBUG
        printf("----------------------------\nTUNE_FAIL\n");
        TUNE_CalPID(id);
        printf("Ku:%f Tu:%f Kp:%f Ki:%f Kd:%f\n", this_o->Ku, this_o->Tu, this_o->Kp, this_o->Ki, this_o->Kd);
#endif
        TUNE_FsmReset(id);
        *outputVal = outputInSetVLessFBV;
        return TUNE_FAIL;
    case TUNE_SUCESS:
        TUNE_CalPID(id);
        TUNE_FsmReset(id);
        *outputVal = outputInSetVLessFBV;
        return TUNE_SUCESS;
    default:
        return TUNE_INIT;
    }
}
/*---------------------------------------------------------------------------------------
 函数原型:   static void TUNE_PeakValReset(int32_t channel,float setpoint)
 功    能:  峰值初始化
输入参数:	id:整定ID
            channel:存储峰值的通道号
            setpoint：设置值
 输出参数:	NA
 返 回 值:	方差
---------------------------------------------------------------------------------------*/
static void TUNE_PeakValReset(TUNE_ID_t id, int32_t channel, float setpoint)
{

    s_tuneObject[id].lastPeakVal[channel].maxPeak.feedabackVal = setpoint;
    s_tuneObject[id].lastPeakVal[channel].maxPeak.milSecond = 0;
    s_tuneObject[id].lastPeakVal[channel].minPeak.feedabackVal = setpoint;
    s_tuneObject[id].lastPeakVal[channel].minPeak.milSecond = 0;
}
/*---------------------------------------------------------------------------------------
 函数原型:   bool TUNE_CalStdDeviation(float * data,uint32_t len)
 功    能:  计算标准差
 输入参数:	fdata：浮点数据
            lenL:数据个数
 输出参数:	NA
 返 回 值:	标准差
---------------------------------------------------------------------------------------*/
static float TUNE_CalStdDeviation(float *fdata, uint32_t len)
{
    if (fdata == NULL || len == 0)
        return 0;

    float peakAver = 0, variance = 0;

    for (uint32_t i = 0; i < len; i++)
    {
        peakAver += fdata[i];
    }
    peakAver /= LAST_PEAK_NUM;

    for (uint32_t i = 0; i < len; i++)
    {
        variance += powf(fdata[i] - peakAver, 2);
    }
    variance /= (float)len;

    return sqrtf(variance);
}
/*---------------------------------------------------------------------------------------
 函数原型:  bool TUNE_Setsetpoint(TUNE_ID_t id, float setpoint)
 功    能:  设置自整定设定值
 输入参数:	id:自整定ID
            setpoint:自整定设置值

 输出参数:	NA
 返 回 值:	true:设置成功，false:设置失败
---------------------------------------------------------------------------------------*/
bool TUNE_Setsetpoint(TUNE_ID_t id, float setpoint)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    s_tuneObject[id].setpoint = setpoint;
    return true;
}
/*---------------------------------------------------------------------------------------
函数原型:   bool TUNE_SetOutStep(TUNE_ID_t id, float maxStep,float minStep)
功    能:  设置输出阶跃值
输入参数:	id:自整定ID
           maxStep:最大输出阶跃值
           minStep:最大输出阶跃值
输出参数:	NA
返 回 值:	true:设置成功，false:设置失败
---------------------------------------------------------------------------------------*/
bool TUNE_SetOutStep(TUNE_ID_t id, float maxStep, float minStep)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    s_tuneObject[id].maxOutputStep = maxStep;
    s_tuneObject[id].minOutputStep = minStep;
    return true;
}
/*---------------------------------------------------------------------------------------
函数原型:  bool TUNE_SetActerType(TUNE_ID_t id, float maxStep,DRIVER_ACTION_TYPE_t type)
功    能:  设置驱动器类型
输入参数:	id:自整定ID
           type:驱动器类型
输出参数:	NA
返 回 值:	true:设置成功，false:设置失败
---------------------------------------------------------------------------------------*/
bool TUNE_SetActerType(TUNE_ID_t id, DRIVER_ACTION_TYPE_t type)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    s_tuneObject[id].acterType = type;
    return true;
}
/*---------------------------------------------------------------------------------------
 函数原型:   bool TUNE_SetCtrlType(TUNE_ID_t id, TUNE_CONTROLER_TYPE_t type)
 功    能:  设置控制器类型
 输入参数:	id:自整定ID
            type:
                CONTROLER_TYPE_PI，PI控制器,积分项不使用
                CONTROLER_TYPE_PID，PID控制器
 输出参数:	NA
 返 回 值:	true:设置成功，false:设置失败
---------------------------------------------------------------------------------------*/
bool TUNE_SetCtrlType(TUNE_ID_t id, TUNE_CONTROLER_TYPE_t type)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    s_tuneObject[id].cTrlType = type;
    return true;
}

/*---------------------------------------------------------------------------------------
函数原型:   bool TUNE_CalPID(TUNE_ID_t id)
功    能:  计算PID
输入参数:	id:自整定ID
输出参数:	NA
返 回 值:	true:获取成功，否则失败
---------------------------------------------------------------------------------------*/
static bool TUNE_CalPID(TUNE_ID_t id)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    TUNE_OBJ_t *this_o = &s_tuneObject[id];
    uint16_t pos;

    pos = this_o->peakWriPos == 0 ? LAST_PEAK_NUM - 1 : this_o->peakWriPos - 1;

    this_o->Ku = 4.0f * (this_o->maxOutputStep - this_o->minOutputStep) / ((this_o->lastPeakVal[this_o->peakWriPos].maxPeak.feedabackVal - this_o->lastPeakVal[this_o->peakWriPos].minPeak.feedabackVal) * 3.14159f);

    this_o->Tu = (float)(this_o->lastPeakVal[this_o->peakWriPos].maxPeak.milSecond - this_o->lastPeakVal[pos].maxPeak.milSecond);

    this_o->Kp = this_o->cTrlType == CONTROLER_TYPE_PID ? 0.6f * this_o->Ku : 0.8f * this_o->Ku;
    this_o->Ki = 0.5f * this_o->Tu;
    this_o->Kd = 0.125f * this_o->Tu;
    return true;
}

/*---------------------------------------------------------------------------------------
 函数原型:   float TUNE_GetStat(TUNE_ID_t id, TUNE_STAT_t *pStat)
 功    能:  获取PID自整定状态
 输入参数:	id:自整定ID
 输出参数:	stat，自整定状态
 返 回 值:	true:获取成功，否则失败
---------------------------------------------------------------------------------------*/
bool TUNE_GetStat(TUNE_ID_t id, TUNE_STAT_t *pStat)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    TUNE_OBJ_t *this_o = &s_tuneObject[id];
    *pStat = this_o->tuneStat;
    return true;
}

/*---------------------------------------------------------------------------------------
函数原型:   float TUNE_GetKp(TUNE_ID_t id, float *pfactorP)
功    能:   获取整定后的P参数
输入参数:	id:自整定ID
输出参数:	pfactorP:整定后的P参数
返 回 值:	true:获取成功，否则失败
---------------------------------------------------------------------------------------*/
float TUNE_GetKp(TUNE_ID_t id, float *pfactorP)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    TUNE_OBJ_t *this_o = &s_tuneObject[id];
    *pfactorP = this_o->Kp;
    return true;
}

/*---------------------------------------------------------------------------------------
函数原型:   float TUNE_GetKp(TUNE_ID_t id)
功    能:   获取整定后的I参数
输入参数:	id:自整定ID
输出参数:	pfactorI:整定后的I参数
返 回 值:	true:获取成功，否则失败
---------------------------------------------------------------------------------------*/
float TUNE_GetKi(TUNE_ID_t id, float *pfactorI)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    TUNE_OBJ_t *this_o = &s_tuneObject[id];

    *pfactorI = this_o->Ki;

    return true;
}
/*---------------------------------------------------------------------------------------
函数原型:   float TUNE_GetKp(TUNE_ID_t id)
功    能:   获取整定后的D参数
输入参数:	id:自整定ID
输出参数:	pfactorD:整定后的D参数
返 回 值:	true:获取成功，否则失败
---------------------------------------------------------------------------------------*/
float TUNE_GetKd(TUNE_ID_t id, float *pfactorD)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    TUNE_OBJ_t *this_o = &s_tuneObject[id];
    *pfactorD = this_o->Kd;
    return true;
}

/*---------------------------------------------------------------------------------------
 函数原型:   float TUNE_GetKp(TUNE_ID_t id)
 功    能:  获取整定后的PID参数
 输入参数:	id:自整定ID
 输出参数:	NA
 返 回 值:	true:获取成功，否则失败
---------------------------------------------------------------------------------------*/
bool TUNE_GedPID(TUNE_ID_t id, float *paramP, float *paramI, float *paramD)
{
    if (id >= AUTO_TUNE_OBJ_NUM)
        return false;
    if (s_tuneObject[id].tuneId < 0)
        return false;
    if (paramP != NULL)
        TUNE_GetKp(id, paramP);
    if (paramI != NULL)
        TUNE_GetKi(id, paramI);
    if (paramD != NULL)
        TUNE_GetKd(id, paramD);
    return true;
}