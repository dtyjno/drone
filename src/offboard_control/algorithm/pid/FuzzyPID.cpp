#include "FuzzyPID.h"
#include <iostream>

float limit(float value, float max_limit, float min_limit)
{
    if (value > max_limit)
        return max_limit;
    if (value < min_limit)
        return min_limit;
    return value;
}

#define inverse(parameter) 1.0f / (float)parameter

// 高斯型隶属度函数（gaussian membership function）
float gaussmf(float x, float sigma, float c)
{
    return expf(-powf(((x - c) / sigma), 2.0f));
}

// 广义钟形隶属度函数（generalized bell-shaped membership function）
float gbellmf(float x, float a, float b, float c)
{
    return inverse(1.0f + powf(fabsf((x - c) / a), 2.0f * b));
}

// s型隶属度函数（sigmoidal membership function）
float sigmf(float x, float a, float c)
{
    return inverse(1.0f + expf(a * (c - x)));
}

// 梯形隶属度函数（trapezoidal membership function）
float trapmf(float x, float a, float b, float c, float d)
{
    if (x >= a && x < b)
        return (x - a) / (b - a);
    else if (x >= b && x < c)
        return 1.0f;
    else if (x >= c && x <= d)
        return (d - x) / (d - c);
    else
        return 0.0f;
}

// 三角隶属度函数（triangular membership function）
float trimf(float x, float a, float b, float c)
{
    return trapmf(x, a, b, b, c);
}

// z型隶属度函数（z-shaped membership function）
float zmf(float x, float a, float b)
{
    if (x <= a)
        return 1.0f;
    else if (x >= a && x <= (a + b) / 2.0f)
        return 1.0f - 2.0f * powf((x - a) / (b - a), 2.0f);
    else if (x >= (a + b) / 2.0f && x < b)
        return 2.0f * powf((x - b) / (b - a), 2.0f);
    else
        return 0;
}

// 隶属度函数（membership function）
// x: 输入
// mf_type: 隶属度函数类型
// params: 隶属度函数参数
float mf(float x, unsigned int mf_type, float *params)
{
    switch (mf_type)
    {
    case 0:
        return gaussmf(x, params[0], params[1]);
    case 1:
        return gbellmf(x, params[0], params[1], params[2]);
    case 2:
        return sigmf(x, params[0], params[2]);
    case 3:
        return trapmf(x, params[0], params[1], params[2], params[3]);
    case 5:
        return zmf(x, params[0], params[1]);
    default: // set triangular as default membership function
        return trimf(x, params[0], params[1], params[2]);
    }
}

// Union operator
float _or(float a, float b, unsigned int type)
{
    if (type == 1)
    { // algebraic sum
        return a + b - a * b;
    }
    else if (type == 2)
    { // bounded sum
        return fminf(1, a + b);
    }
    else
    { // fuzzy union
        return fmaxf(a, b);
    }
}

// Intersection operator
float _and(float a, float b, unsigned int type)
{
    if (type == 1)
    { // algebraic product
        return a * b;
    }
    else if (type == 2)
    { // bounded product
        return fmaxf(0, a + b - 1);
    }
    else
    { // fuzzy intersection
        return fminf(a, b);
    }
}

// Equilibrium operator
float _equilibrium(float a, float b, float params)
{
    return powf(a * b, 1 - params) * powf(1 - (1 - a) * (1 - b), params);
}

// 模糊数量域的操作（fuzzy quantity fields operation）
float fo(float a, float b, unsigned int type)
{
    if (type < 3)
    {
        return _and(a, b, type);
    }
    else if (type < 6)
    {
        return _or(a, b, type - 3);
    }
    else
    {
        return _equilibrium(a, b, 0.5f);
    }
}

FuzzyPID::FuzzyPID()
{
    // 默认的模糊规则库
    float rule_base[][qf_default] = {
        // delta kp 规则库
        {PB, PB, FUZZY_PM, FUZZY_PM, PS, ZO, ZO},
        {PB, PB, FUZZY_PM, PS, PS, ZO, NS},
        {FUZZY_PM, FUZZY_PM, FUZZY_PM, PS, ZO, NS, NS},
        {FUZZY_PM, FUZZY_PM, PS, ZO, NS, NM, NM},
        {PS, PS, ZO, NS, NS, NM, NM},
        {PS, ZO, NS, NM, NM, NM, NB},
        {ZO, ZO, NM, NM, NM, NB, NB},
        // delta ki 规则库
        {NB, NB, NM, NM, NS, ZO, ZO},
        {NB, NB, NM, NS, NS, ZO, ZO},
        {NB, NM, NS, NS, ZO, PS, PS},
        {NM, NM, NS, ZO, PS, FUZZY_PM, FUZZY_PM},
        {NM, NS, ZO, PS, PS, FUZZY_PM, PB},
        {ZO, ZO, PS, PS, FUZZY_PM, PB, PB},
        {ZO, ZO, PS, FUZZY_PM, FUZZY_PM, PB, PB},
        // delta kd 规则库
        {PS, NS, NB, NB, NB, NM, PS},
        {PS, NS, NB, NM, NM, NS, ZO},
        {ZO, NS, NM, NM, NS, NS, ZO},
        {ZO, NS, NS, NS, NS, NS, ZO},
        {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
        {PB, PS, PS, PS, PS, PS, PB},
        {PB, FUZZY_PM, FUZZY_PM, FUZZY_PM, PS, PS, PB}};
    // 默认的模糊函数参数（membership function parameters）
    float mf_params[4 * qf_default] = {-3, -3, -2, 0,
                                       -3, -2, -1, 0,
                                       -2, -1, 0, 0,
                                       -1, 0, 1, 0,
                                       0, 1, 2, 0,
                                       1, 2, 3, 0,
                                       2, 3, 3, 0};
    // float mf_params[4 * qf_default] = {-0.75, -0.75, -0.5, 0,
    //                                    -0.75, -0.5, -0.25, 0,
    //                                    -0.5, -0.25, 0, 0,
    //                                    -0.25, 0, 0.25, 0,
    //                                    0, 0.25, 0.5, 0,
    //                                    0.25, 0.5, 0.75, 0,
    //                                    0.5, 0.75, 0.75, 0};
    struct FuzzyPID::Fuzzy_params fuzzy_params[8] = {
        {4, 1, 0, mf_params, rule_base, 100, 100, 8},
        {4, 1, 0, mf_params, rule_base, 100, 100, 8},
        {4, 1, 0, mf_params, rule_base, 100, 100, 8},
        {4, 1, 0, mf_params, rule_base, 100, 100, 8},
        {4, 1, 0, mf_params, rule_base, 100, 100, 8},
        {4, 1, 0, mf_params, rule_base, 100, 100, 8},
        {4, 1, 0, mf_params, rule_base, 100, 100, 8},
        {4, 1, 0, mf_params, rule_base, 100, 100, 8},
    };
    Init_FuzzyPID(fuzzy_params);
}

FuzzyPID::FuzzyPID(struct FuzzyPID::Fuzzy_params *fuzzy_params)
{
    Init_FuzzyPID(fuzzy_params);
}

void FuzzyPID::Init_FuzzyPID(struct FuzzyPID::Fuzzy_params *fuzzy_params)
{
    struct Fuzzy_params fuzzy_param = fuzzy_params[0];
    kp = 1;
    ki = 1;
    kd = 1;
    delta_kp_max = 0;
    delta_ki_max = 0;
    delta_kd_max = 0;
    // delta_kp_max = kp / delta_k;
    // delta_ki_max = ki / delta_k;
    // delta_kd_max = kd / delta_k;

    delta_kp = 0;
    delta_ki = 0;
    delta_kd = 0;

    error_max = (float *)malloc(sizeof(float) * fuzzy_param.control_id_count);
    delta_error_max = (float *)malloc(sizeof(float) * fuzzy_param.control_id_count);
    for (int i = 0; i < fuzzy_param.control_id_count; ++i)
    {
        error_max[i] = fuzzy_params[i].max_error;
        delta_error_max[i] = fuzzy_params[i].max_delta_error;
    }
    ////
    // 根据ki和kd是否为0来确定输出的数量
    int output_count = 1;
    if (ki > 1e-4)
    {
        output_count += 1;
        if (kd > 1e-4)
            output_count += 1;
    }

    fuzzy_input_num = 2;
    fuzzy_output_num = output_count;
    fo_type = fuzzy_param.fo_type;
    mf_type = (unsigned int *)malloc(sizeof(unsigned int) * (fuzzy_input_num + fuzzy_output_num));
    for (unsigned int i = 0; i < fuzzy_input_num + fuzzy_output_num; ++i)
    {
        mf_type[i] = fuzzy_param.mf_type;
    }

    fuzzy_output = (float *)malloc(sizeof(float) * fuzzy_output_num);
    for (unsigned int i = 0; i < fuzzy_output_num; ++i)
    {
        fuzzy_output[i] = 0;
    }

    // #ifdef fuzzy_pid_rule_base_deep_copy
    rule_base = (float *)malloc(sizeof(float) * fuzzy_output_num * qf_default * qf_default);
    mf_params = (float *)malloc(sizeof(float) * 4 * qf_default);
    for (unsigned int j = 0; j < 4 * qf_default; ++j)
    {
        mf_params[j] = fuzzy_param.mf_params[j];
    }

    for (unsigned int k = 0; k < fuzzy_output_num * qf_default; ++k)
    {
        for (unsigned int i = 0; i < qf_default; ++i)
        {
            rule_base[k * 7 + i] = fuzzy_param.rule_base[k][i];
        }
    }
    // #else
    // #endif

    fo_type = fo_type;
    df_type = df_type;

    ////
    control_id_count = fuzzy_param.control_id_count;
    last_error = (float *)malloc(sizeof(float) * fuzzy_param.control_id_count);
    for (int i = 0; i < fuzzy_param.control_id_count; ++i)
    {
        last_error[i] = 0;
    }
    current_error = 0;
}

// 中心去模糊化的平均值，只适用于两个输入的多重指标
// Mean of centers defuzzifier, only for two input multiple index
void FuzzyPID::moc(const float *joint_membership, const unsigned int *index, const unsigned int *count)
{

    float denominator_count = 0;
    float numerator_count[fuzzy_output_num];
    for (unsigned int l = 0; l < fuzzy_output_num; ++l)
    {
        numerator_count[l] = 0;
    }

    for (unsigned int i = 0; i < count[0]; ++i)
    {
        for (unsigned int j = 0; j < count[1]; ++j)
        {
            denominator_count += joint_membership[i * count[1] + j];
        }
    }

    for (unsigned int k = 0; k < fuzzy_output_num; ++k)
    {
        for (unsigned int i = 0; i < count[0]; ++i)
        {
            for (unsigned int j = 0; j < count[1]; ++j)
            {
                numerator_count[k] += joint_membership[i * count[1] + j] *
                                      rule_base[k * qf_default * qf_default + index[i] * qf_default +
                                                index[count[0] + j]];
            }
        }
    }

#ifdef fuzzy_pid_debug_print
    printf("output:\n");
#endif
    for (unsigned int l = 0; l < fuzzy_output_num; ++l)
    {
        fuzzy_output[l] = numerator_count[l] / denominator_count;
#ifdef fuzzy_pid_debug_print
        printf("%f / %f = %f\n", numerator_count[l], denominator_count, output[l]);
#endif
    }
}

// 去模糊化器（defuzzifier）
void FuzzyPID::df(const float *joint_membership, const unsigned int *output, const unsigned int *count, int df_type)
{
    if (df_type == 0)
        moc(joint_membership, output, count);
    else
    {
        printf("Waring: No such of defuzzifier!\n");
        moc(joint_membership, output, count);
    }
}

// 模糊控制器（fuzzy controller）
void FuzzyPID::fuzzy_control(float e, float de)
{
    Ke = e;
    Kde = de;

    float membership[qf_default * 2];   // Store membership
    unsigned int index[qf_default * 2]; // Store the index of each membership
    unsigned int count[2] = {0, 0};

    // 通过隶属度函数计算隶属度
    {
        int j = 0;
        for (int i = 0; i < qf_default; ++i)
        {
            float temp = mf(e, mf_type[0], mf_params + 4 * i);
            if (temp > 1e-4)
            {
                membership[j] = temp;
                index[j++] = i;
            }
        }

        count[0] = j;

        for (int i = 0; i < qf_default; ++i)
        {
            float temp = mf(de, mf_type[1], mf_params + 4 * i);
            if (temp > 1e-4)
            {
                membership[j] = temp;
                index[j++] = i;
            }
        }

        count[1] = j - count[0];
    }

#ifdef fuzzy_pid_debug_print
    {
        int j = count[0] + count[1];
        printf("membership:\n");
        for (unsigned int k = 0; k < j; ++k)
        {
            printf("%f\n", membership[k]);
        }

        printf("index:\n");
        for (unsigned int k = 0; k < j; ++k)
        {
            printf("%d\n", index[k]);
        }

        printf("count:\n");
        for (unsigned int k = 0; k < 2; ++k)
        {
            printf("%d\n", count[k]);
        }
    }
#endif

    if (count[0] == 0 || count[1] == 0)
    {
        for (unsigned int l = 0; l < fuzzy_output_num; ++l)
        {
            fuzzy_output[l] = 0;
        }
        return;
    }

    // 计算联合隶属度
    // Joint membership
    float joint_membership[count[0] * count[1]];

    for (unsigned int i = 0; i < count[0]; ++i)
    {
        for (unsigned int j = 0; j < count[1]; ++j)
        {
            joint_membership[i * count[1] + j] = fo(membership[i], membership[count[0] + j], fo_type);
        }
    }

    df(joint_membership, index, count, 0);

    delta_kp = fuzzy_output[0] / 3.0f * delta_kp_max;

    if (fuzzy_output_num >= 2)
        delta_ki = fuzzy_output[1] / 3.0f * delta_ki_max;
    else
        delta_ki = 0;

    if (fuzzy_output_num >= 3)
        delta_kd = fuzzy_output[2] / 3.0f * delta_kd_max;
    else
        delta_ki = 0;

#ifdef fuzzy_pid_debug_print
    printf("kp : %f, ki : %f, kd : %f\n", kp, ki, kd);
    printf("delta kp : %f, delta ki : %f, delta kd : %f\n", delta_kp, delta_ki, delta_kd);
    printf("kp + delta kp : %f, %f, %f\n", kp + delta_kp, ki + delta_ki, kd + delta_kd);
    printf("output : %f\n", output);
#endif
}

// : undefined reference to `FuzzyPID::fuzzy_pid_control(float, float, float&, float&, float&, float)'
void FuzzyPID::fuzzy_pid_control(float real, float idea, int control_id, float &kp, float &ki, float &kd, float delta_k)
{

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    if (delta_k != 0.0f)
    {
        delta_kp_max = kp / delta_k;
        delta_ki_max = ki / delta_k;
        delta_kd_max = kd / delta_k;
    }
    target = idea;
    actual = real;

    if (control_id > control_id_count || control_id < 0)
    {
        printf("control_id is out of range!\n");
        return;
    }
    else
    {
        last_error[control_id] = current_error;
    }
    current_error = idea - real;
    float delta_error = current_error - last_error[control_id];

    limit(delta_error, delta_error_max[control_id], -delta_error_max[control_id]);
    limit(current_error, error_max[control_id], -error_max[control_id]);

    fuzzy_control(current_error / error_max[control_id] * 3.0f, delta_error / delta_error_max[control_id] * 3.0f);

    kp += delta_kp;
    ki += delta_ki;
    kd += delta_kd;
}

void FuzzyPID::showMf(const int tab, float *mf_paras)
{
    string type;
    if (tab == 0)
        type = "gaussmf";
    else if (tab == 1)
        type = "gbellmf";
    else if (tab == 2)
        type = "sigmf";
    else if (tab == 3)
        type = "trapmf";
    else if (tab == 4)
        type = "trimf";
    else if (tab == 5)
        type = "zmf";
    else
        type = "trimf";
    cout << "函数类型：" << type << endl;
    cout << "函数参数列表：" << endl;
    if (type == "gaussmf")
    {
        cout << "sigma: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4) << " ";
        }
        cout << endl;
        cout << "c: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 1) << " ";
        }
        cout << endl;
    }
    else if (type == "gbellmf")
    {
        cout << "a: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4) << " ";
        }
        cout << endl;
        cout << "b: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 1) << " ";
        }
        cout << endl;
        cout << "c: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 2) << " ";
        }
        cout << endl;
    }
    else if (type == "sigmf")
    {
        cout << "a: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4) << " ";
        }
        cout << endl;
        cout << "c: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 2) << " ";
        }
        cout << endl;
    }
    else if (type == "trapmf")
    {
        cout << "a: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4) << " ";
        }
        cout << endl;
        cout << "b: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 1) << " ";
        }
        cout << endl;
        cout << "c: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 2) << " ";
        }
        cout << endl;
        cout << "d: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 3) << " ";
        }
        cout << endl;
    }
    else if (type == "trimf")
    {
        cout << "a: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4) << " ";
        }
        cout << endl;
        cout << "b: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 1) << " ";
        }
        cout << endl;
        cout << "c: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 2) << " ";
        }
        cout << endl;
    }
    else if (type == "zmf")
    {
        cout << "a: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4) << " ";
        }
        cout << endl;
        cout << "b: " << endl;
        for (int i = 1; i < qf_default; i++)
        {
            cout << *(mf_paras + i * 4 + 1) << " ";
        }
        cout << endl;
    }
}

// void delete_fuzzy(struct fuzzy *fuzzy_struct)
// {
//     free(fuzzy_struct->mf_type);
//     free(fuzzy_struct->output);
//     free(fuzzy_struct);
// }

// void delete_pid(struct PID *pid)
// {
//     if (pid->fuzzy_struct != NULL)
//     {
//         delete_fuzzy(pid->fuzzy_struct);
//     }
//     free(pid);
// }

// void delete_pid_vector(struct PID **pid_vector, unsigned int count)
// {
//     for (unsigned int i = 0; i < count; ++i)
//     {
//         delete_pid(pid_vector[i]);
//     }
//     free(pid_vector);
// }

void FuzzyPID::showFuzzyPIDInfo(int control_id)
{
    cout << "Info of this fuzzy controller is as following:" << endl;
    cout << "基本论域e：[" << -error_max[control_id] << "," << error_max[control_id] << "]" << endl;
    cout << "基本论域de：[" << -delta_error_max[control_id] << "," << delta_error_max[control_id] << "]" << endl;
    cout << "基本论域delta_Kp：[" << -delta_kp_max << "," << delta_kp_max << "]" << endl;
    cout << "基本论域delta_Ki：[" << -delta_ki_max << "," << delta_ki_max << "]" << endl;
    cout << "基本论域delta_Kd：[" << -delta_kd_max << "," << delta_kd_max << "]" << endl;
    cout << "误差e的模糊隶属度函数参数：" << endl;
    showMf(mf_type[0], mf_params);
    // cout << "误差变化率de的模糊隶属度函数参数：" << endl;
    // showMf(mf_t_de, de_mf_paras);
    // cout << "delta_Kp的模糊隶属度函数参数：" << endl;
    // showMf(mf_t_Kp, Kp_mf_paras);
    // cout << "delta_Ki的模糊隶属度函数参数：" << endl;
    // showMf(mf_t_Ki, Ki_mf_paras);
    // cout << "delta_Kd的模糊隶属度函数参数：" << endl;
    // showMf(mf_t_Kd, Kd_mf_paras);
    cout << "模糊规则表：" << endl;
    cout << "delta_Kp的模糊规则矩阵" << endl;
    for (int i = 0; i < qf_default; i++)
    {
        for (int j = 0; j < qf_default; j++)
        {
            cout.width(3);
            cout << *(rule_base + i * qf_default + j) << "  ";
        }
        cout << endl;
    }
    cout << "delta_Ki的模糊规则矩阵" << endl;
    for (int i = qf_default; i < 2 * qf_default; i++)
    {
        for (int j = 0; j < qf_default; j++)
        {
            cout.width(3);
            cout << *(rule_base + i * qf_default + j) << "  ";
        }
        cout << endl;
    }
    cout << "delta_Kd的模糊规则矩阵" << endl;
    for (int i = 2 * qf_default; i < 3 * qf_default; i++)
    {
        for (int j = 0; j < qf_default; j++)
        {
            cout.width(3);
            cout << *(rule_base + i * qf_default + j) << "  ";
        }
        cout << endl;
    }
    cout << endl;
    cout << "误差的量化比例因子Ke=" << Ke << endl;
    cout << "误差变化率的量化比例因子Kde=" << Kde << endl;
    cout << "输出的量化比例因子Ku_p=" << delta_kp << endl;
    cout << "输出的量化比例因子Ku_i=" << delta_ki << endl;
    cout << "输出的量化比例因子Ku_d=" << delta_kd << endl;
    cout << "设定目标target=" << target << endl;
    cout << "实际值actual=" << actual << endl;
    cout << "误差e=" << target - actual << endl;
    cout << "Kp=" << kp << endl;
    cout << "Ki=" << ki << endl;
    cout << "Kd=" << kd << endl;
    cout << endl;
}

void FuzzyPID::set_rule_base(int i, int j, float value)
{
    *(rule_base + i * qf_default * qf_default + j) = value;
}
