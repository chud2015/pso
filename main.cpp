#include <QCoreApplication>
#include <math.h>
#include<QTime>
#include<QList>
#include <iostream>


using namespace std;
int const dim = 2;               //粒子维数

//粒子
class PARTICLE
{
public:
     double X[dim];                      //微粒的坐标数组
     double XBest[dim];                //微粒的最好位置数组
     double V[dim];                      //粒子速度数组
     double Fit;                      //微粒适合度
     double FitBest;               //微粒最好位置适合度
};


class PSO
{
public:
    QList<PARTICLE> simple_list;        //初始粒子列表
    QList<PARTICLE>  _new_simple_list;  //更新过的粒子列表
    double X_min[dim];    //区间最小值
    double X_max[dim];    //区间最大值



    int  simple_number;         //粒子个数
    static const int  inertia_pra = 10;        //惯性权重因子
    static const int  local_pra = 2;    //个体学习因子
    static const int  global_pra = 2;   // 全局学习因子

    static const int number = 100;             //迭代次数


    double global_best;               //全局最优
    double X_global_best[dim];             //全局最优点

    PSO(double _X_min[dim],double _X_max[dim],int _simple_num);

    void Initial(void);
    double PSOTimeRandomNum(void);
    double ParameterRandomNum(double _r);
    PARTICLE GetNewSimple(PARTICLE _simple);
    double  FunctionToOptimal(PARTICLE _simple);
    void  GetSimpleFitness(PARTICLE _simple);
    void GetGlobalBest(QList<PARTICLE> _simple_list);
    QList<PARTICLE> GetNewSimpleList(QList<PARTICLE> _simple_list);

};







//在区间范围内初始化一群粒子
void PSO ::Initial(void)
{
    //一维初始化
    /*for(int i = 0;i<simple_number;i++)
    {
        PARTICLE  simple;
        double random=(double)PSORandomNum()/60000.0;
        //simple.X[0] = (random*(X_max-X_min)+X_min);
        simple.X[0] = 3;
        simple.XBest[0] = simple.X[0];
        simple.V[0] = 0;
        simple.Fit = FunctionToOptimal(simple);
        simple.FitBest = simple.Fit;
        simple_list.push_back(simple);
    }*/

    //多维初始化
    for(int i=0; i<simple_number; i++)
    {
        PARTICLE  simple;
        for(int j=0; j<dim; j++)
        {
            double _parameter_random_num = ParameterRandomNum(((double)j+(double)i)/2);
            double random=((double)PSOTimeRandomNum()+_parameter_random_num)/2;
            simple.X[j]=(random*(X_max[j]-X_min[j])+X_min[j]);
            simple.XBest[j]=simple.X[j];
            simple.V[j]=0;

        }
        simple.Fit=FunctionToOptimal(simple);
        simple.FitBest= simple.Fit;


        simple_list.push_back(simple);
    }
}


//根据系统时间产生0-1的随机数
double PSO ::PSOTimeRandomNum(void)
{
    QTime t;
    t= QTime::currentTime();
    qsrand(t.msec()+t.second()*1000);
    int n = qrand();
    double m = (double)n/60000;
    return m;
}


//根据参数产生0-1的随机数
double PSO::ParameterRandomNum(double _r)
{
    double r;
    r = _r;
    int m;
    double s,u,v,p;
    s = 65536.0;
    u = 2052.0;
    v = 13849.0;
    m = (int)(r/s);
    r = r - m*s;
    r = u*(r)+v;
    m = (int)(r/s);
    r = r - m*s;
    p = r/s;
    return (p);
}


//对于每一个粒子进行更新
 PARTICLE PSO ::GetNewSimple(PARTICLE _simple)
 {
     PARTICLE _new_simple;
     for(int i=0;i<dim;i++)
     {
         double parameter_random_num = ParameterRandomNum(double(i));
         double local_rand = (double)PSOTimeRandomNum();
         double global_rand =  parameter_random_num;
         _new_simple.V[i] = inertia_pra*_simple.V[i]
                 +local_pra*local_rand*(_simple.XBest[i]-_simple.X[i])
                 +global_pra*global_rand*(global_best-_simple.X[i]);

         _new_simple.X[i] = _simple.X[i]+_new_simple.V[i];

         _new_simple.Fit = FunctionToOptimal(_new_simple);
     }

     if( _new_simple.Fit>_simple.FitBest)
     {
         _new_simple.FitBest=_new_simple.Fit;
         for(int i=0;i<dim;i++)
         {
             _new_simple.XBest[i]=_new_simple.X[i];
         }

     }
     else
     {
         _new_simple.FitBest=_simple.Fit;
         for(int i=0;i<dim;i++)
         {
             _new_simple.XBest[i]=_simple.X[i];
         }

     }


     return _new_simple;

 }


 //得到初始粒子群中的全局最优
 void PSO::GetGlobalBest(QList<PARTICLE> _simple_list)
 {
     _simple_list = simple_list;
     global_best = _simple_list.at(0).FitBest;
     for(int i = 0;i < _simple_list.size();i++)
     {
         if(_simple_list.at(i).FitBest > global_best)
         {
             global_best = _simple_list.at(i).FitBest;
             for(int j = 0; j < dim; j++)
             {
                 X_global_best[j] = _simple_list.at(i).XBest[j];
             }

         }
         else
         {
             global_best = global_best;
             for(int j = 0; j < dim; j++)
             {
                 X_global_best[j] = _simple_list.at(0).XBest[j];
             }

         }
     //return global_best;
    }
 }





//对粒子群中的每个粒子进行更新得到新的粒子群，同时更新全局最优值
 //每调用一次，全局最优值即更新一次
 QList<PARTICLE>PSO:: GetNewSimpleList(QList<PARTICLE> _simple_list)
 {

     QList<PARTICLE>     fun_new_simple_list;

     PARTICLE  _new_simple;


     for(int i=0;i<_simple_list.size();i++)
     {
         _new_simple=GetNewSimple(_simple_list.at(i));
         fun_new_simple_list.push_back(_new_simple);
         if(_new_simple.FitBest > global_best)
         {
             global_best = _new_simple.FitBest;
             for(int j = 0; j < dim;j++)
             {
                 X_global_best[j] = _new_simple.XBest[j];
             }
         }
         else
         {
             global_best = global_best;
         }
     }
     return fun_new_simple_list;
 }



double PSO:: FunctionToOptimal(PARTICLE _simple)
{
    return -(_simple.X[0]*_simple.X[0]+_simple.X[1]*_simple.X[1]);
}





//求粒子的适合度
void  PSO::GetSimpleFitness(PARTICLE _simple)
{
    _simple.Fit=FunctionToOptimal(_simple);
}






PSO::PSO(double _X_min[dim], double _X_max[dim], int _simple_num)
{
    X_min[0] = _X_min[0];
    X_max[0] = _X_max[0];
    X_min[1] = _X_min[1];
    X_max[1] = _X_max[1];
    simple_number = _simple_num;
    //PARTICLE _new_simple;
    Initial();                                 //初始化
    GetGlobalBest(simple_list);               //得到初始粒子群中的全局最优
    _new_simple_list = GetNewSimpleList(simple_list);           //第一次更新
    for(int i = 0; i < number;i++)               //迭代
    {
        QList<PARTICLE> new_simple_list;
        new_simple_list = GetNewSimpleList(_new_simple_list);
        _new_simple_list.clear();
         //GetGlobalBest(_new_simple_list);
        _new_simple_list=new_simple_list;
        new_simple_list.clear();
    }

}



int main()
{
    //QCoreApplication a(argc, argv);
    double X_min[2];
    X_min[0] = -1;
    X_min[1] = -1;
    double X_max[2];
    X_max[0] = 1;
    X_max[1] = 1;
    //PSO dd;
    PSO dd(X_min,X_max,20);

    cout<<dd.global_best<<endl;
    cout<<dd.X_global_best[0]<<","<<dd.X_global_best[1]<<endl;

    return 0;
}
