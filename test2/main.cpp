#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>

using namespace std;
using namespace cv;
using namespace ceres;

//vector,用于存放x、y的观测数据
//待估计函数为y=3.5x^3+1.6x^2+0.3x+7.8
vector<double> xs;
vector<double> ys;

//定义CostFunctor结构体用于描述代价函数
struct CostFunctor{
  
  double x_guan,y_guan;
  
  //构造函数，用已知的x、y数据对其赋值
  CostFunctor(double x,double y)
  {
    x_guan = x;
    y_guan = y;
  }
  
  //重载括号运算符，两个参数分别是估计的参数和由该参数计算得到的残差
  //注意这里的const，一个都不能省略，否则就会报错
  template <typename T>
  bool operator()(const T* const params,T* residual)const
  {
    residual[0]=y_guan-(params[0]*x_guan*x_guan*x_guan+params[1]*x_guan*x_guan+params[2]*x_guan+params[3]);
    return true;  
  }
};

//生成实验数据
void generateData()
{
  RNG rng;
  //RNG::gaussian( σ) 返回一个均值为0，标准差为σ的随机数。
  double w_sigma = 1.0;
  for(int i=0;i<100;i++)
  {
    double x = i;
    double y = 3.5*x*x*x+1.6*x*x+0.3*x+7.8;
    xs.push_back(x);
    ys.push_back(y+rng.gaussian(w_sigma));
  }
  for(int i=0;i<xs.size();i++)
  {
    cout<<"x:"<<xs[i]<<" y:"<<ys[i]<<endl;
  }
}

//简单描述我们优化的目的就是为了使我们估计参数算出的y'和实际观测的y的差值之和最小
//所以代价函数(CostFunction)就是y'-y，其对应每一组观测值与估计值的残差。
//由于我们优化的是残差之和，因此需要把代价函数全部加起来，使这个函数最小，而不是单独的使某一个残差最小
//默认情况下，我们认为各组的残差是等权的，也就是核函数系数为1。
//但有时可能会出现粗差等情况，有可能不等权，但这里不考虑。
//这个求和以后的函数便是我们优化的目标函数
//通过不断调整我们的参数值，使这个目标函数最终达到最小，即认为优化完成
int main(int argc, char **argv) {
  
  generateData();
  
  //创建一个长度为4的double数组用于存放参数
  double params[4]={1.0};

  //第一步，创建Problem对象，并对每一组观测数据添加ResidualBlock
  //由于每一组观测点都会得到一个残差，而我们的目的是最小化所有残差的和
  //所以采用for循环依次把每个残差都添加进来
  Problem problem;
  for(int i=0;i<xs.size();i++)
  {
    //利用我们之前写的结构体、仿函数，创建代价函数对象，注意初始化的方式
    //尖括号中的参数分别为误差类型，输出维度(因变量个数)，输入维度(待估计参数的个数)
    CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor,1,4>(new CostFunctor(xs[i],ys[i]));
    //三个参数分别为代价函数、核函数和待估参数
    problem.AddResidualBlock(cost_function,NULL,params);
  }
  
  //第二步，配置Solver
  Solver::Options options;
  //配置增量方程的解法
  options.linear_solver_type=ceres::DENSE_QR;
  //是否输出到cout
  options.minimizer_progress_to_stdout=true;
  
  //第三步，创建Summary对象用于输出迭代结果
  Solver::Summary summary;
  
  //第四步，执行求解
  Solve(options,&problem,&summary);
  
  //第五步，输出求解结果
  cout<<summary.BriefReport()<<endl;
  
  cout<<"p0:"<<params[0]<<endl;
  cout<<"p1:"<<params[1]<<endl;
  cout<<"p2:"<<params[2]<<endl;
  cout<<"p3:"<<params[3]<<endl;
  return 0;
}