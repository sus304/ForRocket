#include <iostream>
#include <fstream>
#include <vector>

#include "Eigen/Core"
#include "boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

class Rocket {
    public:
        double mass = 10.0  // [kg]
        double thrust = 300.0  // [N]
        double tb = 3.0  // [s]
    private:
}

class Dynamics6DoF {
    public:
    private:
}

class Solver {
    public:
    private:
}



struct Rocket
{
    using state = std::array<double, 2>;
    double x0 = 0.0;
    double v0 = 0.0;
    double thrust = 100.0;
    double m = 5.0;

    void operator()(const state& x, state& dx, const double t)
    {
        double pos = x[0];
        double vel = x[1];
        double F = 0.0;

        if (t < 5.0)
        {
            F = thrust;
        }
        else
        {
            F = 0.0;
        }
        double acc = F / m - 9.80665;

        dx[0] = vel;
        dx[1] = acc;
    };

};


struct RocketObserver : public Rocket
{
    using state = Rocket::state;
    void operator()(const state& x, double t)
    {
        std::cout << t << ',' << thrust << ',' << x[0] << ',' << x[1] << std::endl;
    };
};




int main()
{
    Rocket rocket;
    Rocket::state state0 = {0.0, 0.0};
    odeint::runge_kutta4<Rocket::state> stepper;
    RocketObserver observer;
    odeint::integrate_const(stepper, rocket, state0, 0.0, 25.0, 0.1, std::ref(observer));
    // odeint::integrate_const(stepper, rocket, state0, 0.0, 25.0, 0.1);
    std::cout << "time=25 " << "x=" << state0[0] << " " << "y=" << state0[1] << std::endl;
    return 0;
};


// /////////////////////////////////////////////////////////
// struct lv_system {
// public:
//    using state = std::array<double, 2>;
// public:
//    double alpha;
//    double beta;
//    double gamma;
//    double delta;
// public:
//    lv_system(double alpha_,double beta_,double gamma_,double delta_)
//       :alpha(alpha_),beta(beta_),gamma(gamma_),delta(delta_){}

//    void operator()(const state& x, state& dx, double t){
//       dx[0] = x[0] * (alpha - beta* x[1]);
//       dx[1] = - x[1] * (gamma - delta * x[0]);
//    }
// };

// struct csv_observer{
//     using state = lv_system::state;
//     std::ofstream fout;
//     csv_observer(const std::string& FileName) :fout(FileName){};
//     void operator()(const state& x, double t)
//     {
//         fout << t << "," << x[0] << "," << x[1] << std::endl;
//     }
// };

// int main(){
//     lv_system System(2.0, 3.0, 4.0, 5.0);
//     lv_system::state State = {1.0,0.5};

//     std::cout << "time=0 " << "x=" << State[0] << " " << "y=" << State[1] << std::endl;

//     //オイラー法を使ってみる
//     boost::numeric::odeint::euler<lv_system::state> Stepper;
//     csv_observer Observer("result1.csv");
//     boost::numeric::odeint::integrate_const(Stepper, System, State, 0.0, 5.0, 0.05, std::ref(Observer));

//     std::cout << "time=10 " << "x=" << State[0] << " " << "y=" << State[1] << std::endl;
// }

// ///////////////////////////////////////////////////////////////////
// using namespace boost::numeric::odeint;
// using state_type = std::array<double,3>;
// const double p = 10;
// const double r = 28;
// const double b = 8./3;

// void lorenz_system(const state_type& x, state_type& dxdt, const double t)
// {
//     // x=x[0], y=x[1], z=x[2]
//     dxdt[0] = -p*x[0] + p*x[1];              // dx/dt
//     dxdt[1] = -x[0]*x[2] + r*x[0] - x[1] ;   // dy/dt
//     dxdt[2] = x[0]*x[1] - b*x[2];            // dz/dt
// }

// int main() {
//     std::vector<double> timelog;        // ☆
//     std::vector<state_type> statelog;   // ☆

//     state_type state0= { 10, 1, 1 };
//     integrate(lorenz_system, state0, 0., 25., 0.01,
//         [&](const state_type& state, const double t)
//         {
//             timelog.push_back(t);
//             statelog.push_back(state);
//         }
//     );

//     //  ☆ バイナリで出力
//     std::ofstream ofs("lorenz.dat",std::ios::binary);
//     const size_t N = timelog.size();
//     ofs.write(reinterpret_cast<const char*>(&N),    sizeof(N));
//     ofs.write(reinterpret_cast<char*>(&timelog[0]), sizeof(double)*N);
//     ofs.write(reinterpret_cast<char*>(&statelog[0]),sizeof(state_type)*N);
//     return 0;
// }

// ///////////////////////////////////////////////
// namespace odeint = boost::numeric::odeint;
// using state_type = std::array< double, 1 >;
// auto exponential = [](const state_type &x, state_type &dxdt, const double /* t */) {dxdt[ 0 ] = x[ 0 ];};
// auto x0 = state_type{ 1.0 }; // 初期状態
// auto t0 = 0.0; // 開始パラメータ
// auto t1 = 1.0; // 終了パラメータ
// auto dt = 0.1; // ステップ
// auto stepper = odeint::runge_kutta4< state_type >();
// odeint::integrate_const(stepper, exponential, x0, t0, t1, dt,
//   [](const state_type &x, const double t)
//   {
//     std::cout << t << "\t"
//               << x[ 0 ] << std::endl;
//   } 
// );