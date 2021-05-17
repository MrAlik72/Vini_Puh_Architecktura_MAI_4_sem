#include <iostream>
#include <cmath>
#include <future>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <mutex>
// Управление ПУхом через сокеты. 
using namespace std;

class PIDImpl
{
public:
    // Kp - proportional gain
    // Ki - Integral gain
    // Kd - derivative gain
    // dt - loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki);
    ~PIDImpl();
    // Returns the manipulated variable given a setpoint and current process value
    double calculate(double setpoint, double pv);

private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};

/**
* Implementation
*/
PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki) : _dt(dt),
                                                                                       _max(max),
                                                                                       _min(min),
                                                                                       _Kp(Kp),
                                                                                       _Kd(Kd),
                                                                                       _Ki(Ki),
                                                                                       _pre_error(0),
                                                                                       _integral(0)
{
}

double PIDImpl::calculate(double setpoint, double pv)
{

    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    //std::cout<<"out="<<output<< "Pout = "<< Pout <<" " <<Iout <<" "<< Dout <<std::endl;
    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    std::cout << "outB=" << output << std::endl;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

enum ECmd
{
    ENone,
    EExit
};

ECmd cmd = ENone;

int counter = 0;

std::mutex mtx;

float targetH = 10.;

float getHZadFromUser()
{
    return 10.;
}

class Bear
{
    float h;
    float m;
    float v;
    float a;

public:
    Bear()
    {
        v = 0;  // m/s
        a = 0;  // m/s2
        m = 10; // kg
        h = 0;  // Текущая высота
    }
    float getH()
    {
        return h;
    }
    float getV()
    {
        return v;
    }
    float getM()
    {
        return m;
    }
    float getA()
    {
        return a;
    }
    virtual void calc(float dt, float extrnForce)
    {
        float dh;
        v = v + a * dt;
        dh = v * dt;
        h = h + dh;
        a = (extrnForce - 9.8 * m) / m; // m * a = mg - F

        if (h < 0)
        { // Stay on the ground
            h = 0.;
            v = 0.;
            a = 0.;
        }
    }

    void eat(float mass)
    {
        m = m + mass;
    }

    virtual void show()
    {
        std::cout << "h=" << h << std::endl;
    }
};

class Engine;

class Engine
{
    float enginePower = 0; // n 0..500 n
    float maxPower;        // n
public:
    Engine(float imaxPower)
    {
        enginePower = 0.;
        maxPower = imaxPower;
    }

    void setPower(float enginePowerPercent)
    {
        if (enginePowerPercent > 100.)
            enginePowerPercent = 100;
        if (enginePowerPercent < -100.)
            enginePowerPercent = -100;

        enginePower = enginePowerPercent * (maxPower / 100.);
    }

    float getForce()
    {
        return enginePower;
    }
};

class SmartFlyingBear : public Bear
{
    PIDImpl *pid;
    Engine *pengine;
    float hZad;

public:
    SmartFlyingBear(PIDImpl *ppid, Engine *ppengine)
    {
        pid = ppid;
        pengine = ppengine;
        hZad = 0;
    }

    void setHZad(float val)
    {
        hZad = val;
    }

    virtual void calc(float dt, float extrnForce)
    {
        float enginePowerPercent = 0;
        if (pid != nullptr)
        {
            enginePowerPercent = pid->calculate(hZad, getH());
        }

        if (pengine != nullptr)
        {
            pengine->setPower(enginePowerPercent);
            Bear::calc(dt, extrnForce + pengine->getForce());
        }
    }
};

int model()
{

    //PIDImpl pid(0.1, 100, -100, 0.05, 0.001, 0.1);
    //Bear pooh;
    SmartFlyingBear superpooh(new PIDImpl(0.1, 100, -100, 0.15, 0.001, 0.01), new Engine(500.));

    float t = 0;    // Время моделирования
    float dt = 0.1; // s
    float lastT = -5;

    float hZad = 0; // Измененение высоты за шаг моделирования

    while (cmd != EExit)
    {
        mtx.lock();
        hZad = targetH; // Неблокирующая функция для получения высоты
        mtx.unlock();
        if (abs(superpooh.getH() - hZad) < 1 && abs(t - lastT) > 5)
        {
            superpooh.eat(0.1);
            lastT = t;
        }
        superpooh.setHZad(hZad);
        superpooh.show();
        superpooh.calc(dt, 0);

        t = t + dt;

        if (t < 200)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else
        {
            std::cout << "Масса пуха = " << superpooh.getM() << std::endl;
            std::cout << "Скорость пуха = " << superpooh.getV() << std::endl;
            std::cout << "Ускорение пуха= " << superpooh.getA() << std::endl;
            std::cout << "Тик Так часики = " << t << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return 0;
}

int main()
{
    try
    {
        boost::asio::io_context io_context;
        boost::asio::ip::udp::endpoint reciever(boost::asio::ip::udp::v4(), 9889);
        boost::asio::ip::udp::socket socket(io_context, reciever);

        std::thread th(model);
        while (cmd != EExit)
        {
            boost::array<char, 1> recv_b;
            boost::asio::ip::udp::endpoint remote_endpoint;
            size_t len = socket.receive_from(boost::asio::buffer(recv_b), remote_endpoint);
            std::cout.write(recv_b.data(), len);
            if (*recv_b.data() == 'q') // Ты зашел не в тот улей. Напишите уравнение гравитационного поля или программа уничтожит ваш кумпутер.
            {
                std::cout << "Спасибо! И до скорых встреч." << std::endl;
                cmd = EExit;
                th.join();
            }
            if (*recv_b.data() == 'd') // меняет целевую высоту на нуль
            {
                std::cout << "Мордой в пол, нахальный медведь. Черная дыра по курсу!" << std::endl;
                mtx.lock();
                targetH = 0.;
                mtx.unlock();
            }
            if (*recv_b.data() == 'u') // Подъем чувак. Пора в космос 
            {
                std::cout << "Пора на геостационарну орбиту! Подъем до 35 786 км" << std::endl;
                mtx.lock();
                targetH = 35786000. ;
                mtx.unlock();
            }
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    return 0;
} 