#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <cpprest/http_listener.h>
#include <cpprest/filestream.h>
#include <cpprest/uri.h>
#include <cpprest/json.h>

using namespace utility;                           // Common utilities like string conversions
using namespace web;                               // Common features like URIs.
using namespace web::http;                         // Common HTTP functionality
using namespace web::http::experimental::listener; // HTTP listener
using namespace concurrency::streams;              // Asynchronous streams

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
PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki)
    : _dt(dt), _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0), _integral(0)
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

    //std::cout << "outB=" << output << std::endl;

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
    unique_ptr<PIDImpl> pid;
    unique_ptr<Engine> pengine;
    float hZad;

public:
    SmartFlyingBear(PIDImpl *ppid, Engine *ppengine) : pid(ppid), pengine(ppengine)
    {
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
            std::cout << "Масса пуха = = " << superpooh.getM() << std::endl;
            std::cout << "Скорость пуха = " << superpooh.getV() << std::endl;
            std::cout << "Ускорение пуха = " << superpooh.getA() << std::endl;
            std::cout << "Тик Так часики= " << t << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return 0;
}

void handle_signal(int s)
{
    cmd = EExit;
}

bool is_authentificated(http_request message)
{
    auto headers = message.headers();
    if (message.headers().find("Authorization") == headers.end())
        return false;
    auto authHeader = headers["Authorization"];
    auto credsPos = authHeader.find("Basic");
    if (credsPos == std::string::npos)
        return false;

    auto base64 = authHeader.substr(credsPos + std::string("Basic").length() + 1);
    if (base64.empty())
        return false;
    auto bytes = utility::conversions::from_base64(base64);
    std::string creds(bytes.begin(), bytes.end());
    auto colonPos = creds.find(":");
    if (colonPos == std::string::npos)
        return false;
    auto user = creds.substr(0, colonPos);
    auto password = creds.substr(colonPos + 1, creds.size() - colonPos - 1);

    if (user == "POOH" && password == "MED")
    {
        return true;
    }
    else
    {
        return false;
    }
}

void handle_get(http_request message)
{
    cout << "OBRABOTKA: " << message.to_string() << endl;
    if (is_authentificated(message))
    {
        json::value jsonObject;
        jsonObject[U("target_Visota")] = json::value::number(targetH);
        message.reply(status_codes::OK, jsonObject);
    }
    else
    {
        message.reply(status_codes::Forbidden);
    }
}

void handle_post(http_request message)
{
    cout << "Obrabotka post: " << message.to_string() << endl;

    if (is_authentificated(message))
    {
        json::value jsonObject;
        try
        {
            message.extract_json()
                .then([&jsonObject](json::value jo) {
                    cout << "Val:" << jo.serialize() << endl;
                    jsonObject = jo;
                    const lock_guard<mutex> lock(mtx);
                    targetH = jsonObject.at(U("target_Visota")).as_number().to_double();
                    cout << "Val:" << targetH << endl;
                })
                .wait();
        }
        catch (const exception &e)
        {
            printf("Error exception:%s\n", e.what());
        }
        message.reply(status_codes::OK, jsonObject);
    }
    else
    {
        message.reply(status_codes::Forbidden);
    }
}

void handle_quit(http_request message)
{
    cout << "obrabotka quit: " << message.to_string() << endl;
    if (is_authentificated(message))
    {
        cmd = EExit;
        message.reply(status_codes::OK);
    }
    else
    {
        message.reply(status_codes::Forbidden);
    }
}

int main()
{

    int rc = 0;

    web::http::experimental::listener::http_listener
        listener(U("http://localhost:8080/"));
    listener.support(methods::GET, handle_get);
    listener.support(methods::POST, handle_post);
    listener.support(methods::DEL, handle_quit);

    try
    {
        std::thread th(model);
        listener.open()
            .then([&listener]() { printf("\nStarting svrRest\n"); })
            .wait();
        while (cmd != EExit)
            ;
        th.join();
    }
    catch (const std::exception &e)
    {
        printf("Error exception:%s\n", e.what());
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    return rc;
}