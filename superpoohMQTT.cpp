#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <mosquitto.h>
#include <mutex>
#include <thread>

#define mqtt_host "localhost"
#define mqtt_port 1883

using namespace std;

class PIDImpl {
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
    : _dt(dt)
    , _max(max)
    , _min(min)
    , _Kp(Kp)
    , _Kd(Kd)
    , _Ki(Ki)
    , _pre_error(0)
    , _integral(0)
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

enum ECmd {
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

class Bear {
    float h;
    float m;
    float v;
    float a;

public:
    Bear()
    {
        v = 0; // m/s
        a = 0; // m/s2
        m = 10; // kg
        h = 0; // Текущая высота
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

        if (h < 0) { // Stay on the ground
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

class Engine {
    float enginePower = 0; // n 0..500 n
    float maxPower; // n
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

class SmartFlyingBear : public Bear {
    PIDImpl* pid;
    Engine* pengine;
    float hZad;

public:
    SmartFlyingBear(PIDImpl* ppid, Engine* ppengine)
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
        if (pid != nullptr) {
            enginePowerPercent = pid->calculate(hZad, getH());
        }

        if (pengine != nullptr) {
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

    float t = 0; // Время моделирования
    float dt = 0.1; // s
    float lastT = -5;

    float hZad = 0; // Измененение высоты за шаг моделирования

    while (cmd != EExit) {
        mtx.lock();
        hZad = targetH; // Неблокирующая функция для получения высоты
        mtx.unlock();
        if (abs(superpooh.getH() - hZad) < 1 && abs(t - lastT) > 5) {
            superpooh.eat(0.1);
            lastT = t;
        }
        superpooh.setHZad(hZad);
        superpooh.show();
        superpooh.calc(dt, 0);

        t = t + dt;

        if (t < 200) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } else {
            std::cout << "Масса пуха =  " << superpooh.getM() << std::endl;
            std::cout << "Скорость пуха " << superpooh.getV() << std::endl;
            std::cout << "Ускорение пуха= = " << superpooh.getA() << std::endl;
            std::cout << "Тик Так часики = " << t << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    return 0;
}

void handle_signal(int s)
{
    cmd = EExit;
}

void message_callback(struct mosquitto* mosq, void* obj, const struct mosquitto_message* message)
{
    bool match = 0;
    printf("got message '%.*s' for topic '%s'\n", message->payloadlen, (char*)message->payload, message->topic);

    mosquitto_topic_matches_sub("/pooh/height", message->topic, &match);
    if (match) {
        std::cout << "Пора лететь, а полет будет жарким." << std::endl; // задает высоту 
        mtx.lock();
        targetH = std::atof((char*)message->payload);
        mtx.unlock();
    }

    mosquitto_topic_matches_sub("/pooh/land", message->topic, &match); //Садись пух, на землю мать
    if (match) {
        std::cout << "Садись, приземляйся, не бойся." << std::endl;
        mtx.lock();
        targetH = 0.;
        mtx.unlock();
    }

    mosquitto_topic_matches_sub("/pooh/quit", message->topic, &match); // Пока.
    if (match) {
        std::cout <<"Спасибо! И до скорых встреч." << std::endl;
        cmd = EExit;
    }
}

int main()
{

    char clientid[24];
    struct mosquitto* mosq;
    int rc = 0;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    mosquitto_lib_init();

    memset(clientid, 0, 24);
    snprintf(clientid, 23, "mysql_log_%d", getpid());
    mosq = mosquitto_new(clientid, true, 0);

    try {
        std::thread th(model);
        if (mosq) {
            mosquitto_message_callback_set(mosq, message_callback);

            rc = mosquitto_connect(mosq, mqtt_host, mqtt_port, 1555);

            mosquitto_subscribe(mosq, NULL, "/pooh/height", 0);
            mosquitto_subscribe(mosq, NULL, "/pooh/land", 0);
            mosquitto_subscribe(mosq, NULL, "/pooh/quit", 0);

            while (cmd != EExit) {
                rc = mosquitto_loop(mosq, -1, 1);
                if (cmd != EExit && rc) {
                    printf("Oshibka connecta!\n");
                    sleep(5);
                    mosquitto_reconnect(mosq);
                }
            }
            th.join();
            mosquitto_destroy(mosq);
        }
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    mosquitto_lib_cleanup();
    return rc;
}