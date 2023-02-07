#include <iostream>
#include <map>
#include <cstdlib>
using namespace std;

//class delaration for RcBrainConfigParams
class RcBrainConfigParams{
    public: 
        //member variables
        double maxSteerAngle;
        double maxSpeed;
        double steerAngleStep;
        double speedStep;
        double kpStep;
        double kiStep;
        double kdStep;
        //constructor
        RcBrainConfigParams(double maxSteerAngle,double maxSpeed,double steerAngleStep,double speedStep,double kpStep,double kiStep,double kdStep);
};
//implementation of RcBrainConfigParams class
RcBrainConfigParams::RcBrainConfigParams(double maxSteerAngle,double maxSpeed,double steerAngleStep,double speedStep,double kpStep,double kiStep,double kdStep){
    this->maxSteerAngle = maxSteerAngle;
    this->maxSpeed = maxSpeed;
    this->steerAngleStep = steerAngleStep;
    this->speedStep = speedStep;
    this->kpStep = kpStep;
    this->kiStep = kiStep;
    this->kdStep = kdStep;
}

class RcBrainThread{
    public:
        double speed;
        double steerAngle;
        bool pida;
        double pids_kp;
        double pids_ki;
        double pids_kd;
        double pids_tf;
        double parameterIncrement;
        RcBrainConfigParams* limit_configParam;
        double startSpeed;
        double startSteerAngle;
        RcBrainConfigParams* default_configParam;
        RcBrainConfigParams* configParam;
        bool currentState[8]  = {false,false,false,false,false, false, false, false};
        //inite constructor
        RcBrainThread();
        //public methods:
        int displayInfo();
        int _stateDict(map<string,string>* data);
        map<string,string> getMessage(string key);
        int _updateSpeed();
        int _updateSteerAngle();
        int _updateParameters(string currentKey);
        int _updatePID(string currentKey);
        int _updateMotionState(string currentKey); 
};

RcBrainThread::RcBrainThread(){
    this->speed = 0.0;
    this->steerAngle = 0.0;
    this->pida = false;
    this->pids_kp = 0.115000;
    this->pids_ki = 0.810000;
    this->pids_kd = 0.000222;
    this->pids_tf = 0.040000;
    //this values do not change
    this->parameterIncrement = 0.1;
    this->limit_configParam = limit_configParam = new RcBrainConfigParams(21.0, 30.0, 3.0, 4.0, 0.001, 0.001, 0.000001);
    this->startSpeed = 9.0;
    this->startSteerAngle = 1.0;
    cout<<"new RcBrainConfig, free after finish"<<"\n";
    this->default_configParam = new RcBrainConfigParams(20.5,20.0,1.5,2.0, 0.001, 0.001, 0.000001);
    this->configParam = new RcBrainConfigParams(*(this->default_configParam));//deep copy
}

int RcBrainThread::displayInfo(){
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif
    
    string info = 
            "speed:          "  + to_string(this->speed) +                     "[W/S]" +
            "\nangle:         " + to_string(this->steerAngle) +                "[A/D]" +
            "\npid:           " + to_string(this->pida) +                      "[P]"   +
            "\npid KP:        " + to_string(this->pids_kp) +                   "[Z/X]" +
            "\npid KI:        " + to_string(this->pids_ki) +                   "[V/B]" +
            "\npid KD:        " + to_string(this->pids_kd) +                   "[N/M]" +
            "\nmaxSpeed :     " + to_string(this->configParam->maxSpeed) +      "[T/G]" +
            "\nmaxSteerAngle: " + to_string(this->configParam->maxSteerAngle) + "[Y/H]" +
            "\nacceleration:  " + to_string(this->configParam->speedStep) +     "[U/J]" +
            "\nsteerStep:     " + to_string(this->configParam->steerAngleStep) +"[I/K]" +
            "\nReset Params:  [R]" +
            "\nExit           [Esc]"
    ;
    cout << info << "\n";
    return 0;
}

int RcBrainThread::_stateDict(map<string,string>* data){
    map<string,string> data;
    if(this->currentState[4]){
        (*data)["action"] = "3";
        (*data)["steerAngle"] = to_string((double)this->steerAngle);
    }else if(this->currentState[0] || this->currentState[1]){
        (*data)["action"]        =  "1";
        (*data)["speed"]         =  to_string((double)(this->speed/100.0));
    }else if(this->currentState[2] || this->currentState[3]){
        (*data)["action"]        =  "2";
        (*data)["steerAngle"]    =  to_string((double)this->steerAngle);
    }else if(this->currentState[5]){
        (*data)["action"] = "4";
        (*data)["activate"] = this->pida;
        this->currentState[5] = false;
    }else if(this->currentState[6]){
        (*data)["action"] = "6";
        (*data)["kp"] = to_string(this->pids_kp);
        (*data)["ki"] = to_string(this->pids_ki);
        (*data)["kd"] = to_string(this->pids_kd);
        (*data)["tf"] = to_string(this->pids_tf);
        this->currentState[6] = false;
    }else if(this->currentState[7]){
        (*data)["action"] = "2";
        (*data)["steerAngle"] = to_string(0.0);
        this->currentState[7] = false;
    }else{
        data = nullptr;
    }
    return 0;
}

int RcBrainThread::_updateSpeed(){
    if (this->currentState[4]) {
        this->currentState[0] = false;
        this->currentState[1] = false;
        this->speed = 0;
        return;
    }

    //forward
    if (this->currentState[0]) {
        if (this->speed == 0) {
            this->speed = this->startSpeed;
        } else if (this->speed == -this->startSpeed) {
            this->speed = 0;
        } else if (this->speed < this->configParam->maxSpeed) {
            if (this->configParam->maxSpeed - this->speed < this->configParam->speedStep) {
                this->speed = this->configParam->maxSpeed;
            } else {
                this->speed += this->configParam->speedStep;
            }
        }
    }
    //backwards
    else if (this->currentState[1]) {
        if (this->speed == 0) {
            this->speed = -this->startSpeed;
        } else if (this->speed == this->startSpeed) {
            this->speed = 0;
        } else if (this->speed > -this->configParam->maxSpeed) {
            if (abs(this->configParam->maxSpeed + this->speed) < this->configParam->speedStep) {
                this->speed = -this->configParam->maxSpeed;
            } else {
                this->speed -= this->configParam->speedStep;
            }
        }
    }
    return 0;
}

int RcBrainThread::_updateSteerAngle(){
    if (currentState[2]) {
        if (steerAngle == 0) {
            steerAngle = -startSteerAngle;
        }
        else if (steerAngle > -this->configParam->maxSteerAngle) {
            if (this->configParam->maxSteerAngle + steerAngle < this->configParam->steerAngleStep) {
                steerAngle = -this->configParam->maxSteerAngle;
            }
            else {
                steerAngle -= this->configParam->steerAngleStep;
            }
        }
    }

    if (currentState[3]) {
        if (steerAngle == 0) {
            steerAngle = startSteerAngle;
        }
        else if (steerAngle < this->configParam->maxSteerAngle) {
            if (this->configParam->maxSteerAngle - steerAngle < this->configParam->steerAngleStep) {
                steerAngle = this->configParam->maxSteerAngle;
            }
            else {
                steerAngle += this->configParam->steerAngleStep;
            }
        }
    }

    if (!currentState[2] && !currentState[3]) {
        steerAngle = 0;
    }
    return 0;
}

int RcBrainThread::_updateParameters(string currentKey){
    if (currentKey == "p.r") {
        speed = 0.0;
        steerAngle = 0.0;
        configParam = default_configParam;
    }
    //--------------- MAX SPEED ------------------------------
    else if (currentKey == "p.t") {
        if (this->configParam->maxSpeed < this->limit_configParam->maxSpeed) {
            this->configParam->maxSpeed += parameterIncrement;
        }
    } else if (currentKey == "p.g") {
        if (startSpeed < this->configParam->maxSpeed) {
            this->configParam->maxSpeed -= parameterIncrement;
        }
    }
    //--------------- MAX STEER ANGLE ------------------------
    else if (currentKey == "p.y") {
        if (this->configParam->maxSteerAngle < this->limit_configParam->maxSteerAngle) {
            this->configParam->maxSteerAngle += parameterIncrement;
        }
    } else if (currentKey == "p.h") {
        if (startSteerAngle < this->configParam->maxSteerAngle) {
            this->configParam->maxSteerAngle -= parameterIncrement;
        }
    }
    //--------------- SPEED STEP ------------------------------
    else if (currentKey == "p.u") {
        if (this->configParam->speedStep < this->limit_configParam->speedStep) {
            this->configParam->speedStep += parameterIncrement;
        }
    } else if (currentKey == "p.j") {
        if (0.1 < this->configParam->speedStep) {
            this->configParam->speedStep -= parameterIncrement;
        }
    }
    //--------------- STEER STEP ------------------------------
    else if (currentKey == "p.i") {
        if (this->configParam->steerAngleStep < this->limit_configParam->steerAngleStep) {
            this->configParam->steerAngleStep += parameterIncrement;
        }
    } else if (currentKey == "p.k") {
        if (0.1 < this->configParam->steerAngleStep) {
            this->configParam->steerAngleStep -= parameterIncrement;
        }
    }
    return 0;
}

int RcBrainThread::_updatePID(string currentKey){
    if (currentKey == "p.p") {
        pida = !pida;
        currentState[5] = true;
    } else if (currentKey == "p.z") {
        pids_kp += this->configParam->kpStep;
        currentState[6] = true;
    } else if (currentKey == "p.x") {
        pids_kp -= this->configParam->kpStep;
        currentState[6] = true;
    } else if (currentKey == "p.v") {
        pids_ki += this->configParam->kiStep;
        currentState[6] = true;
    } else if (currentKey == "p.b") {
        pids_ki -= this->configParam->kiStep;
        currentState[6] = true;
    } else if (currentKey == "p.n") {
        pids_kd += this->configParam->kdStep;
        currentState[6] = true;
    } else if (currentKey == "p.m") {
        pids_kd -= this->configParam->kdStep;
        currentState[6] = true;
    }
    return 0;
}

int RcBrainThread::_updateMotionState(string currentKey){
    if (currentKey == "p.w") {
        currentState[0] = true;
    } else if (currentKey == "r.w") {
        currentState[0] = false;
    } else if (currentKey == "p.s") {
        currentState[1] = true;
    } else if (currentKey == "r.s") {
        currentState[1] = false;
    } else if (currentKey == "p.a") {
        currentState[2] = true;
    } else if (currentKey == "r.a") {
        currentState[2] = false;
        currentState[7] = true;
    } else if (currentKey == "p.d") {
        currentState[3] = true;
    } else if (currentKey == "r.d") {
        currentState[3] = false;
        currentState[7] = true;
    } else if (currentKey == "p.space") {
        currentState[4] = true;
    } else if (currentKey == "r.space") {
        currentState[4] = false;
    }
    return 0;
}

map<string,string> RcBrainThread::getMessage(string key){
    this->_updateMotionState(key);
    this->_updateSpeed();
    this->_updateSteerAngle();
    this->_updatePID(key);
    this->_updateParameters(key);
    this->displayInfo();
    map<string,string> m;
    this->_stateDict(&m);
    return m; 
}


