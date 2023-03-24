
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <math.h>
#include <iostream>
#include <string> // header file for string

using namespace std;
class _Serial{
    public:
    char debug = 0;
    const char* dev;
    int FD=0;
    _Serial(const char* device){
        this->dev = device;
    }
    int begin(int baudrate){
        this->FD = serialOpen (this->dev, baudrate); 
        return this->FD;
    }
    int available(){
        return serialDataAvail (FD) ;
    }
    void print(const char* str){
        this->_debug(str);
        serialPuts (FD, str);  
    }
    void println(const char* str){
        this->print(str);
        this->print("\n") ;  
    }
    void print(float i, unsigned int precision){
        if (precision >= 10)precision=9;
        string str = to_string(i);
        string rounded = str.substr(0, str.find(".")+precision);
        this->print(rounded.c_str());
    }
    void println(float i, unsigned int precision){
        this->print(i,precision);
        this->print("\n");
    }
    void print(int i){
        string str = to_string(i);
        this->print(str.c_str());       
    }
    void println(int i){
        string str = to_string(i);
        this->println(str.c_str());    
    }
    private:
        void _debug(const char* str){
            if (debug){
                cout << str; 
            }
        }

};
