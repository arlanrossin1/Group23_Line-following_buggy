#include "mbed.h"
#include <QEI.h>
class motor {
    protected:
        DigitalOut b, d;
        PwmOut p;
    public:
        motor(PinName bipolar, PinName direction, PinName pwmpin) : 
        b(bipolar), d(direction), p(pwmpin){set();} 
        void set() {
            b = 1;
            d = 0;
            p.period_us(100); // 10kHz    
            p.write(0.5);
        }
        void off() {p.write(0.5);}
        void on(float s) {p.write(s/200 + 0.5);}
    };//on(s) -> -100 <= s <= 100
class tcrt {  
    protected:
        DigitalOut tled;
        AnalogIn reads;
        float v, vlim;
        bool w;
    public: 
        tcrt(PinName l, PinName r, float limit) : 
        tled(l), reads(r), vlim(limit){tled = 1;}
        float reading() {
            wait_us(150);
            v = reads.read();
            return v;
        }
        bool white() {
            if (v > vlim) w = true; //on the line if >vlim
            else w = false;
            return w;
        }
    };
//variables
    bool tf = false;
    int c = 0;
//component setting
    DigitalOut enable(PC_11);
    motor l(PD_2, PB_2, PB_1);
    motor r(PA_13, PA_14, PA_15);
    QEI lenc(PB_8, PB_9, NC, 512);
    QEI renc(PC_12, PC_10, NC, 512);
    tcrt ls(PB_5, PC_2, 0.4);
    tcrt mid(PB_15, PC_1, 0.45);
    tcrt rs(PB_14, PC_3, 0.4);
    Serial hm10(PA_11, PA_12);
//motor control
    Ticker pid, toggle;
    float kp = 0.5, ki = 0.037, kd = 14, po = 70, attacht = 0.001;
    double e = 0, ep = 0, ep1 = 0, ep2 = 0, ep3 = 0, ep4 = 0, ep5 = 0, ep6 = 0, ep7 = 0, ep8 = 0;
    double et = 0, u = 0, lf = 0, rf = 0;
    void pidISR(){
        ep8 = ep7, ep7 = ep6, ep6 = ep5, ep5 = ep4, ep4 = ep3;
        ep3 = ep2, ep2 = ep1, ep1 = ep, ep = e;
        e = (ls.reading() - rs.reading())*(1.34 - 0.3*mid.reading());
        et += e;
        et -= ep8;
        u = kp*e + ki*et + kd*(e-ep);
        if (u > 0) {
            lf = (1-u); 
            rf = 1;
        }
        else {
            lf = 1; 
            rf = (1+u);
        }
        l.on(po*lf);
        r.on(po*rf);
    }
//hm10 setting
    void BLEISR() {
        if (hm10.readable()) {
            char a = hm10.getc();
            if (a == 't') tf = true;
            if (a == 'b') po = -20;
            if (a == 'o') po = 0;
            if (a == 's') po = 30;
            if (a == 'f') po = 70;
        }
    }
int main() {
    hm10.attach(&BLEISR, Serial::RxIrq);
    hm10.baud(9600);
    enable = 1;
    pid.attach(&pidISR, attacht);
    while (!tf) wait_us(2);
    pid.detach();
    //stop
    l.on(-100); 
    r.on(-100); 
    wait_ms(1);
    l.off(); 
    r.off(); 
} 
