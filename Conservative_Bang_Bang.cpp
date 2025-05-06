#include "mbed.h"
#include <QEI.h>


class bmotor {
    protected:
        DigitalOut b;
        PwmOut p;
    public:
        bmotor(PinName bipolar, PinName pwmpin) : b(bipolar), p(pwmpin){set();} 
        void set() {
            b = 1;
            p.period_us(100); // 10kHz
            off();
        }
        void off() {
            p.write(0.5);
        }
        void on(float s) {
            p.write(s/200 + 0.5);
        }
        void go(float s) {
            p.write(s);
        }
};//on(s) -> -100 <= s <= 100
class encoder {
    protected:
        QEI enc;
        Timer timer;
        int prevPulses;
        float distance, speed;
    public:
        encoder(PinName pinA, PinName pinB) : enc(pinA, pinB, NC, 512), timer() {
            timer.start();
            enc.reset();
            distance = 0.0f;
            speed = 0.0f;
        }
        void clear() {
            enc.reset();
        }
        int readPul() {
            return enc.getPulses();
        }
        int readRev() {
            return enc.getRevolutions();
        }
        bool enctimer(int t) {
            bool pass = false;
            if (t <= abs(enc.getPulses())) {
                pass = true;
            }
            return pass;   
        }
};

class atcrt {
    protected:
        DigitalOut led;
        AnalogIn reads;
        float v, VDD, vlim;
        bool w;
    public: 
        atcrt(PinName l, PinName r, float vdd, float limit) : led(l), reads(r), vlim(limit), VDD(vdd){off();}
        void off() {
            led = 0;
        }
        void on() {
            led = 1;
        }
        void readSensor() {
            on();
            wait_ms(1);
            v = reads.read();
            v *= VDD;
            if (v >= vlim) w = true; //on the line if >=3.3V
            else w = false;
            off();
        }
        float reading() {
            readSensor();
            return v;
        }
        float initialReading(){
            vlim = reads.read() + 0.5f;
            return vlim;
        }
        bool white() {
            readSensor();
            return w;
        }
};

//stateISR
    typedef enum {straight, lturn, rturn} ProgramState; ProgramState state;
    void straightISR() {state = straight;}
    void lturnISR() {state = lturn;}
    void rturnISR() {state = rturn;}
//hm10 setting
    Serial hm10(PA_11, PA_12);
    DigitalOut led(LED2);
    
//component setting
    DigitalOut enable(PC_11);
    bmotor l(PD_2, PB_1), r(PA_13, PA_15);
    DigitalOut ld(PB_2), rd(PA_14);
    //umotor left(PD_2, PB_2, PB_1), right(PA_13, PA_14, PA_15);
    encoder lenc(PB_8, PB_9), renc(PC_12, PC_10);
    atcrt ls(PB_5, PC_2, 3.3, 0.9), rs(PB_14, PC_3, 3.3, 0.9); 
    atcrt mid(PB_15, PC_1, 3.3, 1.0); //bb
    
    
// variable setup for ISRs
bool start = false;
bool end = false;
Ticker ticker_pulses;               
Ticker ticker_control;
Ticker ticker_updateSpeed;
Ticker ticker_lcontrol;
Ticker ticker_rcontrol;
int curr = 0;
int prevr = 0;
int curl = 0;
int prevl = 0;
int pl = 0;
int pr = 0;
//float speedl = 0.0f;
//float speedr = 0.0f;
float lpow = 30.0f;
float rpow = 30.0f;
bool linefound = true;
Timeout lineEnd;

//updates pulses and calculates speed according to a 0.01 second interval
void pulsesISR()                        
{
    prevl = curl;
    prevr = curr;
    curl = lenc.readPul();
    curr = renc.readPul(); 
    pl = curl - prevl;
    //speedl = (0.0804 * 3.14159 * pl)/2.56;
    pr = curr - prevr;
    //speedr = (0.0804 * 3.14159 * pr)/2.56;
}

//updates speed of motors according to the values of lpow and rpow which is determined by the speed control ISRs and mainline code 
void updateSpeedISR()
{
   l.on(lpow), r.on(rpow);
}

// speed control increases/decreases depending on speed of each wheel only 1 should be active at a time 
void controlISR() //for straight lines 
{
        if (pl < 19){        //pl and pr is the number of pulses in the time interval set
             lpow += 1;      //by ticker_pulses
        }
        else if (pl > 21){
             lpow -= 1;
        }
        if (pr < 19) {
             rpow += 1;
        }
        else if (pr > 21){
             rpow -= 1;
        }
}

void lcontrolISR() // for left turns 
{
        if (pl < 19){
            rpow += 1;              //lpow is set according to the value of rpow during lturn
        }                           //so rpow has to be increased or decreased
        else if (pl > 21){           
            rpow -= 1;
        }     
}
void rcontrolISR() //for right turns 
{
        if (pr < 19){               //same as lcontrolISR but for rturn
             lpow += 1;
        }
        else if (pr > 21){
             lpow -= 1;
        }      
}
void endCheck() //check whether it's a line break or end of line
{
if (state == lturn){       //this might happen when the buggy doesn't turn sharp enough to the left
    l.go(-30), r.on(30);   //and loses the line because of that so the buggy will now turn aggressively 
                           //to see if it catches the line again
}
else if (state == rturn){  //as above but for right
    l.on(30), r.on(-30);
}
else if (state == straight){     //to slow the buggy down in case it is the end of the line 
    ticker_control.detach();     //mainly here for higher speeds
    l.on(25), r.on(25);
}
while (linefound == false){        //loops through the 3 sensors and if any detect the line sets linefound
    if (mid.white() == true){      //as true so the timeout function has no effect     
        linefound = true;          
        straightISR();
    }
    else if (rs.white() == true){
        linefound = true;
        rturnISR();
    }
    else if (ls.white() == true){
        linefound = true;
        lturnISR();
    }
}
}
void lineE() //stops buggy if line end
{
    if (linefound == false){        //if the line wasn't found during endcheck before this was called 
       enable = 0;                  //the buggy will stop
       end = true;
    } 
}

void check() //checks if a sensor is reading a line then sends it to the appropriate state 
{
    if (mid.white() == true){          //middle sensor first as the other 2 will be detected at this point 
        straightISR();                 //so would never enter the straight line code otherwise 
    }                                  //also keeps buggy speed higher
    else if (rs.white() == true){
        rturnISR();
    }
    else if (ls.white() == true){
        lturnISR();
    }
    else {
        linefound = false;
        lineEnd.attach(&lineE, 0.3);  
        endCheck();
        //timeout 0.3s seperate function to repeatedly check for line and set bool to true or false
        //then depending whether value is true or false return to mainline or ending at timeout
    }

}
void turnaround(){
                //this is deliberately intensive as finding the line again for the turnaround is the only thing that matters when this is called
                ticker_control.detach(), ticker_rcontrol.detach(), ticker_lcontrol.detach(), ticker_updateSpeed.detach(); //otherwise speed control might interfere with the turn 
                l.on(30), r.on(-30);
                bool one80 = false;
                wait(0.5); //needed so the sensor clears the initial white line 
                while (one80 == false){
                    if (mid.white() == true){   //resumes straight line code when middle sensor detects the line again 
                        one80 = true;
                        ticker_updateSpeed.attach(&updateSpeedISR,0.02); //reattaches speed ticker for mainline code
                        straightISR();
                    }
                }
}
void BLEISR() //interrupt code for BLE commands
{
if (hm10.readable()) {
            char a = hm10.getc();
            if(a == 't'){
                //turnaround COMMAND RECEIVED
                turnaround(); 
            }
    }
}


int main() {
    //initial ticker setup
    ticker_pulses.attach(&pulsesISR,0.02);       //the 3 tickers work together so should be set to the same time 
    ticker_control.attach(&controlISR,0.02);     //also true for lcontrol and rcontrol
    ticker_updateSpeed.attach(&updateSpeedISR,0.02);
    //hm10 setup
    hm10.attach(&BLEISR, Serial::RxIrq); //interrupts when BLE command is recieved
    hm10.baud(9600);



            //motor setup
            enable = 1;
            l.set(), r.set();
            l.on(lpow);
            r.on(rpow);
    //state machine
    end = false;
    while (!end) {
        switch (state) {
            default : 
                
            case (straight) :
                ticker_rcontrol.detach(),ticker_lcontrol.detach(), ticker_control.attach(&controlISR,0.02); //ensures only the straight line speed control is active 
                while (state == straight) {
                    //sets motor power equal for after a turn
                    if (lpow > rpow){
                        rpow = lpow;
                    }
                    else {
                        lpow = rpow;
                    }
                    check();
                 
                }
                break;
            case (lturn) :
                ticker_control.detach(), ticker_rcontrol.detach(), ticker_lcontrol.attach(&lcontrolISR,0.02);   //sets up left speed control and turns off the others 
                while (state == lturn) {
                    lpow = rpow * 0.25;  //sets left motor pwm proportionally to the right motor which is adjusted with control ISRs
                    check();
                }
                break;
            case (rturn) :
                ticker_control.detach(), ticker_lcontrol.detach(), ticker_rcontrol.attach(&rcontrolISR,0.02);  //sets up right speed control and turns off the others 
                while (state == rturn) {
                    rpow = lpow * 0.25;   //sets right motor pwm proportionally to the left motor which is adjusted with control ISRs
                    check();   
                }
                break;
    }

} //task: pid, main code
}
