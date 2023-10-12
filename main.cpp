#include "mbed.h"       //Libraries for basic mBed functions.
#include "TextLCD.h"    //Library to control LCD screen.
#include "time.h"       //Library to start a timer. 

//------------------PIN OUTS------------------

//Defines the digitial pins of the connections to the
//NUCLEO board. Specified by Univeristy of Bath.

//Bi-directional LED on PCB. 
DigitalOut LED_BIRA(PB_7);      
DigitalOut LED_BIRB(PA_15);

//Uni-directional LED on PCB
DigitalOut LED2(PC_0);     

//Button built into the NUCLEO board. Interrupt detects
//when it has been pressed. 
InterruptIn Change_Mode(BUTTON1);   

//Two digital channels on the rotary encoder (on PCB)
DigitalIn Rot_A(PA_4);
DigitalIn Rot_B(PA_1);

//Defines fan PWM pin (PCB) as a PWMOut signal.
PwmOut PWM_Sig(PB_0);

//Interrupt for fan tachometer signal (PCB).
InterruptIn Tach_Sig(PA_0);

//Pinout for the LCD screen. 
//Pins for data select, enable and data buses pins 4-7 specified.
TextLCD lcd(PB_15, PB_14, PB_10, PA_8, PB_2, PB_1); // rs, e, d4-d7

//------------------TIMERS------------------

//Timer used to track tachometer pulse times.
Timer t;

//Define tickets for timed ISR functions. 
Ticker Encoder_tick;            //For checking encoder values.
Ticker Closed_Loop_tick;        //For calculating closed_loop duty cycle.

//------------------CONSTANTS/DEPENDENCIES------------------

//Counter for encoder
//Has a limit of 100 to prevent user continously turning encoder and 
//to calculate desired speed of fan. 
int enc_count = 0;                          
const int Enc_Count_Upper_Limit = 100;      
   
//Desired and measured speed of the fan (RPM). Desired speed has a minimum speed of 400 
//as that was the lowest observed speed where closed loop control could be performed satifactorily.
//Maximum fan RPM observed was 1950 however this differed between 
//fans so lowered it to ensure functionality is consistent between different hardwares.
int desired_speed;
int current_speed;
const int Minimum_Speed = 400;
const int Maximum_Speed = 1900;

//Selected control mode to drive the fan.
//False = Open loop
//True  = Closed loop. 
bool Control_Mode = false;  

//PID constants used for closed-loop control. 
const float KP = 0.11;       
const float KI = 0.12;
const float KD = 0.05;
//Time in seconds for how often PID is recorded. 
const float PID_Time_Between = 0.020;
//Uses a scalar to turn the total PID signal into a duty cycle. 
const int PID_Scalar = 255;

//Duty cycle for closed loop control, intialised at 0. 
float closed_loop_duty_cycle = 0;  


//------------------FUNCTIONS------------------

void Encoder_Count()
{
    //Timed ISR (Ticker) that reads the current state of the encoder
    //and updates the counted value. Uses a finite state machine to 
    //perform X4 encoding. 
    //
    //Increments/decrements enc_count while keeping it between 0-100.

    //Tracks current state of encoder.
    static int current_state;       

    //Reads both binary states of the channels and calcaultes state integer.
    int A = Rot_A.read();           
    int B = Rot_B.read();           
    int next_state = (2 * A) + B;

    //If the state has changed since last measured. 
    if (next_state != current_state){
        //Finite state machine that updates the count value based on the quadrature 
        //output table in https://moodle.bath.ac.uk/pluginfile.php/2146779/mod_resource/content/1/3717670.pdf
        switch (current_state){
            case 0:
                if (next_state == 2){
                    enc_count--;
                }
                else if (next_state == 1){
                    enc_count++;
                }
                break;
            case 1:
                if (next_state == 0){
                    enc_count--;
                }
                else if (next_state == 3){
                    enc_count++;
                }
                break;
            case 2:
                if (next_state == 3){
                    enc_count--;
                }
                else if (next_state == 0){
                    enc_count++;
                }
                break;
            case 3:
                if (next_state == 1){
                    enc_count--;
                }
                else if (next_state == 2){
                    enc_count++;
                }
                break;
        }
        //Updates current state to the new state.
        current_state = next_state;
    }
    //Ensures that the enc_count remains between 0 and the upper specified limit.
    enc_count = enc_count < 0 ? 0 : enc_count;
    enc_count = enc_count > Enc_Count_Upper_Limit ? Enc_Count_Upper_Limit : enc_count;

    //Change uni-directional LED to reflect if fan is being driven.
    LED2.write(((enc_count == 0) ? 0 : 1));

}

void Read_Tach()
{
    //ISR that calculates the current RPM of the fan based on the frequency
    //of pulses sent from the tachometer.
    //
    //Updates value for current_speed.

    //Stores a list of the last 10 durations between pulses and calculated speeds.
    //Used for smoothening of speed data. 
    static int tach_timestamp_list[10] = {0,0,0,0,0,0,0,0,0,0};
    static int tach_curr_speed_list[10] = {0,0,0,0,0,0,0,0,0,0};

    //Takes the time since the last pulse was recorded in microseconds. 
    int pulse_time = t.elapsed_time().count();

    //Filters out any pulses that occur two quickly for the desired speed.
    //Larger pulse frequencies are filtered at fan speeds lower than 800 RPM. 
    int threshold = (current_speed < 800) ? 0.8*1000000*60/(desired_speed*2) : 15000;

    //If the recorded pulse time is valid.
    if (pulse_time > threshold)
    {
        //Shift the list of times to the left and add them up. 
        int sum = 0;
        int speed_sum = 0;
        for (int i=0;i<=8;i++)
        {
            sum += tach_timestamp_list[i+1];
            tach_timestamp_list[i] = tach_timestamp_list[i+1];

            speed_sum += tach_curr_speed_list[i+1];
            tach_curr_speed_list[i] = tach_curr_speed_list[i+1];        
        }
        //Only add the currently detected time if the pulse isn't too long, as the calcualted speed
        //could be incorrect. Extends the last value on the end if this is the case.   
        tach_timestamp_list[9] = abs(pulse_time - tach_timestamp_list[8]) < 150000 ? pulse_time : tach_timestamp_list[8];    

        //Calculate average time between pulses.
        sum += pulse_time;
        float av_time = float(sum)/10;
        //Calculates current speed of the fan from readings. 
        int read_speed = 60 * 1000000/(2*av_time);

        //Repeats process for the list of fan speeds
        tach_curr_speed_list[9] = read_speed;
        speed_sum += read_speed;
        //Current speed is the average of the last 10 calculated speeds. 
        current_speed = speed_sum/10;
        
        //Timer is reset to prepare for next tachometer reading. 
        t.reset();
    }
}

void Closed_Loop()
{
    //Timed ISR (Ticker) that calculates the duty cycle for closed loop
    //control mode.
    //
    //Updates value for closed_loop_duty_cycle. 

    //Total and previous errors are updated every iteration. 
    static int prev_error = 0;  
    static double total_error = 0;

    //Error is calculated as difference between desired and current speed.
    int error = (desired_speed - current_speed);
    total_error += (error * PID_Time_Between);

    //Calculate PID components. 
    float P = KP * error;
    float I = KI * total_error;
    float D = KD * (error - prev_error)/PID_Time_Between;
    int PID_total = P + I + D;

    //Update previous error.
    prev_error = error;
    //Closed loop duty cycle is calculated from total PID calculated.  
    closed_loop_duty_cycle = float(PID_total)/PID_Scalar;     
}
        
void Control_Mode_Change(){
    //ISR for toggling the contorl mode whenever the NUCLEO button is pressed.
    //
    //Toggles Control_Mode.
    //Toggles colour of bi-directonal LED.
    
    //Control mode is inverted.
    Control_Mode = !Control_Mode;       
    //LED colour changed to reflect control mode.
    LED_BIRA = (Control_Mode);          
    LED_BIRB = !LED_BIRA;
}

//------------------MAIN LOOP------------------

int main()
{
    //Main function that initially runs when the system is turned on. 

    //Initialise states of timers, LEDs and PWM signals so that the starting
    //state is open loop with 0% duty cycle. 
    t.start();
    LED2.write(Control_Mode);
    LED_BIRA = (Control_Mode);          
    LED_BIRB = !LED_BIRA;
    PWM_Sig.period(0.05);
    PWM_Sig.write(0);
    float open_loop_duty_cycle = 0;

    //Attach methods to the ISRs/tickers and the time periods they should run.
    Encoder_tick.attach(&Encoder_Count, 25ms);
    Closed_Loop_tick.attach(&Closed_Loop, 20ms);
    Change_Mode.rise(&Control_Mode_Change);
    Tach_Sig.fall(&Read_Tach);

    while (true) {
        //Calculate open loop duty cycles based on the proportion of encoder count to it's limit.
        //Adds a base duty cycle to overcome the frictional forces of the fan.
        //A base duty cycle of 0.18 was found to be optimal. 
        open_loop_duty_cycle = float(enc_count)/Enc_Count_Upper_Limit;
        open_loop_duty_cycle += (enc_count < 1) ? 0 : 0.18 * (1-open_loop_duty_cycle);

        //The closed loop duty cycle has the base open loop duty cycle added.
        closed_loop_duty_cycle += open_loop_duty_cycle;

        //Calculates the desired speed based of the proportion of the encoder count and its limit.
        //Only adds the minimum speed of the fan if desired to not be at rest. 
        desired_speed = (Maximum_Speed-Minimum_Speed) * float(enc_count)/Enc_Count_Upper_Limit;
        desired_speed += (enc_count < 1) ? 0 : Minimum_Speed;

        //Prints both speed RPMs and control mode to LCD screen.
        lcd.cls();
        lcd.printf("Des:%d   %s\nCur:%d", desired_speed, (Control_Mode ? "CL":"OL"),current_speed);

        //The fan sometimes twitched at speeds of 300 RPM or remained stationary even when the 
        //desired speed was greater than 0. If the tachometer pulses get too wide (less than 4 per second)
        //it is assumed the motor is off.
        // 
        //The fan is given a 'nudge' of a high duty cycle to break it out of incorrect stationary states.
        if (current_speed == 300 || (current_speed == 0 && desired_speed > 1) || t.elapsed_time().count() > 250000)
        {
            current_speed = 0;
            t.reset();        
            if (desired_speed > 1)
            {
                PWM_Sig.write(0.5);
            }    
        }
        //Motor duty cycle is driven low at higher speeds to prevent getting stuck near the limit when the
        //desired speed is lower.  
        else if ((abs(Maximum_Speed - current_speed) < 100) && (abs(Maximum_Speed - desired_speed) > 100)){
                PWM_Sig.write(0.3);
        }

        //Main loop is run 5 times a second to save computational time. 
        ThisThread::sleep_for(200ms); 
        //'Correct' duty cycle is written to the motor PWM.   
        PWM_Sig.write(Control_Mode ? closed_loop_duty_cycle : open_loop_duty_cycle);   
    }
}