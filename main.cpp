#include "mbed.h"
#include "LSM9DS1.h"
#include "Servo.h"
 
using namespace  std;


#define PI 3.14159
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -4.94 // Declination (degrees) in Atlanta,GA.


PwmOut LED_G(D3);
PwmOut LED_R(D5);
 
Servo servoPitch(D9);
Servo servoRoll(D6);

//AnalogIn  batteryVol(A4);
DigitalOut capLED(A5);
AnalogIn  capIN(A6);
DigitalOut myled(LED1);


Serial pc(USBTX, USBRX);



float range = 0.0006;   // uiterste waarden
float position = 0.5;   // midden positie default

float calcRoll(float ax, float ay, float az, float mx, float my, float mz){
    float roll = atan2(ay, az);
    roll  *= 180.0 / PI;
    return roll;
}

float calcPitch(float ax, float ay, float az, float mx, float my, float mz){
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    pitch *= 180.0 / PI;
    return pitch;
}
    



int main()
{
    //LED_G.period_ms(1);
    //LED_R.period_ms(1);
    
    LED_G = 0.2;
    LED_R = 0.15;
    
    pc.baud(38400); //115200

    pc.printf("Begin programma\n");
    //LSM9DS1 lol(p9, p10, 0x6B, 0x1E);
    LSM9DS1 IMU(D0, D1, 0xD6, 0x3C);
    IMU.begin();
    if (!IMU.begin()) {
        pc.printf("Failed to communicate with LSM9DS1.\n");
        myled = true;
    }
    //IMU.calibrate(1);
    //IMU.calibrateMag(0);
    
    capLED = 1; // Lampje capacitieve aan
    bool capLEDState = true;
    bool capSwitch = false;
    servoPitch = 0.5;
    servoRoll = 0.5;
    wait(1);

    double prevRoll = 0.0;
    double prevPitch = 0.0;
    double lRoll = 0.0;
    double lPitch = 0.0;
    
    while(1) {
        //pc.printf("percentage: %3.3f%%\n", batteryVol.read()*100.0f);
        while(!IMU.magAvailable(X_AXIS));
        IMU.readMag();
        while(!IMU.accelAvailable());
        IMU.readAccel();
        while(!IMU.gyroAvailable());
        IMU.readGyro();
        
        // HOEK BEREKENEN
        double tempRoll = calcRoll(IMU.calcAccel(IMU.ax), IMU.calcAccel(IMU.ay), IMU.calcAccel(IMU.az), IMU.calcMag(IMU.mx),IMU.calcMag(IMU.my), IMU.calcMag(IMU.mz));
        double tempPitch = calcPitch(IMU.calcAccel(IMU.ax), IMU.calcAccel(IMU.ay), IMU.calcAccel(IMU.az), IMU.calcMag(IMU.mx),IMU.calcMag(IMU.my), IMU.calcMag(IMU.mz));
        
        tempRoll = tempRoll + 3.0f; // compensatie scheve sensor tijdelijk
        
        
        double diffRoll = abs(tempRoll - prevRoll);
        if (diffRoll >= 4.0f){
            if ((tempRoll - prevRoll) > 3.0f){
                lRoll = prevRoll + 4.0f;
            } else if ((tempRoll - prevRoll) < -3.0f){
                lRoll = prevRoll - 4.0f;
            }
        } else {
            lRoll = tempRoll;
        }
        pc.printf("afwijking roll: %f", diffRoll);
        
        
        double diffPitch = abs(tempPitch - prevPitch);
        if (diffPitch >= 4.0f){
            if ((tempPitch - prevPitch) > 3.0f){
                lPitch = prevPitch + 4.0f;
            } else if ((tempPitch - prevPitch) < -3.0f){
                lPitch = prevPitch - 4.0f;
            }
        } else {
            lPitch = tempPitch;
        }
        pc.printf("afwijking pitch: %f", diffPitch);
        
        
        prevRoll = lRoll;
        prevPitch = lPitch;

        //pc.printf("Afwijking roll > 8graden, te snel bewogen. lPitch: %f, prevPitch: %f\n", abs(lPitch), abs(prevPitch));


        


        // PWM SIGNAAL BEREKENEN
        /*
            0.0125 = 40deg = pwmout 0.0
            
        */
        double cRoll = (0.5f + (0.0122)*lRoll);
        double cPitch = (0.5f + (0.0122)*lPitch);
        // INVERTEREN
        cPitch =  (1.0f - cPitch);
        cRoll = (1.0f - cRoll);

        
        servoPitch.calibrate(range, 30.0);
        servoRoll.calibrate(range, 30.0); 
 
        
        
        
        if (capSwitch == true && capIN <= 0.8){
            capSwitch = false;
            //pc.printf("Capswitch uit\n");
                        
            if (capLEDState == false){
                capLED = 1;
                capLEDState = true;
            } else if (capLEDState == true){
                capLED = 0;
                capLEDState = false;
            }
        } else if (capIN >= 0.8){
            //pc.printf("Capswitch aan\n");
            capSwitch = true;

        }
        
        if (cPitch > 1) {
            cPitch = 1;
        }
        if (cPitch < 0) {
            cPitch = 0;
        }
        
        if (cRoll > 1){
            cRoll = 1;
        }
        
        if (cRoll < 0){
            cRoll = 0;
        }
        // Als we actief mogen levelen
        if (capLEDState == true){
            
            if(lPitch >= 1.0 || lPitch <= -1.0){
                servoPitch = cPitch;
            }
            if(lRoll >= 1.0 || lRoll <= -1.0){
                servoRoll = cRoll;
            }
        }

        pc.printf("Pitch: %f,    Roll: %f degress, pwmPitch%: %f, pwmRoll%: %f\n\r",lPitch,lRoll,cPitch,cRoll);

        //wait(0.1);
    }
}

