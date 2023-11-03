#ifndef _MECHANISMH_H_
#define _MECHANISMH_H_

#if MACHINE_STYLE == ROBOT_H
#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include "MeanFilterLib.h"
#include <HardwareSerial.h>
#include <MultiStepper.h>
#include <config.h>
#include <math.h>
#include <EEPROM.h>

/**
 * @class Calibration
 * @brief .
 * 
 * Esta clase .
 * 
 */
class MechanismH{
    public:
        MechanismH();
        void init();
        void moveToPoint(double, double, int bCompensacion = 0);
        void moveToPointV2(double, double, int bCompensacion = 0);
        void accelRamp(double, double, int typeMove = 0,int bandElectro = 0,int numElectro = 0);
        void getActualPosition(double *,double *);
        void setSpeedMotors(int);
        void setSpeedRampFunction(double);
        void setAccelRampFunction(double);
        void reInitVariables(void);
    private:
};
#endif
#endif