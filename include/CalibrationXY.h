#ifndef _CALIBRATION_XY_H_
#define _CALIBRATION_XY_H_

//#if MACHINE_STYLE == ROBOT_H
#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include "MeanFilterLib.h"
#include <HardwareSerial.h>
#include <MultiStepper.h>
#include <config.h>
#include <EEPROM.h>
#include <CuteBuzzerSounds.h>

/**
 * @class Calibration
 * @brief .
 * 
 * Esta clase .
 * 
 */
class CalibrationXY{
    public:
        CalibrationXY();
        void initCalibration();
        int startCalibration();
    private:
};
//#endif
#endif