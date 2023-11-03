#ifndef CONFIGURE_H
#define CONFIGURE_H
//------------------------------------------------------------------------------


//---------------------------------------------------------------
#define myDebug

//#define _realizaJugada            //Habilita los comentarios en la funcion realizaJugada
//#define activateSensors             //Habilita los sensores
//#define debugSensors              //Se define si se muestra constantemente la impresion de sensores

//#define generalDebug

//#define testWorkSpace               //Habilita hacer un recorrido inicial al rededor del area de trabajo
//#define testPiecesAndCore           //Habilita el recorrido del electroiman apagado por debajo de las piezas en las posiciones iniciales

#define stateMachineActivate        //El programa entra automaticamente a la maquina de estados

//#define modoDemostracion
//#define testSensoresSoloCambios    //Este test solo imprime en el serial cada vez que algo cambia en el tablero, se debe comentar el define de debugSensors

//#define automaticInitSculpture      //El programa inicia la reproduccion automaticamente (Modo escultura)
#define SelectModeBluetooth
//#define automaticInitPlayMode

//===================================================================================
//Para entrar al modo play, comentar la definicion anterior automaticInitSculpture y descomentar
//las siguientes lineas para pasar automaticamente algunas de las verificaciones
//---------------------------------------------------------------------------------- 
#define passReceiveInitialFen
#define passCheckMovement
#define passCompareEngineMechanism
#define passCheckMate


//#define CambiosPCBLalo
#define CambiosPCBTavo

//====================================================================================
//#define normalElectro                         
#define invertElectro                           //Para Lalo es invert Electro

#define pwmMagnet 0                             //Nivel de pwm, 0 es el maximo ciclo de trabajo

//#define activarTestSensors   //para test de sensores porr bluetooth App de react native
#define completeTest         //Para hcer el test completo, electroimanes, motores , buzzer
//#define testTemp

#define version4Electro
//#define version1Electro

//#define curvasV1
#define curvasV2


//#define pinoutv1                //Configuracion del pinout del primer pcb
#define pinoutv2                //Configuracion del pinout del PCB que tengo yo
//#define pinoutv3                    //Configuracion del pinout del PCB que tiene Lalo

#ifdef pinoutv1

#define magnet1 13
#define magnet2 12
#define magnet3 22
#define magnet4 23

#endif

#ifdef pinoutv2

#define magnet1 13      //13
#define magnet2 12      //12
#define magnet3 22      //22
#define magnet4 23      //23

#endif

#ifdef pinoutv3

#define magnet1 19     //19 
#define magnet2 22     //23
#define magnet3 21     //21
#define magnet4 23     //22

#endif


//#define electromagnetConf1    //Configuracion de electroiman para las placas anteriores
#define electromagnetConf2    //Configuracion de electroiman para las placas nuevas 


//#define checkArea                 //Habilita el movimiento cuadrado sobre los escaques

#define centrarPiezas               //Habilita la funcion de centrar piezas en sus pocisionees iniciales



//===================================================
//CONFIGURACION DE COMUNICACION PARA EL MODO PLAY
//---------------------------------------------------
//#define receiveSerialMessages
#define receiveBluetoothMessages
//===================================================

//===================================================
//CONFIGURACION DE CALIBRACION ENTRE PROGRAMAS
//---------------------------------------------------
//#define calibBetweenGames
//===================================================

//===================================================
//DEFINICIONES PARA SELECCIONAR EL TIPO DE MOVIMIENTO
//---------------------------------------------------
//#define funcMoveTo
#define funcAccelRamp
//#define accelRampDebug
//===================================================


//#define relacionMicroSteps1 
//#define relacionMicroSteps4
//#define relacionMicroSteps16
//#define relacionMicroSteps32
//#define relacionMicroSteps64
#define relacionMicroSteps128

//#define originalSpeed
#define halfSpeed


//===================================================
//DEFINICIONES PARA CONFIGURAR VELOCIDAD Y ACELERACION GLOBALES
//---------------------------------------------------
#ifdef relacionMicroSteps1
#define defGlobalSpeed     400     //Funciona 500       //400
#define defGlobalAccel     800     //Funciona 1000      //800
#endif

#ifdef relacionMicroSteps4
#define defGlobalSpeed     6000     //Antes 8000
#define defGlobalAccel     12000     //Antes 16000
#endif

#ifdef relacionMicroSteps16
#define defGlobalSpeed     9000     //Original 12000
#define defGlobalAccel     18000     //Original 24000
#endif

#ifdef relacionMicroSteps32
#define defGlobalSpeed     2000     //Antes 4000
#define defGlobalAccel     8000     //Antes 5000
#endif

#ifdef relacionMicroSteps64
#define defGlobalSpeed     25000     //
#define defGlobalAccel     50000     //
#endif

#ifdef relacionMicroSteps128
#ifdef originalSpeed
    #define defGlobalSpeed     50000     //
    #define defGlobalAccel     100000    //
#endif

#ifdef halfSpeed
    #define defGlobalSpeed     25000     //
    #define defGlobalAccel     50000    //
#endif

#endif
//===================================================


//===================================================
//CONFIGURACION DE ROTACION DE LA MATRIZ DE SENSORES
//---------------------------------------------------
#define matrizPos1 1    // (1) Matriz rotada 180 grados en sentido horario
#define matrizPos2 2    // (2) Matriz rotada 90 grados en sentido horario
#define matrizPos3 3    // (3) Matriz original
#define matrizPos4 4    // (4) Matriz rotada 270 grados en sentido horario

#ifdef CambiosPCBTavo
#define PosMatrizSensor matrizPos2    //matrizPos4  
#endif

#ifdef CambiosPCBLalo
#define PosMatrizSensor matrizPos2  
#endif
//===================================================


//===================================================
//CONFIGURACION DE TIPO DE SENSORES
//---------------------------------------------------
#define newSensors
//#define oldSensors
//===================================================


//===================================================
//CONFIGURACION PARA EL RECORRIDO SIN PIEZA
//---------------------------------------------------
#define moveDirectPosPiece                  //Se mueve hacia la pieza objetivo en una trayectoria directa

//.....Velocidad en trayectoria sin pieza usando moveToPoint.....
#ifdef relacionMicroSteps1
#define maximumVelocityDirect       400    //Funciona 500     //400

//.....Velocidad en trayectoria sin pieza usando accelRamp
#define maximunVelocityDirectRamp   400    //Funciona 500     //400
#define maximunAccelDirectRamp      800    //Funciona 1000    //800
#endif

#ifdef relacionMicroSteps4
#define maximumVelocityDirect       6000    //Antes 5000    Define la velocidad cuando se dirige en una trayectoria directa

//.....Velocidad en trayectoria sin pieza usando accelRamp
#define maximunVelocityDirectRamp   6000    //Antes 4000
#define maximunAccelDirectRamp      12000
#endif

#ifdef relacionMicroSteps16
#define maximumVelocityDirect       9000    //Original 12000    Define la velocidad cuando se dirige en una trayectoria directa

//.....Velocidad en trayectoria sin pieza usando accelRamp
#define maximunVelocityDirectRamp   9000    //Original 12000
#define maximunAccelDirectRamp      18000    //Original 24000
#endif

#ifdef relacionMicroSteps32
#define maximumVelocityDirect       2000    //Antes 5000    Define la velocidad cuando se dirige en una trayectoria directa

//.....Velocidad en trayectoria sin pieza usando accelRamp
#define maximunVelocityDirectRamp   2000    //Antes 4000
#endif


#ifdef relacionMicroSteps64
#define maximumVelocityDirect       20000    //

//.....Velocidad en trayectoria sin pieza usando accelRamp
#define maximunVelocityDirectRamp   25000    //
#define maximunAccelDirectRamp      50000    //
#endif

#ifdef relacionMicroSteps128
#ifdef originalSpeed
#define maximumVelocityDirect       40000    //
//.....Velocidad en trayectoria sin pieza usando accelRamp
#define maximunVelocityDirectRamp   50000    //
#define maximunAccelDirectRamp      100000    //
#endif
#ifdef halfSpeed
#define maximumVelocityDirect       20000    //
//.....Velocidad en trayectoria sin pieza usando accelRamp
#define maximunVelocityDirectRamp   25000    //
#define maximunAccelDirectRamp      50000    //
#endif
#endif
//================================================================

#define desfaseElectroiman 3                //Original 6 Valor de la distancia donde se apaga el electroiman antes de encenderse nuevamente para centrar las piezas
#define centrarElectromagnet                //Habilita accion de apagar y encender electroiman para centrar las piezas


//================================================================
#define ROBOT_4THREADS       1  
#define ROBOT_H              2  
//---------------------------------------------------------------
#define FOUR_DRIVERS         1
#define TWO_DRIVERS          2
//---------------------------------------------------------------
#define MACHINE_STYLE ROBOT_H       //Change this
#define BOARD_CHESS TWO_DRIVERS     //Change this
//================================================================

#include "MechanismH.h"

#if BOARD_CHESS == FOUR_DRIVERS
//================MOTOR 1====================
#define MOTOR_0_DIR_PIN (14)
#define MOTOR_0_STEP_PIN (25)
//================MOTOR 2====================
#define MOTOR_1_DIR_PIN (32)   //Antes 33
#define MOTOR_1_STEP_PIN (33) //Antes 32
//================MOTOR 3====================
#define MOTOR_2_DIR_PIN (18)   
#define MOTOR_2_STEP_PIN (13)  
//================MOTOR 4====================
#define MOTOR_3_DIR_PIN (21)     //Antes 19
#define MOTOR_3_STEP_PIN (19)    //Antes 21

#define MOTOR_0_ENABLE_PIN                  17
#define MOTOR_1_ENABLE_PIN                  17
#define ENABLE_PIN 17

#define hall1 39
#define hall2 36

//DEFINICIONES TEMPOREALES PARA PRUEBAS
#define LED_PIN     23
#define HALL3       35
//=====================================

//==================== I2C =================

//#define SDA_PIN 22
//#define SCL_PIN 23
#define I2C_SLAVE_ADDR 0x05
#define I2C_ESP32_ADDR 0x04
#define I2C_ARDUINO_ADDR 0x06

// el maximo es 124
#define MAX_SLAVE_RESPONSE_LENGTH 32

//===========================================

//Es el valor de la resistencia colocada en los drivers, por ejemplo 0.11f representa 0.11 ohms.
#define R_SENSE 0.11f

//STALL_VALUE y STALL_VALUE2 es la sensibilidad para la deteccion de colision la cual tambien depende de la corriente.
#define STALL_VALUE 20 //20
#define STALL_VALUE2 20 //20
#define STALL_VALUE3 20 //20
#define STALL_VALUE4 20 //20

//MICROSTEPPING es para configurar las distintas configuraciones de de microstepping que tiene el motor
// entre las opciones estan 1,2,4,8,16,32,64.
#define MICROSTEPPING 16

#define NORMAL_CURRENT 1200 //800
#define CURRENT_IN_CALIBRATION 1200 //800

#define SERIAL_PORT2 Serial1 // TMC2208/TMC2224 HardwareSerial port

#define DRIVER_ADDRESS1 0b00 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS2 0b01 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS3 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS4 0b11 // TMC2209 Driver address according to MS1 and MS2


//Definitions to read from Serial
#define BAUD                                115200
#define STRING_SIZE                         128
#define XOFF                                0x13
#define XON                                 0x11

//Definitions for Steppers and Mechanics
#define pulleyRadius                        6.283185307180
#define neededStepsfor1Turn                 3200
#define maximumVelocitySteppsperSeconds     7000     //Antes 7000
#define MyPI                                3.1415926535897932384626433832795

#endif


#if BOARD_CHESS == TWO_DRIVERS

#ifdef pinoutv1
//================MOTOR 1====================
#define MOTOR_0_DIR_PIN (14)
#define MOTOR_0_STEP_PIN (25)
//================MOTOR 2====================
#define MOTOR_1_DIR_PIN (33) 
#define MOTOR_1_STEP_PIN (32)
//================MOTOR 3====================
#define MOTOR_2_DIR_PIN (0) 
#define MOTOR_2_STEP_PIN (0)  
//================MOTOR 4====================
#define MOTOR_3_DIR_PIN (0)     //Antes 19
#define MOTOR_3_STEP_PIN (0)    //Antes 21

#define MOTOR_0_ENABLE_PIN                  18
#define MOTOR_1_ENABLE_PIN                  18
#define ENABLE_PIN 18

#define hall1 39
#define hall2 36
#define HALL3       13
#define LED_PIN     13

#define ELECTRO_ENABLE_PIN 0
#define BUZZER_PIN 5

#endif

#ifdef pinoutv2
//================MOTOR 1====================
#define MOTOR_0_DIR_PIN (33)
#define MOTOR_0_STEP_PIN (32)
//================MOTOR 2====================
#define MOTOR_1_DIR_PIN (14) 
#define MOTOR_1_STEP_PIN (25)

#define ENABLE_PIN 18
#define ELECTRO_ENABLE_PIN 18

#define LED_PIN     0

#define BUZZER_PIN 5

#endif

#ifdef pinoutv3
//================MOTOR 1====================
#define MOTOR_0_DIR_PIN (32)
#define MOTOR_0_STEP_PIN (33)
//================MOTOR 2====================
#define MOTOR_1_DIR_PIN (17) 
#define MOTOR_1_STEP_PIN (5)

#define ENABLE_PIN 18
#define ELECTRO_ENABLE_PIN 18

#define LED_PIN     0

#define BUZZER_PIN 25

#endif


//=====================================

//==================== I2C =================

//#define SDA_PIN 22
//#define SCL_PIN 23
//#define I2C_SLAVE_ADDR 0x05
//#define I2C_ESP32_ADDR 0x04
//#define I2C_ARDUINO_ADDR 0x06

// el maximo es 124
//#define MAX_SLAVE_RESPONSE_LENGTH 32
//===========================================

#define EEPROM_SIZE 512

//Es el valor de la resistencia colocada en los drivers, por ejemplo 0.11f representa 0.11 ohms.

#ifdef pinoutv1 
#define R_SENSE 0.11f
#endif

#ifdef pinoutv2 
#define R_SENSE 0.10f
#endif

#ifdef pinoutv3 
#define R_SENSE 0.10f
#endif

//STALL_VALUE y STALL_VALUE2 es la sensibilidad para la deteccion de colision la cual tambien depende de la corriente.
#define STALL_VALUE 20 //20
#define STALL_VALUE2 20 //20
#define STALL_VALUE3 20 //20
#define STALL_VALUE4 20 //20

//MICROSTEPPING es para configurar las distintas configuraciones de de microstepping que tiene el motor
// entre las opciones estan 8,16,32,64.
#ifdef relacionMicroSteps1
#define MICROSTEPPING 0  
#endif

#ifdef relacionMicroSteps4
#define MICROSTEPPING 4  
#endif

#ifdef relacionMicroSteps16
#define MICROSTEPPING 16
#endif

#ifdef relacionMicroSteps32
#define MICROSTEPPING 32
#endif

#ifdef relacionMicroSteps64
#define MICROSTEPPING 64
#endif

#ifdef relacionMicroSteps128
#define MICROSTEPPING 128
#endif


#ifdef pinoutv1
#define NORMAL_CURRENT 1200 //1200
#define CURRENT_IN_CALIBRATION 1200 //1200
#endif

#ifdef pinoutv2
#define NORMAL_CURRENT 500 //1200
#define CURRENT_IN_CALIBRATION 500 //1200
#endif

#ifdef pinoutv3
#define NORMAL_CURRENT 500 //900
#define CURRENT_IN_CALIBRATION 500 //900
#endif


#define I_HOLD 1 //12

#define SERIAL_PORT2 Serial1 // TMC2208/TMC2224 HardwareSerial port

#ifdef pinoutv1
#define DRIVER_ADDRESS1 0b00 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS2 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS3 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS4 0b11 // TMC2209 Driver address according to MS1 and MS2
#endif


#ifdef pinoutv2
#define DRIVER_ADDRESS1 0b00 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS2 0b11 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS3 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS4 0b01 // TMC2209 Driver address according to MS1 and MS2

#endif

#ifdef pinoutv3
#define DRIVER_ADDRESS1 0b00 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS2 0b11 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS3 0b10 // TMC2209 Driver address according to MS1 and MS2

#define DRIVER_ADDRESS4 0b01 // TMC2209 Driver address according to MS1 and MS2

#endif

//Definitions to read from Serial
#define BAUD                                115200
#define STRING_SIZE                         128
#define XOFF                                0x13
#define XON                                 0x11

//Definitions for Steppers and Mechanics
#define pulleyRadius                        6.283185307180

#ifdef relacionMicroSteps1
#define neededStepsfor1Turn                 200     //3200
#endif

#ifdef relacionMicroSteps4
#define neededStepsfor1Turn                 800     //3200
#endif

#ifdef relacionMicroSteps16
#define neededStepsfor1Turn                 3200     //3200
#endif

#ifdef relacionMicroSteps32
#define neededStepsfor1Turn                 6400     //3200
#endif

#ifdef relacionMicroSteps64
#define neededStepsfor1Turn                 12800     //3200
#endif

#ifdef relacionMicroSteps128
#define neededStepsfor1Turn                 25600     //3200
#endif

#ifdef relacionMicroSteps1
#define maximumVelocitySteppsperSeconds     800     //Funciona 1000 //800
#endif

#ifdef relacionMicroSteps4
#define maximumVelocitySteppsperSeconds     6000     //Antes 7000     9800
#endif

#ifdef relacionMicroSteps16
#define maximumVelocitySteppsperSeconds     9000     //Original 12000
#endif

#ifdef relacionMicroSteps32
#define maximumVelocitySteppsperSeconds     2000     //Antes 7000     9800
#endif

#ifdef relacionMicroSteps64
#define maximumVelocitySteppsperSeconds     25000     //Antes 7000     9800
#endif

#ifdef relacionMicroSteps128
#ifdef originalSpeed
#define maximumVelocitySteppsperSeconds     50000     //Antes 7000     9800
#endif
#ifdef halfSpeed
#define maximumVelocitySteppsperSeconds     25000     //Antes 7000     9800
#endif
#endif

#define MyPI                                3.1415926535897932384626433832795


//The line below makes the compiler to copy the config.h once, even is it's called multiples times. 
// #pragma one
/////////////////////////////////////////////////////////////
//Tiempo entre sensores para que alcance a switchear compuertas en ms
#define timeBsensors 100
#define CONSTOFREJECTION 1000
#define SIMPLE 0
#define FULL 1
#define GROUPOFSENSORS 2
#define REJECTION 3

#ifdef pinoutv1
 //Pins below ares used to select the 8bits muxes 
 #define mux8_0 17
 #define mux8_1 5
 #define mux8_2 16
 //Pins for the 4 selectors of the 16bits muxes. 
 #define mux16_0 4
 #define mux16_1 2
 #define mux16_2 15

//Data IN pins from 16 bits muxes
 #define mux16Out_1 35
 #define mux16Out_2 34
 #define mux16Out_3 21
 #define mux16Out_4 19
 #endif

#ifdef pinoutv2
 //Pins below ares used to select the 8bits muxes 
 #define mux8_0 17
 #define mux8_1 2
 #define mux8_2 16
 //Pins for the 4 selectors of the 16bits muxes. 
 #define mux16_0 5
 #define mux16_1 19
 #define mux16_2 21

//Data IN pins from 16 bits muxes
 #define mux16Out_1 34
 #define mux16Out_2 35
 #define mux16Out_3 15
 #define mux16Out_4 4
#endif


#ifdef pinoutv3
 //Pins below ares used to select the 8bits muxes
 #define mux8_0 12//12 4
 #define mux8_1 14//14 13
 #define mux8_2 16//4  2
 //Pins for the 4 selectors of the 16bits muxes.
 #define mux16_0 2//13  25
 #define mux16_1 15//2  14
 #define mux16_2 13//25  12
//Data IN pins from 16 bits muxes
 #define mux16Out_1 35//35  39 
 #define mux16Out_2 34//34  34 
 #define mux16Out_3 39//39  35 
 #define mux16Out_4 36//36  36 
#endif

//Para direccionar un sensor, debemos conocer 3 cosas, la entrada del ESP al que está conectado, el mux de 16 al que está conectado su mux de 8 y finalmente la dirección en ese mux de 8. 
//Con 8 bits somos capaces de direccionar los 500 sensores. 
//La estructura del byte es el siguiente 0 000 0000 

#endif

#endif // CONFIGURE_H