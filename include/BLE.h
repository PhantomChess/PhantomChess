#pragma once
//#define BLUECOMMENTS
#include <Arduino.h>
#include "config.h"



#define SERVICE_UUID5 "fd31a840-22e7-11eb-adc1-0242ac120002" //General service


//====General Config====
#define GENERAL_UUID_TEST_SENSORS               "7b204548-30c3-11eb-adc1-0242ac120002"                  //Esta caracteristica es para el test de sensores
#define GENERAL_UUID_ERRORMSG                   "7b204d4a-30c3-11eb-adc1-0242ac120002"

//=============== Definicion de los UUID de las Caracteristicas NewCharacteristic (3)
#define GENERAL_UUID_RECEIVE_MOVEMENT           "c60c786b-bf3f-49d8-bd9e-c268e0519a7b"      //Enviar movimiento, se envia en de la siguiente forma "M f7-f5", es la coordenada de inicio y la cordenada final del movimiento
#define GENERAL_UUID_STATUS_BOARD               "06034924-77e8-433e-ac4c-27302e5e853f"      //Al realizar un movimiento de pieza en el tablero dicho movimiento se envia a traves de la siguiente caracteristica
#define GENERAL_UUID_SELECT_MODE                "c08d3691-e60f-4467-b2d0-4a4b7c72777e"      //Seleccionar modo, escultura = "1" play = "2"

//Actualizacion por Ota
#define FILE_UUID_RECEIVE_OTA                   "93601602-bbc2-4e53-95bd-a3ba326bc04b"      //La caracteristica para la actualizacion por OTA
//====================================================================================

/**
 * @class Bluetooth
 * @brief Se encarga de gestionar la comunicacion por bluetooth
 * @param timeOutBt es el tiempo, en milisegundos, que se va a esperar para recibir respuesta del dispositivo bluetooth conectado.
 * @param dataBt es donde se va a almacenar la informacion recibida.
 */
class Bluetooth {
    private:
        
    public:
        Bluetooth(); 
        int init(String = "Chess");

        static void setPlaylistName(String );
        static void setPathAmount(int);
        static void setPathName(String);
        static void setAddPathNamePlaylist(String);
        static void setPathNameNotify(String);
        static void setPathPosition(int);
        static void setPlayMode(int);
        static void setPathProgress(int);
        static void setPlaylistNameFiles(String );
        static void setPlaylistToggle();

        static void setLedSpeed(int);
        static void setCycleMode(int);
        static void setLedDirection(int);
        static void setBrightness(uint16_t);
        static void setIndexPalette(int);
        static void setRed();
        static void setGreen();
        static void setBlue();
        static void setPositions();
        static void setAmountOfColors();

        static void setVersion(String);
        static void setSimpleVersion(String);
        static void setName(String);
        static void setNotifySensorChange(String);

        //===================Funciones para bluetooth Chess==================
        static String verifNewCommandBluetooth();
        static void setStatus(String);
        static String getModeChess();
        //====================================================================



        static void setTimePaths(String);
        static void setTimeOffOn(String);


        static void setMotorSpeed(int);
        static void setPercentage(int);
        static void setCalibrationStatus(bool);

        static void setTestResult(String);
};