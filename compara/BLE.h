#ifndef _BLE_H_
#define _BLE_H_

#include "Arduino.h"


//===============Original===============
/*
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
*/
//======================================

//=============== Change ===============
#include <NimBLEDevice.h>
//======================================

#include "esp_ota_ops.h"

#define SOFTWARE_VERSION_MAJOR 0
#define SOFTWARE_VERSION_MINOR 1
#define SOFTWARE_VERSION_PATCH 0
#define HARDWARE_VERSION_MAJOR 1
#define HARDWARE_VERSION_MINOR 2

class BLE; // forward declaration

class BLECustomServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      // deviceConnected = true;
      // // code here for when device connects
    };

    void onDisconnect(BLEServer* pServer) {
      // deviceConnected = false;
      // // code here for when device disconnects
    }
};

class otaCallback: public BLECharacteristicCallbacks {
  public:
    otaCallback(BLE* ble) {
      _p_ble = ble;
    }
    BLE* _p_ble;

    void onWrite(BLECharacteristic *pCharacteristic);
};


//=========================================================
//Definir los callbacks de las funciones a utilizar
//---------------------------------------------------------
class generalCallbacks_name : public BLECharacteristicCallbacks
{
  public:
    void onWrite(BLECharacteristic *characteristic);

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic);

    #endif
};

class callbackReceiveMovementBluetooth : public BLECharacteristicCallbacks
{
  public:
    void onWrite(BLECharacteristic *characteristicMovementBluetooth);

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic);

    #endif
};

class generalCallbacks_status: public BLECharacteristicCallbacks {
    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic){
            Serial.print("READ status: ");
            //Serial.println(characteristic->getValue().c_str());
            Serial.println(characteristic->getValue());             //Cambio para la libreria NimBLE
        
        }
    #endif
};

class generalCallbacks_selectMode : public BLECharacteristicCallbacks
{
  public:
    void onWrite(BLECharacteristic *characteristic);

    #ifdef DEBUGGING_BLUETOOTH
        void onRead(BLECharacteristic *characteristic);

    #endif
};

//=========================================================

class BLE
{
  public:

    BLE(void);
    ~BLE(void);

    bool begin(const char* localName);
    String verifNewCommandBluetooth(void);
    void setStatus(String);
    String getModeChess(void);
  
  private:
    String local_name;

    BLEServer *pServer = NULL;

    BLEService *pESPOTAService = NULL;
    BLECharacteristic * pESPOTAIdCharacteristic = NULL;

    BLEService *pService = NULL;
    BLECharacteristic * pVersionCharacteristic = NULL;
    BLECharacteristic * pOtaCharacteristic = NULL;



    //======================================================
    //Se crea el apuntador para el servicio
    //------------------------------------------------------
    BLEService *pServiceGeneralConfig = NULL;
    //======================================================
    
    //======================================================
    //Se define la caracteristica como un apuntador de tipo de dato BLECharacteristic
    //------------------------------------------------------
    BLECharacteristic * generalCharacteristic_name = NULL;

    BLECharacteristic * characteristicReceiveMovementBluetooth = NULL;

    BLECharacteristic * generalCharacteristic_status = NULL;

    BLECharacteristic * generalCharacteristic_selectMode = NULL;
    //======================================================
};

#endif