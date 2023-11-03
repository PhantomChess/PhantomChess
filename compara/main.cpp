#include <AccelStepper.h>
#include <CircularBuffer.h>
#include <MultiStepper.h>
#include <TMCStepper.h>
#include <config.h>
#include <CalibrationThreads.h>
#include <CalibrationXY.h>
#include <math.h>
#include <stdio.h>
#include <EEPROM.h>
//#include "Bluetooth.h"
#include <Servo.h>

//Incluye los archivos para la clase BLE
//===========================================
#include "BLE.h"
//===========================================

//Definicion del objeto BLE
//===========================================
BLE BT;
//===========================================

using namespace std;

#include "esp_int_wdt.h"
#include "esp_task_wdt.h"
 
#include "SPIFFS.h"
#include "FS.h"

#include "Mechanism4Threads.h"

CircularBuffer<String, 100> commands; // uses 538 bytes
String commandRead;
extern int desfaseEnX = 0;
extern int desfaseEnY = 0;

/*
extern String Game0,Game1,Game2,Game3,Game4,Game5,Game6,Game7,Game8,Game9,Game10,Game11,Game12,Game13,Game14,Game15,Game16,Game17,Game18,Game19,Game20,Game21,Game22,Game23,Game24,
Game25,Game26,Game27,Game28,Game29,Game30,Game31,Game32,Game33,Game34,Game35,Game36,Game37,Game38,Game39,Game40,Game41,Game42,Game43,Game44,Game45,Game46,Game47,Game48,Game49,
Game50,Game51,Game52,Game53,Game54,Game55,Game56,Game57,Game58,Game59,Game60,Game61,Game62,Game63,Game64,Game65,Game66,Game67,Game68,Game69,Game70,Game71,Game72,Game73,Game74,
Game75,Game76,Game77,Game78,Game79,Game80,Game81,Game82,Game83,Game84,Game85,Game86,Game87,Game88,Game89,Game90,Game91,Game92,Game93,Game94,Game95,Game96,Game97,Game98,Game99;
*/

extern const char *Game0,*Game1,*Game2,*Game3,*Game4,*Game5,*Game6,*Game7,*Game8,*Game9,*Game10,*Game11,*Game12,*Game13,*Game14,*Game15,*Game16,*Game17,*Game18,*Game19,*Game20,*Game21,*Game22,*Game23,*Game24,
*Game25,*Game26,*Game27,*Game28,*Game29,*Game30,*Game31,*Game32,*Game33,*Game34,*Game35,*Game36,*Game37,*Game38,*Game39,*Game40,*Game41,*Game42,*Game43,*Game44,*Game45,*Game46,*Game47,*Game48,*Game49,
*Game50,*Game51,*Game52,*Game53,*Game54,*Game55,*Game56,*Game57,*Game58,*Game59,*Game60,*Game61,*Game62,*Game63,*Game64,*Game65,*Game66,*Game67,*Game68,*Game69,*Game70,*Game71,*Game72,*Game73,*Game74,
*Game75,*Game76,*Game77,*Game78,*Game79,*Game80,*Game81,*Game82,*Game83,*Game84,*Game85,*Game86,*Game87,*Game88,*Game89,*Game90,*Game91,*Game92,*Game93,*Game94,*Game95,*Game96,*Game97,*Game98,*Game99;

extern const char *Game100,*Game101,*Game102,*Game103,*Game104,*Game105,*Game106,*Game107,*Game108,*Game109,*Game110,*Game111,*Game112,*Game113,*Game114,*Game115,*Game116,*Game117,*Game118,*Game119,*Game120,*Game121,*Game122,*Game123,*Game124,
*Game125,*Game126,*Game127,*Game128,*Game129,*Game130,*Game131,*Game132,*Game133,*Game134,*Game135,*Game136,*Game137,*Game138,*Game139,*Game140,*Game141,*Game142,*Game143,*Game144,*Game145,*Game146,*Game147,*Game148,*Game149,
*Game150,*Game151,*Game152,*Game153,*Game154,*Game155,*Game156,*Game157,*Game158,*Game159,*Game160,*Game161,*Game162,*Game163,*Game164,*Game165,*Game166,*Game167,*Game168,*Game169,*Game170,*Game171,*Game172,*Game173,*Game174,
*Game175,*Game176,*Game177,*Game178,*Game179,*Game180,*Game181,*Game182,*Game183,*Game184,*Game185,*Game186,*Game187,*Game188,*Game189,*Game190,*Game191,*Game192,*Game193,*Game194,*Game195,*Game196,*Game197,*Game198,*Game199;

float a, b, c, d, e, f, g;
int numdeDatos;
float offsetx = 90;
float offsety = 90;
int indexB = 0;
int indexN = 0;
float xB = 3.5;     // Antes 4.0
float yB = 3.5;     
float xN = -3.5;    // Antes -4.0
float yN = -3.5;

float xB2 = 4.5;
float xN2 = -4.5;
int bandActivate = 0;
int bandDesactivate = 0;
//====================
//====================
//====================
char matriz[8][8];
bool matrizBin[10][10];
bool matrizBinVerif[10][10];
char vectBlancas[16] = { 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v'};
char vectNegras[16]  = { 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v'};
int indexVectBlancas = 0;
int indexVectNegras = 0;
String FEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";

//String FEN2 = "r2qkbnr/p2pp1pp/b1n2p2/1pp5/4P1Q1/2N3PN/PPPP1P1P/R1B1KB1R";
String FEN2 = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";

extern TMC2209Stepper driver;
extern TMC2209Stepper driver2;
extern TMC2209Stepper driver3;
extern TMC2209Stepper driver4;

struct BoardPosition {
    double x;
    double y;
};

int flag = 0;

int contCiclos = 0;
int cont_m = 0;
int contGames = 0 ;
int verifElectro;

/*================== Variables de salida de maquina de Estado =================*/
int fileCommand = 0;
int receiveInstruction = 0;
int checkInstruction = 0;
int intermediateCalibration = 0;
int correctPosition = 0;

int mode = 0;

int initialFen = 0;
int actualTurn = 0;
int newFen = 0;
int sensorChange = 0;
int correctMovement = 0;
int resultEngineMec = 0;
int resultCheckMate = 0;
/*=============================================================================*/

struct decodeMovementInfo{
    char charXIniDecode;
    char charYIniDecode;
    char charXFinDecode;
    char charYFinDecode;
    char charPiezaDecode;
    int intAccionDecode;
};

decodeMovementInfo decodeInfoM;
String movementFileGlobal = "";
bool colorChessGlobal = false;
bool globalTurn = false;
bool bandEndGame = false;
/*================== Variables de salida de maquina de Estado Ejemplo=================*/

    int idReadMode = 1;
    int idSculptureMode = 2;
    int idPlayMode = 3;
    int idReadFileCommand = 4;
    int idReadMovementFile = 5;
    int idDecodeInstruction = 6;
    int idMechanicMovement = 7;
    int idCheckPosition = 8;
    int idReorderAutomatic = 9;
    int idIntermediateCalib = 10;
    int idEndGame = 11;
    int idDeleteFileChess = 12;
    int idWriteFileChess = 13;
    int idPlayNextFile = 14;
    int idEndSculpture = 15;
    int idReceiveInitialFen = 16;
    int idUpgradeInitialFen = 17;
    int idInitBoardFromFen = 18;
    int idCheckTurn = 19;
    int idPlayerMoveChess = 20;
    int idReceiveNewFen = 21;
    int idMechanicMoveChess = 22;
    int idDetectMove = 23;
    int idSendNewFen = 24;
    int idCheckMovement = 25;
    int idReturnPieceToPreviousPosition = 26;
    int idCompareEngineMechanism = 27;
    int idUpdateFen = 28;
    int idCheckMate = 29;

    int currentState;
    int currentInput;

    //==============Prototipo de funciones ==========================
    // Acciones de los estados y condiciones de transiciones
    void stateReadMode(void);
    void stateSculptureMode(void);
    void statePlayMode(void);
    void stateReadFileCommand(void);
    void stateReadMovementFile(void);
    void stateDecodeInstruction(void);
    void stateMechanicMovement(void);
    void stateCheckPosition(void);
    void stateReorderAutomatic(void);
    void stateIntermediateCalib(void);
    void stateEndGame(void);
    void stateDeleteFileChess(void);
    void stateWriteFileChess(void);
    void statePlayNextFile(void);
    void stateEndSculpture(void);
    void stateReceiveInitialFen(void);
    void stateUpgradeInitialFen(void);
    void stateInitBoardFromFen(void);
    void stateCheckTurn(void);
    void statePlayerMoveChess(void);
    void stateReceiveNewFen(void);
    void stateMechanicMoveChess(void);
    void stateDetectMove(void);
    void stateSendNewFen(void);
    void stateCheckMovement(void);
    void stateReturnPieceToPreviousPosition(void);
    void stateCompareEngineMechanism(void);
    void stateUpdateFen(void);
    void stateCheckMate(void);

    // Salidas asociadas a las transiciones
    void readMode(void);
    void sculptureMode(void);
    void playMode(void);
    void readFileCommand(void);
    void readMovementFile(void);
    void decodeInstruction(void);
    void mechanicMovement(void);
    void checkPosition(void);
    void reorderAutomatic(void);
    void intermediateCalib(void);
    void endGame(void);
    void deleteFileChess(void);
    void writeFileChess(void);
    void playNextFile(void);
    void endSculpture(void);
    void receiveInitialFen(void);
    void upgradeInitialFen(void);
    void initBoardFromFen(void);
    void checkTurn(void);
    void playerMoveChess(void);
    void receiveNewFen(void);
    void mechanicMoveChess(void);
    void detectMove(void);
    void sendNewFen(void);
    void checkMovement(void);
    void returnPieceToPreviousPosition(void);
    void compareEngineMechanism(void);
    void updateFen(void);
    void checkMate(void);

    void initStateMachine(void);
    void runStateMachine(void); 
    void updateStateMachine(void);
    void changeState(int);
/*================================================================================================*/

//=================Funciones Ajedrez=============================
double longitud = 50; 
void coordenadas(char, char, double*, double*);
void infoChessMovement(char[], char*, char*, char*, char*, char*, int*);
void chess_King(char, char, char, char, int, bool, char);
void chess_Queen(char, char, char, char, int, bool, char);
void chess_Rook(char, char, char, char, int, bool, char);
void chess_Bishop(char, char, char, char, int, bool, char);
void chess_Knight(char, char, char, char, int, bool, char);
void chess_Pawn(char, char, char, char, int, bool, char);

void comerVersion3(char, char, int , bool, char);
void enroque_corto(bool);
void enroque_largo(bool);
void moveChessPiece( char, char, char, char, int, bool);
void moveChessPieceKnight(char, char, char, char, int, bool, char);

void shortCastlingRook(bool);
void longCastlingRook(bool);

void updateFenFromMatriz(bool, String,String);
void printMatriz();
void upgradeMatriz(char, char, char, char, int, bool, char);
char convertIntChar(int);
String castlingFEN(int,int,int,int,int,int);
String capturaAlPasoFen(char,char,bool);
void removeChessAuto(void);
void imprimirVectPiecesD(void);
char consultaPiezaTablero(char, char);
void initializeMatriz(void);
void consultaCoordPiezasMuertas(char,bool,double*,double*);
void reorderAuto(void);
void reorderAutoV2(void);
void reorderAutoForzado(void);
void moveOnTheLine( double, double, double, double);
void moveOnTheLinev2( double, double, double, double, int numElectro = 0);
void moveOnTheLineIni(double, double, double,  double);
void moveOnTheLineIniv2(double, double, double,  double);

void puntosDeCurva(double,double,double,double,double,double,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*,double*);
void puntosDeCurvaV2(double ,double ,double ,double ,double ,double , double *,double *,int);


void generateMatrizFromFEN(String);
void initFromFEN(String);
void upgradeInitFen(String);

void activateElectromagnet();
void deactivateElectromagnet();
void testPWMElectromagnetDeactivate();
void testPWMElectromagnetActivate();

int mainElectro = 1;

void activateElectromagnetV4E(int);
void deactivateElectromagnetV4E(int);
void testPWMElectromagnetDeactivateV4E(int);
void testPWMElectromagnetActivateV4E(int);



void configDriver(void);

int checkFinalPosition(char , char);
int checkFinalPositionGraveyard(bool);
void checkAreaChess(float ,float ,int );
int readFromSerial();
String readCommandInput(void);
BoardPosition getBoardPositionFromString(String position);

//===========================Files Functions==========================
void createNewFileChess(fs::FS &, const char * , const char * );
void readFileChess(fs::FS &, const char * );
void addContentFileChess(fs::FS &, const char * , const char *);
void deleteFile(fs::FS &, const char * );
void listFilesChess(fs::FS &, const char *);
void listFilesChessTest(fs::FS &, const char *);
bool readFromFile(fs::FS &, int);
void IdDeleteFile(fs::FS &,const char * );

void saveGamesFirstTime(void);


int idActualFile = 0;
int totalFiles = 0;
const char * dirName;
int numMovement = 1;
String movesInFile;

//*------------------ Funciones para comunicacion con Novag -----------------*/
void initGame(void);
void sendMoveFromSerial(void);
void recibeMove(void);
void realizaJugada(char , char, char, char, int, bool, char);
String jugadaAnterior = " ";
String resultCastlingFEN = "KQkq";
String resultCapturaAlPasoFen = "-";
int rookKingW = 0;
int rookQueenW = 0;
int rookKingB = 0;
int rookQueenB = 0;
int kingW = 0;
int kingB = 0;

int receiveMovementSerial(void);
void menu(void);

/*=========================================== Funciones Sensores ===========================================*/
//Globar Variables for ChessBoard
//byte dirSensor[6][6][6]; // Direccion de los sensores por Barra, Cuadro y Sensor
byte dirSensor[10][10]; // Direccion de los sensores por Barra, Cuadro y Sensor
bool muxValues [10][10][10];//Valores guardados de los  de sensores por Mux, Barra Intermedia, Barra sensores. 
bool orderedSensorValues[10][10];//Valores en orden para mandar a processing. 
bool previousSensorValures[100][5];

bool muxSquaresValues[6][6][6];//Valores guardador del resultador de 5 sensores en Or por cuadro. 
bool orderedSquaresValues[100];
bool valorPasadoCuadroAlu[100];

bool chessAnterior[10][10] = {0};
int iniChangeX = 0;
int iniChangeY = 0;
int finChangeX = 0;
int finChangeY = 0;

int totalPiecesonBoard = 32;
int PiecesonBoardAnterior = 32;
int totalPiezasMuertas = 0;
int contSensoresPiezas = 0;
int contSensoresPiezasMuertas = 0;
int contAnteriorPiezasMuertas = 0;
int bandAnteriorComio = 0;
bool comioPieza = true;
int specialMove = 0;
bool vectGraveyard[36];
int periodo = 1000;
unsigned long TiempoAhora = 0;

void readSensors3(void);
void sensorsDir(void);
void readRawChessBoard(void); 
void detectChessBoard(void);
void detectChessBoardVerif(void);
int detectChange(void);
String upgradeChessboardFromSensors(bool*);
int contChessSensors(void);


void centrarPiezasIni(void);

void impresionSerialEasyPeasy(int );

void compareVirtualMatrizVsSensors(void);

void centrarPiezasAlgortimoConSensores(void);
void detectChessBoardAlgoritmoSensores(void);


void testVibrador(void);


//======================BLUETOOTH==========================
//Bluetooth BluetoothChess;
bool deviceConnected = false;
String textoEnviado;
//=========================================================


Servo myservo;
/*================================================================================================*/

//Si el mecanismo es tipo H
     
     #if MACHINE_STYLE == ROBOT_H
      MechanismH Robot;
      CalibrationXY haloCalib;
     #endif

//Si el mecanismo es de hilos

     #if MACHINE_STYLE == ROBOT_4THREADS
      Mechanism Robot;
      Calibration haloCalib;
     #endif

/**------------------------------------------------------------------------------------------------
 **                                         SETUP
 *------------------------------------------------------------------------------------------------**/
void setup()
{
        Serial.begin(115200);
        //====Init watchdog====
        esp_task_wdt_init(15, false);

//=======================================
/*
        int totalPuntosEnCurva = 7;

        double arrayPuntosX[totalPuntosEnCurva + 1] = {0};   //Se contempla un elemento mas del vector para contemplar desde el punto inicial hasta el punto final de la curva
        double arrayPuntosY[totalPuntosEnCurva + 1] = {0};
        puntosDeCurvaV2(0 ,50 ,25 ,50 ,25 ,25, arrayPuntosX,arrayPuntosY,totalPuntosEnCurva);
        for(int i = 0; i < (totalPuntosEnCurva + 1); i++)
        {
            Serial.print(arrayPuntosX[i]);
            Serial.print("\t");
            Serial.println(arrayPuntosY[i]);
        }
        while(true)
        {

        }
*/
//=======================================
        pinMode(LED_PIN, OUTPUT);
        pinMode(12, OUTPUT);
        pinMode(23, OUTPUT);
        pinMode(22, OUTPUT);
        
        #ifdef version4Electro
        deactivateElectromagnetV4E(1);
        deactivateElectromagnetV4E(2);
        deactivateElectromagnetV4E(3);
        deactivateElectromagnetV4E(4);
        #endif


        //Inicializacion de BLE
        //===========================================
        Serial.print("HW v");
        Serial.print(HARDWARE_VERSION_MAJOR);
        Serial.print(".");
        Serial.println(HARDWARE_VERSION_MINOR);

        Serial.print("SW v");
        Serial.print(SOFTWARE_VERSION_MAJOR);
        Serial.print(".");
        Serial.print(SOFTWARE_VERSION_MINOR);
        Serial.print(".");
        Serial.println(SOFTWARE_VERSION_PATCH);

        BT.begin("Chess Bluetooth T");
        //===========================================
        
        while(!SPIFFS.begin(true)){
        Serial.println("SPIFFS Mount Failed");
        //return;
        }
        
        listFilesChess(SPIFFS,"/");
        Serial.println("Numero de Archivos");
        Serial.println(totalFiles);
        delay(5000);
        //saveGamesFirstTime();
        if(totalFiles == 0)
        {
            //saveGamesFirstTime();
            const char *contentsGame;

    String nameFileIni = "";

    while(!SPIFFS.begin(true)){
    Serial.println("SPIFFS Mount Failed");
    }
    Serial.println("Saving Games ...");

    createNewFileChess(SPIFFS, "/Game0.txt", Game0);

    createNewFileChess(SPIFFS, "/Game1.txt", Game1);
    
    createNewFileChess(SPIFFS, "/Game2.txt", Game2);

    createNewFileChess(SPIFFS, "/Game3.txt", Game3);

    createNewFileChess(SPIFFS, "/Game4.txt", Game4);

    createNewFileChess(SPIFFS, "/Game5.txt", Game5);
    
    createNewFileChess(SPIFFS, "/Game6.txt", Game6);

    createNewFileChess(SPIFFS, "/Game7.txt", Game7);

    createNewFileChess(SPIFFS, "/Game8.txt", Game8);

    createNewFileChess(SPIFFS, "/Game9.txt", Game9);

    createNewFileChess(SPIFFS, "/Game10.txt", Game10);

    createNewFileChess(SPIFFS, "/Game11.txt", Game11);
    
    createNewFileChess(SPIFFS, "/Game12.txt", Game12);

    createNewFileChess(SPIFFS, "/Game13.txt", Game13);

    createNewFileChess(SPIFFS, "/Game14.txt", Game14);

    createNewFileChess(SPIFFS, "/Game15.txt", Game15);

    createNewFileChess(SPIFFS, "/Game16.txt", Game16);

    createNewFileChess(SPIFFS, "/Game17.txt", Game17);

    createNewFileChess(SPIFFS, "/Game18.txt", Game18);

    createNewFileChess(SPIFFS, "/Game19.txt", Game19);

    createNewFileChess(SPIFFS, "/Game20.txt", Game20);

    createNewFileChess(SPIFFS, "/Game21.txt", Game21);
    
    createNewFileChess(SPIFFS, "/Game22.txt", Game22);

    createNewFileChess(SPIFFS, "/Game23.txt", Game23);

    createNewFileChess(SPIFFS, "/Game24.txt", Game24);

    createNewFileChess(SPIFFS, "/Game25.txt", Game25);

    createNewFileChess(SPIFFS, "/Game26.txt", Game26);

    createNewFileChess(SPIFFS, "/Game27.txt", Game27);

    createNewFileChess(SPIFFS, "/Game28.txt", Game28);

    createNewFileChess(SPIFFS, "/Game29.txt", Game29);

    createNewFileChess(SPIFFS, "/Game30.txt", Game30);

    createNewFileChess(SPIFFS, "/Game31.txt", Game31);
    
    createNewFileChess(SPIFFS, "/Game32.txt", Game32);

    createNewFileChess(SPIFFS, "/Game33.txt", Game33);

    createNewFileChess(SPIFFS, "/Game34.txt", Game34);

    createNewFileChess(SPIFFS, "/Game35.txt", Game35);

    createNewFileChess(SPIFFS, "/Game36.txt", Game36);

    createNewFileChess(SPIFFS, "/Game37.txt", Game37);

    createNewFileChess(SPIFFS, "/Game38.txt", Game38);

    createNewFileChess(SPIFFS, "/Game39.txt", Game39);

    createNewFileChess(SPIFFS, "/Game40.txt", Game40);

    createNewFileChess(SPIFFS, "/Game41.txt", Game41);
    
    createNewFileChess(SPIFFS, "/Game42.txt", Game42);

    createNewFileChess(SPIFFS, "/Game43.txt", Game43);

    createNewFileChess(SPIFFS, "/Game44.txt", Game44);

    createNewFileChess(SPIFFS, "/Game45.txt", Game45);

    createNewFileChess(SPIFFS, "/Game46.txt", Game46);

    createNewFileChess(SPIFFS, "/Game47.txt", Game47);

    createNewFileChess(SPIFFS, "/Game48.txt", Game48);

    createNewFileChess(SPIFFS, "/Game49.txt", Game49);

    createNewFileChess(SPIFFS, "/Game50.txt", Game50);

    createNewFileChess(SPIFFS, "/Game51.txt", Game51);
    
    createNewFileChess(SPIFFS, "/Game52.txt", Game52);

    createNewFileChess(SPIFFS, "/Game53.txt", Game53);

    createNewFileChess(SPIFFS, "/Game54.txt", Game54);

    createNewFileChess(SPIFFS, "/Game55.txt", Game55);

    createNewFileChess(SPIFFS, "/Game56.txt", Game56);

    createNewFileChess(SPIFFS, "/Game57.txt", Game57);

    createNewFileChess(SPIFFS, "/Game58.txt", Game58);

    createNewFileChess(SPIFFS, "/Game59.txt", Game59);

    createNewFileChess(SPIFFS, "/Game60.txt", Game60);

    createNewFileChess(SPIFFS, "/Game61.txt", Game61);
    
    createNewFileChess(SPIFFS, "/Game62.txt", Game62);

    createNewFileChess(SPIFFS, "/Game63.txt", Game63);

    createNewFileChess(SPIFFS, "/Game64.txt", Game64);

    createNewFileChess(SPIFFS, "/Game65.txt", Game65);

    createNewFileChess(SPIFFS, "/Game66.txt", Game66);

    createNewFileChess(SPIFFS, "/Game67.txt", Game67);

    createNewFileChess(SPIFFS, "/Game68.txt", Game68);

    createNewFileChess(SPIFFS, "/Game69.txt", Game69);

    createNewFileChess(SPIFFS, "/Game70.txt", Game70);

    createNewFileChess(SPIFFS, "/Game71.txt", Game71);
    
    createNewFileChess(SPIFFS, "/Game72.txt", Game72);

    createNewFileChess(SPIFFS, "/Game73.txt", Game73);

    createNewFileChess(SPIFFS, "/Game74.txt", Game74);

    createNewFileChess(SPIFFS, "/Game75.txt", Game75);

    createNewFileChess(SPIFFS, "/Game76.txt", Game76);

    createNewFileChess(SPIFFS, "/Game77.txt", Game77);

    createNewFileChess(SPIFFS, "/Game78.txt", Game78);

    createNewFileChess(SPIFFS, "/Game79.txt", Game79);

    createNewFileChess(SPIFFS, "/Game80.txt", Game80);

    createNewFileChess(SPIFFS, "/Game81.txt", Game81);
    
    createNewFileChess(SPIFFS, "/Game82.txt", Game82);

    createNewFileChess(SPIFFS, "/Game83.txt", Game83);

    createNewFileChess(SPIFFS, "/Game84.txt", Game84);

    createNewFileChess(SPIFFS, "/Game85.txt", Game85);

    createNewFileChess(SPIFFS, "/Game86.txt", Game86);

    createNewFileChess(SPIFFS, "/Game87.txt", Game87);

    createNewFileChess(SPIFFS, "/Game88.txt", Game88);

    createNewFileChess(SPIFFS, "/Game89.txt", Game89);

    createNewFileChess(SPIFFS, "/Game90.txt", Game90);

    createNewFileChess(SPIFFS, "/Game91.txt", Game91);
    
    createNewFileChess(SPIFFS, "/Game92.txt", Game92);

    createNewFileChess(SPIFFS, "/Game93.txt", Game93);

    createNewFileChess(SPIFFS, "/Game94.txt", Game94);

    createNewFileChess(SPIFFS, "/Game95.txt", Game95);

    createNewFileChess(SPIFFS, "/Game96.txt", Game96);

    createNewFileChess(SPIFFS, "/Game97.txt", Game97);

    createNewFileChess(SPIFFS, "/Game98.txt", Game98);

    createNewFileChess(SPIFFS, "/Game99.txt", Game99);


    createNewFileChess(SPIFFS, "/Game100.txt", Game100);

    createNewFileChess(SPIFFS, "/Game101.txt", Game101);
    
    createNewFileChess(SPIFFS, "/Game102.txt", Game102);

    createNewFileChess(SPIFFS, "/Game103.txt", Game103);

    createNewFileChess(SPIFFS, "/Game104.txt", Game104);

    createNewFileChess(SPIFFS, "/Game105.txt", Game105);
    
    createNewFileChess(SPIFFS, "/Game106.txt", Game106);

    createNewFileChess(SPIFFS, "/Game107.txt", Game107);

    createNewFileChess(SPIFFS, "/Game108.txt", Game108);

    createNewFileChess(SPIFFS, "/Game109.txt", Game109);

    createNewFileChess(SPIFFS, "/Game110.txt", Game110);

    createNewFileChess(SPIFFS, "/Game111.txt", Game111);
    
    createNewFileChess(SPIFFS, "/Game112.txt", Game112);

    createNewFileChess(SPIFFS, "/Game113.txt", Game113);

    createNewFileChess(SPIFFS, "/Game114.txt", Game114);

    createNewFileChess(SPIFFS, "/Game115.txt", Game115);

    createNewFileChess(SPIFFS, "/Game116.txt", Game116);

    createNewFileChess(SPIFFS, "/Game117.txt", Game117);

    createNewFileChess(SPIFFS, "/Game118.txt", Game118);

    createNewFileChess(SPIFFS, "/Game119.txt", Game119);

    createNewFileChess(SPIFFS, "/Game120.txt", Game120);

    createNewFileChess(SPIFFS, "/Game121.txt", Game121);
    
    createNewFileChess(SPIFFS, "/Game122.txt", Game122);

    createNewFileChess(SPIFFS, "/Game123.txt", Game123);

    createNewFileChess(SPIFFS, "/Game124.txt", Game124);

    createNewFileChess(SPIFFS, "/Game125.txt", Game125);

    createNewFileChess(SPIFFS, "/Game126.txt", Game126);

    createNewFileChess(SPIFFS, "/Game127.txt", Game127);

    createNewFileChess(SPIFFS, "/Game128.txt", Game128);

    createNewFileChess(SPIFFS, "/Game129.txt", Game129);

    createNewFileChess(SPIFFS, "/Game130.txt", Game130);

    createNewFileChess(SPIFFS, "/Game131.txt", Game131);
    
    createNewFileChess(SPIFFS, "/Game132.txt", Game132);

    createNewFileChess(SPIFFS, "/Game133.txt", Game133);

    createNewFileChess(SPIFFS, "/Game134.txt", Game134);

    createNewFileChess(SPIFFS, "/Game135.txt", Game135);

    createNewFileChess(SPIFFS, "/Game136.txt", Game136);

    createNewFileChess(SPIFFS, "/Game137.txt", Game137);

    createNewFileChess(SPIFFS, "/Game138.txt", Game138);

    createNewFileChess(SPIFFS, "/Game139.txt", Game139);

    createNewFileChess(SPIFFS, "/Game140.txt", Game140);

    createNewFileChess(SPIFFS, "/Game141.txt", Game141);
    
    createNewFileChess(SPIFFS, "/Game142.txt", Game142);

    createNewFileChess(SPIFFS, "/Game143.txt", Game143);

    createNewFileChess(SPIFFS, "/Game144.txt", Game144);

    createNewFileChess(SPIFFS, "/Game145.txt", Game145);

    createNewFileChess(SPIFFS, "/Game146.txt", Game146);

    createNewFileChess(SPIFFS, "/Game147.txt", Game147);

    createNewFileChess(SPIFFS, "/Game148.txt", Game148);

    createNewFileChess(SPIFFS, "/Game149.txt", Game149);

    createNewFileChess(SPIFFS, "/Game150.txt", Game150);

    createNewFileChess(SPIFFS, "/Game151.txt", Game151);
    
    createNewFileChess(SPIFFS, "/Game152.txt", Game152);

    createNewFileChess(SPIFFS, "/Game153.txt", Game153);

    createNewFileChess(SPIFFS, "/Game154.txt", Game154);

    createNewFileChess(SPIFFS, "/Game155.txt", Game155);

    createNewFileChess(SPIFFS, "/Game156.txt", Game156);

    createNewFileChess(SPIFFS, "/Game157.txt", Game157);

    createNewFileChess(SPIFFS, "/Game158.txt", Game158);

    createNewFileChess(SPIFFS, "/Game159.txt", Game159);

    createNewFileChess(SPIFFS, "/Game160.txt", Game160);

    createNewFileChess(SPIFFS, "/Game161.txt", Game161);
    
    createNewFileChess(SPIFFS, "/Game162.txt", Game162);

    createNewFileChess(SPIFFS, "/Game163.txt", Game163);

    createNewFileChess(SPIFFS, "/Game164.txt", Game164);

    createNewFileChess(SPIFFS, "/Game165.txt", Game165);

    createNewFileChess(SPIFFS, "/Game166.txt", Game166);

    createNewFileChess(SPIFFS, "/Game167.txt", Game167);

    createNewFileChess(SPIFFS, "/Game168.txt", Game168);

    createNewFileChess(SPIFFS, "/Game169.txt", Game169);

    createNewFileChess(SPIFFS, "/Game170.txt", Game170);

    createNewFileChess(SPIFFS, "/Game171.txt", Game171);
    
    createNewFileChess(SPIFFS, "/Game172.txt", Game172);

    createNewFileChess(SPIFFS, "/Game173.txt", Game173);

    createNewFileChess(SPIFFS, "/Game174.txt", Game174);

    createNewFileChess(SPIFFS, "/Game175.txt", Game175);

    createNewFileChess(SPIFFS, "/Game176.txt", Game176);

    createNewFileChess(SPIFFS, "/Game177.txt", Game177);

    createNewFileChess(SPIFFS, "/Game178.txt", Game178);

    createNewFileChess(SPIFFS, "/Game179.txt", Game179);

    createNewFileChess(SPIFFS, "/Game180.txt", Game180);

    createNewFileChess(SPIFFS, "/Game181.txt", Game181);
    
    createNewFileChess(SPIFFS, "/Game182.txt", Game182);

    createNewFileChess(SPIFFS, "/Game183.txt", Game183);

    createNewFileChess(SPIFFS, "/Game184.txt", Game184);

    createNewFileChess(SPIFFS, "/Game185.txt", Game185);

    createNewFileChess(SPIFFS, "/Game186.txt", Game186);

    createNewFileChess(SPIFFS, "/Game187.txt", Game187);

    createNewFileChess(SPIFFS, "/Game188.txt", Game188);

    createNewFileChess(SPIFFS, "/Game189.txt", Game189);

    createNewFileChess(SPIFFS, "/Game190.txt", Game190);

    createNewFileChess(SPIFFS, "/Game191.txt", Game191);
    
    createNewFileChess(SPIFFS, "/Game192.txt", Game192);

    createNewFileChess(SPIFFS, "/Game193.txt", Game193);

    createNewFileChess(SPIFFS, "/Game194.txt", Game194);

    createNewFileChess(SPIFFS, "/Game195.txt", Game195);

    createNewFileChess(SPIFFS, "/Game196.txt", Game196);

    createNewFileChess(SPIFFS, "/Game197.txt", Game197);

    createNewFileChess(SPIFFS, "/Game198.txt", Game198);

    createNewFileChess(SPIFFS, "/Game199.txt", Game199);
    
        }

        #ifdef automaticInitSculpture
        Serial.println("Modo Escultura inicia automaticamente");
        #endif

        #ifdef SelectModeBluetooth
        Serial.println("Modo de inicio desde Bluetooth");
        #endif

        #ifdef automaticInitPlayMode
        Serial.println("Modo play inicia automaticamente");
        #endif

        #ifdef activateSensors
        Serial.println("Version con sensores activados");
        #else
        Serial.println("Version sin sensores");
        #endif

        #ifdef newSensors
        Serial.println("Tipo de sensores: Nuevos");
        #endif
        
        #ifdef oldSensors
        Serial.println("Tipo de sensores: Viejos");
        #endif

        

        /*
        
        BluetoothChess.init("Chess");

        float outputValue1 = 100;
            float outputValue2 = 300;
            float outputValue3 = 500;
        while (true)
        {
           
            float outputValue = 0;
            
            outputValue = 10;
            
            char outputString[8]; 
            outputValue = random(0,100);
            outputValue1++;
            //dtostrf(outputValue, 1, 2, outputString);
            //BluetoothChess.setName(outputString);
            dtostrf(outputValue1, 1, 2, outputString);
            BluetoothChess.setName(outputString);
            Serial.print("Dato A: ");
            Serial.println(outputString);

            outputValue = random(0,100);
            outputValue2++;
            //dtostrf(outputValue, 1, 2, outputString);
            //BluetoothChess.setStatus(outputString);
            dtostrf(outputValue2, 1, 2, outputString);
            BluetoothChess.setStatus(outputString);
            Serial.print("Dato B: ");
            Serial.println(outputString);

            outputValue = random(0,100);
            outputValue3++;
            //dtostrf(outputValue, 1, 2, outputString);
            //BluetoothChess.setLedSpeed(outputString);
            dtostrf(outputValue3, 1, 2, outputString);
            BluetoothChess.setLedSpeed(outputString);
            Serial.print("Dato C: ");
            Serial.println(outputString);
            Serial.println(" ");

            
            
        }
        */
        
        pinMode(mux8_0, OUTPUT);
        pinMode(mux8_1, OUTPUT);
        pinMode(mux8_2, OUTPUT);

        pinMode(mux16_0, OUTPUT);
        pinMode(mux16_1, OUTPUT);
        pinMode(mux16_2, OUTPUT);

        pinMode(mux16Out_1, INPUT);
        pinMode(mux16Out_2, INPUT);
        pinMode(mux16Out_3, INPUT);
        pinMode(mux16Out_4, INPUT);

        //Funcion de test de sensores

        //Funcion de test de vibrador y sensores

        //Funcion de test electroiman con sensores 

        
        haloCalib.initCalibration();
        #ifdef version1Electromagnet
        deactivateElectromagnet();
        #endif

        #ifdef version4Electro
        deactivateElectromagnetV4E(1);
        deactivateElectromagnetV4E(2);
        deactivateElectromagnetV4E(3);
        deactivateElectromagnetV4E(4);
     

        /*

        delay(5000);
        activateElectromagnetV4E(1);
        delay(3000);
        deactivateElectromagnetV4E(1);

        delay(5000);
        activateElectromagnetV4E(2);
        delay(3000);
        deactivateElectromagnetV4E(2);

        delay(5000);
        activateElectromagnetV4E(3);
        delay(3000);
        deactivateElectromagnetV4E(3);

        delay(5000);
        activateElectromagnetV4E(4);
        delay(3000);
        deactivateElectromagnetV4E(4);
        */


        #endif

        haloCalib.startCalibration();
        #ifdef debug
        Serial.println("Termina Calibracion");
        #endif

        #ifdef stateMachineActivate
        //=========Test State Machine ========
        Robot.init();
        configDriver();


    

        Robot.accelRamp(225,175,2,0,2);
        Robot.accelRamp(175,175,2,0,1);
        Robot.accelRamp(175,225,2,0,1);
        Robot.accelRamp(-175,225,2,0,1);
        Robot.accelRamp(-175,175,2,0,1);
        Robot.accelRamp(-225,175,2,0,4);
        Robot.accelRamp(-225,-175,2,0,4);
        Robot.accelRamp(-175,-175,2,0,1);
        Robot.accelRamp(-175,-225,2,0,3);
        Robot.accelRamp(175,-225,2,0,3);
        Robot.accelRamp(175,-175,2,0,1);
        Robot.accelRamp(225,-175,2,0,2);
        Robot.accelRamp(0,0,2,0,1);

        /*
        Robot.setSpeedMotors(defGlobalSpeed);
        Robot.setSpeedRampFunction(defGlobalSpeed);
        Robot.setAccelRampFunction(defGlobalAccel);

        Robot.accelRamp(25,25,2);
        while(true)
        {    
            Robot.setSpeedMotors(defGlobalSpeed);
        Robot.setSpeedRampFunction(defGlobalSpeed);
        Robot.setAccelRampFunction(defGlobalAccel);
        moveOnTheLinev2(25,25,75,75,mainElectro);
        delay(3000);
        Robot.setSpeedMotors(defGlobalSpeed);
        Robot.setSpeedRampFunction(defGlobalSpeed);
        Robot.setAccelRampFunction(defGlobalAccel);
        moveOnTheLinev2(75,75,25,25,mainElectro);
        delay(3000);
        
        }
        */


        #ifdef modoDemostracion
        Robot.setSpeedMotors(defGlobalSpeed);
        Robot.setSpeedRampFunction(defGlobalSpeed);
        Robot.setAccelRampFunction(defGlobalAccel);
        while(true)
        {
            #ifdef version1Electro
            testPWMElectromagnetActivate();
            Robot.accelRamp(125,125,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(25,25,2);
            Robot.accelRamp(125,125,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(-125,125,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(125,125,2);
            Robot.accelRamp(-125,125,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(125,-125,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(-125,125,2);
            Robot.accelRamp(125,-125,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(-125,-125,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(125,-125,2);
            Robot.accelRamp(-125,-125,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(-125,75,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(-125,-125,2);
            Robot.accelRamp(-125,75,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(175,75,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(-125,75,2);
            Robot.accelRamp(175,75,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(-75,-175,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(175,75,2);
            Robot.accelRamp(-75,-175,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(-75,175,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(-75,-175,2);
            Robot.accelRamp(-75,175,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(175,175,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(-75,175,2);
            Robot.accelRamp(175,175,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(175,-75,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(175,175,2);
            Robot.accelRamp(175,-75,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(75,25,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(175,-75,2);
            Robot.accelRamp(75,25,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(75,-75,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(75,25,2);
            Robot.accelRamp(75,-75,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(-75,-75,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(75,-75,2);
            Robot.accelRamp(-75,-75,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(-75,75,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(-75,-75,2);
            Robot.accelRamp(-75,75,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(25,-25,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(-75,75,2);
            Robot.accelRamp(25,-25,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(25,125,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(25,-25,2);
            Robot.accelRamp(25,125,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(-100,0,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(25,125,2);
            Robot.accelRamp(-100,0,2);
            //-------------------------------

            testPWMElectromagnetActivate();
            Robot.accelRamp(0,0,2,1);
            activateElectromagnet();

            deactivateElectromagnet();
            Robot.accelRamp(-100,0,2);
            Robot.accelRamp(0,0,2);
            //-------------------------------
            #endif


            #ifdef version4Electro
/*
            int Electro = 3;
            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(0,175,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(0,-175,2,0,Electro);
            Robot.accelRamp(0,175,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-175,0,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(0,0,2,0,Electro);
            Robot.accelRamp(175,0,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-175,0,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(175,0,2,0,Electro);
            Robot.accelRamp(-175,0,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(0,0,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(175,175,2,0,Electro);
            Robot.accelRamp(-175,-175,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(175,175,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-175,-175,2,0,Electro);
            Robot.accelRamp(0,0,2,0,Electro);
            //-------------------------------
            */
/*
            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(175,75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-125,75,2,0,Electro);
            Robot.accelRamp(175,75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-75,-175,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(175,75,2,0,Electro);
            Robot.accelRamp(-75,-175,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-75,175,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-75,-175,2,0,Electro);
            Robot.accelRamp(-75,175,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(175,175,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-75,175,2,0,Electro);
            Robot.accelRamp(175,175,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(175,-75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(175,175,2,0,Electro);
            Robot.accelRamp(175,-75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(75,25,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(175,-75,2,0,Electro);
            Robot.accelRamp(75,25,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(75,-75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(75,25,2,0,Electro);
            Robot.accelRamp(75,-75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-75,-75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(75,-75,2,0,Electro);
            Robot.accelRamp(-75,-75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-75,75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-75,-75,2,0,Electro);
            Robot.accelRamp(-75,75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(25,-25,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-75,75,2,0,Electro);
            Robot.accelRamp(25,-25,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(25,125,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(25,-25,2,0,Electro);
            Robot.accelRamp(25,125,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-100,0,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(25,125,2,0,Electro);
            Robot.accelRamp(-100,0,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(0,0,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-100,0,2,0,Electro);
            Robot.accelRamp(0,0,2,0,Electro);
            //-------------------------------
            */

            
            int Electro = 3;
            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(125,125,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(25,25,2,0,Electro);
            Robot.accelRamp(125,125,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-125,125,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(125,125,2,0,Electro);
            Robot.accelRamp(-125,125,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(125,-125,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-125,125,2,0,Electro);
            Robot.accelRamp(125,-125,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-125,-125,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(125,-125,2,0,Electro);
            Robot.accelRamp(-125,-125,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-125,75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-125,-125,2,0,Electro);
            Robot.accelRamp(-125,75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(175,75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-125,75,2,0,Electro);
            Robot.accelRamp(175,75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-75,-175,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(175,75,2,0,Electro);
            Robot.accelRamp(-75,-175,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-75,175,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-75,-175,2,0,Electro);
            Robot.accelRamp(-75,175,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(175,175,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-75,175,2,0,Electro);
            Robot.accelRamp(175,175,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(175,-75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(175,175,2,0,Electro);
            Robot.accelRamp(175,-75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(75,25,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(175,-75,2,0,Electro);
            Robot.accelRamp(75,25,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(75,-75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(75,25,2,0,Electro);
            Robot.accelRamp(75,-75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-75,-75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(75,-75,2,0,Electro);
            Robot.accelRamp(-75,-75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-75,75,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-75,-75,2,0,Electro);
            Robot.accelRamp(-75,75,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(25,-25,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-75,75,2,0,Electro);
            Robot.accelRamp(25,-25,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(25,125,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(25,-25,2,0,Electro);
            Robot.accelRamp(25,125,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(-100,0,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(25,125,2,0,Electro);
            Robot.accelRamp(-100,0,2,0,Electro);
            //-------------------------------

            testPWMElectromagnetActivateV4E(Electro);
            Robot.accelRamp(0,0,2,1,Electro);
            activateElectromagnetV4E(Electro);

            deactivateElectromagnetV4E(Electro);
            Robot.accelRamp(-100,0,2,0,Electro);
            Robot.accelRamp(0,0,2,0,Electro);
            //-------------------------------

            
            #endif
            
        }




        #endif




        Robot.setSpeedRampFunction(500);
        Robot.setAccelRampFunction(8000);

        #ifdef activateSensors
        centrarPiezasAlgortimoConSensores();
        #endif
        Serial.println("Entra a la funcion initStateMachine");

        Robot.setSpeedMotors(defGlobalSpeed);
        Robot.setSpeedRampFunction(defGlobalSpeed);
        Robot.setAccelRampFunction(defGlobalAccel);
        

        initStateMachine();
        /*
        sensorsDir();

        while(true)
        {
            impresionSerialEasyPeasy(50);
        }
        */ 

        while(true)
        {
            runStateMachine();
        }
        #endif
        Robot.init();
        configDriver();
        Robot.setSpeedMotors(defGlobalSpeed);
        Robot.setSpeedRampFunction(defGlobalSpeed);
        Robot.setAccelRampFunction(defGlobalAccel);
        
        flag = 1;
}
//*------------------------------------------------------------------------------------------------*/

/**------------------------------------------------------------------------------------------------
 **                                         LOOP
 *------------------------------------------------------------------------------------------------**/
void loop()
{
	menu();
}
//*------------------------------------------------------------------------------------------------*/

int readFromSerial()
{
    int numData = 0;
    while (Serial.available()) {
        char uartValue = Serial.read();
        if (uartValue == ',' || uartValue == '\n') {
            //limitar commandRead a ser un commando valido para poder ser pusheado
            commands.push(commandRead);
            Serial.print("Commando ingresado al Buffer: ");
            Serial.println(commandRead);
            commandRead = "";
            numData += 1;
        } else {
            commandRead.concat(uartValue);
        }
    }
    a = commands.shift().toFloat();
    b = commands.shift().toFloat();
    c = commands.shift().toFloat();
    d = commands.shift().toFloat();
    e = commands.shift().toFloat();
    f = commands.shift().toFloat();
    g = commands.shift().toFloat();

    return numData;
}

/**================================================================================================
 **                                      Funcion readMovementFile
 *?  Esta funcion busca la jugada que sigue dentro de los archivos de partidas guaedadas para la reproduccion del
 *?  modo Escultura
 *@param name type
 *@return void
 *================================================================================================**/
bool readFromFile(fs::FS &fs, int nMovement)
{
    const char * dirNameFile = dirName;

    char mov_chess[7] = { 'v', 'v', 'v', 'v', 'v', 'v', 'v' };
    char char_actual = 'v';
    int index_mov = 0;
    int contMovement = 1;
    
    //====run program========================================================
    
    int band_vec = 0;
    int contOpenFile = 0;
    
    File myFileChess = fs.open(dirName);
    while(!myFileChess || myFileChess.isDirectory()){
        Serial.println("Failed to open");
        Serial.printf("Dir Name file: %s\r\n", dirName);
        Serial.println("");

        myFileChess.close();                 //Lo agregue para ver si esta relacionado con el problema de que de manera aleatoria se queda sin poder leer los archivos              
        myFileChess = fs.open(dirName);
        listFilesChessTest(SPIFFS,"/");
        delay(100);
        contOpenFile++;
        if(contOpenFile == 20)
        {
            movementFileGlobal = "1-0";
            return false;
            
        }
    }
    Serial.printf("Dir Name file: %s\r\n", dirName);
    Serial.println("");
    Serial.println("File Content:");

    int L = movesInFile.length();
    int indexA = 0;
    
    //while(myFileChess.available()){
    while(indexA < L){
        if (index_mov == 0) {
                //char_actual = (myFileChess.read());
                char_actual = movesInFile[indexA];
                indexA++;
        }
        if(char_actual == ' ') {
                index_mov = 0;
                do {
                    //if (myFileChess.available()) {
                    if (indexA < L) {
                        //char_actual = (myFileChess.read());
                          char_actual = movesInFile[indexA];
                          indexA++;
                    } else {
                        break;
                    }

                    if (char_actual != ' ') {
                        mov_chess[index_mov] = char_actual;
                        index_mov++;
                    }
                    
                } while (char_actual != ' ');

                for (int i = 0; i < 7; i++) //Busca si hay un punto el el vector para descartarlo como una instruccion de movimiento
                {
                    if (mov_chess[i] == '.') {
                        band_vec = 1;
                        for (int j = 0; j < 7; j++) {
                            mov_chess[j] = 'v'; //Inicializa nuevamente el vector
                        }
                    }
                }
                
                if (band_vec == 0) {

                    //infoChessMovement(mov_chess, Ap_x_ini, Ap_y_ini, Ap_x_fin, Ap_y_fin, Ap_pieza, Ap_accion);
                    String stringJugada = "";
                    for(int i = 0; i < 7; i++)
                    {
                            stringJugada = stringJugada + mov_chess[i];
                    }
                    if (contMovement == nMovement)
                    {
                        Serial.print("Movement: ");
                        Serial.print(contMovement);
                        Serial.print(" ");
                        Serial.println(stringJugada);
                        BT.setStatus(stringJugada);     //Test para enviar info por bluetooth

                        movementFileGlobal = stringJugada;
                        numMovement++;
                    }
                    contMovement++;
                }
                for(int i = 0; i < 7; i++)
                {
                    mov_chess[i] = 'v';

                }
                band_vec = 0;
            }
    }
    myFileChess.close();
    Serial.println("Close File");
    return true;
}
/*================================================================================================*/



/*================================================================================================*/



/*================================================================================================*/

//=========================Descripcion de simbolos==============================
//    PIEZAS
//    King: the letter K
//    Queen: the letter Q
//    Rook: the letter R
//    Bishop: the letter B
//    Knight: the letter N
//    Pawn: no letter assigned

//    Acciones
//    - : Mueve pieza
//   ' ': Mueve pieza
//    x : Come pieza
//    + : Pone en jaque
//    o : Enroque

//    acc
//    1 : solo mueve
//    2 : mueve y promocion
//    3 : mueve y come
//    4 : mueve y jaque
//    5 : mueve, promocion y jaque
//    6 : mueve, come y jaque
//    7 : Enroque corto
//    8 : Enroque largo
//    9 : Passant
//   10 : mueve, come y promocion

void infoChessMovement(char v[7], char* ini_c1, char* ini_c2, char* fin_c1, char* fin_c2, char* pieza, int* acc)
{
    char v_pieza[5] = { 'K', 'Q', 'R', 'B', 'N' };
    char v_accion[5] = { '-', ' ', 'x', '+', 'O' };
    char ini_fin[4] = { 'V', 'V', 'V', 'V' };
    char accion;
    int peon = 1;
    int come = 0;
    int jaque = 0;
    int enroque = 5;
    int move = 0;
    int promocion = 0;
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 5; j++) {
            if (v[i] == v_pieza[j]) {
                if(i != 0){
                    peon = 1;
                    promocion = 1;
                }
                else{
                    peon = 0;
                }
                *pieza = v_pieza[j];
                v[i] = 'v';
            }
            if (v[i] == v_accion[j]) {
                accion = v_accion[j];
                v[i] = 'v';
                if (accion == '-' || accion == ' ') {
                    move = 1;
                }
                if (accion == 'x') {
                    come = 3;
                }
                if (accion == '+') {
                    jaque = 3;
                }
                if (accion == 'O') {
                    enroque++;
                }
            }
        }
        if (peon == 1) {
            *pieza = 'P';
        }
    }
    if (enroque == 7) {
        *pieza = 'S'; //Enroque corto
    }
    if (enroque == 8) {
        *pieza = 'L'; //Enroque Largo
    }
    int k = 0;
    for (int i = 0; i < 7; i++) {
        if (v[i] != 'v') {
            ini_fin[k] = v[i];
            k++;
        }
    }
    *ini_c1 = ini_fin[0];
    *ini_c2 = ini_fin[1];
    *fin_c1 = ini_fin[2];
    *fin_c2 = ini_fin[3];

    Serial.println("Vector ini_fin");
    Serial.println(ini_fin[0]);
    Serial.println(ini_fin[1]);
    Serial.println("\n");
    Serial.println(ini_fin[2]);
    Serial.println(ini_fin[3]);

    if (enroque == 5) {
        *acc = come + jaque + move + promocion;
    } else {
        *acc = enroque;
    }

    if(come == 3 && promocion == 1)
    {
        *acc = 10;
    }

    if(ini_fin[0] == '1' && ini_fin[1] == '0'){
        Serial.println("Blancas ganan");
        *acc = 0;
    }
    if(ini_fin[0] == '0' && ini_fin[1] == '1'){
        Serial.println("Negras ganan");
        *acc = 0;
    }
    if(ini_fin[0] == '1' && ini_fin[1] == '2'){
        Serial.println("Empate");
        *acc = 0;
    }
    if(ini_fin[1] == 'e' && ini_fin[2] == 'p'){
        Serial.println("Passant");
        *acc = 9;
    }
}

void moveChessPiece(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color, char pieza)
{
    double compActualX, compActualY;
    double* apActualX = &compActualX;
    double* apActualY = &compActualY;
    double halfPointX;
    double halfPointY;


    double comp_ini_x, comp_ini_y;
    double* Ap_ini_x = &comp_ini_x;
    double* Ap_ini_y = &comp_ini_y;
    double comp_fin_x, comp_fin_y;
    double* Ap_fin_x = &comp_fin_x;
    double* Ap_fin_y = &comp_fin_y;

    Serial.println("Dentro de funcion moveChessPiece");
    Serial.println("Color del movimiento actual");
    if(chess_color == false)
    {
        Serial.println("Negras");
    }
    if(chess_color == true)
    {
        Serial.println("Blancas");
    }

    if (mode != 2)
    {
#ifdef activateSensors
    sensorsDir();
    detectChessBoard();
#endif
        if (movement == 3 || movement == 6 || movement == 9 || movement == 10)
        {
            comerVersion3(x_fin, y_fin, movement, chess_color, pieza);
#ifdef activateSensors
            int resultFinalPos;
            detectChessBoardVerif();
            resultFinalPos = checkFinalPositionGraveyard(chess_color);
            while (resultFinalPos == 0)
            {
                comerVersion3(x_fin, y_fin, movement, chess_color, pieza);
                detectChessBoardVerif();
                resultFinalPos = checkFinalPositionGraveyard(chess_color);
            }
#endif
        }
    }

    coordenadas(x_ini, y_ini, Ap_ini_x, Ap_ini_y);
    coordenadas(x_fin, y_fin, Ap_fin_x, Ap_fin_y);
    //=================================================================


    #ifdef funcMoveTo
    Robot.setSpeedMotors(maximumVelocityDirect);
    #endif
    #ifdef funcAccelRamp
    Robot.setSpeedMotors(maximunVelocityDirectRamp);
    Robot.setSpeedRampFunction(maximunVelocityDirectRamp);
    Robot.setAccelRampFunction(maximunAccelDirectRamp);
    #endif

    #ifdef moveDirectPosPiece
    
    #ifdef funcMoveTo
    Robot.moveToPointV2(comp_ini_x, comp_ini_y,1);
    
    #endif

    #ifdef funcAccelRamp
    #ifdef version1Electro
    Robot.accelRamp(comp_ini_x,comp_ini_y,2);       //Antes desacelracion,2
    #endif
    #ifdef version4Electro
    Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);       //Antes desacelracion,2
    #endif
    #endif

    #else
    moveOnTheLineIni(comp_ini_x, comp_ini_y, comp_fin_x, comp_fin_y);
    #endif

    #ifdef funcMoveTo
    Robot.setSpeedMotors(defGlobalSpeed);
    
    #endif
    #ifdef funcAccelRamp
    Robot.setSpeedMotors(defGlobalSpeed);
    Robot.setSpeedRampFunction(defGlobalSpeed);
    Robot.setAccelRampFunction(defGlobalAccel);
    #endif

    #ifdef version1Electro
    testPWMElectromagnetActivate();
    #endif

    #ifdef version4Electro
    testPWMElectromagnetActivateV4E(mainElectro);
    #endif
                                                                                
    //==============
    //Funcion que recorre un area del escaque para tomar la pieza
    checkAreaChess(comp_ini_x, comp_ini_y, 6);
    //=================================================================
     
    #ifdef centrarElectromagnet
    double compXanterior = 0;
    double compYanterior = 0;
    
    compXanterior = comp_fin_x;
    compYanterior = comp_fin_y;

    double desfase;
    desfase = desfaseElectroiman;
    if((comp_ini_x != comp_fin_x)&&(comp_ini_y != comp_fin_y))
    {
        desfase = sqrt((desfaseElectroiman*desfaseElectroiman)/2);
    }

    if(comp_ini_x > comp_fin_x)
    {
        compXanterior = compXanterior + desfase;
    }
    if(comp_ini_x < comp_fin_x)
    {
        compXanterior = compXanterior - desfase;
    }

    if(comp_ini_y > comp_fin_y)
    {
        compYanterior = compYanterior + desfase;
    }
    if(comp_ini_y < comp_fin_y)
    {
        compYanterior = compYanterior - desfase;
    }
    #ifdef funcMoveTo
    Robot.moveToPointV2(compXanterior,compYanterior,1);
    #endif

    #ifdef funcMoveTo
    
    testPWMElectromagnetDeactivate();
    #endif
    #endif

    //=================================================================
    #ifdef funcMoveTo
    Robot.moveToPointV2(comp_fin_x,comp_fin_y,1);
    #endif
    #ifdef funcAccelRamp
    #ifdef version1Electro
    Robot.accelRamp(comp_fin_x,comp_fin_y,2,1);
    #endif
    #ifdef version4Electro
    Robot.accelRamp(comp_fin_x,comp_fin_y,2,1,mainElectro);
    #endif
    #endif

    #ifdef centrarElectromagnet

    #ifdef version1Electro
    activateElectromagnet();
    #endif

    #ifdef version4Electro
    activateElectromagnetV4E(mainElectro);
    #endif

    #endif

    #ifdef version1Electro
    deactivateElectromagnet();
    #endif

    #ifdef version4Electro
    deactivateElectromagnetV4E(mainElectro);
    #endif
    
    

    //=================================================================

    if (mode != 2)
    {
        //=======Condicion para el caso de promocion de un peon========
        if (movement == 2 || movement == 5 || movement == 10)
        {
            comerVersion3(x_fin, y_fin, movement, chess_color, pieza);
            //Falta (Funcion que coloca la pieza en el lugar del peon)
        }
        //=============================================================
    }
}

void activateElectromagnet()
{   
    //Serial.println("Funcion activateElectromagnet ... ");
    #ifdef electromagnetConf1
    //Serial.println("->> ACTIVATE ELECTROMAGNET");
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);

   ledcWrite(ledChannel, 254);
   delay(200);
   #endif

   #ifdef electromagnetConf2
   //Serial.println("->> DEACTIVATE ELECTROMAGNET");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 0);
    delay(600); //Antes delay 1000 //Antes 800
   #endif
     
}

void deactivateElectromagnet()
{
    int optionElectro = 1;
    //Serial.println("SE DESACTIVO ELECTROIMAN : ");
    //Serial.println(optionElectro);
    #ifdef electromagnetConf1
    //Serial.println("->> DEACTIVATE ELECTROMAGNET");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 0);
    delay(100); //Antes delay 1000 //Antes 800
    #endif

    #ifdef electromagnetConf2
    //Serial.println("->> ACTIVATE ELECTROMAGNET");
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    //ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
#ifdef normalElectro
     if(optionElectro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
#endif
#ifdef invertElectro
     if(optionElectro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
#endif
   //ledcWrite(ledChannel, 254);
   ledcDetachPin(12);
   ledcDetachPin(13);
   ledcDetachPin(22);
   ledcDetachPin(23);  
   delay(200);
    #endif
    /*
    Serial.println("deactivateElectromagnet ");
    #ifdef electromagnetConf1
    //Serial.println("->> DEACTIVATE ELECTROMAGNET");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 0);
    delay(100); //Antes delay 1000 //Antes 800
    #endif

    #ifdef electromagnetConf2
    //Serial.println("->> ACTIVATE ELECTROMAGNET");
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);

   ledcWrite(ledChannel, 254);
   delay(200);
    #endif
    */
}

void chess_King(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color, char pieza)
{
    moveChessPiece(x_ini, y_ini, x_fin, y_fin, movement, chess_color, pieza);
}

void chess_Queen(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color, char pieza)
{
    moveChessPiece(x_ini, y_ini, x_fin, y_fin, movement, chess_color, pieza);
}

void chess_Rook(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color,char pieza)
{
    moveChessPiece(x_ini, y_ini, x_fin, y_fin, movement, chess_color, pieza);
}

void chess_Bishop(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color, char pieza)
{
    moveChessPiece(x_ini, y_ini, x_fin, y_fin, movement, chess_color, pieza);
}

void chess_Knight(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color, char pieza)
{
    //moveChessPiece(x_ini, y_ini, x_fin, y_fin, movement, chess_color, pieza);
    moveChessPieceKnight(x_ini, y_ini, x_fin, y_fin, movement, chess_color, pieza);
}

void chess_Pawn(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color, char pieza)
{
    moveChessPiece(x_ini, y_ini, x_fin, y_fin, movement, chess_color, pieza);
}


//Para ambos enroques la secuencia ya está definida, sólo cambia según el color de la pieza que va a mover
//y si se trata de un enroque corto o largo.
//Por regla el primer movimiento lo debe realizar el rey y posteriormente la torre
//Por ahora como sólo se están leyendo jugadas desde un archivo no se hacen las verificaciones necesarias
void enroque_corto(bool chess_color)
{
    double comp_ini_x, comp_ini_y;
    double comp_fin_x, comp_fin_y;
    Serial.println("Entro a funcion de enroque corto");
    double x[5];
    double y[5];
    double desfase;

    BoardPosition boardPosition;

    
    //====================================================
    if (chess_color == true) //Si es el turno de las blancas
    {
        //Movemos hacia la posicion del rey blanco "e1"
        boardPosition = getBoardPositionFromString("e1");
        comp_ini_x = boardPosition.x;
        comp_ini_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);
        #endif

        #endif

        
        //Movemos el rey blanco a "g1"
        boardPosition = getBoardPositionFromString("g1");
        comp_fin_x = boardPosition.x;
        comp_fin_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y);
        #endif

        #ifdef version4Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y,mainElectro);
        #endif

        #endif

        //Movemos a la posicion de la torre en "h1"
        boardPosition = getBoardPositionFromString("h1");
        comp_ini_x = boardPosition.x;
        comp_ini_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);
        #endif

        #endif

        //Movemos la torre al centro de "f1"
        boardPosition = getBoardPositionFromString("f1");
        comp_fin_x = boardPosition.x;
        comp_fin_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y);
        #endif

        #ifdef version4Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y,mainElectro);
        #endif

        #endif

    }
     else //Si es el turno de las negras
    {
        //Movemos hacia la posicion del rey negro "e8"
        boardPosition = getBoardPositionFromString("e8");
        comp_ini_x = boardPosition.x;
        comp_ini_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);
        #endif

        #endif

        //Movemos el rey negro a "g8"
        boardPosition = getBoardPositionFromString("g8");
        comp_fin_x = boardPosition.x;
        comp_fin_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y);
        #endif

        #ifdef version4Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y,mainElectro);
        #endif

        #endif


        //Movemos a la posicion de la torre en "h8"
        boardPosition = getBoardPositionFromString("h8");
        comp_ini_x = boardPosition.x;
        comp_ini_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);
        #endif

        #endif

        //Movemos la torre al centro de "f8"
        boardPosition = getBoardPositionFromString("f8");
        comp_fin_x = boardPosition.x;
        comp_fin_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y);
        #endif

        #ifdef version4Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y,mainElectro);
        #endif

        #endif

    }
}

void enroque_largo(bool chess_color)
{
    double comp_ini_x, comp_ini_y;
    double comp_fin_x, comp_fin_y;
    Serial.println("Entro a funcion de enroque largo");
    double x[5];
    double y[5];
    double desfase;

    BoardPosition boardPosition;

    
    //=======================================================
    if (chess_color == true) //Si es el turno de las blancas
    {

        //Movemos hacia la posicion del rey blanco "e1"
        boardPosition = getBoardPositionFromString("e1");
        comp_ini_x = boardPosition.x;
        comp_ini_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);
        #endif

        #endif


        //Movemos el rey blanco a "c1"
        boardPosition = getBoardPositionFromString("c1");
        comp_fin_x = boardPosition.x;
        comp_fin_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y);
        #endif

        #ifdef version4Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y,mainElectro);
        #endif

        #endif


        //Movemos a la posicion de la torre en "a1"
        boardPosition = getBoardPositionFromString("a1");
        comp_ini_x = boardPosition.x;
        comp_ini_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);
        #endif

        #endif

         //Movemos la torre al centro de "d1"
        boardPosition = getBoardPositionFromString("d1");
        comp_fin_x = boardPosition.x;
        comp_fin_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y);
        #endif

        #ifdef version4Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y,mainElectro);
        #endif

        #endif

        
    }
    else //Si es el turno de las negras
    {
        //Movemos hacia la posicion del rey negro "e8"
        boardPosition = getBoardPositionFromString("e8");
        comp_ini_x = boardPosition.x;
        comp_ini_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);
        #endif

        #endif


        //Movemos el rey negro a "c8"
        boardPosition = getBoardPositionFromString("c8");
        comp_fin_x = boardPosition.x;
        comp_fin_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y);
        #endif

        #ifdef version4Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y,mainElectro);
        #endif

        #endif



        //Movemos a la posicion de la torre en "a8"
        boardPosition = getBoardPositionFromString("a8");
        comp_ini_x = boardPosition.x;
        comp_ini_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);
        #endif

        #endif


        //Movemos la torre al centro de "d8"
        boardPosition = getBoardPositionFromString("d8");
        comp_fin_x = boardPosition.x;
        comp_fin_y = boardPosition.y;
        #ifdef funcAccelRamp

        #ifdef version1Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y);
        #endif

        #ifdef version4Electro
        moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y,mainElectro);
        #endif

        #endif

    }
}

BoardPosition getBoardPositionFromString(String position)
{
    BoardPosition boardPosition;
    position.toLowerCase();
    if (position.length() != 2) {
        boardPosition.x = 0;
        boardPosition.y = 0;
        Serial.println("getBoardPositionFromString Error");
        return boardPosition;
    }
    char letterPosition = position.charAt(0);
    char numberPosition = position.charAt(1);
    coordenadas(letterPosition, numberPosition, &boardPosition.x, &boardPosition.y);
    return boardPosition;
}

void coordenadas(char caract_1, char caract_2, double* c_1, double* c_2)
{
    if (caract_1 == 'a') {
        *c_1 = -3.5 * longitud;
    }
    if (caract_1 == 'b') {
        *c_1 = -2.5 * longitud;
    }
    if (caract_1 == 'c') {
        *c_1 = -1.5 * longitud;
    }
    if (caract_1 == 'd') {
        *c_1 = -0.5 * longitud;
    }
    if (caract_1 == 'e') {
        *c_1 = 0.5 * longitud;
    }
    if (caract_1 == 'f') {
        *c_1 = 1.5 * longitud;
    }
    if (caract_1 == 'g') {
        *c_1 = 2.5 * longitud;
    }
    if (caract_1 == 'h') {
        *c_1 = 3.5 * longitud;
    }
    if (caract_2 == '1') {
        *c_2 = -3.5 * longitud;
    }
    if (caract_2 == '2') {
        *c_2 = -2.5 * longitud;
    }
    if (caract_2 == '3') {
        *c_2 = -1.5 * longitud;
    }
    if (caract_2 == '4') {
        *c_2 = -0.5 * longitud;
    }
    if (caract_2 == '5') {
        *c_2 = 0.5 * longitud;
    }
    if (caract_2 == '6') {
        *c_2 = 1.5 * longitud;
    }
    if (caract_2 == '7') {
        *c_2 = 2.5 * longitud;
    }
    if (caract_2 == '8') {
        *c_2 = 3.5 * longitud;
    }
}

void moveChessPieceKnight(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color, char pieza)
{
    double comp_ini_x, comp_ini_y;
    double* Ap_ini_x = &comp_ini_x;
    double* Ap_ini_y = &comp_ini_y;
    double comp_fin_x, comp_fin_y;
    double* Ap_fin_x = &comp_fin_x;
    double* Ap_fin_y = &comp_fin_y;

    testPWMElectromagnetDeactivateV4E(1);
    testPWMElectromagnetDeactivateV4E(2);
    testPWMElectromagnetDeactivateV4E(3);
    testPWMElectromagnetDeactivateV4E(4);

    Serial.println("Dentro de funcion moveChessPiece");
    Serial.println("Color del movimiento actual");
    //Segmento para calcular los 6 puntos intermedios en el movimiento del caballo
    double vectMoveKnightX[6];
    double vectMoveKnightY[6];
    double difAxisX;
    double difAxisY;
/*
    difAxisX = abs(x_fin - x_ini);
    difAxisY = abs(y_fin - y_ini);

    if(difAxisY > difAxisX)         //Como la diferencia es mayor en Y el movimiento ocurre en vertical 
    {
        if(x_ini < x_fin)
        {
            if(y_ini < y_fin)
            {
                vectMoveKnightX[0] = x_ini + 12.5;
                vectMoveKnightY[0] = y_ini + 3.35;

                vectMoveKnightX[1] = x_ini + 21.65;
                vectMoveKnightY[1] = y_ini + 12.5;
                
                vectMoveKnightX[2] = x_ini + 25;
                vectMoveKnightY[2] = y_ini + 25;
                
                vectMoveKnightX[3] = x_ini + 25;
                vectMoveKnightY[3] = y_ini + 75;
                
                vectMoveKnightX[4] = x_ini + 28.35;
                vectMoveKnightY[4] = y_ini + 87.5;
                
                vectMoveKnightX[5] = x_ini + 37.5;
                vectMoveKnightY[5] = y_ini + 96.65; 
            }
            else
            {
                vectMoveKnightX[0] = x_ini + 12.5;
                vectMoveKnightY[0] = y_ini - 3.35;

                vectMoveKnightX[1] = x_ini + 21.65;
                vectMoveKnightY[1] = y_ini - 12.5;
                
                vectMoveKnightX[2] = x_ini + 25;
                vectMoveKnightY[2] = y_ini - 25;
                
                vectMoveKnightX[3] = x_ini + 25;
                vectMoveKnightY[3] = y_ini - 75;
                
                vectMoveKnightX[4] = x_ini + 28.35;
                vectMoveKnightY[4] = y_ini - 87.5;
                
                vectMoveKnightX[5] = x_ini + 37.5;
                vectMoveKnightY[5] = y_ini - 96.65; 
            }
        }
        else
        {
            if(y_ini < y_fin)
            {
                vectMoveKnightX[0] = x_ini - 12.5;
                vectMoveKnightY[0] = y_ini + 3.35;

                vectMoveKnightX[1] = x_ini - 21.65;
                vectMoveKnightY[1] = y_ini + 12.5;
                
                vectMoveKnightX[2] = x_ini - 25;
                vectMoveKnightY[2] = y_ini + 25;
                
                vectMoveKnightX[3] = x_ini - 25;
                vectMoveKnightY[3] = y_ini + 75;
                
                vectMoveKnightX[4] = x_ini - 28.35;
                vectMoveKnightY[4] = y_ini + 87.5;
                
                vectMoveKnightX[5] = x_ini - 37.5;
                vectMoveKnightY[5] = y_ini + 96.65;
            }
            else
            {
                vectMoveKnightX[0] = x_ini - 12.5;
                vectMoveKnightY[0] = y_ini - 3.35;

                vectMoveKnightX[1] = x_ini - 21.65;
                vectMoveKnightY[1] = y_ini - 12.5;
                
                vectMoveKnightX[2] = x_ini - 25;
                vectMoveKnightY[2] = y_ini - 25;
                
                vectMoveKnightX[3] = x_ini - 25;
                vectMoveKnightY[3] = y_ini - 75;
                
                vectMoveKnightX[4] = x_ini - 28.35;
                vectMoveKnightY[4] = y_ini - 87.5;
                
                vectMoveKnightX[5] = x_ini - 37.5;
                vectMoveKnightY[5] = y_ini - 96.65; 
            }
        }

    }
    else                            //En caso contrario el movimiento ocurre en horizontal
    {
        if(x_ini < x_fin)
        {
            if(y_ini < y_fin)
            {
                vectMoveKnightX[0] = x_ini + 3.35;
                vectMoveKnightY[0] = y_ini + 12.5;

                vectMoveKnightX[1] = x_ini + 12.5;
                vectMoveKnightY[1] = y_ini + 21.65;
                
                vectMoveKnightX[2] = x_ini + 25;
                vectMoveKnightY[2] = y_ini + 25;
                
                vectMoveKnightX[3] = x_ini + 75;
                vectMoveKnightY[3] = y_ini + 25;
                
                vectMoveKnightX[4] = x_ini + 87.5;
                vectMoveKnightY[4] = y_ini + 28.35;
                
                vectMoveKnightX[5] = x_ini + 96.65;
                vectMoveKnightY[5] = y_ini + 37.5;
            }
            else
            {
                vectMoveKnightX[0] = x_ini + 3.35;
                vectMoveKnightY[0] = y_ini - 12.5;

                vectMoveKnightX[1] = x_ini + 12.5;
                vectMoveKnightY[1] = y_ini - 21.65;
                
                vectMoveKnightX[2] = x_ini + 25;
                vectMoveKnightY[2] = y_ini - 25;
                
                vectMoveKnightX[3] = x_ini + 75;
                vectMoveKnightY[3] = y_ini - 25;
                
                vectMoveKnightX[4] = x_ini + 87.5;
                vectMoveKnightY[4] = y_ini - 28.35;
                
                vectMoveKnightX[5] = x_ini + 96.65;
                vectMoveKnightY[5] = y_ini - 37.5;
            }
        }
        else
        {
            if(y_ini < y_fin)
            {
                vectMoveKnightX[0] = x_ini - 3.35;
                vectMoveKnightY[0] = y_ini + 12.5;

                vectMoveKnightX[1] = x_ini - 12.5;
                vectMoveKnightY[1] = y_ini + 21.65;
                
                vectMoveKnightX[2] = x_ini - 25;
                vectMoveKnightY[2] = y_ini + 25;
                
                vectMoveKnightX[3] = x_ini - 75;
                vectMoveKnightY[3] = y_ini + 25;
                
                vectMoveKnightX[4] = x_ini - 87.5;
                vectMoveKnightY[4] = y_ini + 28.35;
                
                vectMoveKnightX[5] = x_ini - 96.65;
                vectMoveKnightY[5] = y_ini + 37.5;
            }
            else
            {
                vectMoveKnightX[0] = x_ini - 3.35;
                vectMoveKnightY[0] = y_ini - 12.5;

                vectMoveKnightX[1] = x_ini - 12.5;
                vectMoveKnightY[1] = y_ini - 21.65;
                
                vectMoveKnightX[2] = x_ini - 25;
                vectMoveKnightY[2] = y_ini - 25;
                
                vectMoveKnightX[3] = x_ini - 75;
                vectMoveKnightY[3] = y_ini - 25;
                
                vectMoveKnightX[4] = x_ini - 87.5;
                vectMoveKnightY[4] = y_ini - 28.35;
                
                vectMoveKnightX[5] = x_ini - 96.65;
                vectMoveKnightY[5] = y_ini - 37.5;
            }
        }

    }
*/


    //
    if(chess_color == false)
    {
        Serial.println("Negras");
    }
    if(chess_color == true)
    {
        Serial.println("Blancas");
    }


    if (mode != 2)
    {
#ifdef activateSensors
    sensorsDir();
    detectChessBoard();
#endif
        if (movement == 3 || movement == 6 || movement == 9 || movement == 10)
        {
            comerVersion3(x_fin, y_fin, movement, chess_color, pieza);
#ifdef activateSensors
            int resultFinalPos;
            detectChessBoardVerif();
            resultFinalPos = checkFinalPositionGraveyard(chess_color);
            while (resultFinalPos == 0)
            {
                comerVersion3(x_fin, y_fin, movement, chess_color, pieza);
                detectChessBoardVerif();
                resultFinalPos = checkFinalPositionGraveyard(chess_color);
            }
#endif
        }
    }
    coordenadas(x_ini, y_ini, Ap_ini_x, Ap_ini_y);
    coordenadas(x_fin, y_fin, Ap_fin_x, Ap_fin_y);
    //===============================================================
    #ifdef funcMoveTo
    Robot.moveToPointV2(comp_ini_x, comp_ini_y,1);
    #endif

    #ifdef funcAccelRamp
    Robot.setSpeedMotors(maximunVelocityDirectRamp);
    Robot.setSpeedRampFunction(maximunVelocityDirectRamp);
    Robot.setAccelRampFunction(maximunAccelDirectRamp);
    #ifdef version1Electro
    Robot.accelRamp(comp_ini_x,comp_ini_y,2);   //Antes desacelracion,2
    //Robot.accelRamp(comp_ini_x,comp_ini_y,0);
    #endif
    #ifdef version4Electro
    Robot.accelRamp(comp_ini_x,comp_ini_y,2,0,mainElectro);   //Antes desacelracion,2
    #endif
    #endif

    Robot.setSpeedMotors(defGlobalSpeed);
    Robot.setSpeedRampFunction(defGlobalSpeed);
    Robot.setAccelRampFunction(defGlobalAccel);
    #ifdef version1Electro
    moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y);
    #endif
    #ifdef version4Electro
    moveOnTheLinev2(comp_ini_x,comp_ini_y,comp_fin_x,comp_fin_y,mainElectro);
    #endif
}

void updateFenFromMatriz(bool color_actual, String stringEnroque, String stringCaptura)
{
    char charArrayTemp[90];
    for(int t =0; t < 90; t++)
    {
        charArrayTemp[t] = 'v';
    }

    int k = 0;
    char charValue = 'v';
    int espacioLibre = 0;
    for(int j = 0 ; j < 8; j++)
    {
        for(int i = 0; i < 8; i++)
        {
            if(matriz[i][j] != '.')
            {
                if(espacioLibre == 0)
                {
                    charArrayTemp[k] = matriz[i][j];
                    k++;
                }
                else
                {
                    charValue = convertIntChar(espacioLibre);
                    charArrayTemp[k] = charValue;
                    k++;
                    charArrayTemp[k] = matriz[i][j];
                    k++;
                    espacioLibre = 0;
                }
            }
            else
            {
                espacioLibre++;
            }
        }
        if(espacioLibre != 0)
        {
            charValue = convertIntChar(espacioLibre);
            charArrayTemp[k] = charValue;
            k++;
        }
        if(j <7)
        {
            charArrayTemp[k] = '/';
            k++;
        }
        espacioLibre=0; 
    }
    charArrayTemp[k] = ' ';
    k++;

    if(color_actual == true)
    {
        charArrayTemp[k] = 'b';
        k++;
    }
    if(color_actual == false)
    {
        charArrayTemp[k] = 'w';
        k++;
    }

    charArrayTemp[k] = ' ';
    k++;

    int n = stringEnroque.length();
    char char_array[n + 1];
    
    strcpy(char_array, stringEnroque.c_str());
    for (int i = 0; i < n; i++)
    {
        if(char_array[i] != 'v' || char_array[i] != '-' )
        {
            charArrayTemp[k] = char_array[i];
            k++;
        }
    }

    charArrayTemp[k] = ' ';
    k++;

    int nCap = stringCaptura.length();
    char char_arrayCap[nCap + 1]; 
    
    if(stringCaptura == "-")
    {
        charArrayTemp[k] = '-';
        k++;
    }
    else
    {
        strcpy(char_arrayCap, stringCaptura.c_str());
        for (int i = 0; i < nCap; i++)
        {
            if (char_arrayCap[i] != '-')
            {
                charArrayTemp[k] = char_arrayCap[i];
                k++;
            }
        }
    }
    #ifdef debug
    for(int i = 0; i < 90; i++)
    {
        if(charArrayTemp[i] != 'v')
        {
            Serial.print(charArrayTemp[i]);
        }
    }
    Serial.println("");
    #endif
}

void upgradeMatriz(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color, char pieza){
    char posCoordBoardX[8] = {'a','b','c','d','e','f','g','h'};
    char posCoordBoardY[8] = {'8','7','6','5','4','3','2','1'};

    int posMatrizIniX = -1;
    int posMatrizIniY = -1;
    int posMatrizFinX = -1;
    int posMatrizFinY = -1;

    //Busqueda de las coordenadas para modificar la matriz del tablero
    for(int i = 0; i < 8; i++)
    {
        if(x_ini == posCoordBoardX[i])
        {
            posMatrizIniX = i;
        }
        if(x_fin == posCoordBoardX[i])
        {
            posMatrizFinX = i;
        }
    }

    for(int j = 0; j < 8; j++)
    {
        if(y_ini == posCoordBoardY[j])
        {
            posMatrizIniY = j;
        }
        if(y_fin == posCoordBoardY[j])
        {
            posMatrizFinY = j;
        }
    }

//    movement
//    1 : solo mueve
//    2 : mueve y promocion
//    3 : mueve y come
//    4 : mueve y jaque
//    5 : mueve, promocion y jaque
//    6 : mueve, come y jaque
//    7 : Enroque corto
//    8 : Enroque largo
//    9 : Passant 
//   10 : mueve, come y promocion

    if(pieza == 'S' || pieza == 'L')
    {
        if(pieza == 'S')
        {
            if(chess_color == true)
            {
                matriz[4][7] = '.';
                matriz[6][7] = 'K';
                matriz[7][7] = '.';
                matriz[5][7] = 'R';  
            }
            else
            {
                matriz[4][0] = '.';
                matriz[6][0] = 'k';
                matriz[7][0] = '.';
                matriz[5][0] = 'r'; 
            }
        }
        if(pieza == 'L')
        {
            if(chess_color == true)
            {
                matriz[4][7] = '.';
                matriz[2][7] = 'K';
                matriz[0][7] = '.';
                matriz[3][7] = 'R'; 
            }
            else
            {
                matriz[4][0] = '.';
                matriz[2][0] = 'k';
                matriz[0][0] = '.';
                matriz[3][0] = 'r'; 
            }
        }
    }
    else
    {
        matriz[posMatrizFinX][posMatrizFinY] = matriz[posMatrizIniX][posMatrizIniY];
        matriz[posMatrizIniX][posMatrizIniY] = '.';
    }
}

void printMatriz(){
    #ifdef debug
    int rows = 8;
    int cols = 8;
    for(int j=0;j< rows;j++){
        for(int i=0;i< cols;i++){
            Serial.print(matriz[i][j]);
            Serial.print(" ");
        }
        Serial.println(" ");
    }
    Serial.println("");
    #endif
}

String castlingFEN(int bandRookKingW,int bandRookQueenW,int bandRookKingB,int bandRookQueenB,int bandKingW,int bandKingB)
{
    String result = "";
    char charDato1 = 'K';
    char charDato2 = 'Q';
    char charDato3 = 'k';
    char charDato4 = 'q';
    if(bandRookKingW == 1)
    {
        charDato1 = 'v';
    }
    if(bandRookQueenW == 1)
    {
        charDato2 = 'v';
    }
    if(bandRookKingB == 1)
    {
        charDato3 = 'v';
    }
    if(bandRookQueenB == 1)
    {
        charDato4 = 'v';
    }
    if(bandKingW == 1)
    {
        charDato1 = 'v';
        charDato2 = 'v';
    }
    if(bandKingB == 1)
    {
        charDato3 = 'v';
        charDato4 = 'v';
    }
    result = result + charDato1;
    result = result + charDato2;
    result = result + charDato3;
    result = result + charDato4;
    if(result == "vvvv")
    {
        result = '-';
    }
    return result; 
}

String capturaAlPasoFen(char coordXfin,char coordYfin,bool color_actual)
{
    String result = "-";
    if(color_actual == true)
    {
        if(coordYfin == '4')
        {
            result = "";
            result = result + coordXfin;
            result = result + '3';
        }
    }
    else
    {
        if(coordYfin == '5')
        {
            result = "";
            result = result + coordXfin;
            result = result + '6';
        }
    }
    return result;
}

char convertIntChar(int datoInt)
{
    char datoChar = 'v';
    if(datoInt == 0)
    {
        datoChar = '0';
    }
    if(datoInt == 1)
    {
        datoChar = '1';
    }
    if(datoInt == 2)
    {
        datoChar = '2';
    }
    if(datoInt == 3)
    {
        datoChar = '3';
    }
    if(datoInt == 4)
    {
        datoChar = '4';
    }
    if(datoInt == 5)
    {
        datoChar = '5';
    }
    if(datoInt == 6)
    {
        datoChar = '6';
    }
    if(datoInt == 7)
    {
        datoChar = '7';
    }
    if(datoInt == 8)
    {
        datoChar = '8';
    }
    if(datoInt == 9)
    {
        datoChar = '9';
    }
    return datoChar;
}

void removeChessAuto(void)
{
    sensorsDir();
    detectChessBoard();
    char vectChess[12] = {'r', 'n', 'b', 'q', 'k', 'p', 'R', 'N', 'B', 'Q', 'K', 'P'};
    char coordVectX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    char coordVectY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};
    bool colorChessT = true;
    char pieceActual = 'v';

    for(int j = 0; j < 8; j++)
    {
        for(int i = 0; i < 8; i++)
        {
            if(matriz[i][j] != '.')
            {
                //======== Determinar que pieza se encontro y de que color es ======
                for(int k = 0; k < 12; k++)
                {
                    if(matriz[i][j] == vectChess[k])
                    {
                        pieceActual = vectChess[k];
                        if(k <= 5)
                        {
                            colorChessT = true;
                        }
                        else
                        {
                            colorChessT = false;
                        }
                    }
                }
                comerVersion3(coordVectX[i],coordVectY[j],1,colorChessT,pieceActual);


                #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
                int resultFinalPos;
                detectChessBoardVerif();
                resultFinalPos = checkFinalPositionGraveyard(colorChessT);
                while(resultFinalPos == 0)
                {
                    comerVersion3(coordVectX[i],coordVectY[j],1,colorChessT,pieceActual);
                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPositionGraveyard(colorChessT);
                }
                #endif


                


                matriz[i][j] = '.';
                printMatriz();
                imprimirVectPiecesD();
                
                #ifdef activateSensors
                compareVirtualMatrizVsSensors();
                #endif
                
            }
        }
    }
}

void imprimirVectPiecesD(void)
{
    #ifdef debug
    Serial.println("White Chess");
    for (int i = 0; i < 16; i++)
    {
        Serial.print(vectBlancas[i]);
        Serial.print(" ");
    }
    Serial.println(" ");

    Serial.println("Black Chess");
    for (int i = 0; i < 16; i++)
    {
        Serial.print(vectNegras[i]);
        Serial.print(" ");
    }
    Serial.println(" ");
    #endif
}

char consultaPiezaTablero(char coordXChess, char coordYChess)
{
    char coordVectX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    char coordVectY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};
    char pieceActual = 'v';
    int indX = -1;
    int indY = -1;

    for(int i = 0; i < 8; i++)
    {
        if(coordXChess == coordVectX[i])
        {
            indX = i;
        }
        if(coordYChess == coordVectY[i])
        {
            indY = i;
        }
    }
    pieceActual =  matriz[indX][indY];        
    return pieceActual;
}

void initializeMatriz(void)
{
    char vectChessW[8] = {'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R'};
    char vectChessB[8] = {'r', 'n', 'b', 'q', 'k', 'b', 'n', 'r'};
    for(int i = 0; i < 8; i++){
        matriz[i][0] = vectChessB[i];
    }
    for(int i = 0; i < 8; i++){
        matriz[i][1] = 'p';
    }

    for(int j = 2; j < 6; j++){
        for(int i = 0; i < 8; i++){
            matriz[i][j] = '.';
        }
    }

    for(int i = 0; i < 8; i++){
        matriz[i][7] = vectChessW[i];
    }
    for(int i = 0; i < 8; i++){
        matriz[i][6] = 'P';
    }
}
void reorderAuto()
{
    sensorsDir();
    detectChessBoard();

    double cornerPointX = 0;
    double cornerPointY = 0;

    int resultFinalPos;

    char matriz2[8][8]; 

    double compFinX, compFinY;
    double* ApFinX = &compFinX;
    double* ApFinY = &compFinY;

    double coordIniX, coordIniY;
    double* ApIniX = &coordIniX;
    double* ApIniY = &coordIniY;

    char cVectX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    char cVectY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};
    char charPosXFinal = 'v';
    char charPosYFinal = 'v';
    bool colorActualChess = false;

    //=============Inicializacion de la matriz de referencia Matriz2============
    char vectChessW[8] = {'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R'};
    char vectChessB[8] = {'r', 'n', 'b', 'q', 'k', 'b', 'n', 'r'};
    for(int i = 0; i < 8; i++){
        matriz2[i][0] = vectChessB[i];
    }
    for(int i = 0; i < 8; i++){
        matriz2[i][1] = 'p';
    }

    for(int j = 2; j < 6; j++){
        for(int i = 0; i < 8; i++){
            matriz2[i][j] = '.';
        }
    }

    for(int i = 0; i < 8; i++){
        matriz2[i][7] = vectChessW[i];
    }
    for(int i = 0; i < 8; i++){
        matriz2[i][6] = 'P';
    }
    //=============================================================================


    //-----------------------------------------------
    //Recorre la matriz inicializada para identificar el lugar de cada pieza
    //Genera la coordenada de posicion final
    //Busca la pieza en el vector de piezas muertas
    //Genera la coordenada de posicion inicial
    //Mueve la pieza al lugar que le corresponde
    //Avanza a la siguiente posicion de la matriz
    //-----------------------------------------------      

    for(int j = 0; j < 8; j++)
    {
        for(int i = 0; i < 8; i++)
        {
            if(matriz2[i][j] != '.')
            {
                charPosXFinal = cVectX[i];               
                charPosYFinal = cVectY[j];
                coordenadas(charPosXFinal,charPosYFinal,ApFinX,ApFinY);
                Serial.println("Final Coordinate X");
                Serial.println(compFinX);
                Serial.println("Final Coordinate Y");
                Serial.println(compFinY);
                
                if (matriz2[i][j] >= 'a' && matriz2[i][j] <= 'z')
                {
                    //Piezas Negras
                    colorActualChess = false;
                }
                if (matriz2[i][j] >= 'A' && matriz2[i][j] <= 'Z')
                {
                    //Piezas Blancas
                    colorActualChess = true;
                }

                consultaCoordPiezasMuertas(matriz2[i][j],colorActualChess,ApIniX,ApIniY);
                Serial.println("Ini Coordinate X");
                Serial.println(coordIniX);
                Serial.println("Ini Coordinate Y");
                Serial.println(coordIniY);

                //moveOnTheLineIni(coordIniX, coordIniY, compFinX,compFinY);

                #ifdef funcMoveTo
                Robot.setSpeedMotors(maximumVelocityDirect);
                #endif
                #ifdef funcAccelRamp
                Robot.setSpeedMotors(maximunVelocityDirectRamp);
                Robot.setSpeedRampFunction(maximunVelocityDirectRamp);
                Robot.setAccelRampFunction(maximunAccelDirectRamp);
                #endif
                
                #ifdef moveDirectPosPiece
                
                #ifdef funcMoveTo
                Robot.moveToPointV2(coordIniX, coordIniY,1);
                #endif

                #ifdef funcAccelRamp

                #ifdef version1Electro
                Robot.accelRamp(coordIniX,coordIniY,2); //Antes desacelracion,2
                #endif
/*
                #ifdef version4Electro
               
                float diferenciaX;
                float diferenciaY;
                float puntoInterX;
                float puntoInterY;
                int vectInterPointsX[5] = {-1,-1,-1,-1,-1};
                int vectInterPointsY[5] = {-1,-1,-1,-1,-1};
                
                diferenciaX = abs(coordIniX - compFinX);
                diferenciaY = abs(coordIniY - compFinY);

                vectInterPointsX[0] = coordIniX;
                vectInterPointsY[0] = coordIniY;
                if (diferenciaX > diferenciaY)
                {
                    if (compFinY < coordIniY)
                    {
                        puntoInterX = coordIniX;
                        puntoInterY = coordIniY - 25;

                        vectInterPointsX[1] = puntoInterX;
                        vectInterPointsY[1] = puntoInterY;
                    }
                    // Aqui me falta la condicion en caso de que las coordenadas Y sean las mismas
                    if (compFinY > coordIniY)
                    {
                        puntoInterX = coordIniX;
                        puntoInterY = coordIniY + 25;

                        vectInterPointsX[1] = puntoInterX;
                        vectInterPointsY[1] = puntoInterY;
                    }
                    //=================================================================================
                    if (compFinX < coordIniX)
                    {
                        puntoInterX = compFinX + 25;

                        vectInterPointsX[2] = puntoInterX;
                        vectInterPointsY[2] = puntoInterY;

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];
                    }
                    // Aqui me falta la condicion en caso de que las coordenadas X sean las mismas
                    if (compFinX > coordIniX)
                    {
                        puntoInterX = compFinX - 25;

                        vectInterPointsX[2] = puntoInterX;
                        vectInterPointsY[2] = puntoInterY;

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];
                    }
                    //=================================================================================
                    puntoInterY = compFinY;

                    vectInterPointsX[3] = puntoInterX;
                    vectInterPointsY[3] = puntoInterY;
                }
                else
                {
                    if (compFinX < coordIniX)
                    {
                        puntoInterX = coordIniX - 25;
                        puntoInterY = coordIniY;

                        vectInterPointsX[1] = puntoInterX;
                        vectInterPointsY[1] = puntoInterY;
                    }
                    // Aqui me falta la condicion en caso de que las coordenadas X sean las mismas
                    if (compFinX > coordIniX)
                    {
                        puntoInterX = coordIniX + 25;
                        puntoInterY = coordIniY;

                        vectInterPointsX[1] = puntoInterX;
                        vectInterPointsY[1] = puntoInterY;
                    }
                    //=================================================================================
                    if (compFinY < coordIniY)
                    {
                        puntoInterY = compFinY + 25;

                        vectInterPointsX[2] = puntoInterX;
                        vectInterPointsY[2] = puntoInterY;

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];
                    }
                    // Aqui me falta la condicion en caso de que las coordenadas X sean las mismas
                    if (compFinY > coordIniY)
                    {
                        puntoInterY = compFinY - 25;

                        vectInterPointsX[2] = puntoInterX;
                        vectInterPointsY[2] = puntoInterY;

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];
                    }
                    //=================================================================================
                    puntoInterX = compFinX;

                    vectInterPointsX[3] = puntoInterX;
                    vectInterPointsY[3] = puntoInterY;
                }
                //===============================================================

                vectInterPointsX[4] = compFinX;
                vectInterPointsY[4] = compFinY;


                if (diferenciaX < diferenciaY)
                {
                    if (diferenciaX == 50)
                    {
                        vectInterPointsX[2] = vectInterPointsX[1];
                        vectInterPointsY[2] = compFinY;

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];

                        vectInterPointsX[3] = compFinX;
                        vectInterPointsY[3] = compFinY;

                        vectInterPointsX[4] = -1;
                        vectInterPointsY[4] = -1;
                    }
                    if (diferenciaX == 0)
                    {
                        // El caso donde estan en la misma columna del tablero
                        vectInterPointsX[1] = coordIniX + 25;
                        vectInterPointsY[1] = coordIniY;

                        vectInterPointsX[2] = vectInterPointsX[1];
                        vectInterPointsY[2] = compFinY;

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];

                        vectInterPointsX[3] = compFinX;
                        vectInterPointsY[3] = compFinY;

                        vectInterPointsX[4] = -1;
                        vectInterPointsY[4] = -1;
                    }
                }
                if (diferenciaX > diferenciaY)
                {
                    if (diferenciaY == 50)
                    {
                        vectInterPointsX[2] = compFinX;
                        vectInterPointsY[2] = vectInterPointsY[1];

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];

                        vectInterPointsX[3] = compFinX;
                        vectInterPointsY[3] = compFinY;

                        vectInterPointsX[4] = -1;
                        vectInterPointsY[4] = -1;
                    }
                    if (diferenciaY == 0)
                    {
                        // El caso donde estan en la misma columna del tablero

                        vectInterPointsX[1] = coordIniX;
                        vectInterPointsY[1] = coordIniY + 25;

                        vectInterPointsX[2] = compFinX;
                        vectInterPointsY[2] = vectInterPointsY[1];

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];

                        vectInterPointsX[3] = compFinX;
                        vectInterPointsY[3] = compFinY;

                        vectInterPointsX[4] = -1;
                        vectInterPointsY[4] = -1;
                    }
                }
                if (diferenciaX == 50 && diferenciaY == 50)
                {
                    if (coordIniX < compFinX)
                    {
                        vectInterPointsX[1] = coordIniX + 25;
                        vectInterPointsY[1] = coordIniY;

                        vectInterPointsX[2] = vectInterPointsX[1];
                        vectInterPointsY[2] = compFinY;

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];

                        vectInterPointsX[3] = compFinX;
                        vectInterPointsY[3] = compFinY;

                        vectInterPointsX[4] = -1;
                        vectInterPointsY[4] = -1;
                    }
                    else
                    {
                        vectInterPointsX[1] = coordIniX - 25;
                        vectInterPointsY[1] = coordIniY;

                        vectInterPointsX[2] = vectInterPointsX[1];
                        vectInterPointsY[2] = compFinY;

                        cornerPointX = vectInterPointsX[2];
                        cornerPointY = vectInterPointsY[2];

                        vectInterPointsX[3] = compFinX;
                        vectInterPointsY[3] = compFinY;

                        vectInterPointsX[4] = -1;
                        vectInterPointsY[4] = -1;
                    }
                }
#endif
*/
                
                #ifdef version4Electro
                if (coordIniX < -200) // Si va a sacar del lado izquierdo
                {
                    Robot.accelRamp(coordIniX,coordIniY, 2, 0, 4);
                }
                if (coordIniX > 200) // Si va a sacar del lado derecho
                {
                    Robot.accelRamp(coordIniX,coordIniY, 2, 0, 2);
                }
                if (coordIniY < -200) // Si va a sacar del lado inferior
                {
                    Robot.accelRamp(coordIniX,coordIniY, 2, 0, 3);
                }
                if (coordIniY > 200) // Si va a sacar del lado superior
                {
                    Robot.accelRamp(coordIniX,coordIniY, 2, 0, 1);
                }

                #endif
                
                #endif
                
                #else
                moveOnTheLineIni(coordIniX, coordIniY, compFinX,compFinY);
                #endif

                #ifdef funcMoveTo
                Robot.setSpeedMotors(defGlobalSpeed);
                #endif
                #ifdef funcAccelRamp
                Robot.setSpeedMotors(defGlobalSpeed);
                Robot.setSpeedRampFunction(defGlobalSpeed);
                Robot.setAccelRampFunction(defGlobalAccel);
                #endif

                
                //moveOnTheLine(coordIniX, coordIniY, compFinX,compFinY);
                #ifdef version1Electro
                moveOnTheLinev2(coordIniX, coordIniY, compFinX,compFinY);
                #endif

                #ifdef version4Electro
                if (coordIniX < -200) // Si va a sacar del lado izquierdo
                {
                    moveOnTheLinev2(coordIniX, coordIniY, compFinX,compFinY,4);
                }
                if (coordIniX > 200) // Si va a sacar del lado derecho
                {
                    moveOnTheLinev2(coordIniX, coordIniY, compFinX,compFinY,2);
                }
                if (coordIniY < -200) // Si va a sacar del lado inferior
                {
                    moveOnTheLinev2(coordIniX, coordIniY, compFinX,compFinY,3);
                }
                if (coordIniY > 200) // Si va a sacar del lado superior
                {
                    moveOnTheLinev2(coordIniX, coordIniY, compFinX,compFinY,1);
                }
                #endif

                #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
                int resultFinalPos;
                detectChessBoardVerif();
                resultFinalPos = checkFinalPosition(charPosXFinal,charPosYFinal);
                while(resultFinalPos == 0)
                {
                    //moveOnTheLineIni(coordIniX, coordIniY, compFinX,compFinY);

                    #ifdef funcMoveTo
                    Robot.setSpeedMotors(maximumVelocityDirect);
                    #endif
                    #ifdef funcAccelRamp
                    Robot.setSpeedMotors(maximunVelocityDirectRamp);
                    Robot.setSpeedRampFunction(maximunVelocityDirectRamp);
                    Robot.setAccelRampFunction(maximunAccelDirectRamp);
                    #endif

                    #ifdef moveDirectPosPiece
                    
                    #ifdef funcMoveTo
                    Robot.moveToPointV2(coordIniX, coordIniY,1);
                    #endif

                    #ifdef funcAccelRamp
                    #ifdef version4Electro
                    if (coordIniX < -200) // Si va a sacar del lado izquierdo
                    {
                        Robot.accelRamp(coordIniX, coordIniY, 2, 0, 4);
                    }
                    if (coordIniX > 200) // Si va a sacar del lado derecho
                    {
                        Robot.accelRamp(coordIniX, coordIniY, 2, 0, 2);
                    }
                    if (coordIniY < -200) // Si va a sacar del lado inferior
                    {
                        Robot.accelRamp(coordIniX, coordIniY, 2, 0, 3);
                    }
                    if (coordIniY > 200) // Si va a sacar del lado superior
                    {
                        Robot.accelRamp(coordIniX, coordIniY, 2, 0, 1);
                    }

                    #endif
                    #endif
                    
                    #else
                    moveOnTheLineIni(coordIniX, coordIniY, compFinX,compFinY);
                    #endif

                    #ifdef funcMoveTo
                    Robot.setSpeedMotors(defGlobalSpeed);
                    #endif
                    #ifdef funcAccelRamp
                    Robot.setSpeedMotors(defGlobalSpeed);
                    Robot.setSpeedRampFunction(defGlobalSpeed);
                    Robot.setAccelRampFunction(defGlobalAccel);
                    #endif


                    //moveOnTheLine(coordIniX, coordIniY, compFinX,compFinY);
                    #ifdef version4Electro
                if (coordIniX < -200) // Si va a sacar del lado izquierdo
                {
                    moveOnTheLinev2(coordIniX, coordIniY, compFinX,compFinY,4);
                }
                if (coordIniX > 200) // Si va a sacar del lado derecho
                {
                    moveOnTheLinev2(coordIniX, coordIniY, compFinX,compFinY,2);
                }
                if (coordIniY < -200) // Si va a sacar del lado inferior
                {
                    moveOnTheLinev2(coordIniX, coordIniY, compFinX,compFinY,3);
                }
                if (coordIniY > 200) // Si va a sacar del lado superior
                {
                    moveOnTheLinev2(coordIniX, coordIniY, compFinX,compFinY,1);
                }
                #endif

                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPosition(charPosXFinal,charPosYFinal);
                }
                #endif

                matriz[i][j] = matriz2[i][j];
                printMatriz();
                imprimirVectPiecesD();

                #ifdef activateSensors
                compareVirtualMatrizVsSensors();
                #endif
            }
        }
    }
}

void consultaCoordPiezasMuertas(char piezaObjetivo,bool colorPieza, double* resultCoordX, double* resultCoordY)
{
    double contPosXBlancas = 0;
    double contPosYBlancas = 0; 
    double contPosXNegras = 0;
    double contPosYNegras = 0;

    Serial.println("Function consultaCoordPiezasMuertas");
    if(colorPieza == true)
    {
        contPosXBlancas = -225;
        contPosYBlancas = -225;  
        for(int i = 0; i < 16; i++)
        {
            Serial.print(vectBlancas[i]);
            Serial.println("  ");
            if(i < 8)
            {
                contPosYBlancas = contPosYBlancas + 50;
            }
            if(i >= 8)
            {
                contPosXBlancas = contPosXBlancas + 50;
            }
            if(piezaObjetivo == vectBlancas[i])
            {
                *resultCoordX = contPosXBlancas;
                *resultCoordY = contPosYBlancas;
                Serial.println("Pieza Objetivo");
                Serial.println(piezaObjetivo);
                vectBlancas[i] = 'v';                   //Vacia el espacio en el vector de piezas muertas 
                break;
            }            
            if(i == 7)
            {
                contPosXBlancas = -225;
                contPosYBlancas = 225;
            }
        }
        Serial.println("");
    }

    if(colorPieza == false)
    {
        contPosXNegras = 225;
        contPosYNegras = 225;   
        for(int i = 0; i < 16; i++)
        {
            Serial.print(vectNegras[i]);
            Serial.println("  ");
            if(i < 8)
            {
                contPosYNegras = contPosYNegras - 50;
            }
            if(i >= 8)
            {
                contPosXNegras = contPosXNegras - 50;
            }
            if(piezaObjetivo == vectNegras[i])
            {
                *resultCoordX = contPosXNegras;
                *resultCoordY = contPosYNegras;
                Serial.println("Pieza Objetivo");
                Serial.println(piezaObjetivo);
                vectNegras[i] = 'v';                   //Vacia el espacio en el vector de piezas muertas 
                break;
            }            
            if(i == 7)
            {
                contPosXNegras = 225;
                contPosYNegras = -225;
            }
        }
        Serial.println("");
    }
}
/**================================================================================================
 *                                           moveOnTheLine
 *  Esta funcion mueve piezas a traves de las lineas del tablero para evitar chocar con otras piezas.
 *  Es necesario verificar si la pocision inicial esta fuera del area de juego ya que en esta zona es
 *  necesario hacer un movimiento extra para evitar que la gondola se delice sobre la cara interior de
 * la base del ajedrez.
 *================================================================================================**/

void moveOnTheLine(double xIni,double yIni,double xFin,double yFin)
{

    
    //=======Puntos intermedios para trayectoria del caballo=========
    float diferenciaX;
    float diferenciaY;
    float puntoInterX;
    float puntoInterY;

    testPWMElectromagnetActivate();

    //Funcion para recorrer mas area del escaque
    //==========================================
    checkAreaChess(xIni, yIni, 6);
    //==========================================


    

    
    diferenciaX = abs(xIni - xFin);
    diferenciaY = abs(yIni - yFin);
    //======Condiciones para los casos donde la pieza se encuentra en los escaques de piezas muertas =======*/
    //Como primer movimiento acerca a la pieza en diagonal hasta la esquina del escaque inicial
    if(xIni < -200)
    {
        puntoInterX = xIni + 25;
        if(yFin > yIni)
        {
            puntoInterY = yIni + 25;
        }
        if(yFin <= yIni)
        {
            puntoInterY = yIni - 25;
        }
        #ifdef funcMoveTo
        Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(puntoInterX, puntoInterY,2);
        #endif
    }
    if(xIni > 200)
    {
        puntoInterX = xIni - 25;
        if(yFin > yIni)
        {
            puntoInterY = yIni + 25;
        }
        if(yFin <= yIni)
        {
            puntoInterY = yIni - 25;
        }
        #ifdef funcMoveTo
        Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(puntoInterX, puntoInterY,2);
        #endif
    }
    if(yIni < -200)
    {
        if(xFin < xIni)
        {
            puntoInterX = xIni - 25;
        }
        if(xFin >= xIni)
        {
            puntoInterX = xIni - 25;
        }
        puntoInterY = yIni + 25;
        #ifdef funcMoveTo
        Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(puntoInterX, puntoInterY,2);
        #endif
    }
    if(yIni > 200)
    {
        if(xFin < xIni)
        {
            puntoInterX = xIni - 25;
        }
        if(xFin >= xIni)
        {
            puntoInterX = xIni - 25;
        }
        puntoInterY = yIni - 25;
        #ifdef funcMoveTo
        Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(puntoInterX, puntoInterY,2);
        #endif
    }
    /*================================================================================================*/
    if(diferenciaX > diferenciaY)
    {
        if(xIni < 0) //Si es menor a cero el primere punto intermedio lo hara hacia la derecha
        {
            if((xIni <= 200 && xIni >= -200) && (yIni <= 200 && yIni >= -200))    //Punto inicial dentro del area de juego
            {
                puntoInterX = xIni + 25;
                puntoInterY = yIni;
                #ifdef funcMoveTo
                Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
            }
        }
        else    //Si no es menor a cero el primere punto intermedio lo hara hacia la izquierda
        {
            if((xIni <= 200 && xIni >= -200) && (yIni <= 200 && yIni >= -200))    //Punto inicial dentro del area de juego
            {
                puntoInterX = xIni - 25;
                puntoInterY = yIni;
                #ifdef funcMoveTo
                Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
            }
        }
        //===================================================================
        if(yFin > 0)
        {
            puntoInterY = yFin - 25;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        else
        {
            puntoInterY = yFin + 25;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        if(xFin <= 200 && xFin >= -200)
        {
            puntoInterX = xFin;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        else
        {
            if(xFin <= xIni)
            {
                puntoInterX = xFin + 25;
            }
            else
            {
                puntoInterX = xFin - 25;
            }
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
    }
    else
    {
        if(xIni < 0)
        {
            if((xIni <= 200 && xIni >= -200) && (yIni <= 200 && yIni >= -200))    //Punto inicial dentro del area de juego
            {
                puntoInterX = xIni + 25;
                puntoInterY = yIni;
                #ifdef funcMoveTo
                Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
            }
        }
        else
        {
            if((xIni <= 200 && xIni >= -200) && (yIni <= 200 && yIni >= -200))    //Punto inicial dentro del area de juego
            {
                puntoInterX = xIni - 25;
                puntoInterY = yIni;
                #ifdef funcMoveTo
                Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
            }
        }
        //===================================================================
        if(yFin > 0)
        {
            puntoInterY = yFin - 25;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        else
        {
            puntoInterY = yFin + 25;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        if(xFin <= 200 && xFin >= -200)
        {
            puntoInterX = xFin;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        else
        {
            if(xFin <= xIni)
            {
                puntoInterX = xFin + 25;
            }
            else
            {
                puntoInterX = xFin - 25;
            }
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX,puntoInterY,2);
            #endif
        }
    }
    //===============================================================
    #ifdef centrarElectromagnet
    double compXanterior = 0;
    double compYanterior = 0;
    
    compXanterior = xFin;
    compYanterior = yFin;

    double desfase;
    desfase = desfaseElectroiman;
    if((puntoInterX != xFin)&&(puntoInterY != yFin))
    {
        desfase = sqrt((desfaseElectroiman*desfaseElectroiman)/2);
    }

    if(puntoInterX > xFin)
    {
        compXanterior = compXanterior + desfase;
    }
    if(puntoInterX < xFin)
    {
        compXanterior = compXanterior - desfase;
    }

    if(puntoInterY > yFin)
    {
        compYanterior = compYanterior + desfase;
    }
    if(puntoInterY < yFin)
    {
        compYanterior = compYanterior - desfase;
    }
    #ifdef funcMoveTo
    Robot.moveToPointV2(compXanterior,compYanterior,1);
    #endif

    #ifdef funcMoveTo
    
    testPWMElectromagnetDeactivate();
    #endif

    #endif


    #ifdef funcMoveTo
    Robot.moveToPointV2(xFin,yFin);
    #endif
    #ifdef funcAccelRamp
    Robot.accelRamp(xFin,yFin,2,1);
    #endif

    #ifdef centrarElectromagnet
    activateElectromagnet();
    #endif


    deactivateElectromagnet();
    //testPWMElectromagnetDeactivate();
}


void comerVersion3(char xInicio, char yInicio,int movement, bool chess_color, char pieza)
{   
    int vectPosNegrasX[16] = {225,225,225,225,225,225,225,225,175,125,75,25,-25,-75,-125,-175};                     //Vectores con las coordenadas XY para las posiciones en el cementerio
    int vectPosNegrasY[16] = {175,125,75,25,-25,-75,-125,-175,-225,-225,-225,-225,-225,-225,-225,-225};
    int vectPosBlancasX[16]  = {-225,-225,-225,-225,-225,-225,-225,-225,-175,-125,-75,-25,25,75,125,175};
    int vectPosBlancasY[16]  = {-175,-125,-75,-25,25,75,125,175,225,225,225,225,225,225,225,225};

    Serial.println("Dentro de funcion comerVersion3");
    Serial.println("Color del movimiento actual");
    if(chess_color == false)
    {
        Serial.println("Negras");
    }
    if(chess_color == true)
    {
        Serial.println("Blancas");
    }
    
    
    BoardPosition boardPosition;

    char piezaEliminada;

    piezaEliminada = consultaPiezaTablero(xInicio,yInicio);                                                         //Consulta que pieza va a ser eliminada para almacenar el caracter que identifica la pieza muerta en el cementerio

    int puntoInicioX = 0;
    int puntoInicioY = 0;
    int puntoFinX = 0;
    int puntoFinY = 0;

    coordenadas(xInicio, yInicio, &boardPosition.x, &boardPosition.y);                                              //Busca las coordenadas donde se encuentra la pieza que sera removida de la zona de juego

    puntoInicioX = boardPosition.x;
    puntoInicioY = boardPosition.y;

    if(chess_color == false)                                                                                        //En el caso de que la pieza que come es de color negro
    {
        for(int i = 0; i < 16;i++)
        {
            if(vectBlancas[i] == 'v')                                                                               //En el vector de Piezas blancas muertas busca el ultimo eespacio vacio para colocar la pieza muerta
            {
                puntoFinX = vectPosBlancasX[i];                                                                     //Con el indice del vectBlancas donde encontro un espacio vacio busca las coordenadas correspondientes para llegar a esa en los vectores  vectPosBlancasX y  vectPosBlancasY
                puntoFinY = vectPosBlancasY[i];
                vectBlancas[i] = piezaEliminada;
                indexVectBlancas = i;
                i = 16;
            }
        }
    }

    if(chess_color == true)                                                                                         //En el caso de que la pieza que come es de color blannco
    {
        for(int i = 0; i < 16;i++)
        {
            if(vectNegras[i] == 'v')
            {
                puntoFinX = vectPosNegrasX[i];
                puntoFinY = vectPosNegrasY[i];
                vectNegras[i] = piezaEliminada;
                indexVectNegras = i;
                i = 16;
            }
        }
    }
    //moveOnTheLineIni(puntoInicioX,puntoInicioY,puntoFinX,puntoFinY);

    #ifdef funcMoveTo
    Robot.setSpeedMotors(maximumVelocityDirect);
    #endif
    #ifdef funcAccelRamp
    Robot.setSpeedMotors(maximunVelocityDirectRamp);
    Robot.setSpeedRampFunction(maximunVelocityDirectRamp);
    Robot.setAccelRampFunction(maximunAccelDirectRamp);
    #endif

    #ifdef moveDirectPosPiece
    
    #ifdef funcMoveTo
    Robot.moveToPointV2(puntoInicioX, puntoInicioY,1);
    #endif
    #ifdef funcAccelRamp

    #ifdef version1Electro
    Robot.accelRamp(puntoInicioX,puntoInicioY,2);   //Antes desacelracion,2
    #endif

    #ifdef version4Electro
    Robot.accelRamp(puntoInicioX,puntoInicioY,2,0,mainElectro);   //Antes desacelracion,2
    #endif

    #endif
    
    #else
    moveOnTheLineIni(puntoInicioX,puntoInicioY,puntoFinX,puntoFinY);
    #endif

    #ifdef funcMoveTo
    Robot.setSpeedMotors(defGlobalSpeed);
    #endif
    #ifdef funcAccelRamp
    Robot.setSpeedMotors(defGlobalSpeed);
    Robot.setSpeedRampFunction(defGlobalSpeed);
    Robot.setAccelRampFunction(defGlobalAccel);
    #endif

    //moveOnTheLine(puntoInicioX,puntoInicioY,puntoFinX,puntoFinY);
    #ifdef version1Electro
    moveOnTheLinev2(puntoInicioX,puntoInicioY,puntoFinX,puntoFinY);
    #endif

    #ifdef version4Electro
    moveOnTheLinev2(puntoInicioX,puntoInicioY,puntoFinX,puntoFinY,mainElectro);
    #endif
}

void configDriver(void)
{
    SERIAL_PORT2.begin(115200);

    pinMode(MOTOR_0_STEP_PIN, OUTPUT);
    pinMode(MOTOR_0_DIR_PIN, OUTPUT);
    pinMode(MOTOR_1_STEP_PIN, OUTPUT);
    pinMode(MOTOR_1_DIR_PIN, OUTPUT);

    pinMode(ENABLE_PIN, OUTPUT);

    digitalWrite(MOTOR_0_DIR_PIN, HIGH);
    digitalWrite(MOTOR_1_DIR_PIN, HIGH);
    digitalWrite(ENABLE_PIN, LOW);

    //===========Motor 3 y 4============
    /*
    pinMode(MOTOR_2_STEP_PIN, OUTPUT);
    pinMode(MOTOR_2_DIR_PIN, OUTPUT);

    pinMode(MOTOR_3_STEP_PIN, OUTPUT);
    pinMode(MOTOR_3_DIR_PIN, OUTPUT);

    digitalWrite(MOTOR_2_DIR_PIN, HIGH);

    digitalWrite(MOTOR_3_DIR_PIN, HIGH);
  */
    //===========Motor 1 y 2============
    driver.begin();
    driver.pdn_disable(true); // enables the  PDN/UART comunication.
    driver.toff(4); // Establece el tiempo de disminucion lenta (tiempo de apagado) [1 ... 15]
        // Esta configuración también limita la frecuencia máxima de chopper. Para operar con StealthChop
        // En caso de operar solo con StealthChop, cualquier configuración está bien.
    driver.blank_time(24);
    driver.rms_current(CURRENT_IN_CALIBRATION); // Fija el valor de la corriente
    driver.microsteps(MICROSTEPPING); // Se define el valor de microstepps
    driver.TCOOLTHRS(0xFFFFF); // Velocidad umbral inferior para encender la energía inteligente CoolStep y StallGuard a la salida del DIAG
    driver.semin(0); // Umbral inferior CoolStep [0 ... 15].
        // Si SG_RESULT cae por debajo de este umbral, CoolStep aumenta la corriente a ambas bobinas.
        // 0: deshabilitar CoolStep
    driver.shaft(false); //Establece el sentido de giro del motor mediante la comunicacion UART
    driver.sedn(0b01); // Establece el número de lecturas de StallGuard2 por encima del umbral superior necesario
        // por cada disminución de corriente de la corriente del motor.
    driver.SGTHRS(STALL_VALUE); // Nivel de umbral StallGuard4 [0 ... 255] para la detección de bloqueo. Compensa
        // características específicas del motor y controla la sensibilidad. Un valor más alto da un valor más alto
        // sensibilidad. Un valor más alto hace que StallGuard4 sea más sensible y requiere menos torque para
        // indica una oposicion al movimiento.
    driver2.begin();
    driver2.pdn_disable(true); // Activa la comunicacion PDN/UART
    driver2.toff(4); // Establece el tiempo de disminucion lenta (tiempo de apagado) [1 ... 15]
        // Esta configuración también limita la frecuencia máxima de chopper. Para operar con StealthChop
        // En caso de operar solo con StealthChop, cualquier configuración está bien.
    driver2.blank_time(24);
    driver2.rms_current(CURRENT_IN_CALIBRATION); // Fija el valor de la corriente
    driver2.microsteps(MICROSTEPPING); // Se define el valor de microstepps
    driver2.TCOOLTHRS(0xFFFFF); // Velocidad umbral inferior para encender la energía inteligente CoolStep y StallGuard a la salida del DIAG
    driver2.semin(0); // Umbral inferior CoolStep [0 ... 15].
        // Si SG_RESULT cae por debajo de este umbral, CoolStep aumenta la corriente a ambas bobinas.
        // 0: deshabilitar CoolStep
    //driver.semax(2);
    driver2.shaft(false); // Establece el sentido de giro del motor mediante la comunicacion UART
    driver2.sedn(0b01); // Establece el número de lecturas de StallGuard2 por encima del umbral superior necesario
        // por cada disminución de corriente de la corriente del motor.
    driver2.SGTHRS(STALL_VALUE2); // Nivel de umbral StallGuard4 [0 ... 255] para la detección de bloqueo. Compensa
        // características específicas del motor y controla la sensibilidad. Un valor más alto da un valor más alto
        // sensibilidad. Un valor más alto hace que StallGuard4 sea más sensible y requiere menos torque para
        // indica una oposicion al movimiento.
    //===========Motor 3 y 4============
    driver3.begin();
    driver3.pdn_disable(true); // enables the  PDN/UART comunication.
    driver3.toff(4); // Establece el tiempo de disminucion lenta (tiempo de apagado) [1 ... 15]
        // Esta configuración también limita la frecuencia máxima de chopper. Para operar con StealthChop
        // En caso de operar solo con StealthChop, cualquier configuración está bien.
    driver3.blank_time(24);
    driver3.rms_current(CURRENT_IN_CALIBRATION); // Fija el valor de la corriente
    driver3.microsteps(MICROSTEPPING); // Se define el valor de microstepps
    driver3.TCOOLTHRS(0xFFFFF); // Velocidad umbral inferior para encender la energía inteligente CoolStep y StallGuard a la salida del DIAG
    driver3.semin(0); // Umbral inferior CoolStep [0 ... 15].
        // Si SG_RESULT cae por debajo de este umbral, CoolStep aumenta la corriente a ambas bobinas.
        // 0: deshabilitar CoolStep
    driver3.shaft(false); //Establece el sentido de giro del motor mediante la comunicacion UART
    driver3.sedn(0b01); // Establece el número de lecturas de StallGuard2 por encima del umbral superior necesario
        // por cada disminución de corriente de la corriente del motor.
    driver3.SGTHRS(STALL_VALUE3); // Nivel de umbral StallGuard4 [0 ... 255] para la detección de bloqueo. Compensa
        // características específicas del motor y controla la sensibilidad. Un valor más alto da un valor más alto
        // sensibilidad. Un valor más alto hace que StallGuard4 sea más sensible y requiere menos torque para
        // indica una oposicion al movimiento.
    driver4.begin();
    driver4.pdn_disable(true); // Activa la comunicacion PDN/UART
    driver4.toff(4); // Establece el tiempo de disminucion lenta (tiempo de apagado) [1 ... 15]
        // Esta configuración también limita la frecuencia máxima de chopper. Para operar con StealthChop
        // En caso de operar solo con StealthChop, cualquier configuración está bien.
    driver4.blank_time(24);
    driver4.rms_current(CURRENT_IN_CALIBRATION); // Fija el valor de la corriente
    driver4.microsteps(MICROSTEPPING); // Se define el valor de microstepps
    driver4.TCOOLTHRS(0xFFFFF); // Velocidad umbral inferior para encender la energía inteligente CoolStep y StallGuard a la salida del DIAG
    driver4.semin(0); // Umbral inferior CoolStep [0 ... 15].
        // Si SG_RESULT cae por debajo de este umbral, CoolStep aumenta la corriente a ambas bobinas.
        // 0: deshabilitar CoolStep
    //driver.semax(2);
    driver4.shaft(false); // Establece el sentido de giro del motor mediante la comunicacion UART
    driver4.sedn(0b01); // Establece el número de lecturas de StallGuard2 por encima del umbral superior necesario
        // por cada disminución de corriente de la corriente del motor.
    driver4.SGTHRS(STALL_VALUE4); // Nivel de umbral StallGuard4 [0 ... 255] para la detección de bloqueo. Compensa
        // características específicas del motor y controla la sensibilidad. Un valor más alto da un valor más alto
        // sensibilidad. Un valor más alto hace que StallGuard4 sea más sensible y requiere menos torque para
        // indica una oposicion al movimiento.
    driver.ihold(I_HOLD); //Antes 16
    driver2.ihold(I_HOLD); //Antes 16
    driver3.ihold(I_HOLD); //Antes 16
    driver4.ihold(I_HOLD); //Antes 16
    //EEPROM.begin(EEPROM_SIZE);
    
    int cont = 0;
    while (driver.microsteps() != MICROSTEPPING) {
        driver.microsteps(MICROSTEPPING);
        delay(100);
        cont++;
        if (cont == 5) {
            break;
        }
    }
    cont = 0;
    while (driver2.microsteps() != MICROSTEPPING) {
        driver2.microsteps(MICROSTEPPING);
        delay(100);
        cont++;
        if (cont == 5) {
            break;
        }
    }
    cont = 0;
    while (driver3.microsteps() != MICROSTEPPING) {
        driver3.microsteps(MICROSTEPPING);
        delay(100);
        cont++;
        if (cont == 5) {
            break;
        }
    }
    cont = 0;
    while (driver4.microsteps() != MICROSTEPPING) {
        driver4.microsteps(MICROSTEPPING);
        delay(100);
        cont++;
        if (cont == 5) {
            break;
        }
    }


    EEPROM.begin(EEPROM_SIZE);

}

void generateMatrizFromFEN(String stringFEN)
{
	char vectNum[10] = {'0','1','2','3','4','5','6','7','8','9'};
	char matrizResult[8][8];
	int numEspacios = 0;
	int indexX = 0;
	int indexY = 0;
    String s = stringFEN;
 
    int n = s.length();
    char char_array[n + 1];
	int band_num = 0;
 
    strcpy(char_array, s.c_str());
 
    for (int i = 0; i < n; i++)
	{
		Serial.print(char_array[i]);
	}
	Serial.print(" ");
    
	for(int k = 0; k < n; k++)
	{
		if(char_array[k] != ' ')
		{
			if(char_array[k] == '/')
			{
					indexY++;
					indexX = 0;
			}
			else
			{
				//Verificar si el caracter actual no es un numero
				for(int t = 0; t < 10; t++)
				{
					if(char_array[k] == vectNum[t])         		//Si encuentra un numero lo convierte a entero y llena ese numero de casillas con '.'
					{
						numEspacios = vectNum[t] - '0';
						for(int s = 0; s < numEspacios; s++)
						{
							matriz[indexX][indexY] = '.';
							indexX++;
						}
						band_num = 1;				
					}
				}
				if(band_num == 0)
				{
					matriz[indexX][indexY] = char_array[k];
					indexX++;
				}
			}
			band_num = 0;
		}
		if(char_array[k] == ' ')
		{
			k = n;
		}
	}
	
	Serial.println(" ");
    for(int j=0;j< 8;j++){
        for(int i=0;i< 8;i++){
            Serial.print(matriz[i][j]);
            Serial.print(" ");
        }
        Serial.println(" ");
    }
	Serial.println("Termina funcion generateMatrizFromFEN");
}

void initFromFEN(String newFen)
{
    if(newFen != FEN)
    {
        removeChessAuto();
        generateMatrizFromFEN(newFen);
        reorderAuto();
    }
    colorChessGlobal = true;
    globalTurn = true;
    
}

void upgradeInitFen(String newFen)
{
    if(newFen != FEN)
    {
        FEN = newFen;
        #ifdef debug
        Serial.print("FEN recibido: ");
        Serial.println(FEN);
        #endif
    }
}

/**================================================================================================
 **                                         Funciones Archivos
 *================================================================================================**/
void createNewFileChess(fs::FS &fs3, const char * dirAndNameFile, const char * words){
    Serial.printf("Writing file: %s\r\n", dirAndNameFile);

    File file3 = fs3.open(dirAndNameFile, FILE_WRITE);
    if(!file3){
        Serial.println("Failed to open");
        return;
    }
    if(file3.print(words)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file3.close();
}

void readFileChess(fs::FS &fs, const char * dirAndNameFile){
    Serial.printf("Reading file: %s\r\n", dirAndNameFile);

    File file = fs.open(dirAndNameFile);
    if(!file || file.isDirectory()){
        Serial.println("Failed to open");
        return;
    }
    if(file.available())
    {
        movesInFile = file.readString();
    }
    file.close();
}

void addContentFileChess(fs::FS &fs, const char * dirAndNameFile, const char * message){
    Serial.printf("Add Content to file: %s\r\n", dirAndNameFile);

    File file = fs.open(dirAndNameFile, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open");
        return;
    }
    if(file.print(message)){
        Serial.println("Add message");
    } else {
        Serial.println("Add Failed");
    }
    file.close();
}

void addContentFileChess(fs::FS &fs, const char * dirAndNameFile){
    Serial.printf("Deleting file: %s\r\n", dirAndNameFile);
    if(fs.remove(dirAndNameFile)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void listFilesChess(fs::FS &fs, const char * dirNameFile){
    const char * dirName2;

    File root2 = fs.open(dirNameFile);
    if(!root2){
        Serial.println("Failed open directory");
        return;
    }

    File file2 = root2.openNextFile();
    
    while(file2){
        Serial.print("  File: ");
        dirName2 = file2.name();
        Serial.print(dirName2);
        readFileChess(SPIFFS, dirName2);
        Serial.print("\tSize: ");
        Serial.println(file2.size());
        file2 = root2.openNextFile();
        totalFiles++;
    }
}

void deleteFile(fs::FS &fs, const char * dirAndNameFile){
    if(fs.remove(dirAndNameFile)){
        Serial.println("File deleted");
    }else {
        Serial.println("Delete failed");
    }
}

/*================================================================================================*/
//*------------------ Funciones para comunicacion con Novag -----------------*/
void initGame()
{
    char *message;
    message = "New Game";
    //Banderas para condiciones de enroque en FEN
    
    rookKingW = 0;
    rookQueenW = 0;
    rookKingB = 0;
    rookQueenB = 0;
    kingW = 0;
    kingB = 0;

    Serial.println(message);      //para jugar desde terminal y dos esp32      Serial2.write(message);
                                 //Serial2.flush();
    delay(200);
}


void sendMoveFromSerial(){
        String textoEnviado;
        char *message;
        int movementChess = 0; 
        char piezaChess= 'v';
        
        while(Serial.available() > 0) {
            textoEnviado = Serial.readString();

            int n = textoEnviado.length();
            char array[n + 1];
        
            strcpy(array, textoEnviado.c_str());
            message = array;


            //Serial.println(textoEnviado);         //texto contiene el String con el comando recibido
            Serial.println("Comando Enviado");
            Serial.println(array[4]);
            Serial.println(array[5]);
            Serial.println(array[6]);
            Serial.println(array[7]);
            Serial.println(array[8]);
            Serial2.write(message);
            Serial2.flush();
            if(array[6] == '-')
            {
                movementChess = 1;
            }
            if(array[6] == 'x')
            {
                movementChess = 3;
            }
            piezaChess = consultaPiezaTablero(array[4], array[5]);
            realizaJugada(array[4], array[5], array[7], array[8], movementChess, true, piezaChess);
        }
}

void recibeMove()
{
    String textoRecibido;
    char *message;
    int movementChess = 0;
    char piezaChess = 'v';

    while (Serial2.available() > 0)
    {
        textoRecibido = Serial2.readString();

        int n = textoRecibido.length();
        char array[n + 1];

        strcpy(array, textoRecibido.c_str());
        message = array;

        Serial.println(textoRecibido);
        Serial.println("Received command");
        Serial.println(array[2]);
        Serial.println(array[3]);
        Serial.println(array[4]);
        Serial.println(array[5]);
        Serial.println(array[6]);
        Serial2.flush();
        Serial.println("Received text");
        Serial.println(textoRecibido);
        Serial.println("Previous move");
        Serial.println(jugadaAnterior);
        if (textoRecibido != jugadaAnterior)
        {

            if (array[4] == '-')
            {
                movementChess = 1;
            }
            if (array[4] == 'x')
            {
                movementChess = 3;
            }
            piezaChess = consultaPiezaTablero(array[2], array[3]);
            realizaJugada(array[2], array[3], array[5], array[6], movementChess, false, piezaChess);
        }
        jugadaAnterior = textoRecibido;
    }
}

int receiveMovementSerial()
{
    String textoRecibido = "";
    int movementChess = 0;
    char piezaChess = 'v';
    
    #ifdef receiveSerialMessages
    while (Serial.available() > 0)
    {
        textoRecibido = Serial.readString();
        int n = textoRecibido.length();
        char array[n + 1];

        strcpy(array, textoRecibido.c_str());
        #ifdef debug
        Serial.println("Received text");
        Serial.println(textoRecibido);
        #endif

        if (textoRecibido != jugadaAnterior)
        {
            if (array[4] == '-')
            {
                movementChess = 1;
            }
            if (array[4] == 'x')
            {
                movementChess = 3;
            }
            piezaChess = consultaPiezaTablero(array[2], array[3]);
            decodeInfoM.charXIniDecode = array[2];
            decodeInfoM.charYIniDecode = array[3];
            decodeInfoM.charXFinDecode = array[5];
            decodeInfoM.charYFinDecode = array[6];
            decodeInfoM.charPiezaDecode = piezaChess;
            decodeInfoM.intAccionDecode = movementChess;
            #ifdef debug
            Serial.println("Chess piece");
            Serial.println(piezaChess);
            #endif
            //realizaJugada(array[2], array[3], array[5], array[6], movementChess, false, piezaChess);      //Se quito para la prueba de la maquina de estados
            if (movementChess == 1 || movementChess == 3)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        jugadaAnterior = textoRecibido;
    }
    #endif

    #ifdef receiveBluetoothMessages
    //while (Serial.available() > 0)
    //{

        textoRecibido = BT.verifNewCommandBluetooth();
        //textoRecibido = Serial.readString();
        int n = textoRecibido.length();
        char array[n + 1];

        strcpy(array, textoRecibido.c_str());
        #ifdef debug
        //Serial.println("Received text");        //Comentado porque se cicla y no se pueden seguir bien las jugadas en el serial
        //Serial.println(textoRecibido);
        #endif

        if (textoRecibido != "")    //Esta condicion cambia respecto de la version con serial
        {
            if (array[4] == '-')
            {
                movementChess = 1;
            }
            if (array[4] == 'x')
            {
                movementChess = 3;
            }
            piezaChess = consultaPiezaTablero(array[2], array[3]);
            decodeInfoM.charXIniDecode = array[2];
            decodeInfoM.charYIniDecode = array[3];
            decodeInfoM.charXFinDecode = array[5];
            decodeInfoM.charYFinDecode = array[6];
            decodeInfoM.charPiezaDecode = piezaChess;
            decodeInfoM.intAccionDecode = movementChess;
            #ifdef debug
            Serial.println("Chess piece");
            Serial.println(piezaChess);
            #endif
            //realizaJugada(array[2], array[3], array[5], array[6], movementChess, false, piezaChess);      //Se quito para la prueba de la maquina de estados
            if (movementChess == 1 || movementChess == 3)
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }
        else                        //Esta condicion cambia respecto de la version con serial
        {
            return 0;
        }
        jugadaAnterior = textoRecibido;
    //}

    #endif
}

void realizaJugada(char xIni, char yIni, char xFin, char yFin, int movement, bool chessColor, char pieza)
{
    cont_m++;
    #ifdef _realizaJugada
    Serial.println("En funcion realizaJugada");
    #endif
    
    //resultCastlingFEN =  castlingFEN(rookKingW,rookQueenW,rookKingB,rookQueenB,kingW,kingB);
    //updateFenFromMatriz(false,resultCastlingFEN,resultCapturaAlPasoFen);

    chessColor = !chessColor;
   
    #ifdef _realizaJugada
    Serial.println("Jugada");
    Serial.println(movement);
    
    
    switch (movement)
    {
    case 0:
        Serial.println(" ");
        break;
    case 1:
        Serial.println("Solo Mueve");
        break;
    case 2:
        Serial.println("Mueve y Promocion");
        break;
    case 3:
        Serial.println("Mueve y Come");
        break;
    case 4:
        Serial.println("Mueve y Jaque");
        break;
    case 5:
        Serial.println("Mueve, Promocion y Jaque");
        break;
    case 6:
        Serial.println("Mueve, Come y Jaque");
        break;
    case 7:
        Serial.println("Enroque Corto");
        break;
    case 8:
        Serial.println("Enroque Largo");
        break;
    default:
        Serial.println("Movimiento no valido");
        break;
    }
    #endif

    switch (pieza)
    {
    case 'K':
    case 'k':
        #ifdef _realizaJugada
        Serial.println("King");
        #endif
        chess_King(xIni, yIni, xFin, yFin, movement, chessColor, pieza);
        /*
        //===========Condiciones de primer movimiento del rey para posibilidad de enroque===============
        if (chessColor == true)
        {
            if (kingW == 0)
            {
                kingW = 1;
            }
        }
        if (chessColor == false)
        {
            if (kingB == 0)
            {
                kingB = 1;
            }
        }
        //===============================================================================================
         */
        break;
    case 'Q':
    case 'q':
        #ifdef _realizaJugada
        Serial.println("Queen");
        #endif
        chess_Queen(xIni, yIni, xFin, yFin, movement, chessColor, pieza);
        break;
    case 'R':
    case 'r':
        #ifdef _realizaJugada
        Serial.println("Rook");
        #endif
        chess_Rook(xIni, yIni, xFin, yFin, movement, chessColor, pieza);
        /*
        //===========Condiciones de primer movimiento de las torres para posibilidad de enroque===============
        if (chessColor == true)
        {
            if (xIni == 'a')
            {
                if (rookQueenW == 0)
                {
                    rookQueenW = 1;
                }
            }
            if (xIni == 'h')
            {
                if (rookKingW == 0)
                {
                    rookKingW = 1;
                }
            }
        }
        if (chessColor == false)
        {
            if (xIni == 'a')
            {
                if (rookQueenB == 0)
                {
                    rookQueenB = 1;
                }
            }
            if (xIni == 'h')
            {
                if (rookKingB == 0)
                {
                    rookKingB = 1;
                }
            }
        }
        //===============================================================================================
        */
        break;
    case 'B':
    case 'b':
        #ifdef _realizaJugada
        Serial.println("Bishop");
        #endif
        chess_Bishop(xIni, yIni, xFin, yFin, movement, chessColor, pieza);
        break;
    case 'N':
    case 'n':
        #ifdef _realizaJugada
        Serial.println("Knight");
        #endif
        chess_Knight(xIni, yIni, xFin, yFin, movement, chessColor, pieza);
        break;
    case 'P':
    case 'p':
        if (movement != 0)
        {
            #ifdef _realizaJugada
            Serial.println("Pawn Mueve");
            #endif
            chess_Pawn(xIni, yIni, xFin, yFin, movement, chessColor, pieza);
            resultCapturaAlPasoFen = capturaAlPasoFen(xFin, yFin, chessColor);
        }
        break;
    case 'S':
        /*
        enroque_corto(chessColor);

        //=======Condiciones para desactivar posibilidad de enroque en formato FEN=======
        if (chessColor == true)
        {
            kingW = 1;
        }
        if (chessColor == false)
        {
            kingB = 1;
        }
        //===============================================================================
        */
        break;
    case 'L':
         /*
        enroque_largo(chessColor);
        //=======Condiciones para desactivar posibilidad de enroque en formato FEN=======
        if (chessColor == true)
        {
            kingW = 1;
        }
        if (chessColor == false)
        {
            kingB = 1;
        }
        //===============================================================================
        */
        break;
    default:
        #ifdef _realizaJugada
        Serial.println("Pieza no valida");
        Serial.println(pieza);
        #endif
        break;
    }

    #ifdef _realizaJugada
    Serial.print(xIni);
    Serial.println(yIni);
    Serial.print(xFin);
    Serial.println(yFin);
    //Serial.print("Numero de movimientos: ");
    //Serial.println(cont_m);
    #endif

    if (movement != 0)
    {
        #ifdef _realizaJugada
        printMatriz();
        #endif
        //imprimirVectPiecesD();
        //resultCastlingFEN = castlingFEN(rookKingW, rookQueenW, rookKingB, rookQueenB, kingW, kingB);
        //updateFenFromMatriz(chessColor, resultCastlingFEN, resultCapturaAlPasoFen);
    }
    resultCapturaAlPasoFen = "-";
    movement = 0;
}

void sensorsDir()
{
    /*
    dirSensor[0][4][4] = 0x03;
    dirSensor[0][4][3] = 0x06;
    dirSensor[0][4][2] = 0x04;
    dirSensor[0][4][1] = 0x07;
    dirSensor[0][4][0] = 0x05;
    dirSensor[0][3][4] = 0x16;
    dirSensor[0][3][3] = 0x01;
    dirSensor[0][3][2] = 0x02;
    dirSensor[0][3][1] = 0x00;
    dirSensor[0][3][0] = 0x14;
    dirSensor[0][2][4] = 0x10;
    dirSensor[0][2][3] = 0x12;
    dirSensor[0][2][2] = 0x15;
    dirSensor[0][2][1] = 0x11;
    dirSensor[0][2][0] = 0x13;
    dirSensor[0][1][4] = 0x25;
    dirSensor[0][1][3] = 0x24;
    dirSensor[0][1][2] = 0x17;
    dirSensor[0][1][1] = 0x26;
    dirSensor[0][1][0] = 0x27;
    dirSensor[0][0][4] = 0x30;
    dirSensor[0][0][3] = 0x22;
    dirSensor[0][0][2] = 0x21;
    dirSensor[0][0][1] = 0x20;
    dirSensor[0][0][0] = 0x23;

    dirSensor[1][4][4] = 0x43;
    dirSensor[1][4][3] = 0x46;
    dirSensor[1][4][2] = 0x44;
    dirSensor[1][4][1] = 0x47;
    dirSensor[1][4][0] = 0x45;
    dirSensor[1][3][4] = 0x56;
    dirSensor[1][3][3] = 0x41;
    dirSensor[1][3][2] = 0x42;
    dirSensor[1][3][1] = 0x40;
    dirSensor[1][3][0] = 0x54;
    dirSensor[1][2][4] = 0x50;
    dirSensor[1][2][3] = 0x52;
    dirSensor[1][2][2] = 0x55;
    dirSensor[1][2][1] = 0x51;
    dirSensor[1][2][0] = 0x53;
    dirSensor[1][1][4] = 0x65;
    dirSensor[1][1][3] = 0x64;
    dirSensor[1][1][2] = 0x57;
    dirSensor[1][1][1] = 0x66;
    dirSensor[1][1][0] = 0x67;
    dirSensor[1][0][4] = 0x70;
    dirSensor[1][0][3] = 0x62;
    dirSensor[1][0][2] = 0x61;
    dirSensor[1][0][1] = 0x60;
    dirSensor[1][0][0] = 0x63;

    dirSensor[2][4][4] = 0xF3;
    dirSensor[2][4][3] = 0xF6;
    dirSensor[2][4][2] = 0xF4;
    dirSensor[2][4][1] = 0xF7;
    dirSensor[2][4][0] = 0xF5;
    dirSensor[2][3][4] = 0xE6;
    dirSensor[2][3][3] = 0xF1;
    dirSensor[2][3][2] = 0xF2;
    dirSensor[2][3][1] = 0xF0;
    dirSensor[2][3][0] = 0xE4;
    dirSensor[2][2][4] = 0xE0;
    dirSensor[2][2][3] = 0xE2;
    dirSensor[2][2][2] = 0xE5;
    dirSensor[2][2][1] = 0xE1;
    dirSensor[2][2][0] = 0xE3;
    dirSensor[2][1][4] = 0xD5;
    dirSensor[2][1][3] = 0xD4;
    dirSensor[2][1][2] = 0xE7;
    dirSensor[2][1][1] = 0xD6;
    dirSensor[2][1][0] = 0xD7;
    dirSensor[2][0][4] = 0xC0;
    dirSensor[2][0][3] = 0xD2;
    dirSensor[2][0][2] = 0xD1;
    dirSensor[2][0][1] = 0xD0;
    dirSensor[2][0][0] = 0xD3;

    dirSensor[3][4][4] = 0xB3;
    dirSensor[3][4][3] = 0xB6;
    dirSensor[3][4][2] = 0xB4;
    dirSensor[3][4][1] = 0xB7;
    dirSensor[3][4][0] = 0xB5;
    dirSensor[3][3][4] = 0xA6;
    dirSensor[3][3][3] = 0xB1;
    dirSensor[3][3][2] = 0xB2;
    dirSensor[3][3][1] = 0xB0;
    dirSensor[3][3][0] = 0xA4;
    dirSensor[3][2][4] = 0xA0;
    dirSensor[3][2][3] = 0xA2;
    dirSensor[3][2][2] = 0xA5;
    dirSensor[3][2][1] = 0xA1;
    dirSensor[3][2][0] = 0xA3;
    dirSensor[3][1][4] = 0x95;
    dirSensor[3][1][3] = 0x94;
    dirSensor[3][1][2] = 0xA7;
    dirSensor[3][1][1] = 0x96;
    dirSensor[3][1][0] = 0x97;
    dirSensor[3][0][4] = 0x80;
    dirSensor[3][0][3] = 0x92;
    dirSensor[3][0][2] = 0x91;
    dirSensor[3][0][1] = 0x90;
    dirSensor[3][0][0] = 0x93;
    */

   dirSensor[0][0] = 0x00;
    dirSensor[0][1] = 0x01;
    dirSensor[0][2] = 0x02;
    dirSensor[0][3] = 0x03;
    dirSensor[0][4] = 0x04;
    dirSensor[0][5] = 0x05;
    dirSensor[0][6] = 0x06;
    dirSensor[0][7] = 0x07;
    dirSensor[1][0] = 0x10;
    dirSensor[1][1] = 0x11;
    dirSensor[1][2] = 0x12;
    dirSensor[1][3] = 0x13;
    dirSensor[1][4] = 0x14;
    dirSensor[1][5] = 0x15;
    dirSensor[1][6] = 0x16;
    dirSensor[1][7] = 0x17;
    dirSensor[2][0] = 0x20;    
    dirSensor[2][1] = 0x21;
    dirSensor[2][2] = 0x22;
    dirSensor[2][3] = 0x23;
    dirSensor[2][4] = 0x24;
    dirSensor[2][5] = 0x25;
    dirSensor[2][6] = 0x26;
    dirSensor[2][7] = 0x27;
    dirSensor[3][0] = 0x30;
    dirSensor[3][1] = 0x31;   
    dirSensor[3][2] = 0x32;
    dirSensor[3][3] = 0x33;
    dirSensor[3][4] = 0x34;
    dirSensor[3][5] = 0x35;
    dirSensor[3][6] = 0x36;
    dirSensor[3][7] = 0x37;
    dirSensor[4][0] = 0x40;
    dirSensor[4][1] = 0x41;   
    dirSensor[4][2] = 0x42;
    dirSensor[4][3] = 0x43;
    dirSensor[4][4] = 0x44;
    dirSensor[4][5] = 0x45;
    dirSensor[4][6] = 0x46;
    dirSensor[4][7] = 0x47;
    dirSensor[5][0] = 0x50;
    dirSensor[5][1] = 0x51;     
    dirSensor[5][2] = 0x52;
    dirSensor[5][3] = 0x53;
    dirSensor[5][4] = 0x54;
    dirSensor[5][5] = 0x55;
    dirSensor[5][6] = 0x56;
    dirSensor[5][7] = 0x57;
    dirSensor[6][0] = 0x60;
    dirSensor[6][1] = 0x61;  
    dirSensor[6][2] = 0x62;
    dirSensor[6][3] = 0x63;
    dirSensor[6][4] = 0x64;
    dirSensor[6][5] = 0x65;
    dirSensor[6][6] = 0x66;
    dirSensor[6][7] = 0x67;
    dirSensor[7][0] = 0x70;  
    dirSensor[7][1] = 0x71;
    dirSensor[7][2] = 0x72;
    dirSensor[7][3] = 0x73;
    dirSensor[7][4] = 0x74;
    dirSensor[7][5] = 0x75;
    dirSensor[7][6] = 0x76;
    dirSensor[7][7] = 0x77;

}

void detectChessBoard()
{
    bool matrizBinTemp[10][10];

    #ifdef version1Electro
    testPWMElectromagnetDeactivate();
    #endif

    #ifdef version4Electro
    testPWMElectromagnetDeactivateV4E(1);
    testPWMElectromagnetDeactivateV4E(2);
    testPWMElectromagnetDeactivateV4E(3);
    testPWMElectromagnetDeactivateV4E(4);  
    #endif

    readRawChessBoard();

#ifdef oldSensors
    orderedSensorValues[0][0]=muxValues[3][4][4];
    orderedSensorValues[0][1]=muxValues[3][4][6];
    orderedSensorValues[0][2]=muxValues[3][4][7];
    orderedSensorValues[0][3]=muxValues[3][4][5];
    orderedSensorValues[0][4]=muxValues[3][4][2];
    orderedSensorValues[0][5]=muxValues[3][4][1];
    orderedSensorValues[0][6]=muxValues[3][4][0];
    orderedSensorValues[0][7]=muxValues[3][4][3];
    orderedSensorValues[0][8]=muxValues[3][6][0];
    orderedSensorValues[0][9]=muxValues[3][7][0];

    orderedSensorValues[1][0]=muxValues[3][2][4];
    orderedSensorValues[1][1]=muxValues[3][2][6];
    orderedSensorValues[1][2]=muxValues[3][2][7];
    orderedSensorValues[1][3]=muxValues[3][2][5];
    orderedSensorValues[1][4]=muxValues[3][2][2];
    orderedSensorValues[1][5]=muxValues[3][2][1];
    orderedSensorValues[1][6]=muxValues[3][2][0];
    orderedSensorValues[1][7]=muxValues[3][2][3];
    orderedSensorValues[1][8]=muxValues[3][1][0];
    orderedSensorValues[1][9]=muxValues[3][0][0];

    orderedSensorValues[2][0]=muxValues[4][2][4];
    orderedSensorValues[2][1]=muxValues[4][2][6];
    orderedSensorValues[2][2]=muxValues[4][2][7];
    orderedSensorValues[2][3]=muxValues[4][2][5];
    orderedSensorValues[2][4]=muxValues[4][2][2];
    orderedSensorValues[2][5]=muxValues[4][2][1];
    orderedSensorValues[2][6]=muxValues[4][2][0];
    orderedSensorValues[2][7]=muxValues[4][2][3];
    orderedSensorValues[2][8]=muxValues[3][5][0];
    orderedSensorValues[2][9]=muxValues[3][3][0];

    orderedSensorValues[3][0]=muxValues[4][3][4];
    orderedSensorValues[3][1]=muxValues[4][3][6];
    orderedSensorValues[3][2]=muxValues[4][3][7];
    orderedSensorValues[3][3]=muxValues[4][3][5];
    orderedSensorValues[3][4]=muxValues[4][3][2];
    orderedSensorValues[3][5]=muxValues[4][3][1];
    orderedSensorValues[3][6]=muxValues[4][3][0];
    orderedSensorValues[3][7]=muxValues[4][3][3];
    orderedSensorValues[3][8]=muxValues[4][0][0];
    orderedSensorValues[3][9]=muxValues[4][1][0];

    orderedSensorValues[4][0]=muxValues[4][5][4];
    orderedSensorValues[4][1]=muxValues[4][5][6];
    orderedSensorValues[4][2]=muxValues[4][5][7];
    orderedSensorValues[4][3]=muxValues[4][5][5];
    orderedSensorValues[4][4]=muxValues[4][5][2];
    orderedSensorValues[4][5]=muxValues[4][5][1];
    orderedSensorValues[4][6]=muxValues[4][5][0];
    orderedSensorValues[4][7]=muxValues[4][5][3];
    orderedSensorValues[4][8]=muxValues[4][7][0];
    orderedSensorValues[4][9]=muxValues[4][6][0];

    orderedSensorValues[5][0]=muxValues[2][5][4];
    orderedSensorValues[5][1]=muxValues[2][5][6];
    orderedSensorValues[5][2]=muxValues[2][5][7];
    orderedSensorValues[5][3]=muxValues[2][5][5];
    orderedSensorValues[5][4]=muxValues[2][5][2];
    orderedSensorValues[5][5]=muxValues[2][5][1];
    orderedSensorValues[5][6]=muxValues[2][5][0];
    orderedSensorValues[5][7]=muxValues[2][5][3];
    orderedSensorValues[5][8]=muxValues[2][7][0];
    orderedSensorValues[5][9]=muxValues[2][6][0];

    orderedSensorValues[6][0]=muxValues[2][0][4];
    orderedSensorValues[6][1]=muxValues[2][0][6];
    orderedSensorValues[6][2]=muxValues[2][0][7];
    orderedSensorValues[6][3]=muxValues[2][0][5];
    orderedSensorValues[6][4]=muxValues[2][0][2];
    orderedSensorValues[6][5]=muxValues[2][0][1];
    orderedSensorValues[6][6]=muxValues[2][0][0];
    orderedSensorValues[6][7]=muxValues[2][0][3];
    orderedSensorValues[6][8]=muxValues[2][1][0];
    orderedSensorValues[6][9]=muxValues[2][2][0];

    orderedSensorValues[7][0]=muxValues[1][5][4];
    orderedSensorValues[7][1]=muxValues[1][5][6];
    orderedSensorValues[7][2]=muxValues[1][5][7];
    orderedSensorValues[7][3]=muxValues[1][5][5];
    orderedSensorValues[7][4]=muxValues[1][5][2];
    orderedSensorValues[7][5]=muxValues[1][5][1];
    orderedSensorValues[7][6]=muxValues[1][5][0];
    orderedSensorValues[7][7]=muxValues[1][5][3];
    orderedSensorValues[7][8]=muxValues[1][3][0];
    orderedSensorValues[7][9]=muxValues[2][3][0];

    orderedSensorValues[8][0]=muxValues[1][7][4];
    orderedSensorValues[8][1]=muxValues[1][7][6];
    orderedSensorValues[8][2]=muxValues[1][7][7];
    orderedSensorValues[8][3]=muxValues[1][7][5];
    orderedSensorValues[8][4]=muxValues[1][7][2];
    orderedSensorValues[8][5]=muxValues[1][7][1];
    orderedSensorValues[8][6]=muxValues[1][7][0];
    orderedSensorValues[8][7]=muxValues[1][7][3];
    orderedSensorValues[8][8]=muxValues[1][6][0];
    orderedSensorValues[8][9]=muxValues[1][4][0];

    orderedSensorValues[9][0]=muxValues[1][0][4];
    orderedSensorValues[9][1]=muxValues[1][0][6];
    orderedSensorValues[9][2]=muxValues[1][0][7];
    orderedSensorValues[9][3]=muxValues[1][0][5];
    orderedSensorValues[9][4]=muxValues[1][0][2];
    orderedSensorValues[9][5]=muxValues[1][0][1];
    orderedSensorValues[9][6]=muxValues[1][0][0];
    orderedSensorValues[9][7]=muxValues[1][0][3];
    orderedSensorValues[9][8]=muxValues[1][1][0];
    orderedSensorValues[9][9]=muxValues[1][2][0];
#endif
    
#ifdef newSensors
    orderedSensorValues[0][0]=muxValues[3][4][4];
    orderedSensorValues[0][1]=muxValues[3][4][6];
    orderedSensorValues[0][2]=muxValues[3][4][7];
    orderedSensorValues[0][3]=muxValues[3][4][5];
    orderedSensorValues[0][4]=muxValues[3][4][2];
    orderedSensorValues[0][5]=muxValues[3][4][1];
    orderedSensorValues[0][6]=muxValues[3][4][0];
    orderedSensorValues[0][7]=muxValues[3][4][3];
    orderedSensorValues[0][8]=muxValues[3][7][0];
    orderedSensorValues[0][9]=muxValues[3][6][0];
    orderedSensorValues[1][0]=muxValues[3][1][4];
    orderedSensorValues[1][1]=muxValues[3][1][6];
    orderedSensorValues[1][2]=muxValues[3][1][7];
    orderedSensorValues[1][3]=muxValues[3][1][5];
    orderedSensorValues[1][4]=muxValues[3][1][2];
    orderedSensorValues[1][5]=muxValues[3][1][1];
    orderedSensorValues[1][6]=muxValues[3][1][0];
    orderedSensorValues[1][7]=muxValues[3][1][3];
    orderedSensorValues[1][8]=muxValues[3][5][0];
    orderedSensorValues[1][9]=muxValues[3][2][0];
    orderedSensorValues[2][0]=muxValues[4][0][4];
    orderedSensorValues[2][1]=muxValues[4][0][6];
    orderedSensorValues[2][2]=muxValues[4][0][7];
    orderedSensorValues[2][3]=muxValues[4][0][5];
    orderedSensorValues[2][4]=muxValues[4][0][2];
    orderedSensorValues[2][5]=muxValues[4][0][1];
    orderedSensorValues[2][6]=muxValues[4][0][0];
    orderedSensorValues[2][7]=muxValues[4][0][3];
    orderedSensorValues[2][8]=muxValues[3][0][0];
    orderedSensorValues[2][9]=muxValues[3][3][0];
    orderedSensorValues[3][0]=muxValues[4][6][4];
    orderedSensorValues[3][1]=muxValues[4][6][6];
    orderedSensorValues[3][2]=muxValues[4][6][7];
    orderedSensorValues[3][3]=muxValues[4][6][5];
    orderedSensorValues[3][4]=muxValues[4][6][2];
    orderedSensorValues[3][5]=muxValues[4][6][1];
    orderedSensorValues[3][6]=muxValues[4][6][0];
    orderedSensorValues[3][7]=muxValues[4][6][3];
    orderedSensorValues[3][8]=muxValues[4][2][0];
    orderedSensorValues[3][9]=muxValues[4][1][0];
    orderedSensorValues[4][0]=muxValues[4][3][4];
    orderedSensorValues[4][1]=muxValues[4][3][6];
    orderedSensorValues[4][2]=muxValues[4][3][7];
    orderedSensorValues[4][3]=muxValues[4][3][5];
    orderedSensorValues[4][4]=muxValues[4][3][2];
    orderedSensorValues[4][5]=muxValues[4][3][1];
    orderedSensorValues[4][6]=muxValues[4][3][0];
    orderedSensorValues[4][7]=muxValues[4][3][3];
    orderedSensorValues[4][8]=muxValues[4][5][0];
    orderedSensorValues[4][9]=muxValues[4][7][0];
    orderedSensorValues[5][0]=muxValues[2][6][4];
    orderedSensorValues[5][1]=muxValues[2][6][6];
    orderedSensorValues[5][2]=muxValues[2][6][7];
    orderedSensorValues[5][3]=muxValues[2][6][5];
    orderedSensorValues[5][4]=muxValues[2][6][2];
    orderedSensorValues[5][5]=muxValues[2][6][1];
    orderedSensorValues[5][6]=muxValues[2][6][0];
    orderedSensorValues[5][7]=muxValues[2][6][3];
    orderedSensorValues[5][8]=muxValues[2][5][0];
    orderedSensorValues[5][9]=muxValues[2][7][0];
    orderedSensorValues[6][0]=muxValues[2][0][4];
    orderedSensorValues[6][1]=muxValues[2][0][6];
    orderedSensorValues[6][2]=muxValues[2][0][7];
    orderedSensorValues[6][3]=muxValues[2][0][5];
    orderedSensorValues[6][4]=muxValues[2][0][2];
    orderedSensorValues[6][5]=muxValues[2][0][1];
    orderedSensorValues[6][6]=muxValues[2][0][0];
    orderedSensorValues[6][7]=muxValues[2][0][3];
    orderedSensorValues[6][8]=muxValues[2][1][0];
    orderedSensorValues[6][9]=muxValues[2][2][0];
    orderedSensorValues[7][0]=muxValues[1][4][4];
    orderedSensorValues[7][1]=muxValues[1][4][6];
    orderedSensorValues[7][2]=muxValues[1][4][7];
    orderedSensorValues[7][3]=muxValues[1][4][5];
    orderedSensorValues[7][4]=muxValues[1][4][2];
    orderedSensorValues[7][5]=muxValues[1][4][1];
    orderedSensorValues[7][6]=muxValues[1][4][0];
    orderedSensorValues[7][7]=muxValues[1][4][3];
    orderedSensorValues[7][8]=muxValues[2][3][0];
    orderedSensorValues[7][9]=muxValues[1][3][0];
    orderedSensorValues[8][0]=muxValues[1][6][4];
    orderedSensorValues[8][1]=muxValues[1][6][6];
    orderedSensorValues[8][2]=muxValues[1][6][7];
    orderedSensorValues[8][3]=muxValues[1][6][5];
    orderedSensorValues[8][4]=muxValues[1][6][2];
    orderedSensorValues[8][5]=muxValues[1][6][1];
    orderedSensorValues[8][6]=muxValues[1][6][0];
    orderedSensorValues[8][7]=muxValues[1][6][3];
    orderedSensorValues[8][8]=muxValues[1][5][0];
    orderedSensorValues[8][9]=muxValues[1][7][0];
    orderedSensorValues[9][0]=muxValues[1][0][4];
    orderedSensorValues[9][1]=muxValues[1][0][6];
    orderedSensorValues[9][2]=muxValues[1][0][7];
    orderedSensorValues[9][3]=muxValues[1][0][5];
    orderedSensorValues[9][4]=muxValues[1][0][2];
    orderedSensorValues[9][5]=muxValues[1][0][1];
    orderedSensorValues[9][6]=muxValues[1][0][0];
    orderedSensorValues[9][7]=muxValues[1][0][3];
    orderedSensorValues[9][8]=muxValues[1][2][0];
    orderedSensorValues[9][9]=muxValues[1][1][0];
#endif


//================  Pasar el vector a la matriz binaria Temporal=================
int a = 0;
int b = 9;

for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinTemp[j][i] = orderedSensorValues[a][b];
        b = b - 1;
    }
    a = a + 1;
    b = 9;
}

#if PosMatrizSensor == matrizPos1
//================  Matriz en posicion 1 (Original) =================
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBin[i][j] = matrizBinTemp[i][j];
    }
}
#endif

#ifdef debugSensors
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        //matrizBinVerif[i][j] = matrizBinTemp[i][j];
        Serial.print(matrizBin[i][j]);
    }
    Serial.println("");
}
#endif

#if PosMatrizSensor == matrizPos2
//================  Matriz en posicion 2 (Rota 90grados en sentido horario) =================
 a = 0;
 b = 9;
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBin[i][j] = matrizBinTemp[a][b];
        b = b-1;
    }
    a = a + 1;
    b = 9;
}
#endif

#if PosMatrizSensor == matrizPos3
//================  Matriz en posicion 3 (Rota 180grados en sentido horario) =================
 a = 9;
 b = 9;
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBin[i][j] = matrizBinTemp[a][b];
        a = a-1;
    }
    a = 9;
    b = b - 1;
}
#endif

#if PosMatrizSensor == matrizPos4
//================  Matriz en posicion 4 (Rota 270grados en sentido horario) =================
 a = 9;
 b = 0;
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBin[i][j] = matrizBinTemp[a][b];
        b = b + 1;
    }
    a = a - 1;
    b = 0;
}
#endif

}

void readRawChessBoard()
{
  /*  
  for (int k = 0; k < 4; k++) //Representa las barras de sensores conectadas en orden al Mux de 16.
  {
    for (int i = 0; i < 5; i++) //Representa la direccion de los sensores a nivel de Mux16 y Mux8
    {
        for (int j = 0; j < 5; j++)  //Representa la direccion de los sensores a nivel de Mux16 y Mux8
        {
                    //Las direcciones estan dadas en valor decodificado de los sensores a nivel de selectores de Mux. 
                    //Por ello si queremos acceder al sensor [0][0][0] que tiene la direccion 0x23, significa que las compuertas deben tener el siguiente codigo 0010 0011
                    //Es decir que el mux de 8 seleccionara 011 y el mux de 16 seleccionara 0010. 
                    //Para mandar estas senales digitalmente a las compuertas lo que hacemos es separar el byte de direccion y lo escribimos en las salidas digitales correspondientes. 
                      digitalWrite(mux8_0, bitRead(dirSensor[k][i][j], 0)); 
                      digitalWrite(mux8_1, bitRead(dirSensor[k][i][j], 1));
                      digitalWrite(mux8_2, bitRead(dirSensor[k][i][j], 2));

                      digitalWrite(mux16_0, bitRead(dirSensor[k][i][j], 4));
                      digitalWrite(mux16_1, bitRead(dirSensor[k][i][j], 5));
                      digitalWrite(mux16_2, bitRead(dirSensor[k][i][j], 6));
                      digitalWrite(mux16_3, bitRead(dirSensor[k][i][j], 7));

                    delayMicroseconds(timeBsensors); // Este delay es encesario para que le tiempo de switchear las compuertas antes de leer los datos, en caso de que no existiera, leeriamos ruido. 
                      
                      //Podemos ver que podemos leer 5 datos en paralelo por cada uno de los puertos de entrada al ESP32.
                      muxValues[0][k][i][j]= (digitalRead(mux16Out_0));
                      muxValues[1][k][i][j]= (digitalRead(mux16Out_1));
                      muxValues[2][k][i][j]= (digitalRead(mux16Out_2));
                      muxValues[3][k][i][j]= (digitalRead(mux16Out_3));
                      muxValues[4][k][i][j]= (digitalRead(mux16Out_4));
                      

                  
                  //Hacemos el Or cuando se hayan juntado ya los 5 datos que estamos esperando. El cuadro en cuestion esta definido por Z que es el MUX, K que es la barra a la que esta conectado, I que es el numero de Cuadro. 
                      if (j == 4)
                      {
                              for (int z = 0; z < 5; z++) //En realidad cuando se cumpla la condicion anterior tendremos 25 daatos que es el equivalente a 5 cuadros
                          {
                                      if (muxValues[z][k][i][0] == 0 || muxValues[z][k][i][1] == 0 || muxValues[z][k][i][2] == 0 || muxValues[z][k][i][3] == 0 || muxValues[z][k][i][4] == 0  )
                                  {
                                    muxSquaresValues[z][k][i] = 0;
                                  }else
                                  {
                                    muxSquaresValues[z][k][i] = 1;
                                  }
                            } 
                      }
        }
    }
  }
  */

 for (int i = 0; i < 8; i++) //Representa la direccion de los sensores a nivel de MuxFIRST 8
    {
        for (int j = 0; j < 8; j++)  //Representa la direccion de los sensores a nivel de Mux8
        {
                        //Las direcciones estan dadas en valor decodificado de los sensores a nivel de selectores de Mux. 
                        //Por ello si queremos acceder al sensor [0][0][0] que tiene la direccion 0x23, significa que las compuertas deben tener el siguiente codigo 0010 0011
                        //Es decir que el mux de 8 seleccionara 011 y el mux de 16 seleccionara 0010. 
                        //Para mandar estas senales digitalmente a las compuertas lo que hacemos es separar el byte de direccion y lo escribimos en las salidas digitales correspondientes. 
                      digitalWrite(mux8_0, bitRead(dirSensor[i][j], 0)); 
                      digitalWrite(mux8_1, bitRead(dirSensor[i][j], 1));
                      digitalWrite(mux8_2, bitRead(dirSensor[i][j], 2));

                      digitalWrite(mux16_0, bitRead(dirSensor[i][j], 4));
                      digitalWrite(mux16_1, bitRead(dirSensor[i][j], 5));
                      digitalWrite(mux16_2, bitRead(dirSensor[i][j], 6));


                      delayMicroseconds(timeBsensors); // Este delay es encesario para que le tiempo de switchear las compuertas antes de leer los datos, en caso de que no existiera, leeriamos ruido. 
                      
                      //Podemos ver que podemos leer 4 datos en paralelo por cada uno de los puertos de entrada al ESP32.
                      muxValues[1][i][j] = (digitalRead(mux16Out_1));
                      muxValues[2][i][j] = (digitalRead(mux16Out_2));
                      muxValues[3][i][j] = (digitalRead(mux16Out_3));
                      muxValues[4][i][j] = (digitalRead(mux16Out_4));

        }
    }
}

int detectChange()
{
    bool changeFlag = LOW;
    int colocaPieza = 0;
    int levantaPieza = 0;
    int iAnterior = -1;
    int jAnterior = -1;
    int iActual = -1;
    int jActual = -1;

    //detectChessBoard();
    //Serial.println("Tablero chessAnterior");

    for (int j = 0; j < 10; j++)
    {
        for (int i = 0; i < 10; i++)
        {
            chessAnterior[i][j] = matrizBin[i][j];
        }
    }
    TiempoAhora = millis();
    while (changeFlag != HIGH)
    {
        detectChessBoard();
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                if (chessAnterior[i][j] != matrizBin[i][j])
                {
                    changeFlag = HIGH;
                    if (matrizBin[i][j] == false)
                    {
                        /*
                        Serial.println("Se coloco Pieza");
                        Serial.println("Coordenada x");
                        Serial.println(i);
                        Serial.println("Coordenada y");
                        Serial.println(j);
                        */
                        finChangeX = i;
                        finChangeY = j;
                        iActual = i;
                        jActual = j;
                        colocaPieza = 1;
                        //testVibrador();
                    }
                    if (matrizBin[i][j] == true)
                    {
                        /*
                        Serial.println("Se levanto Pieza");
                        Serial.println("Coordenada x");
                        Serial.println(i);
                        Serial.println("Coordenada y");
                        Serial.println(j);
                        */
                        iniChangeX = i;
                        iniChangeY = j;
                        iActual = i;
                        jActual = j;
                        colocaPieza = 0;
                    }
                }
            }
        }

        if (millis() < (TiempoAhora + periodo))
        {

            if (iActual != iAnterior || jActual != jAnterior)
            {
                TiempoAhora = millis();
            }
            changeFlag = LOW;
        }
        iAnterior = iActual;
        jAnterior = jActual;
    }
    for (int j = 0; j < 10; j++)
    {
        for (int i = 0; i < 10; i++)
        {
            chessAnterior[i][j] = matrizBin[i][j];
        }
    }
    if (colocaPieza == 1)
    {
        //La funcion de pureba de vibrador se debe agregar aqui
        //testVibrador();
        return 1;
    }
    if (colocaPieza == 0)
    {
        return 0;
    }
}

String upgradeChessboardFromSensors(bool* variableColor)
{
    char coordIniX = 'v';
    char coordIniY = 'v';
    char coordFinX = 'v';
    char coordFinY = 'v';
    char piezaEnMovimiento = 'v';
    bool colorPiezaActual = false;
    String comandoJugada = "";
    char arrayComandoJugada[9];
   
    char CoordBoardX[8] = {'a','b','c','d','e','f','g','h'};
    char CoordBoardY[8] = {'8','7','6','5','4','3','2','1'};
    if(contAnteriorPiezasMuertas == contSensoresPiezasMuertas)
    {
        comioPieza = false;   //No hay piezas comidas en el turno
    }
    else
    {
        comioPieza = true;
    }
    contAnteriorPiezasMuertas = contSensoresPiezasMuertas;
    if(iniChangeX > 0 && iniChangeX < 9)
    {
        coordIniX = CoordBoardX[iniChangeX-1];
    }
    if(iniChangeY > 0 && iniChangeY < 9)
    {
        coordIniY = CoordBoardY[iniChangeY-1];
    }
    if(finChangeX > 0 && finChangeX < 9)
    {
        coordFinX = CoordBoardX[finChangeX-1];
    }
    if(finChangeY > 0 && finChangeY < 9)
    {
        coordFinY = CoordBoardY[finChangeY-1];
    }
    if(comioPieza == false )
    {
        piezaEnMovimiento = consultaPiezaTablero(coordIniX, coordIniY);
    }
    if(piezaEnMovimiento >= 'a' && piezaEnMovimiento <= 'z')
    {
        //Piezas Negras
        colorPiezaActual = false;
    }
    if(piezaEnMovimiento >= 'A' && piezaEnMovimiento <= 'Z')
    {
        //Piezas Blancas
        colorPiezaActual = true;
    }
    if(comioPieza == false )
    {
        upgradeMatriz(coordIniX, coordIniY, coordFinX, coordFinY,0,colorPiezaActual,piezaEnMovimiento); //Actualiza la matriz virtual basado solo en la lectura 
        
        #ifdef debug                                                                                                //de los sensores y el mapeo de piezas virtuales
        Serial.println("Actual Piece");
        Serial.println(piezaEnMovimiento);
        #endif
        (*variableColor) = colorPiezaActual;
        //=====================================Seccion para generar el comando del movimento que se envia a Arena ==============================
        arrayComandoJugada[0] = 'M';
        arrayComandoJugada[1] = ' ';
        arrayComandoJugada[2] = '1';
        arrayComandoJugada[3] = ' ';
        arrayComandoJugada[4] = coordIniX;
        arrayComandoJugada[5] = coordIniY;

        if(bandAnteriorComio == 0)
        {
            arrayComandoJugada[6] = '-';
        }
        else
        {
            arrayComandoJugada[6] = 'x';
            bandAnteriorComio = 0;
        }
        arrayComandoJugada[7] = coordFinX;
        arrayComandoJugada[8] = coordFinY;
        for(int k = 0; k < 9; k++)
        {
            comandoJugada = comandoJugada + arrayComandoJugada[k];
        }
        //=========================================================================================================================================

        //Seccion para verificar si la pieza que se mueve es un rey y si se mueve mas de dos lugares 
        if(piezaEnMovimiento == 'K' || piezaEnMovimiento == 'k')
        {
            if(colorPiezaActual == true)                    //Estan moviendo piezas blancas
            {
                if(coordIniX == 'e' && coordFinX == 'g')    //Enroque corto
                {
                    specialMove = 10;                       //Para identificar que es un enroque en proceso
                }
                if(coordIniX == 'e' && coordFinX == 'c')    //Enroque largo
                {
                    specialMove = 11;                       //Para identificar que es un enroque en proceso
                }
                kingW = 1;                                  //Desactiva la bandera de enroque para las piezas blancas  -> Para formato FEN
            }

            if(colorPiezaActual == false)                   //Estan moviendo piezas negras
            {
                if(coordIniX == 'e' && coordFinX == 'g')    //Enroque corto
                {
                    specialMove = 12;                       //Para identificar que es un enroque en proceso
                }
                if(coordIniX == 'e' && coordFinX == 'c')    //Enroque largo
                {
                    specialMove = 13;                       //Para identificar que es un enroque en proceso
                }
                kingB = 1;                                  //Desactiva la bandera de enroque para las piezas blancas  -> Para formato FEN
            }
        }
        //Se verifica si se movio una torre e indica de que lado ya no es posible realizar un enroque
        if(piezaEnMovimiento == 'R' || piezaEnMovimiento == 'r')
        {
            //===========Condiciones de primer movimiento de las torres para posibilidad de enroque===============
        if (colorPiezaActual == true)       //Condicion para torres Blancas
        {
            if (coordIniX == 'a')           //Torre lado Reina
            {
                if (rookQueenW == 0)
                {
                    rookQueenW = 1;
                }
            }
            if (coordIniX == 'h')           //Torre lado Rey
            {
                if (rookKingW == 0)
                {
                    rookKingW = 1;
                }
            }
        }
        if (colorPiezaActual == false)      //Condicion para torres Negras
        {
            if (coordIniX == 'a')           //Torre lado Reina
            {
                if (rookQueenB == 0)
                {
                    rookQueenB = 1;
                }
            }
            if (coordIniX == 'h')           //Torre lado Rey
            {
                if (rookKingB == 0)
                {
                    rookKingB = 1;
                }
            }
        }
        //===============================================================================================


        }
    }
    if(comioPieza == true)
    {
        matriz[iniChangeX-1][iniChangeY-1] = '.';
        comandoJugada = "";
        totalPiezasMuertas++;
        bandAnteriorComio = 1;
    }
    return comandoJugada;
}

int contChessSensors()    //Solo se contemplan las 32 piezas del juego, falta considerar piezas extras para coronacion
{
    int totalPiezas = 0;
    contSensoresPiezas = 0;
    contSensoresPiezasMuertas = 0;

    for (int j = 0; j < 10; j++)
    {
        for (int i = 0; i < 10; i++)
        {
            if ((i > 0 && i < 9) && (j > 0 && j < 9))
            {
                if (matrizBin[i][j] == 0)
                {
                    contSensoresPiezas++;
                }
            }
            else
            {
                if (matrizBin[i][j] == 0)
                {
                    contSensoresPiezasMuertas++;
                }
            }
        }
    }
    totalPiezas = contSensoresPiezas + contSensoresPiezasMuertas;
    if(totalPiezas == 32)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/*=========================================== Funciones para Maquina de Estados ===========================================*/

// Acciones de los estados y condiciones de transiciones
void stateReadMode()  //readMode
{
  if (mode == 1)
    changeState(idSculptureMode);
  if (mode == 2)
    changeState(idPlayMode);
}

void stateSculptureMode()   //sculptureMode
{
    changeState(idReadFileCommand);
}

void statePlayMode()   //playMode
{
    changeState(idReceiveInitialFen);                                                                      
}

void stateReadFileCommand()   //readFileCommand
{
  if (fileCommand == 4)
    changeState(idDeleteFileChess);
  if (fileCommand == 3)
    changeState(idWriteFileChess);
  if (fileCommand == 2)
    changeState(idPlayNextFile);
  if (fileCommand == 1)
    changeState(idEndSculpture);
}

void stateReadMovementFile()   //readMovementFile
{
  if (receiveInstruction == 1)
    changeState(idDecodeInstruction);
}

void stateDecodeInstruction()   //decodeInstruccion
{
  if (checkInstruction == 1)
    changeState(idMechanicMovement);
  if (checkInstruction == 0)
    changeState(idReorderAutomatic);
}

void stateMechanicMovement()   //mechanicMovement
{
  if (checkInstruction == 1)
    changeState(idCheckPosition);
}

void stateCheckPosition()   //checkPosition
{
  if (correctPosition == 1)
    changeState(idReadMovementFile);
  if (correctPosition == 0)
    changeState(idDecodeInstruction);
}

void stateReorderAutomatic()   //reorderAutomatic
{
  if (intermediateCalibration == 1)
    changeState(idIntermediateCalib);
  if (intermediateCalibration == 0)
    changeState(idEndGame);
}

void stateIntermediateCalib()   //intermediateCalib
{
    changeState(idEndGame);
}

void stateEndGame()   //endGame
{
  if (mode == 1  && fileCommand == 2)
    changeState(idReadFileCommand);
  if (mode == 2 || (mode == 1  && fileCommand == 1))
    changeState(idReadMode);
}

void stateDeleteFileChess()   //deleteFileChess
{
    changeState(idReadFileCommand);
}

void stateWriteFileChess()   //writeFileChess
{
    changeState(idReadFileCommand);
}

void statePlayNextFile()   //playNextFile
{
    if (fileCommand == 2)
        changeState(idReadMovementFile);
}

void stateEndSculpture()   //endSculpture
{
    if (fileCommand == 1)
        changeState(idReorderAutomatic);
}

void stateReceiveInitialFen()   //receiveInitialFen              
{
    if (initialFen == 1)
      changeState(idUpgradeInitialFen);
    if (initialFen == 2)
      changeState(idInitBoardFromFen);
}

void stateUpgradeInitialFen()   //upgradeInitialFen
{
      changeState(idInitBoardFromFen);
}

void stateInitBoardFromFen()   //initBoardFromFen
{
      changeState(idCheckTurn);
}

void stateCheckTurn()   //checkTurn
{
    if (actualTurn == 1)
      changeState(idPlayerMoveChess);
    if (actualTurn == 2)
      changeState(idReceiveNewFen);
}

void statePlayerMoveChess()   //playerMoveChess
{
    changeState(idDetectMove);
}

void stateReceiveNewFen()   //receiveNewFen
{
    changeState(idMechanicMoveChess);
}

void stateMechanicMoveChess()   //mechanicMomevent
{
    changeState(idDetectMove);
}

void stateDetectMove()   //detectMove
{
    if (actualTurn == 1 && sensorChange == 1)
      changeState(idSendNewFen);
    if (actualTurn == 2 && sensorChange == 1)
      changeState(idCompareEngineMechanism);
    if (sensorChange == 2)
      changeState(idDetectMove);
    if (actualTurn == 1 && sensorChange == 0)
      changeState(idPlayerMoveChess);
    if (actualTurn == 2 && sensorChange == 0)
      changeState(idMechanicMoveChess);    
}

void stateSendNewFen()   //sendNewFen
{
    changeState(idCheckMovement);
}

void stateCheckMovement()   //checkMovement
{
    if (correctMovement == 1)
      changeState(idCompareEngineMechanism);
    if (correctMovement == 2)
      changeState(idReturnPieceToPreviousPosition);
}

void stateReturnPieceToPreviousPosition()   //returnPieceToPrevious
{
      changeState(idCheckTurn);
}

void stateCompareEngineMechanism()   //compareEngineMechanism
{
    if (resultEngineMec == 1)
      changeState(idUpdateFen);
    if (resultEngineMec == 2)
      changeState(idCheckTurn);
}

void stateUpdateFen()   //updateFen
{
    changeState(idCheckMate);
}

void stateCheckMate()   //checkMate
{
    if (resultCheckMate == 1)
      changeState(idCheckTurn);
    if (resultCheckMate == 2)
      changeState(idReorderAutomatic);
}

//STATE MACHINE FUNCTIONS

/*================================================================================================*/
//State "readMode"
//Output:  mode = 1  -> (State: SculptureMode)
//         mode = 2  -> (State: PlayMode)
void readMode()
{
    String newData = "";
    rookKingW = 0;
    rookQueenW = 0;
    rookKingB = 0;
    rookQueenB = 0;
    kingW = 0;
    kingB = 0;
    #ifdef debug
    Serial.println("Initialize Virtual Board");
    #endif
    initializeMatriz();
    printMatriz();
    resultCastlingFEN = castlingFEN(rookKingW,rookQueenW,rookKingB,rookQueenB,kingW,kingB);
    updateFenFromMatriz(false,resultCastlingFEN,resultCapturaAlPasoFen);

    #ifdef testWorkSpace
    #ifdef funcMoveTo
    Robot.moveToPointV2(225,225);
    Robot.moveToPointV2(-225,225);
    Robot.moveToPointV2(-225,-225);
    Robot.moveToPointV2(225,-225);
    Robot.moveToPointV2(225,225);
    Robot.moveToPointV2(0,0);
    #endif

    #ifdef funcAccelRamp


    #ifdef version1Electro
    Robot.accelRamp(225,225,2);
    
    Robot.accelRamp(-225,225,2);
    
    Robot.accelRamp(-225,-225,2);
    
    Robot.accelRamp(225,-225,2);
    
    Robot.accelRamp(225,225,2);

    Robot.accelRamp(0,0,2);
    #endif


    #ifdef version4Electro

    Robot.accelRamp(-175,-175,2,0,1);
    
    Robot.accelRamp(-175,-225,2,0,3);

    Robot.accelRamp(175,-225,2,0,3);


    Robot.accelRamp(175,-175,2,0,1);

    Robot.accelRamp(225,-175,2,0,2);
    
    Robot.accelRamp(225,175,2,0,2);


    Robot.accelRamp(175,175,2,0,1);
    
    Robot.accelRamp(175,225,2,0,1);

    Robot.accelRamp(-175,225,2,0,1);

    
    Robot.accelRamp(-175,175,2,0,1);

    Robot.accelRamp(-225,175,2,0,4);
    
    Robot.accelRamp(-225,-175,2,0,4);



    
    

    #endif
    
    #endif

    #endif


    #ifdef testPiecesAndCore
    #ifdef funcMoveTo
    Robot.moveToPointV2(175,175);
    Robot.moveToPointV2(125,175);
    Robot.moveToPointV2(75,175);
    Robot.moveToPointV2(25,175);
    Robot.moveToPointV2(-25,175);
    Robot.moveToPointV2(-75,175);
    Robot.moveToPointV2(-125,175);
    Robot.moveToPointV2(-175,175);

    Robot.moveToPointV2(-175,125);
    Robot.moveToPointV2(-125,125);
    Robot.moveToPointV2(-75,125);
    Robot.moveToPointV2(-25,125);
    Robot.moveToPointV2(25,125);
    Robot.moveToPointV2(75,125);
    Robot.moveToPointV2(125,125);
    Robot.moveToPointV2(175,125);

    Robot.moveToPointV2(175,-125);
    Robot.moveToPointV2(125,-125);
    Robot.moveToPointV2(75,-125);
    Robot.moveToPointV2(25,-125);
    Robot.moveToPointV2(-25,-125);
    Robot.moveToPointV2(-75,-125);
    Robot.moveToPointV2(-125,-125);
    Robot.moveToPointV2(-175,-125);

    Robot.moveToPointV2(-175,-175);
    Robot.moveToPointV2(-125,-175);
    Robot.moveToPointV2(-75,-175);
    Robot.moveToPointV2(-25,-175);
    Robot.moveToPointV2(25,-175);
    Robot.moveToPointV2(75,-175);
    Robot.moveToPointV2(125,-175);
    Robot.moveToPointV2(175,-175);
    #endif

    #ifdef funcAccelRamp
    Robot.accelRamp(175,175,2);
    Robot.accelRamp(125,175,2);
    Robot.accelRamp(75,175,2);
    Robot.accelRamp(25,175,2);
    Robot.accelRamp(-25,175,2);
    Robot.accelRamp(-75,175,2);
    Robot.accelRamp(-125,175,2);
    Robot.accelRamp(-175,175,2);

    Robot.accelRamp(-175,125,2);
    Robot.accelRamp(-125,125,2);
    Robot.accelRamp(-75,125,2);
    Robot.accelRamp(-25,125,2);
    Robot.accelRamp(25,125,2);
    Robot.accelRamp(75,125,2);
    Robot.accelRamp(125,125,2);
    Robot.accelRamp(175,125,2);

    Robot.accelRamp(175,-125,2);
    Robot.accelRamp(125,-125,2);
    Robot.accelRamp(75,-125,2);
    Robot.accelRamp(25,-125,2);
    Robot.accelRamp(-25,-125,2);
    Robot.accelRamp(-75,-125,2);
    Robot.accelRamp(-125,-125,2);
    Robot.accelRamp(-175,-125,2);

    Robot.accelRamp(-175,-175,2);
    Robot.accelRamp(-125,-175,2);
    Robot.accelRamp(-75,-175,2);
    Robot.accelRamp(-25,-175,2);
    Robot.accelRamp(25,-175,2);
    Robot.accelRamp(75,-175,2);
    Robot.accelRamp(125,-175,2);
    Robot.accelRamp(175,-175,2);
    #endif

    #endif

    #ifdef centrarPiezas
    centrarPiezasIni();
    #endif

    contGames++;
	indexB = 0;
	indexN = 0;
	xB = 4.0;
	yB = 3.5;
	xN = -4.0;
	yN = -3.5;
	xB2 = 4.5;
	xN2 = -4.5;

    specialMove = 0;
    intermediateCalibration = 1;
    mode = 0;
    fileCommand = 0;
    numMovement = 1;
    #ifdef debug
    Serial.println("Run function readMode");
    #endif
    mode = 0;
    while (mode == 0)
    {
        #ifdef SelectModeBluetooth
        newData = BT.getModeChess();
        #endif
        
        #ifdef automaticInitSculpture
        newData = readCommandInput();
        newData = "1";
        #endif

        #ifdef automaticInitPlayMode
        newData = readCommandInput();
        #endif


        if (newData == "1")
        {
            #ifdef debug
            Serial.println("State B");
            #endif
            mode = 1;
        }
        if (newData == "2")
        {
            #ifdef debug
            Serial.println("State C");
            #endif
            mode = 2;
        }
    }
}
/*================================================================================================*/
//State "sculptureMode"
//Output -> (State: readFileCommand)
void sculptureMode()
{
    #ifdef debug
    Serial.println("Run function sculptureMode");
    #endif
}
/*================================================================================================*/
//State "playMode"
//Output -> (State: receiveInitialFen)
void playMode()
{
    #ifdef debug
    Serial.println("Run function playMode");
    #endif
    Serial.println("New Game");
}
/*================================================================================================*/
//State "readFileCommand"
//Output fileCommand = 1 -> (State: endSculpture)
//       fileCommand = 2 -> (State: playNextFile)
//       fileCommand = 3 -> (State: newFile)
//       fileCommand = 4 -> (State: deleteFile)
void readFileCommand()
{
  String newData = "";
  #ifdef debug
  Serial.println("Run function readFileCommand");
  #endif
  delay(1000);  //Para que inicie correctamente spiffs
  fileCommand = 0;
  while (fileCommand == 0)
  {
      newData = readCommandInput();

      #ifdef SelectModeBluetooth
      newData = BT.getModeChess();
      if(newData == "1")            //Esto es porque desde bluetooth leo solo un valor de configuracion del modo
      {                             //Esto es temporal solo para probar la funcionalidad de bluetooth de elegir un modo
        newData = "2";
      }
      #endif

      #ifdef automaticInitSculpture     //Lo mencionado anteriormente no ocurria con el define  por la siguiente linea
      newData = "2";
      #endif
      if (newData == "1")
      {
          Serial.println("State P");
          fileCommand = 1;
      }
      if (newData == "2")
      {
          Serial.println("State N");
          fileCommand = 2;
      }
      if (newData == "3")
      {
          Serial.println("State M");
          fileCommand = 3;
      }
      if (newData == "4")
      {
          Serial.println("State L");
          fileCommand = 4;
      }
  }
}
/*================================================================================================*/
//State "readMovementFile"
//Output receiveInstruction = 1 -> (State: decodeInstruccion)
void readMovementFile()
{
    bool result;
  listFilesChessTest(SPIFFS,"/");
  #ifdef debug
  Serial.println("Run function readMovementFile");
  Serial.println("function readMovementFile");
  Serial.println("dirName");
  #endif
  Serial.print(dirName);
  if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    
    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    result  = readFromFile(SPIFFS,numMovement);
    receiveInstruction = 1;
}
/*================================================================================================*/
//State "decodeInstruccion"
//Output checkInstruction = 1 -> (State: mechanicMovement)
//       checkInstruction = 0 -> (State: reorderAutomatic)
/**================================================================================================
 **                                      Funcion decodeInstruction
 *?  Descompone el comando LAN para obtener la informacion necesaria para realizar la jugada (coordenada inicial y final del movimiento,pieza,movimiento)
 *@param name type  
 *@return void
 *================================================================================================**/
void decodeInstruction()
{
    #ifdef debug
    Serial.println("Run function decodeInstruction");
    #endif
    char char_x_ini, char_y_ini, char_x_fin, char_y_fin, char_pieza;
    int int_accion = 0;
    char *Ap_y_ini = &char_y_ini;
    char *Ap_x_ini = &char_x_ini;
    char *Ap_y_fin = &char_y_fin;
    char *Ap_x_fin = &char_x_fin;
    char *Ap_pieza = &char_pieza;
    int *Ap_accion = &int_accion;
    //bool color_chess = false;
    char char_actual = 'v';
    int index_mov = 0;
    char mov_chess[7];

    int n = movementFileGlobal.length();
    char char_array[n + 1];
    
    strcpy(char_array, movementFileGlobal.c_str());

    for(int i = 0; i < n; i++)
    {
        mov_chess[i] = char_array[i];
    }
    //Determina si se ha terminado la reproduccion de la partida al encontrar la instrucion del resultado
    if(mov_chess[0] == '1' || mov_chess[0] == '0')
    {
        bandEndGame = true;
        checkInstruction = 0;
    }
    else
    {
        bandEndGame = false;
        checkInstruction = 1;
    }
    //===================================================================================================
    Serial.println(" ");

        cont_m++;
        colorChessGlobal = !colorChessGlobal;
        infoChessMovement(mov_chess, Ap_x_ini, Ap_y_ini, Ap_x_fin, Ap_y_fin, Ap_pieza, Ap_accion);
        decodeInfoM.charXIniDecode = char_x_ini;
        decodeInfoM.charYIniDecode = char_y_ini;
        decodeInfoM.charXFinDecode = char_x_fin;
        decodeInfoM.charYFinDecode = char_y_fin;
        decodeInfoM.charPiezaDecode = char_pieza;
        decodeInfoM.intAccionDecode = int_accion;

        for (int i = 0; i < 7; i++)
        {
            mov_chess[i] = 'v';
        }
        Serial.println("Play");
        Serial.println(colorChessGlobal);
        Serial.println(int_accion);
        switch (int_accion)
        {
        case 0:
            Serial.println(" ");
            break;
        case 1:
            Serial.println("Only Move");
            break;
        case 2:
            Serial.println("Move and Promotion");
            break;
        case 3:
            Serial.println("Move and Eat");
            break;
        case 4:
            Serial.println("Move and Check");
            break;
        case 5:
            Serial.println("Move, Promotion and Check");
            break;
        case 6:
            Serial.println("Move, Eat and Check");
            break;
        case 7:
            Serial.println("Short Castling");
            break;
        case 8:
            Serial.println("Long Castling");
            break;
        default:
            Serial.println("Invalid Movement");
            break;
        }

        switch (char_pieza)
        {
        case 'K':
            Serial.println("King");
            //chess_King(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);

            //===========Condiciones de primer movimiento del rey para posibilidad de enroque===============
            if (colorChessGlobal == true)
            {
                if (kingW == 0)
                {
                    kingW = 1;
                }
            }
            if (colorChessGlobal == false)
            {
                if (kingB == 0)
                {
                    kingB = 1;
                }
            }
            //===============================================================================================

            break;
        case 'Q':
            Serial.println("Queen");
            //chess_Queen(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);
            break;
        case 'R':
            Serial.println("Rook");
            //chess_Rook(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);

            //===========Condiciones de primer movimiento de las torres para posibilidad de enroque===============
            if (colorChessGlobal == true)
            {
                if (char_x_ini == 'a')
                {
                    if (rookQueenW == 0)
                    {
                        rookQueenW = 1;
                    }
                }
                if (char_x_ini == 'h')
                {
                    if (rookKingW == 0)
                    {
                        rookKingW = 1;
                    }
                }
            }
            if (colorChessGlobal == false)
            {
                if (char_x_ini == 'a')
                {
                    if (rookQueenB == 0)
                    {
                        rookQueenB = 1;
                    }
                }
                if (char_x_ini == 'h')
                {
                    if (rookKingB == 0)
                    {
                        rookKingB = 1;
                    }
                }
            }
            //===============================================================================================
            break;
        case 'B':
            Serial.println("Bishop");
            //chess_Bishop(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);
            break;
        case 'N':
            Serial.println("Knight");
            //chess_Knight(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);
            break;
        case 'P':
            if (int_accion != 0)
            {
                Serial.println("Pawn Mueve");
                //chess_Pawn(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);
                resultCapturaAlPasoFen = capturaAlPasoFen(char_x_fin, char_y_fin, colorChessGlobal);
            }
            break;
        case 'S':
            //enroque_corto(colorChessGlobal);

            //=======Condiciones para desactivar posibilidad de enroque en formato FEN=======
            if (colorChessGlobal == true)
            {
                kingW = 1;
            }
            if (colorChessGlobal == false)
            {
                kingB = 1;
            }
            //===============================================================================

            break;
        case 'L':
            //enroque_largo(colorChessGlobal);

            //=======Condiciones para desactivar posibilidad de enroque en formato FEN=======
            if (colorChessGlobal == true)
            {
                kingW = 1;
            }
            if (colorChessGlobal == false)
            {
                kingB = 1;
            }
            //===============================================================================
            break;
        default:
            Serial.println("Pieza no valida");
            break;
        }
        Serial.print(char_x_ini);
        Serial.println(char_y_ini);
        Serial.print(char_x_fin);
        Serial.println(char_y_fin);

        //Serial.print("Numero de movimientos: ");
        //Serial.println(cont_m);
        /*
        if (int_accion != 0)
        {
            upgradeMatriz(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);
            printMatriz();
            imprimirVectPiecesD();
            resultCastlingFEN = castlingFEN(rookKingW, rookQueenW, rookKingB, rookQueenB, kingW, kingB);
            updateFenFromMatriz(colorChessGlobal, resultCastlingFEN, resultCapturaAlPasoFen);
        }
        */
        resultCapturaAlPasoFen = "-";
        int_accion = 0;
    //Serial.println("Termino de leer Archivo");
}
/*================================================================================================*/
//State "mechanicMovement"
//Output checkInstruction = 1 -> (State: checkPosition)
/**================================================================================================
 **                                      Funcion mechanicMovement
 *?  Esta función realiza el movimiento de la pieza sobre el tablero de juego, segun las coordenadas iniciales y finales del movimiento;
 *@param name type  
 *@return void
 *================================================================================================**/
void mechanicMovement()
{
    sensorsDir();
    detectChessBoard();
    int resultFinalPos;

    #ifdef debug
    Serial.println("Run function mechanicMovement");
    #endif
    char char_x_ini, char_y_ini, char_x_fin, char_y_fin, char_pieza;
    int int_accion = 0;

    char_x_ini = decodeInfoM.charXIniDecode;
    char_y_ini = decodeInfoM.charYIniDecode;
    char_x_fin = decodeInfoM.charXFinDecode;
    char_y_fin = decodeInfoM.charYFinDecode;
    char_pieza = decodeInfoM.charPiezaDecode;
    int_accion = decodeInfoM.intAccionDecode;
    
    
        switch (char_pieza)
        {
        case 'K':
            //Serial.println("King");
            chess_King(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);

            #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
                detectChessBoardVerif();
                resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                while(resultFinalPos == 0)
                {
                    //Dentro del ciclo while, se cambia el argumento int_accion por "1" en la funcion chess_King
                    //Esto para que solo se repita el movimiento de ocupar el lugar y no el de comer pieza
                    chess_King(char_x_ini, char_y_ini, char_x_fin, char_y_fin, 1, colorChessGlobal, char_pieza);
                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                }
                #endif

            break;
        case 'Q':
            //Serial.println("Queen");
            chess_Queen(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);

            #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
            
                detectChessBoardVerif();
                resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                while(resultFinalPos == 0)
                {
                    //Dentro del ciclo while, se cambia el argumento int_accion por "1" en la funcion chess_Queen
                    //Esto para que solo se repita el movimiento de ocupar el lugar y no el de comer pieza
                    chess_Queen(char_x_ini, char_y_ini, char_x_fin, char_y_fin, 1, colorChessGlobal, char_pieza);
                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                }
                #endif

            break;
        case 'R':
            //Serial.println("Rook");
            chess_Rook(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);
            
            #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
                
                detectChessBoardVerif();
                resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                while(resultFinalPos == 0)
                {
                    //Dentro del ciclo while, se cambia el argumento int_accion por "1" en la funcion chess_Rook
                    //Esto para que solo se repita el movimiento de ocupar el lugar y no el de comer pieza
                    chess_Rook(char_x_ini, char_y_ini, char_x_fin, char_y_fin, 1, colorChessGlobal, char_pieza);
                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                }
                #endif

            break;
        case 'B':
            //Serial.println("Bishop");
            chess_Bishop(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);

            #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
                
                detectChessBoardVerif();
                resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                while(resultFinalPos == 0)
                {
                    //Dentro del ciclo while, se cambia el argumento int_accion por "1" en la funcion chess_Bishop
                    //Esto para que solo se repita el movimiento de ocupar el lugar y no el de comer pieza
                    chess_Bishop(char_x_ini, char_y_ini, char_x_fin, char_y_fin, 1, colorChessGlobal, char_pieza);
                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                }
                #endif

            break;
        case 'N':
            //Serial.println("Knight");
            chess_Knight(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);

            #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
                
                detectChessBoardVerif();
                resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                while(resultFinalPos == 0)
                {
                    //Dentro del ciclo while, se cambia el argumento int_accion por "1" en la funcion chess_Knight
                    //Esto para que solo se repita el movimiento de ocupar el lugar y no el de comer pieza
                    chess_Knight(char_x_ini, char_y_ini, char_x_fin, char_y_fin, 1, colorChessGlobal, char_pieza);
                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                }
                #endif

            break;
        case 'P':
            if (int_accion != 0)
            {
                //Serial.println("Pawn Mueve");
                chess_Pawn(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);
                //resultCapturaAlPasoFen = capturaAlPasoFen(char_x_fin, char_y_fin, colorChessGlobal);

                sensorsDir();
                #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
                int resultFinalPos;
                Serial.println("Matriz fuera de while");
                deactivateElectromagnet();
                detectChessBoardVerif();

                resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                while(resultFinalPos == 0)
                {
                    //Dentro del ciclo while, se cambia el argumento int_accion por "1" en la funcion chess_Pawn
                    //Esto para que solo se repita el movimiento de ocupar el lugar y no el de comer pieza
                    chess_Pawn(char_x_ini, char_y_ini, char_x_fin, char_y_fin, 1, colorChessGlobal, char_pieza);
                    Serial.println("Matriz dentro de while");
                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPosition(char_x_fin,char_y_fin);
                }
                #endif
                Serial.println("Sale del while");


            }
            break;
        case 'S':
            enroque_corto(colorChessGlobal);
            break;
        case 'L':
            enroque_largo(colorChessGlobal);
            break;
        default:
            //Serial.println("Pieza no valida");
            break;
        }

        if (int_accion != 0)
        {
            upgradeMatriz(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, colorChessGlobal, char_pieza);
            printMatriz();
            imprimirVectPiecesD();
            resultCastlingFEN = castlingFEN(rookKingW, rookQueenW, rookKingB, rookQueenB, kingW, kingB);
            updateFenFromMatriz(colorChessGlobal, resultCastlingFEN, resultCapturaAlPasoFen);
        }
        #ifdef activateSensors
        compareVirtualMatrizVsSensors();    //Funcion que compara la matriz virtual del tablero con los sensores
        #endif






}
/*=================================================================================================================*/
//State "checkPosition"
//Output correctPosition = 1 -> (State: readMovementFile)
//       correctPosition = 0 -> (State: decodeInstruction)
void checkPosition()
{
    #ifdef debug
    Serial.println("Run function checkPosition");
    #endif
    correctPosition = 1;
}
/*================================================================================================*/
//State "reorderAutomatic"
//Output intermediateCalibration = 1 -> (State: intermediateCalib)
//       intermediateCalibration = 0 -> (State: endGame)
void reorderAutomatic()
{
    #ifdef debug
    Serial.println("Run function reorderAutomatic");
    #endif
    
    removeChessAuto();      //Descomentar para reordenar
    reorderAuto();          //Descomentar para reordenar
    
   //reorderAutoForzado();
   initializeMatriz();

   //Serial.println("White Chess");
    for (int i = 0; i < 16; i++)
    {
        //Serial.print(vectBlancas[i]);
        //Serial.print(" ");
        vectBlancas[i] = 'v';
    }
    //Serial.println(" ");

    //Serial.println("Black Chess");
    for (int i = 0; i < 16; i++)
    {
        //Serial.print(vectNegras[i]);
        //Serial.print(" ");
        vectNegras[i] = 'v';
    }
    //Serial.println(" ");


//haloCalib.startCalibration();
//===================================================
//BLOQUE PARA LA CALIBRACION ENTRE PROGRAMAS
//---------------------------------------------------
#ifdef calibBetweenGames

#ifdef funcMoveTo
Robot.moveToPointV2(0,0);
#endif

#ifdef funcAccelRamp
Robot.accelRamp(0,0,2,0,1);
#endif

//haloCalib.initCalibration();
haloCalib.startCalibration();

Robot.reInitVariables();

Robot.setSpeedRampFunction(defGlobalSpeed);
Robot.setAccelRampFunction(defGlobalAccel);
Robot.setSpeedMotors(defGlobalSpeed);

#ifdef activateSensors
centrarPiezasAlgortimoConSensores();
#endif

Robot.setSpeedRampFunction(defGlobalSpeed);
Robot.setAccelRampFunction(defGlobalAccel);
Robot.setSpeedMotors(defGlobalSpeed);

#endif

#ifdef centrarPiezas
    centrarPiezasIni();
#endif


//===================================================

  initializeMatriz();
  //centrarPiezasIni();       //<-- Esta linea hace que se centren las piezas entre cada juego
 
 //==========Problema  -> al iniciar nuevamente la lectura de la lista de archivos lleva mal el orden del cementerio
 //==========Solucion  -> Cada vez que termina una partida todas las variables se inicializan
    String newData = "";
    rookKingW = 0;
    rookQueenW = 0;
    rookKingB = 0;
    rookQueenB = 0;
    kingW = 0;
    kingB = 0;
    Serial.println("Initialize Virtual Board al final de la funcion reorder automatic");
    initializeMatriz();
    printMatriz();
    resultCastlingFEN = castlingFEN(rookKingW,rookQueenW,rookKingB,rookQueenB,kingW,kingB);
    updateFenFromMatriz(false,resultCastlingFEN,resultCapturaAlPasoFen);

    colorChessGlobal = false;



}
/*================================================================================================*/
//State "intermediateCalib"
//Output -> (State: endGame)
void intermediateCalib()
{
    #ifdef debug
    Serial.println("Run function intermediateCalib");
    #endif
  //Funcion intermediateCalib     <-------------------------------
}
/*================================================================================================*/
//State "endGame"
//Output mode = 1 &&  fileCommand = 2                   -> (State: readFileCommand)
//       (mode = 2) || (mode = 1 && fileCommand = 1)    -> (State: readMode)
void endGame()
{
    #ifdef debug
    Serial.println("Run function endGame");
    #endif
    #ifdef funcMoveTo
    Robot.moveToPointV2(0,0);
    #endif
    #ifdef funcAccelRamp
    Robot.accelRamp(0,0,2);
    #endif
}
/*================================================================================================*/
//State "deleteFileChess"
//Output -> (State: readFileCommand)
void deleteFileChess()
{
    String newData = "";
    int intData;
    #ifdef debug
    Serial.println("Run function deleteFileChess");
    Serial.println("Input ID File");
    #endif
    Serial.flush();
    while(newData == "")
    {
        newData = readCommandInput();
    }
    
    int n = newData.length();
    char char_array[n + 1]; 
    strcpy(char_array, newData.c_str());
    
    intData = atoi(char_array);
    Serial.println("ID File");
    Serial.println(intData);

    idActualFile = intData;
    IdDeleteFile(SPIFFS,"/");
    idActualFile = 0;
}
/*================================================================================================*/
//State "writeFileChess"
//Output -> (State: readFileCommand)
void writeFileChess()
{
    const char *nameGame;
    const char *contentsGame;
    String newData = "";
    String contentsFile = "";
    #ifdef debug
    Serial.println("Run function writeFileChess");
    Serial.println("Input Name File");
    #endif
    Serial.flush();
    while(newData == "")
    {
        newData = readCommandInput();
    }
    int n = newData.length();
    char char_array[n + 1]; 
    strcpy(char_array, newData.c_str());
    nameGame = char_array;

    Serial.println("Input Contents File");
    Serial.flush();
    while(contentsFile == "")
    {
        contentsFile = readCommandInput();
    }
    int m = contentsFile.length();
    char char_array2[m + 1];
    strcpy(char_array2, contentsFile.c_str());
    contentsGame = char_array2;

    if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS Mount Failed");
        return;
    }
    Serial.println("nameGame");
    Serial.println(nameGame);
    createNewFileChess(SPIFFS, nameGame, contentsGame);
}
/*================================================================================================*/
//State "playNextFile"
//Output fileCommand = 2                       -> (State: readMovementFile)
void playNextFile()
{
    for(int i = 0; i < 16; i++)
    {
        vectBlancas[i] = 'v';
        vectNegras[i] = 'v';
    }
    
    /*
    idActualFile++;
    if(idActualFile == 199)
    {
        idActualFile = 1;

    }
    */
    
    idActualFile = 1;
    #ifdef debug
    Serial.println("Run function playNextFile");
    #endif
    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    
    listFilesChessTest(SPIFFS,"/");
    Serial.print(dirName);

    numMovement = 1;
}
/*================================================================================================*/
//State "endSculpture"
//Output fileCommand = 1                        -> (State: reorderAutomatic)
void endSculpture()
{
    #ifdef debug
    Serial.println("Run function endSculpture");
    #endif
}
/*================================================================================================*/
//State "receiveInitialFen"
//Output initialFen = 1                        -> (State: upgradeInitialFen)
//       initialFen = 2                        -> (State: initBoardFromFen)
void receiveInitialFen()
{
    #ifdef debug
    Serial.println("Run function receiveInitialFen");
    #endif

    String newData = "";
    initialFen = 0;

    #ifdef passReceiveInitialFen
    initialFen = 1;
    #endif

    while (initialFen == 0)
    {
        newData = readCommandInput();
        if (newData == "1")
        {
            initialFen = 1;
        }
        if (newData == "2")
        {
            initialFen = 2;
        }
    }
}
/*================================================================================================*/
//State "upgradeInitialFen"
//Output -> (State: initBoardFromFen)
void upgradeInitialFen()
{
    #ifdef debug
    Serial.println("Run function upgradeInitialFen");
    #endif
    upgradeInitFen(FEN2);
}
/*================================================================================================*/
//State "initBoardFromFeN"
//Output -> (State: checkTurn)
void initBoardFromFen()
{
    #ifdef debug
    Serial.println("Run function initBoardFromFen");
    #endif
    initFromFEN(FEN2);
}
/*================================================================================================*/
//State "checkTurn"
//Output    actualTurn = 1 -> (State: playerMoveChess)
//          actualTurn = 2 -> (State: receiveNewFen)
void checkTurn()
{
    #ifdef debug
    Serial.println("Run function checkTurn");
    #endif
    specialMove = 0;
    /*
    if (colorChessGlobal == true)
    {
        actualTurn = 1;
    }
    else
    {
        actualTurn = 2;
    }
    */
   if(globalTurn == true)
    {
        actualTurn = 1;
    }
    else
    {
        actualTurn = 2;
    }
    globalTurn = !globalTurn;
}
/*================================================================================================*/
//State "playerMoveChess"
//Output    -> (State: detectMove)
void playerMoveChess()
{
    sensorsDir();
    detectChessBoard();
    #ifdef debug
    Serial.println("Run function playerMoveChess");
    #endif
}
/*================================================================================================*/
//State "receiveNewFen"
//Output    -> (State: mechanicMoveChess)
void receiveNewFen()
{
    int receiveNewMovement = 0;
    #ifdef debug
    Serial.println("Run function receiveNewFen");
    #endif
    receiveNewMovement = 0;
    while (receiveNewMovement != 1)
    {
        receiveNewMovement = receiveMovementSerial();
    }
}
/*================================================================================================*/
//State "mechanicMoveChess"
//Output    -> (State: detectMove)
void mechanicMoveChess()
{
    sensorsDir();
    detectChessBoard();
    #ifdef debug
    Serial.println("Run function mechanicMoveChess");
    #endif
    if(decodeInfoM.intAccionDecode == 1)
    {
        realizaJugada(decodeInfoM.charXIniDecode, decodeInfoM.charYIniDecode, decodeInfoM.charXFinDecode, decodeInfoM.charYFinDecode, decodeInfoM.intAccionDecode, false, decodeInfoM.charPiezaDecode);
        #ifdef activateSensors
            #ifdef debug
            Serial.println("En funcion realizaJugada");
            #endif
        int resultFinalPos;
        detectChessBoardVerif();
        resultFinalPos = checkFinalPosition(decodeInfoM.charXFinDecode,decodeInfoM.charYFinDecode);
        while(resultFinalPos == 0)
        {
            realizaJugada(decodeInfoM.charXIniDecode, decodeInfoM.charYIniDecode, decodeInfoM.charXFinDecode, decodeInfoM.charYFinDecode, decodeInfoM.intAccionDecode, false, decodeInfoM.charPiezaDecode);
            detectChessBoardVerif();
            resultFinalPos = checkFinalPosition(decodeInfoM.charXFinDecode,decodeInfoM.charYFinDecode);
        }
        #endif
    }
    if(decodeInfoM.intAccionDecode == 3)
    {
        comerVersion3(decodeInfoM.charXFinDecode, decodeInfoM.charYFinDecode, decodeInfoM.intAccionDecode, colorChessGlobal, decodeInfoM.charPiezaDecode);
        decodeInfoM.intAccionDecode = 1;
        #ifdef activateSensors
        #ifdef debug
        Serial.println("En funcion realizaJugada");
        #endif
        int resultFinalPos;
        detectChessBoardVerif();
        resultFinalPos = checkFinalPositionGraveyard(colorChessGlobal);
        while(resultFinalPos == 0)
        {
            comerVersion3(decodeInfoM.charXFinDecode, decodeInfoM.charYFinDecode, decodeInfoM.intAccionDecode, colorChessGlobal, decodeInfoM.charPiezaDecode);
            decodeInfoM.intAccionDecode = 1;
            detectChessBoardVerif();
            resultFinalPos = checkFinalPositionGraveyard(colorChessGlobal);
        }
        #endif
    }
    if(specialMove == 10)
    {
        //Movimiento de la torre para enroque corto Blancas
        shortCastlingRook(true);
        #ifdef activateSensors
            #ifdef debug
            Serial.println("En funcion realizaJugada");
            #endif
        int resultFinalPos;
        detectChessBoardVerif();
        resultFinalPos = checkFinalPosition('f','1');
        while(resultFinalPos == 0)
        {
            shortCastlingRook(true);
            detectChessBoardVerif();
            resultFinalPos = checkFinalPosition('f','1');
        }
        #endif
    }
    if(specialMove == 11)
    {
        //Movimiento de la torre para enroque largo Blancas
        longCastlingRook(true);
        #ifdef activateSensors
            #ifdef debug
            Serial.println("En funcion realizaJugada");
            #endif
        int resultFinalPos;
        detectChessBoardVerif();
        resultFinalPos = checkFinalPosition('d','1');
        while(resultFinalPos == 0)
        {
            longCastlingRook(true);
            detectChessBoardVerif();
            resultFinalPos = checkFinalPosition('d','1');
        }
        #endif
    }
    if(specialMove == 12)
    {
        //Movimiento de la torre para enroque corto Negras
        shortCastlingRook(false);
        #ifdef activateSensors
            #ifdef debug
            Serial.println("En funcion realizaJugada");
            #endif
        int resultFinalPos;
        detectChessBoardVerif();
        resultFinalPos = checkFinalPosition('f','8');
        while(resultFinalPos == 0)
        {
            shortCastlingRook(false);
            detectChessBoardVerif();
            resultFinalPos = checkFinalPosition('f','8');
        }
        #endif
    }
    if(specialMove == 13)
    {
        //Movimiento de la torre para enroque largo Negras
        longCastlingRook(false);
        #ifdef activateSensors
            #ifdef debug
            Serial.println("En funcion realizaJugada");
            #endif
        int resultFinalPos;
        detectChessBoardVerif();
        resultFinalPos = checkFinalPosition('d','8');
        while(resultFinalPos == 0)
        {
            longCastlingRook(false);
            detectChessBoardVerif();
            resultFinalPos = checkFinalPosition('d','8');
        }
        #endif
    }
    #ifdef debug
    Serial.println("End Mechanic Movement");
    #endif


}
/*================================================================================================*/
//State "detectMove"
//Output    actualTurn = 1 && sensorChange = 1 -> (State: sendNewFen)
//          actualTurn = 2 && sensorChange = 1 -> (State: compareEngineMechanism)
//          sensorChange = 2                   -> (State: detectMove)
//          actualTurn = 1 && sensorChange = 0 -> (State: playerMoveChess)
//          actualTurn = 2 && sensorChange = 0 -> (State: mechanicMoveChess)
void detectMove()
{
    #ifdef debug
    Serial.println("Run function detectMove");
    #endif
    specialMove = 0;
    readSensors3();
}
/*================================================================================================*/
//State "sendNewFen"
//Output    -> (State: checkMovement)
void sendNewFen()
{
    #ifdef debug
    Serial.println("Run function sendNewFen");
    #endif
}
/*================================================================================================*/
//State "checkMovement"
//Output    correctMovement = 1 -> (State: compareEngineMechanism)
//          correctMovement = 2 -> (State: returnPieceToPrevious)
void checkMovement()
{
    #ifdef debug
    Serial.println("Run function checkMovement");
    #endif
    String newData = "";
    correctMovement = 0;

    #ifdef passCheckMovement
    correctMovement = 1;
    #endif


    while (correctMovement == 0)
    {
        newData = readCommandInput();
        if (newData == "1")
        {
            correctMovement = 1;
        }
        if (newData == "2")
        {
            correctMovement = 2;
        }
    }
}
/*================================================================================================*/
//State "returnPieceToPreviousPosition"
//Output    -> (State: checkTurn)
void returnPieceToPreviousPosition()
{
    #ifdef debug
    Serial.println("Run function returnPieceToPreviousPosition");
    #endif
}
/*================================================================================================*/
//State "compareEngineMechanism"
//Output    resultEngineMec = 1 -> (State: updateFen)
//          resultEngineMec = 2 -> (State: checkTurn)
void compareEngineMechanism()
{
    #ifdef debug
    Serial.println("Run function compareEngineMechanism");
    #endif
    String newData = "";
    resultEngineMec = 0;

    #ifdef passCompareEngineMechanism
    resultEngineMec = 1;
    #endif

    while (resultEngineMec == 0)
    {
        newData = readCommandInput();
        if (newData == "1")
        {
            resultEngineMec = 1;
        }
        if (newData == "2")
        {
            resultEngineMec = 2;
        }
    }
    Serial.flush();
}
/*================================================================================================*/
//State "updateFen"
//Output    -> (State: checkMate)
void updateFen()
{
    #ifdef debug
    Serial.println("Run function updateFen");
    #endif
    printMatriz();
    resultCastlingFEN = castlingFEN(rookKingW, rookQueenW, rookKingB, rookQueenB, kingW, kingB);
    updateFenFromMatriz(colorChessGlobal, resultCastlingFEN, resultCapturaAlPasoFen);
    colorChessGlobal = !colorChessGlobal;
}
/*================================================================================================*/
//State "checkMate"
//Output    resultCheckMate = 1 -> (State: checkTurn)
//          resultCheckMate = 2 -> (State: reorderAutomatic)
void checkMate()
{
    #ifdef debug
    Serial.println("Run function checkMate");
    #endif
    String newData = "";
    resultCheckMate = 0;

    #ifdef passCheckMate
    resultCheckMate = 1;
    #endif

    while (resultCheckMate == 0)
    {
        newData = readCommandInput();
        if (newData == "1")
        {
            resultCheckMate = 1;
        }
        if (newData == "2")
        {
            resultCheckMate = 2;
        }
    }
    Serial.flush();
}
/*================================================================================================*/

void initStateMachine()
{
  Serial.begin(115200);
  #ifdef debug
  Serial.println("");
  #endif
  currentState = idReadMode;
  readMode();
}

void runStateMachine()
{
  updateStateMachine();
}

/**================================================================================================
 **                                      Funcion updateStateMchine
 *?  Esta funcion actualiza el estado actual de la máquina de estados
 *@param currentState Global int
 *@return void
 *================================================================================================**/
void updateStateMachine()
{
  switch (currentState)
  {
    case 1: stateReadMode();                        break;
    case 2: stateSculptureMode();                   break;    
    case 3: statePlayMode();                        break;    
    case 4: stateReadFileCommand();                 break;    
    case 5: stateReadMovementFile();                break;    
    case 6: stateDecodeInstruction();               break;    
    case 7: stateMechanicMovement();                break;    
    case 8: stateCheckPosition();                   break;
    case 9: stateReorderAutomatic();                break;    
    case 10: stateIntermediateCalib();              break;    
    case 11: stateEndGame();                        break;    
    case 12: stateDeleteFileChess();                break;
    case 13: stateWriteFileChess();                 break;    
    case 14: statePlayNextFile();                   break;    
    case 15: stateEndSculpture();                   break;
    case 16: stateReceiveInitialFen();              break;  
    case 17: stateUpgradeInitialFen();              break;  
    case 18: stateInitBoardFromFen();               break;  
    case 19: stateCheckTurn();                      break;  
    case 20: statePlayerMoveChess();                break;  
    case 21: stateReceiveNewFen();                  break;  
    case 22: stateMechanicMoveChess();              break;  
    case 23: stateDetectMove();                     break;  
    case 24: stateSendNewFen();                     break;  
    case 25: stateCheckMovement();                  break;  
    case 26: stateReturnPieceToPreviousPosition();  break;  
    case 27: stateCompareEngineMechanism();         break;
    case 28: stateUpdateFen();                      break;
    case 29: stateCheckMate();                      break;      
  }
}

/**================================================================================================
 **                                      Funcion updateStateMchine
 *?  Esta funcion cambia de estado y dispara las transiciones en la máquina de estados
 *@param currentState Global int
 *@return void
 *================================================================================================**/
void changeState(int newState)
{
  currentState = newState;

  switch (currentState)
  {
    case 1: readMode();                         break;
    case 2: sculptureMode();                    break;
    case 3: playMode();                         break;
    case 4: readFileCommand();                  break;
    case 5: readMovementFile();                 break;
    case 6: decodeInstruction();                break;   
    case 7: mechanicMovement();                 break;   
    case 8: checkPosition();                    break;
    case 9: reorderAutomatic();                 break;
    case 10: intermediateCalib();               break;     
    case 11: endGame();                         break;     
    case 12: deleteFileChess();                 break;
    case 13: writeFileChess();                  break;     
    case 14: playNextFile();                    break;     
    case 15: endSculpture();                    break;
    case 16: receiveInitialFen();               break;     
    case 17: upgradeInitialFen();               break;     
    case 18: initBoardFromFen();                break;     
    case 19: checkTurn();                       break;
    case 20: playerMoveChess();                 break;     
    case 21: receiveNewFen();                   break;     
    case 22: mechanicMoveChess();               break;
    case 23: detectMove();                      break;     
    case 24: sendNewFen();                      break;     
    case 25: checkMovement();                   break;
    case 26: returnPieceToPreviousPosition();   break;     
    case 27: compareEngineMechanism();          break;     
    case 28: updateFen();                       break;
    case 29: checkMate();                       break;          
    default:                                    break;
  }
}

//Function to represent the state readMode
String readCommandInput()
{
    String inputData = "";
    if (Serial.available() > 0)
    {
        //inputData = Serial.readString();
        inputData = Serial.readStringUntil('\n');
    }
    //Serial.print(inputData);
    return inputData;
}

/**================================================================================================
 **                                      Funcion listFilesChessTest
 *?  Esta funcion busca busca un archivo en la memoria SPIFFS mediante un indentificador numerico
 *?  definido en la variable "idActualFile"
 *@param dirnName Global int
 *@return void
 *================================================================================================**/
void listFilesChessTest(fs::FS &fs,const char * dirNameFile2){
    File root = fs.open(dirNameFile2);
    if(!root){
        Serial.println("Failed open directory");
        return;
    }
    int contFiles = 0;
    File file = root.openNextFile();
    while(file){
        if(idActualFile == contFiles)
        {
            Serial.print("contFiles:");
            Serial.println(contFiles);
            Serial.print("  File: ");
            dirName = file.name();
            Serial.print(dirName);
            readFileChess(SPIFFS, dirName);
            Serial.print("\tSize: ");
            Serial.println(file.size());
            return;
        }
        else
        {
            file = root.openNextFile();
        }
        contFiles++;
    }
}

void readSensors3()
{
    int resultChangeChess = -1;
    String comandoJugadaR = "";
    sensorChange = 2;
    sensorsDir();
    int resultContPiezas = -1;
    bool colorJugadorActual = 0;
    resultChangeChess = detectChange();
    resultContPiezas = contChessSensors(); //Devuelve 1 si estan todas las piezas 0 si faltan piezas en el tablero
    if (resultContPiezas == 1)
    {
        comandoJugadaR = upgradeChessboardFromSensors(&colorJugadorActual);
        BT.setStatus(comandoJugadaR);     //Test para enviar info por bluetooth
        if(comandoJugadaR != "")
        {
            Serial.println(comandoJugadaR);
        }

        if (comioPieza == false)
        {
            #ifdef debug
            Serial.println("Move piece");
            #endif
            sensorChange = 1;
        }
        else
        {
            #ifdef debug
            Serial.println("Eat piece");
            #endif
            sensorChange = 0;
        }
        if(specialMove == 10 || specialMove == 11 || specialMove == 12 || specialMove == 13)
        {
            #ifdef debug
            Serial.println("Castling movement in progress");
            #endif
            decodeInfoM.intAccionDecode = 0;
            sensorChange = 0;
        }
    }
}

int checkFinalPosition(char finalPosX,char finalPosY)
{
    char vectPosX[8] = {'a','b','c','d','e','f','g','h'};    //Antes
    char vectPosY[8] = {'8','7','6','5','4','3','2','1'};    //Antes
    int axisX = -1;
    int axisY = -1;

    for(int i = 0; i < 8; i++)
    {
        if(finalPosX == vectPosX[i])
        {
            axisX = i + 1;
        }
        if(finalPosY == vectPosY[i])
        {
            axisY = i + 1;
        }
    }
    #ifdef debug
    Serial.println("Final posX , Final posY");
    Serial.print(finalPosX);
    Serial.print(" ");
    Serial.println(finalPosY);
    #endif
/*
    Serial.println("axisX      axisY");
    Serial.print(axisX);
    Serial.print(" ");
    Serial.println(axisY);

    Serial.println("Result matrizBinVerif [axisX][axisY]");
    Serial.println(matrizBinVerif[axisX][axisY]);
*/
    if(matrizBinVerif[axisX][axisY] == false)                //Antes [axisX][axisY]
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void menu()
{
    Serial.println("Menu");
    Serial.println("0) State Machine");
    Serial.println("1) Run Test Curve");
    Serial.println("2) Run function moveToPoint");
    Serial.println("3) Run function activateElectromagnet");
    Serial.println("4) Run function deactivateElectromagnet");

    numdeDatos = readFromSerial();
    while (numdeDatos == 0)
    {
        numdeDatos = readFromSerial();
        if (numdeDatos == 1 && a == 1)
        {
            /*
            //Solo se mueve en una pequena curva
            Robot.moveToPointV2(0, 25);
            Robot.moveToPointV2(3.35, 37.5);
            Robot.moveToPointV2(12.5, 46.65);
            Robot.moveToPointV2(25, 50);
            Robot.moveToPointV2(50, 50);
            */

           //Movimiento del caballo de la posicion 25,25 a lla posicion 75,125
            Robot.moveToPointV2(25, 25);
            Robot.moveToPointV2(37.5, 28.35);
            Robot.moveToPointV2(46.65, 37.5);
            Robot.moveToPointV2(50, 50);
            Robot.moveToPointV2(50, 100);
            Robot.moveToPointV2(53.35, 112.5);
            Robot.moveToPointV2(62.5, 121.65);
            Robot.moveToPointV2(75, 125);


            delay(2000);

            Robot.moveToPointV2(75, 150);
            Robot.moveToPointV2(-25, 150);
            Robot.moveToPointV2(-25, 25);
            Robot.moveToPointV2(25, 25);

            delay(2000);
            /*
            Serial.println("Chess fron file");
            
            //Funcion run Ajedrez
            //=============================================================
            
            //=============================================================
            for(int i = 0; i<30; i++)
            {
                Serial.println("");
            }
            */
        }
        if (numdeDatos == 1 && a == 2)
        {
            Serial.println("Move To Point");
            
            while(a != -1)
            {
                numdeDatos = readFromSerial();
                if(numdeDatos == 2)
                {
                    //Robot.moveToPointV2(a, b);
                    #ifdef funcMoveTo
                    Robot.moveToPointV2(a, b);
                    #endif
                    #ifdef funcAccelRamp
                    Robot.accelRamp(a,b,2,0,1);
                    #endif
                    numdeDatos = 1;
                }
            }
            for(int i = 0; i<30; i++)
            {
                Serial.println("");
            }
        }
        if (numdeDatos == 1 && a == 3)
        {
            Serial.println("Activate Electromagnet");
            
            activateElectromagnet();
            for(int i = 0; i<30; i++)
            {
                Serial.println("");
            }
        }
        if (numdeDatos == 1 && a == 4)
        {
            Serial.println("Deactivate Electromagnet");
            
            deactivateElectromagnet();
            for(int i = 0; i<30; i++)
            {
                Serial.println("");
            }
        }
        if (numdeDatos == 1 && a == 5)
        {
            int contTestMove = 0;
            Serial.println("Test movement");
            
            while(a != -1)
            {
                
            }
            deactivateElectromagnet();
            #ifdef funcMoveTo
            Robot.moveToPointV2(0,0);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(0,0,2);
            #endif
            for(int i = 0; i<30; i++)
            {
                Serial.println("");
            }
        }
    }
}


void IdDeleteFile(fs::FS &fs,const char * dirNameFile2){
    File root = fs.open(dirNameFile2);
    if(!root){
        Serial.println("Failed open directory");
        return;
    }
    int contFiles = 0;
    File file = root.openNextFile();
    while(file){
        if(idActualFile == contFiles)
        {
            Serial.print("contFiles:");
            Serial.println(contFiles);
            Serial.print("Delete  File: ");
            dirName = file.name();
            Serial.print(dirName);
            deleteFile(SPIFFS, dirName);
            Serial.print("\tSize: ");
            Serial.println(file.size());
            return;
        }
        else
        {
            file = root.openNextFile();
        }
        contFiles++;
    }
}

void detectChessBoardVerif()
{
    //Serial.println("Entra a funcion detectChessBoardVerif");
    bool matrizBinTemp[10][10];

    #ifdef version1Electro
    testPWMElectromagnetDeactivate();
    #endif

    #ifdef version4Electro
    testPWMElectromagnetDeactivateV4E(1);
    testPWMElectromagnetDeactivateV4E(2);
    testPWMElectromagnetDeactivateV4E(3);
    testPWMElectromagnetDeactivateV4E(4);
    #endif

    //Serial.println("Desactiva electroiman en funcion detectChessBoardVerif");
    readRawChessBoard();

#ifdef oldSensors
    orderedSensorValues[0][0]=muxValues[3][4][4];
    orderedSensorValues[0][1]=muxValues[3][4][6];
    orderedSensorValues[0][2]=muxValues[3][4][7];
    orderedSensorValues[0][3]=muxValues[3][4][5];
    orderedSensorValues[0][4]=muxValues[3][4][2];
    orderedSensorValues[0][5]=muxValues[3][4][1];
    orderedSensorValues[0][6]=muxValues[3][4][0];
    orderedSensorValues[0][7]=muxValues[3][4][3];
    orderedSensorValues[0][8]=muxValues[3][6][0];
    orderedSensorValues[0][9]=muxValues[3][7][0];

    orderedSensorValues[1][0]=muxValues[3][2][4];
    orderedSensorValues[1][1]=muxValues[3][2][6];
    orderedSensorValues[1][2]=muxValues[3][2][7];
    orderedSensorValues[1][3]=muxValues[3][2][5];
    orderedSensorValues[1][4]=muxValues[3][2][2];
    orderedSensorValues[1][5]=muxValues[3][2][1];
    orderedSensorValues[1][6]=muxValues[3][2][0];
    orderedSensorValues[1][7]=muxValues[3][2][3];
    orderedSensorValues[1][8]=muxValues[3][1][0];
    orderedSensorValues[1][9]=muxValues[3][0][0];

    orderedSensorValues[2][0]=muxValues[4][2][4];
    orderedSensorValues[2][1]=muxValues[4][2][6];
    orderedSensorValues[2][2]=muxValues[4][2][7];
    orderedSensorValues[2][3]=muxValues[4][2][5];
    orderedSensorValues[2][4]=muxValues[4][2][2];
    orderedSensorValues[2][5]=muxValues[4][2][1];
    orderedSensorValues[2][6]=muxValues[4][2][0];
    orderedSensorValues[2][7]=muxValues[4][2][3];
    orderedSensorValues[2][8]=muxValues[3][5][0];
    orderedSensorValues[2][9]=muxValues[3][3][0];

    orderedSensorValues[3][0]=muxValues[4][3][4];
    orderedSensorValues[3][1]=muxValues[4][3][6];
    orderedSensorValues[3][2]=muxValues[4][3][7];
    orderedSensorValues[3][3]=muxValues[4][3][5];
    orderedSensorValues[3][4]=muxValues[4][3][2];
    orderedSensorValues[3][5]=muxValues[4][3][1];
    orderedSensorValues[3][6]=muxValues[4][3][0];
    orderedSensorValues[3][7]=muxValues[4][3][3];
    orderedSensorValues[3][8]=muxValues[4][0][0];
    orderedSensorValues[3][9]=muxValues[4][1][0];

    orderedSensorValues[4][0]=muxValues[4][5][4];
    orderedSensorValues[4][1]=muxValues[4][5][6];
    orderedSensorValues[4][2]=muxValues[4][5][7];
    orderedSensorValues[4][3]=muxValues[4][5][5];
    orderedSensorValues[4][4]=muxValues[4][5][2];
    orderedSensorValues[4][5]=muxValues[4][5][1];
    orderedSensorValues[4][6]=muxValues[4][5][0];
    orderedSensorValues[4][7]=muxValues[4][5][3];
    orderedSensorValues[4][8]=muxValues[4][7][0];
    orderedSensorValues[4][9]=muxValues[4][6][0];

    orderedSensorValues[5][0]=muxValues[2][5][4];
    orderedSensorValues[5][1]=muxValues[2][5][6];
    orderedSensorValues[5][2]=muxValues[2][5][7];
    orderedSensorValues[5][3]=muxValues[2][5][5];
    orderedSensorValues[5][4]=muxValues[2][5][2];
    orderedSensorValues[5][5]=muxValues[2][5][1];
    orderedSensorValues[5][6]=muxValues[2][5][0];
    orderedSensorValues[5][7]=muxValues[2][5][3];
    orderedSensorValues[5][8]=muxValues[2][7][0];
    orderedSensorValues[5][9]=muxValues[2][6][0];

    orderedSensorValues[6][0]=muxValues[2][0][4];
    orderedSensorValues[6][1]=muxValues[2][0][6];
    orderedSensorValues[6][2]=muxValues[2][0][7];
    orderedSensorValues[6][3]=muxValues[2][0][5];
    orderedSensorValues[6][4]=muxValues[2][0][2];
    orderedSensorValues[6][5]=muxValues[2][0][1];
    orderedSensorValues[6][6]=muxValues[2][0][0];
    orderedSensorValues[6][7]=muxValues[2][0][3];
    orderedSensorValues[6][8]=muxValues[2][1][0];
    orderedSensorValues[6][9]=muxValues[2][2][0];

    orderedSensorValues[7][0]=muxValues[1][5][4];
    orderedSensorValues[7][1]=muxValues[1][5][6];
    orderedSensorValues[7][2]=muxValues[1][5][7];
    orderedSensorValues[7][3]=muxValues[1][5][5];
    orderedSensorValues[7][4]=muxValues[1][5][2];
    orderedSensorValues[7][5]=muxValues[1][5][1];
    orderedSensorValues[7][6]=muxValues[1][5][0];
    orderedSensorValues[7][7]=muxValues[1][5][3];
    orderedSensorValues[7][8]=muxValues[1][3][0];
    orderedSensorValues[7][9]=muxValues[2][3][0];

    orderedSensorValues[8][0]=muxValues[1][7][4];
    orderedSensorValues[8][1]=muxValues[1][7][6];
    orderedSensorValues[8][2]=muxValues[1][7][7];
    orderedSensorValues[8][3]=muxValues[1][7][5];
    orderedSensorValues[8][4]=muxValues[1][7][2];
    orderedSensorValues[8][5]=muxValues[1][7][1];
    orderedSensorValues[8][6]=muxValues[1][7][0];
    orderedSensorValues[8][7]=muxValues[1][7][3];
    orderedSensorValues[8][8]=muxValues[1][6][0];
    orderedSensorValues[8][9]=muxValues[1][4][0];

    orderedSensorValues[9][0]=muxValues[1][0][4];
    orderedSensorValues[9][1]=muxValues[1][0][6];
    orderedSensorValues[9][2]=muxValues[1][0][7];
    orderedSensorValues[9][3]=muxValues[1][0][5];
    orderedSensorValues[9][4]=muxValues[1][0][2];
    orderedSensorValues[9][5]=muxValues[1][0][1];
    orderedSensorValues[9][6]=muxValues[1][0][0];
    orderedSensorValues[9][7]=muxValues[1][0][3];
    orderedSensorValues[9][8]=muxValues[1][1][0];
    orderedSensorValues[9][9]=muxValues[1][2][0];
#endif

#ifdef newSensors
    orderedSensorValues[0][0]=muxValues[3][4][4];
    orderedSensorValues[0][1]=muxValues[3][4][6];
    orderedSensorValues[0][2]=muxValues[3][4][7];
    orderedSensorValues[0][3]=muxValues[3][4][5];
    orderedSensorValues[0][4]=muxValues[3][4][2];
    orderedSensorValues[0][5]=muxValues[3][4][1];
    orderedSensorValues[0][6]=muxValues[3][4][0];
    orderedSensorValues[0][7]=muxValues[3][4][3];
    orderedSensorValues[0][8]=muxValues[3][7][0];
    orderedSensorValues[0][9]=muxValues[3][6][0];
    orderedSensorValues[1][0]=muxValues[3][1][4];
    orderedSensorValues[1][1]=muxValues[3][1][6];
    orderedSensorValues[1][2]=muxValues[3][1][7];
    orderedSensorValues[1][3]=muxValues[3][1][5];
    orderedSensorValues[1][4]=muxValues[3][1][2];
    orderedSensorValues[1][5]=muxValues[3][1][1];
    orderedSensorValues[1][6]=muxValues[3][1][0];
    orderedSensorValues[1][7]=muxValues[3][1][3];
    orderedSensorValues[1][8]=muxValues[3][5][0];
    orderedSensorValues[1][9]=muxValues[3][2][0];
    orderedSensorValues[2][0]=muxValues[4][0][4];
    orderedSensorValues[2][1]=muxValues[4][0][6];
    orderedSensorValues[2][2]=muxValues[4][0][7];
    orderedSensorValues[2][3]=muxValues[4][0][5];
    orderedSensorValues[2][4]=muxValues[4][0][2];
    orderedSensorValues[2][5]=muxValues[4][0][1];
    orderedSensorValues[2][6]=muxValues[4][0][0];
    orderedSensorValues[2][7]=muxValues[4][0][3];
    orderedSensorValues[2][8]=muxValues[3][0][0];
    orderedSensorValues[2][9]=muxValues[3][3][0];
    orderedSensorValues[3][0]=muxValues[4][6][4];
    orderedSensorValues[3][1]=muxValues[4][6][6];
    orderedSensorValues[3][2]=muxValues[4][6][7];
    orderedSensorValues[3][3]=muxValues[4][6][5];
    orderedSensorValues[3][4]=muxValues[4][6][2];
    orderedSensorValues[3][5]=muxValues[4][6][1];
    orderedSensorValues[3][6]=muxValues[4][6][0];
    orderedSensorValues[3][7]=muxValues[4][6][3];
    orderedSensorValues[3][8]=muxValues[4][2][0];
    orderedSensorValues[3][9]=muxValues[4][1][0];
    orderedSensorValues[4][0]=muxValues[4][3][4];
    orderedSensorValues[4][1]=muxValues[4][3][6];
    orderedSensorValues[4][2]=muxValues[4][3][7];
    orderedSensorValues[4][3]=muxValues[4][3][5];
    orderedSensorValues[4][4]=muxValues[4][3][2];
    orderedSensorValues[4][5]=muxValues[4][3][1];
    orderedSensorValues[4][6]=muxValues[4][3][0];
    orderedSensorValues[4][7]=muxValues[4][3][3];
    orderedSensorValues[4][8]=muxValues[4][5][0];
    orderedSensorValues[4][9]=muxValues[4][7][0];
    orderedSensorValues[5][0]=muxValues[2][6][4];
    orderedSensorValues[5][1]=muxValues[2][6][6];
    orderedSensorValues[5][2]=muxValues[2][6][7];
    orderedSensorValues[5][3]=muxValues[2][6][5];
    orderedSensorValues[5][4]=muxValues[2][6][2];
    orderedSensorValues[5][5]=muxValues[2][6][1];
    orderedSensorValues[5][6]=muxValues[2][6][0];
    orderedSensorValues[5][7]=muxValues[2][6][3];
    orderedSensorValues[5][8]=muxValues[2][5][0];
    orderedSensorValues[5][9]=muxValues[2][7][0];
    orderedSensorValues[6][0]=muxValues[2][0][4];
    orderedSensorValues[6][1]=muxValues[2][0][6];
    orderedSensorValues[6][2]=muxValues[2][0][7];
    orderedSensorValues[6][3]=muxValues[2][0][5];
    orderedSensorValues[6][4]=muxValues[2][0][2];
    orderedSensorValues[6][5]=muxValues[2][0][1];
    orderedSensorValues[6][6]=muxValues[2][0][0];
    orderedSensorValues[6][7]=muxValues[2][0][3];
    orderedSensorValues[6][8]=muxValues[2][1][0];
    orderedSensorValues[6][9]=muxValues[2][2][0];
    orderedSensorValues[7][0]=muxValues[1][4][4];
    orderedSensorValues[7][1]=muxValues[1][4][6];
    orderedSensorValues[7][2]=muxValues[1][4][7];
    orderedSensorValues[7][3]=muxValues[1][4][5];
    orderedSensorValues[7][4]=muxValues[1][4][2];
    orderedSensorValues[7][5]=muxValues[1][4][1];
    orderedSensorValues[7][6]=muxValues[1][4][0];
    orderedSensorValues[7][7]=muxValues[1][4][3];
    orderedSensorValues[7][8]=muxValues[2][3][0];
    orderedSensorValues[7][9]=muxValues[1][3][0];
    orderedSensorValues[8][0]=muxValues[1][6][4];
    orderedSensorValues[8][1]=muxValues[1][6][6];
    orderedSensorValues[8][2]=muxValues[1][6][7];
    orderedSensorValues[8][3]=muxValues[1][6][5];
    orderedSensorValues[8][4]=muxValues[1][6][2];
    orderedSensorValues[8][5]=muxValues[1][6][1];
    orderedSensorValues[8][6]=muxValues[1][6][0];
    orderedSensorValues[8][7]=muxValues[1][6][3];
    orderedSensorValues[8][8]=muxValues[1][5][0];
    orderedSensorValues[8][9]=muxValues[1][7][0];
    orderedSensorValues[9][0]=muxValues[1][0][4];
    orderedSensorValues[9][1]=muxValues[1][0][6];
    orderedSensorValues[9][2]=muxValues[1][0][7];
    orderedSensorValues[9][3]=muxValues[1][0][5];
    orderedSensorValues[9][4]=muxValues[1][0][2];
    orderedSensorValues[9][5]=muxValues[1][0][1];
    orderedSensorValues[9][6]=muxValues[1][0][0];
    orderedSensorValues[9][7]=muxValues[1][0][3];
    orderedSensorValues[9][8]=muxValues[1][2][0];
    orderedSensorValues[9][9]=muxValues[1][1][0];
#endif

//================  Pasar el vector a la matriz binaria Temporal=================
int a = 0;
int b = 9;

for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinTemp[j][i] = orderedSensorValues[a][b];
        b = b - 1;
    }
    a = a + 1;
    b = 9;
}

#if PosMatrizSensor == matrizPos1
//================  Matriz en posicion 1 (Original) =================
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinVerif[i][j] = matrizBinTemp[i][j];
    }
}
#endif

#if PosMatrizSensor == matrizPos2
//================  Matriz en posicion 2 (Rota 90grados en sentido horario) =================
 a = 0;
 b = 9;
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinVerif[i][j] = matrizBinTemp[a][b];
        b = b-1;
    }
    a = a + 1;
    b = 9;
}
#endif

#if PosMatrizSensor == matrizPos3
//================  Matriz en posicion 3 (Rota 180grados en sentido horario) =================
 a = 9;
 b = 9;
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinVerif[i][j] = matrizBinTemp[a][b];
        a = a-1;
    }
    a = 9;
    b = b - 1;
}
#endif

#if PosMatrizSensor == matrizPos4
//================  Matriz en posicion 4 (Rota 270grados en sentido horario) =================
 a = 9;
 b = 0;
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinVerif[i][j] = matrizBinTemp[a][b];
        b = b + 1;
    }
    a = a - 1;
    b = 0;
}
#endif

#ifdef debugSensors
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        //matrizBinVerif[i][j] = matrizBinTemp[i][j];
        Serial.print(matrizBinVerif[i][j]);
    }
    Serial.println("");
}
#endif

//======================================================================
}

//Funcion que realiza el movimiento de enroque coorto para la torre, ya que el movimiento del rey se realiza 
//previamente, esta funcion se utiliza en el modo de juego de Play Mode
void shortCastlingRook(bool chess_color)
{
    Serial.println("Movimiento de la torre en enroque corto");
    double x[5];
    double y[5];

    BoardPosition boardPosition;

    if (chess_color == true) //Si es el turno de las blancas
    {
        deactivateElectromagnet();
        //Movemos a la posicion de la torre en "h1"
        boardPosition = getBoardPositionFromString("h1");
        
        #ifdef funcMoveTo
        Robot.moveToPointV2(boardPosition.x,boardPosition.y,1);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(boardPosition.x,boardPosition.y,2);
        #endif
        activateElectromagnet();

        //Movemos la torre a la linea inferior del limite del tablero
        boardPosition.x = 3.5 * longitud;
        boardPosition.y = -4.0 * longitud;
        x[0] = boardPosition.x;
        y[0] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[0],y[0]);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[0],y[0],2);
        #endif
        delay(500); //Antes delay 1000

        //Movemos la torre sobre la linea inferior del tablero hasta "f1"
        boardPosition.x = 1.5 * longitud;
        boardPosition.y = -4.0 * longitud;
        x[1] = boardPosition.x;
        y[1] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[1],y[1]);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[1],y[1],2);
        #endif
        delay(500); //Antes delay 1000

        //Movemos la torre al centro de "f1"
        boardPosition = getBoardPositionFromString("f1");
        x[2] = boardPosition.x;
        y[2] = boardPosition.y;

        //moveThroughThisPoints(x, y, 3);
        
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[2],y[2],1);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[2],y[2],2);
        #endif
        deactivateElectromagnet();
    } else //Si es el turno de las negras
    {
        deactivateElectromagnet();
        //Movemos a la posicion de la torre en "h8"
        boardPosition = getBoardPositionFromString("h8");
        
        #ifdef funcMoveTo
        Robot.moveToPointV2(boardPosition.x,boardPosition.y,1);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(boardPosition.x,boardPosition.y,2);
        #endif
        activateElectromagnet();

        //Movemos la torre a la linea inferior del limite del tablero
        boardPosition.x = 3.5 * longitud;
        boardPosition.y = 4.0 * longitud;
        x[0] = boardPosition.x;
        y[0] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[0],y[0]);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[0],y[0],2);
        #endif
        delay(500); //Antes delay 1000

        //Movemos la torre sobre la linea inferior del tablero hasta "f8"
        boardPosition.x = 1.5 * longitud;
        boardPosition.y = 4.0 * longitud;
        x[1] = boardPosition.x;
        y[1] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[1],y[1]);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[1],y[1],2);
        #endif
        delay(500); //Antes delay 1000

        //Movemos la torre al centro de "f8"
        boardPosition = getBoardPositionFromString("f8");
        x[2] = boardPosition.x;
        y[2] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[2],y[2],1);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[2],y[2],2);
        #endif
        //moveThroughThisPoints(x, y, 3);
        deactivateElectromagnet();
    }
}

//Funcion que realiza el movimiento de enroque largo para la torre, ya que el movimiento del rey se realiza 
//previamente, esta funcion se utiliza en el modo de juego de Play Mode
void longCastlingRook(bool chess_color)
{
    double x[5];
    double y[5];
    BoardPosition boardPosition;

    if (chess_color == true) //Si es el turno de las blancas
    {
        deactivateElectromagnet();
        //Movemos a la posicion de la torre en "a1"
        boardPosition = getBoardPositionFromString("a1");
        
        #ifdef funcMoveTo
        Robot.moveToPointV2(boardPosition.x,boardPosition.y,1);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(boardPosition.x,boardPosition.y,2);
        #endif
        activateElectromagnet();

        //Movemos la torre a la linea inferior del limite del tablero
        boardPosition.x = -3.5 * longitud;
        boardPosition.y = -4.0 * longitud;
        x[0] = boardPosition.x;
        y[0] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[0],y[0]);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[0],y[0],2);
        #endif
        delay(500); //Antes delay 1000

        //Movemos la torre sobre la linea inferior del tablero hasta "d1"
        boardPosition.x = -0.5 * longitud;
        boardPosition.y = -4.0 * longitud;
        x[1] = boardPosition.x;
        y[1] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[1],y[1]);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[1],y[1],2);
        #endif
        delay(500); //Antes delay 1000

        //Movemos la torre al centro de "d1"
        boardPosition = getBoardPositionFromString("d1");
        x[2] = boardPosition.x;
        y[2] = boardPosition.y;
        
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[2],y[2],1);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[2],y[2],2);
        #endif
        //moveThroughThisPoints(x, y, 3);
        deactivateElectromagnet();
    } else //Si es el turno de las negras
    {
        deactivateElectromagnet();
        //Movemos a la posicion de la torre en "a8"
        boardPosition = getBoardPositionFromString("a8");
        
        #ifdef funcMoveTo
        Robot.moveToPointV2(boardPosition.x,boardPosition.y,1);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(boardPosition.x,boardPosition.y,2);
        #endif
        activateElectromagnet();

        //Movemos la torre a la linea inferior del limite del tablero
        boardPosition.x = -3.5 * longitud;
        boardPosition.y = 4.0 * longitud;
        x[0] = boardPosition.x;
        y[0] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[0],y[0]);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[0],y[0],2);
        #endif
        delay(500); //Antes delay 1000

        //Movemos la torre sobre la linea inferior del tablero hasta "d8"
        boardPosition.x = -0.5 * longitud;
        boardPosition.y = 4.0 * longitud;
        x[1] = boardPosition.x;
        y[1] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[1],y[1]);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[1],y[1],2);
        #endif
        delay(500); //Antes delay 1000

        //Movemos la torre al centro de "d8"
        boardPosition = getBoardPositionFromString("d8");
        x[2] = boardPosition.x;
        y[2] = boardPosition.y;
        #ifdef funcMoveTo
        Robot.moveToPointV2(x[2],y[2],1);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(x[2],y[2],2);
        #endif

        //moveThroughThisPoints(x, y, 3);
        deactivateElectromagnet();
    }
}

int checkFinalPositionGraveyard(bool colorInMovement)
{
    int axisX = -1;
    int axisY = -1;

    int vectPosXBlancas[16] = {0,0,0,0,0,0,0,0,1,2,3,4,5,6,7,8};                                        //antes  {8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0}
    int vectPosYBlancas[16] = {8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0};                                         //antes  {0,0,0,0,0,0,0,0,1,2,3,4,5,6,7,8}

    int vectPosXNegras[16] = {9,9,9,9,9,9,9,9,8,7,6,5,4,3,2,1};                                         //antes   {1,2,3,4,5,6,7,8,9,9,9,9,9,9,9,9}
    int vectPosYNegras[16] = {1,2,3,4,5,6,7,8,9,9,9,9,9,9,9,9};                                         //antes   {9,9,9,9,9,9,9,9,8,7,6,5,4,3,2,1}

    if(colorInMovement == true)
    {
        axisX = vectPosXNegras[indexVectNegras];
        axisY = vectPosYNegras[indexVectNegras];
    }
    if(colorInMovement == false)
    {
        axisX = vectPosXBlancas[indexVectBlancas];
        axisY = vectPosYBlancas[indexVectBlancas];
    }

    Serial.println("axisX      axisY");
    Serial.print(axisX);
    Serial.print(" ");
    Serial.println(axisY);

    Serial.println("Result matrizBinVerif [axisX][axisY]");
    Serial.println(matrizBinVerif[axisX][axisY]);                          


    if(matrizBinVerif[axisX][axisY] == false)                     
    {
        return 1;
    }
    else
    {
        if(colorInMovement == true)
        {
            vectNegras[indexVectNegras] = 'v';
        }
        else
        {
            vectBlancas[indexVectBlancas] = 'v';
        }
        return 0;
    }
}

void checkAreaChess(float pointIniX, float pointIniY, int r)
{
    #ifdef checkArea
    
    pointIniX = pointIniX + r;
    pointIniY = pointIniY + r;
    #ifdef funcMoveTo
    Robot.moveToPointV2(pointIniX, pointIniY,1);
    #endif
    #ifdef funcAccelRamp
    Robot.accelRamp(pointIniX,pointIniY,2);
    #endif
    delay(200);

    pointIniX = pointIniX - (r*2);
    #ifdef funcMoveTo
    Robot.moveToPointV2(pointIniX, pointIniY,1);
    #endif
    #ifdef funcAccelRamp
    Robot.accelRamp(pointIniX,pointIniY,2);
    #endif
    delay(200);
    
    pointIniY = pointIniY - (r*2);
    #ifdef funcMoveTo
    Robot.moveToPointV2(pointIniX, pointIniY,1);
    #endif
    #ifdef funcAccelRamp
    Robot.accelRamp(pointIniX,pointIniY,2);
    #endif
    delay(200);

    pointIniX = pointIniX + (r*2);
    #ifdef funcMoveTo
    Robot.moveToPointV2(pointIniX, pointIniY,1);
    #endif
    #ifdef funcAccelRamp
    Robot.accelRamp(pointIniX,pointIniY,2);
    #endif
    delay(200);

    pointIniY = pointIniY + (r*2);
    #ifdef funcMoveTo
    Robot.moveToPointV2(pointIniX, pointIniY,1);
    #endif
    #ifdef funcAccelRamp
    Robot.accelRamp(pointIniX,pointIniY,2);
    #endif
    delay(200);

    pointIniX = pointIniX - r;
    pointIniY = pointIniY - r;
    #ifdef funcMoveTo
    Robot.moveToPointV2(pointIniX, pointIniY,1);
    #endif
    #ifdef funcAccelRamp
    Robot.accelRamp(pointIniX,pointIniY,2);
    #endif
    delay(200);
    #endif
}

void testPWMElectromagnetDeactivate()
{
    int optionElectro =1;
    //Serial.println("SE desactivo ELECTROIMAN : ");
    //Serial.println(optionElectro);
    #ifdef electromagnetConf1
    //Serial.println("->> DEACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 0);   //ANTES 1
    #endif

    #ifdef electromagnetConf2
    //Serial.println("->> ACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;        //ANTES 5000
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    //ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
#ifdef normalElectro
     if(optionElectro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
#endif
#ifdef invertElectro
     if(optionElectro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
#endif
    //ledcWrite(ledChannel, 254); //Antes 220

    #endif
    ledcDetachPin(12);
   ledcDetachPin(13);
   ledcDetachPin(22);
   ledcDetachPin(23);
    /*
    Serial.println("deactivateElectromagnet ......");
    #ifdef electromagnetConf1
    //Serial.println("->> DEACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 0);   //ANTES 1
    #endif

    #ifdef electromagnetConf2
    //Serial.println("->> ACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;        //ANTES 5000
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);

    ledcWrite(ledChannel, 254); //Antes 220
    

    #endif
    */
}


void testPWMElectromagnetActivate()
{
    Serial.println("Funcion activateElectromagnet ...");
    #ifdef electromagnetConf1
    //Serial.println("->> ACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;        //ANTES 5000
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);

    ledcWrite(ledChannel, 254); //Antes 220
    delay(200); //Antes 1000
    #endif

    #ifdef electromagnetConf2
    //Serial.println("->> DEACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 0);   //ANTES 1
    delay(200); //Antes 1000
    #endif
    
}

void impresionSerialEasyPeasy(int tiempo)
{
  detectChessBoard();

                for (int i = 0; i < 10; i++)
                {
                  for (int j = 0;  j < 10 ; j++)
                  {
                    Serial.print(orderedSensorValues[i][j]);
                  }
                  Serial.println();
                }
                Serial.println();
                Serial.println();
                if (tiempo != 0)
                {
                  delay(tiempo);
                }
  

  }

  void centrarPiezasIni()
  {
      int resultFinalPos;
      //sensorsDir();
      //detectChessBoard();
      int resultContPiezas = -1;

      double compFinX, compFinY;
      double *ApFinX = &compFinX;
      double *ApFinY = &compFinY;

      double compIniX, compIniY;

      char cVectX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
      char cVectY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};
      char charPosXFinal = 'v';
      char charPosYFinal = 'v';
      bool colorActualChess = false;

      //-----------------------------------------------
      //Recorre la matriz inicializada para identificar el lugar de cada pieza
      //Genera la coordenada de posicion final
      //Mueve la pieza a una posicion distinta al centro
      //Inicia un movimiento para regresar al centro pero un instante antes desactiva el electroiman mientras continua el movimiento al centro
      //Al llegar al centro Activa el eelectroiman al maximo y finalmente lo apaga

      for (int j = 0; j < 8; j++)
      {
          for (int i = 0; i < 8; i++)
          {
                if (matriz[i][j] != '.')
                {
                  charPosXFinal = cVectX[i];
                  charPosYFinal = cVectY[j];
                  coordenadas(charPosXFinal, charPosYFinal, ApFinX, ApFinY);
                  //Serial.println("Coordenada Final X");
                  //Serial.println(compFinX);
                  //Serial.println("Coordenada Final Y");
                  //Serial.println(compFinY);

                  if (matriz[i][j] >= 'a' && matriz[i][j] <= 'z')
                  {
                      //Piezas Negras
                      colorActualChess = false;
                  }
                  if (matriz[i][j] >= 'A' && matriz[i][j] <= 'Z')
                  {
                      //Piezas Blancas
                      colorActualChess = true;
                  }

                  //Robot.moveToPointV2(compFinX,compFinY,1);
                  //moveOnTheLineIni(compFinX,compFinY,compFinX,compFinY);

                    #ifdef funcMoveTo
                    Robot.setSpeedMotors(maximumVelocityDirect);
                    #endif
                    #ifdef funcAccelRamp
                    Robot.setSpeedMotors(maximunVelocityDirectRamp);
                    Robot.setSpeedRampFunction(maximunVelocityDirectRamp);
                    Robot.setAccelRampFunction(maximunAccelDirectRamp);
                    #endif

                    #ifdef moveDirectPosPiece
                    
                    #ifdef funcMoveTo
                    Robot.moveToPointV2(compFinX,compFinY,1);
                    #endif

                    #ifdef funcAccelRamp

                    #ifdef version1Electro
                    Robot.accelRamp(compFinX,compFinY,2);       //Antes desacelracion,2
                    #endif

                    #ifdef version4Electro
                    Robot.accelRamp(compFinX,compFinY,2,0,mainElectro);       //Antes desacelracion,2
                    #endif

                    #endif
                    
                    #else
                    moveOnTheLineIni(compFinX,compFinY,compFinX,compFinY);
                    #endif

                    #ifdef funcMoveTo
                    Robot.setSpeedMotors(defGlobalSpeed);
                    #endif
                    #ifdef funcAccelRamp
                    Robot.setSpeedMotors(defGlobalSpeed);
                    Robot.setSpeedRampFunction(defGlobalSpeed);
                    Robot.setAccelRampFunction(defGlobalAccel);
                    #endif

                    #ifdef version1Electro
                    testPWMElectromagnetActivate();
                    #endif

                     #ifdef version4Electro
                    testPWMElectromagnetActivateV4E(mainElectro);
                    #endif

                  compIniX = compFinX + 10;
                  compIniY = compFinY + 10;
                  #ifdef funcMoveTo
                  Robot.moveToPointV2(compIniX,compIniY,1);
                  #endif
                  #ifdef funcAccelRamp

                  #ifdef version1Electro
                  Robot.accelRamp(compIniX,compIniY,2);
                  #endif

                   #ifdef version4Electro
                    Robot.accelRamp(compIniX,compIniY,2,0,mainElectro);
                   #endif

                  #endif

                  #ifdef centrarElectromagnet
                  double compXanterior = 0;
                  double compYanterior = 0;

                  compXanterior = compFinX;
                  compYanterior = compFinY;

                  double desfase;
                  desfase = desfaseElectroiman;
                  if((compIniX != compFinX)&&(compIniY != compFinY))
                  {
                      desfase = sqrt((desfaseElectroiman*desfaseElectroiman)/2);
                  }

                  if (compIniX > compFinX)
                  {
                      compXanterior = compXanterior + desfase;
                  }
                  if (compIniX < compFinX)
                  {
                      compXanterior = compXanterior - desfase;
                  }

                  if (compIniY > compFinY)
                  {
                      compYanterior = compYanterior + desfase;
                  }
                  if (compIniY < compFinY)
                  {
                      compYanterior = compYanterior - desfase;
                  }
                  #ifdef funcMoveTo
                  Robot.moveToPointV2(compXanterior, compYanterior, 1);
                  #endif


                  #ifdef funcMoveTo
        
                  testPWMElectromagnetDeactivate();
                  #endif
                  #endif

                  //=================================================================
                  #ifdef funcMoveTo
                  Robot.moveToPointV2(compFinX, compFinY,1);
                  #endif
                  #ifdef funcAccelRamp

                  #ifdef version1Electro
                  Robot.accelRamp(compFinX,compFinY,2,1);
                  #endif

                  #ifdef version4Electro
                  Robot.accelRamp(compFinX,compFinY,2,1,mainElectro);
                  #endif

                  #endif

                  #ifdef centrarElectromagnet

                  #ifdef version1Electro
                  activateElectromagnet();
                  #endif

                   #ifdef version4Electro
                    activateElectromagnetV4E(mainElectro);
                   #endif

                  #endif

                  #ifdef version1Electro
                  deactivateElectromagnet();
                  #endif

                   #ifdef version4Electro
                    deactivateElectromagnetV4E(mainElectro);
                   #endif
                  
              }
          }
      }
  }

/*================================================================================================*/
void moveOnTheLineIni(double xIni,double yIni,double xFin,double yFin)
{
    double compX, compY;
    double* ApX = &compX;
    double* ApY = &compY;
    //=======Puntos intermedios para trayectoria del caballo=========
    float diferenciaX;
    float diferenciaY;
    float puntoInterX;
    float puntoInterY;
    #ifdef debug
    Serial.println("Dentro de Funcion moveOnTheLineIni");
    #endif
    Robot.getActualPosition(ApX,ApY);
    #ifdef debug
    Serial.println("---Coordennadas anteriores");
    
    #ifdef activateSensors
    compX = compX - desfaseEnX;
    compY = compY - desfaseEnY;
    #endif

    Serial.println(compX);
    Serial.println(compY);
    #endif

    deactivateElectromagnet();

    //Robot.moveToPointV2(xIni, yIni,1); //Se dirige directamente a la posicion inicial con el electroiman desactivado
    
    //Genera la trayectoria para ir a la posicion de la pieza moviendose entre lineas
    //================================================================================================
    
    diferenciaX = abs(compX - xIni);
    diferenciaY = abs(compY - yIni);
    //======Condiciones para los casos donde la pieza se encuentra en los escaques de piezas muertas =======*/
    //Como primer movimiento acerca a la pieza en diagonal hasta la esquina del escaque inicial
    if(compX < -200)
    {
        puntoInterX = compX + 25;
        if(yIni > compY)
        {
            puntoInterY = compY + 25;
        }
        if(yIni <= compY)
        {
            puntoInterY = compY - 25;
        }
        #ifdef funcMoveTo
        Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(puntoInterX,puntoInterY,2);
        #endif
    }
    if(compX > 200)
    {
        puntoInterX = compX - 25;
        if(yIni > compY)
        {
            puntoInterY = compY + 25;
        }
        if(yIni <= compY)
        {
            puntoInterY = compY - 25;
        }
        #ifdef funcMoveTo
        Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(puntoInterX, puntoInterY,2);
        #endif
    }
    if(compY < -200)
    {
        if(xIni < compX)
        {
            puntoInterX = compX - 25;
        }
        if(xIni >= compX)
        {
            puntoInterX = compX - 25;
        }
        puntoInterY = compY + 25;
        #ifdef funcMoveTo
        Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(puntoInterX, puntoInterY,2);
        #endif
    }
    if(compY > 200)
    {
        if(xIni < compX)
        {
            puntoInterX = compX - 25;
        }
        if(xIni >= compX)
        {
            puntoInterX = compX - 25;
        }
        puntoInterY = compY - 25;
        #ifdef funcMoveTo
        Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        Robot.accelRamp(puntoInterX, puntoInterY,2);
        #endif
    }
    /*================================================================================================*/
    if(diferenciaX > diferenciaY)
    {
        if(compX < 0)
        {
            if((compX <= 200 && compX >= -200) && (compY <= 200 && compY >= -200))    //Punto inicial dentro del area de juego
            {
                puntoInterX = compX + 25;
                puntoInterY = compY;
                #ifdef funcMoveTo
                Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
            }
        }
        else
        {
            if((compX <= 200 && compX >= -200) && (compY <= 200 && compY >= -200))    //Punto inicial dentro del area de juego
            {
                puntoInterX = compX - 25;
                puntoInterY = compY;
                #ifdef funcMoveTo
                Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
            }
        }
        //==================================================================
        if(yIni > 0)
        {
            puntoInterY = yIni - 25;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        else
        {
            puntoInterY = yIni + 25;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        if(xIni <= 200 && xIni >= -200)
        {
            puntoInterX = xIni;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        else
        {
            if(xIni <= compX)
            {
                puntoInterX = xIni + 25;
            }
            else
            {
                puntoInterX = xIni - 25;
            }
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
    }
    else
    {
        if(compX < 0)
        {
            if((compX <= 200 && compX >= -200) && (compY <= 200 && compY >= -200))    //Punto inicial dentro del area de juego
            {
                puntoInterX = compX + 25;
                puntoInterY = compY;
                #ifdef funcMoveTo
                Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
            }
        }
        else
        {
            if((compX <= 200 && compX >= -200) && (compY <= 200 && compY >= -200))    //Punto inicial dentro del area de juego
            {
                puntoInterX = compX - 25;
                puntoInterY = compY;
                #ifdef funcMoveTo
                Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
            }
        }
        //===================================================================
        if(yIni > 0)
        {
            puntoInterY = yIni - 25;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        else
        {
            puntoInterY = yIni + 25;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        if(xIni <= 200 && xIni >= -200)
        {
            puntoInterX = xIni;
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
        else
        {
            if(xIni <= compX)
            {
                puntoInterX = xIni + 25;
            }
            else
            {
                puntoInterX = xIni - 25;
            }
            #ifdef funcMoveTo
            Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
        }
    }
    //===============================================================
    #ifdef funcMoveTo
    Robot.moveToPointV2(xIni,yIni);
    #endif
    #ifdef funcAccelRamp
    Robot.accelRamp(xIni,yIni,2);
    #endif

    //================================================================================================
}


void compareVirtualMatrizVsSensors()
{
    sensorsDir();
    int indexBinX;
    int indexBinY;
 
    #ifdef debugSensors
        for (int j = 0; j < 10; j++)
        {
            for (int i = 0; i < 10; i++)
            {
                Serial.print(matrizBinVerif[i][j]);
                Serial.print(" ");
            }
            Serial.println("");
        }
    #endif

    //==========Compara matriz de sensores y tablero virtual en la zona de juego========
    detectChessBoardVerif();
        for(int j = 0; j < 8; j++)
        {
            for(int i = 0; i < 8; i++)
            {
                indexBinX = i + 1;
                indexBinY = j + 1;
                if(matriz[i][j] != '.')
                {
                    
                    while(matrizBinVerif[indexBinX][indexBinY] == true)
                    {
                        detectChessBoardVerif();
                        //Serial.println("matriz virtual no coincide con sensores");
                        //digitalWrite(23,HIGH);
                        //digitalWrite(22,HIGH);
                        #ifdef debugSensors
                        Serial.println("");
                        for (int j = 0; j < 10; j++)
                        {
                            for (int i = 0; i < 10; i++)
                            {
                                Serial.print(matrizBinVerif[i][j]);
                                Serial.print(" ");
                            }
                            Serial.println("");
                        }
                        delay(1000);
                        #endif
                        
                    }
                    //digitalWrite(23,LOW);
                    //digitalWrite(22,LOW);
                }
            }
        }
        //===================================================================================

        detectChessBoardVerif();
        int axisX = -1;
        int axisY = -1;

        int vectPosXBlancas[16] = {0,0,0,0,0,0,0,0,1,2,3,4,5,6,7,8};    //antes  {8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0}
        int vectPosYBlancas[16] = {8,7,6,5,4,3,2,1,0,0,0,0,0,0,0,0};    //antes  {0,0,0,0,0,0,0,0,1,2,3,4,5,6,7,8}

        int vectPosXNegras[16] = {9,9,9,9,9,9,9,9,8,7,6,5,4,3,2,1};     //antes   {1,2,3,4,5,6,7,8,9,9,9,9,9,9,9,9}
        int vectPosYNegras[16] = {1,2,3,4,5,6,7,8,9,9,9,9,9,9,9,9};     //antes   {9,9,9,9,9,9,9,9,8,7,6,5,4,3,2,1}

        for(int i = 0; i < 16; i++)
        {
            axisX = vectPosXBlancas[i];
            axisY = vectPosYBlancas[i];
            if(vectBlancas[i] != 'v')
            {
                while(matrizBinVerif[axisX][axisY] == true)
                    {
                        detectChessBoardVerif();
                        //Serial.println("matriz virtual no coincide con sensores");
                        //digitalWrite(23,HIGH);
                        //digitalWrite(22,HIGH);
                        #ifdef debugSensors
                        Serial.println("");
                        for (int j = 0; j < 10; j++)
                        {
                            for (int i = 0; i < 10; i++)
                            {
                                Serial.print(matrizBinVerif[i][j]);
                                Serial.print(" ");
                            }
                            Serial.println("");
                        }
                        delay(1000);
                        #endif
                        
                    }
                    //digitalWrite(23,LOW);
                    //digitalWrite(22,LOW);
            }
        }

        detectChessBoardVerif();

        for(int i = 0; i < 16; i++)
        {
            axisX = vectPosXNegras[i];
            axisY = vectPosYNegras[i];
            if(vectNegras[i] != 'v')
            {
                while(matrizBinVerif[axisX][axisY] == true)
                    {
                        detectChessBoardVerif();
                        //Serial.println("matriz virtual no coincide con sensores");
                        //digitalWrite(23,HIGH);
                        //digitalWrite(22,HIGH);
                        #ifdef debugSensors
                        Serial.println("");
                        for (int j = 0; j < 10; j++)
                        {
                            for (int i = 0; i < 10; i++)
                            {
                                Serial.print(matrizBinVerif[i][j]);
                                Serial.print(" ");
                            }
                            Serial.println("");
                        }
                        delay(1000);
                        #endif
                        
                    }
                    //digitalWrite(23,LOW);
                    //digitalWrite(22,LOW);
            }
        }

}

void centrarPiezasAlgortimoConSensores()
{
    int pasosEnX = 0;

    int posIzqX = 0;
    int posDerX = 0;
    float posMediaX = 0;

    int pasosEnY = 25;    

    int posIzqY = 0;
    int posDerY = 0;
    float posMediaY = 0;

    float difSuperiorX = 0;
    float difInferiorX = 0;
    float difSuperiorY = 0;
    float difInferiorY = 0;

    sensorsDir();

    //Algoritmo para buscar el punto medio sobre el eje x
    //detectChessBoardVerif();
    detectChessBoardAlgoritmoSensores();
    #ifdef version1Electro
    deactivateElectromagnet();
    #endif
    #ifdef version4Electro
    deactivateElectromagnetV4E(1);
    #endif

    #ifdef funcMoveTo
    Robot.moveToPointV2(pasosEnX, pasosEnY);
    #endif
    #ifdef funcAccelRamp
    #ifdef version1Electro
    Robot.accelRamp(pasosEnX, pasosEnY,0);
    #endif

    #ifdef version4Electro
    Robot.accelRamp(pasosEnX, pasosEnY,0,0,1);
    #endif

    #endif
    
    #ifdef version1Electro
    activateElectromagnet();
    #endif
    #ifdef version4Electro
    activateElectromagnetV4E(1);
    #endif

    #ifdef relacionMicroSteps4
    Robot.setSpeedMotors(500);
    Robot.setSpeedRampFunction(500);
    #endif
    #ifdef relacionMicroSteps16
    Robot.setSpeedMotors(2000);
    Robot.setSpeedRampFunction(2000);
    #endif
    #ifdef relacionMicroSteps32
    Robot.setSpeedMotors(2000);
    #endif
    // Recorrrido en eje x
    while (matrizBinVerif[8][4] == 1)  //Antes [8][1]
    {
        //detectChessBoardVerif();
        detectChessBoardAlgoritmoSensores();
        // Serial.println(matrizBinVerif[1][1]);
        pasosEnX = pasosEnX + 1;
        #ifdef funcMoveTo
        Robot.moveToPointV2(pasosEnX, pasosEnY);
        #endif
        #ifdef funcAccelRamp
        #ifdef version1Electro
        Robot.accelRamp(pasosEnX, pasosEnY,0);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(pasosEnX, pasosEnY,0,0,1);
        #endif

        #endif
        
        if(pasosEnX > 190)
        {
            #ifdef funcMoveTo
            Robot.moveToPointV2(0,0);
            #endif
            #ifdef funcAccelRamp
            #ifdef version1Electro
            Robot.accelRamp(0,0,0);
            #endif

            #ifdef version4Electro
            Robot.accelRamp(0,0,0,0,1);
            #endif

            #endif
            Serial.println("Llego al limite del tablero (Lado Derecho)");
            #ifdef version1Electro
            deactivateElectromagnet();
            #endif
            #ifdef version4Electro
            deactivateElectromagnetV4E(1);
            #endif
            while(true)
            {
                //Infinite Loop
            }
        }
    }
    Serial.println("Detecta sensor de la derecha");
    Serial.println("Posicion: ");
    Serial.println(pasosEnX);
    posDerX = pasosEnX;

    #ifdef version1Electro
    deactivateElectromagnet();
    #endif
    #ifdef version4Electro
    deactivateElectromagnetV4E(1);
    #endif
    //detectChessBoardVerif();
    detectChessBoardAlgoritmoSensores();
    pasosEnX = 0;
    pasosEnY = 25;
    #ifdef funcMoveTo
    Robot.moveToPointV2(pasosEnX, pasosEnY);
    #endif
    #ifdef funcAccelRamp
    #ifdef version1Electro
    Robot.accelRamp(pasosEnX, pasosEnY,0);
    #endif

    #ifdef version4Electro
    Robot.accelRamp(pasosEnX, pasosEnY,0,0,1);
    #endif

    #endif

    #ifdef version1Electro
    activateElectromagnet();
    #endif

    #ifdef version4Electro
    activateElectromagnetV4E(1);
    #endif
    
    
    while (matrizBinVerif[1][4] == 1)  //Antes [8][1]
    {
        //detectChessBoardVerif();
        detectChessBoardAlgoritmoSensores();
        // Serial.println(matrizBinVerif[1][1]);
        pasosEnX = pasosEnX - 1;
        #ifdef funcMoveTo
        Robot.moveToPointV2(pasosEnX, pasosEnY);
        #endif
        #ifdef funcAccelRamp
        #ifdef version1Electro
        Robot.accelRamp(pasosEnX, pasosEnY,0);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(pasosEnX, pasosEnY,0,0,1);
        #endif

        #endif
        
        if(pasosEnX < -190)
        {
            #ifdef funcMoveTo
            Robot.moveToPointV2(0,0);
            #endif
            #ifdef funcAccelRamp
            #ifdef version1Electro
            Robot.accelRamp(0,0,0);
            #endif

            #ifdef version4Electro
            Robot.accelRamp(0,0,0,0,1);
            #endif

            #endif
            Serial.println("Llego al limite del tablero (Lado Izquierdo)");
            #ifdef version1Electro
            deactivateElectromagnet();
            #endif
            #ifdef version4Electro
            deactivateElectromagnetV4E(1);
            #endif
            while(true)
            {
                //Infinite Loop
            }
        }
    }
    Serial.println("Detecta sensor de la izquierda");
    Serial.println("Posicion: ");
    Serial.println(pasosEnX);
    posIzqX = pasosEnX;

    posMediaX = (posDerX - posIzqX) / 2;
    Serial.println("Pos Media en X");
    Serial.println(posMediaX);
    #ifdef version1Electro
    deactivateElectromagnet();
    #endif

    #ifdef version4Electro
    deactivateElectromagnetV4E(1);
    #endif

    #ifdef funcMoveTo
    Robot.moveToPointV2(0,0);
    #endif
    #ifdef funcAccelRamp
    #ifdef version1Electro
    Robot.accelRamp(0,0,0);
    #endif

    #ifdef version4Electro
    Robot.accelRamp(0,0,0,0,1);
    #endif

    #endif
    Robot.setSpeedMotors(2000);
    Robot.setSpeedRampFunction(2000);
    #ifdef version1Electro
    activateElectromagnet();
    deactivateElectromagnet();
    #endif
    #ifdef version4Electro
    activateElectromagnetV4E(1);
    deactivateElectromagnetV4E(1);
    #endif


    //Algoritmo para buscar el punto medio sobre el eje y
    pasosEnX = 25;
    pasosEnY = 0;  

    //detectChessBoardVerif();
    detectChessBoardAlgoritmoSensores();
    #ifdef version1Electro
    testPWMElectromagnetDeactivate();
    #endif
    #ifdef version4Electro
    testPWMElectromagnetDeactivateV4E(1);
    #endif
    
    #ifdef funcMoveTo
    Robot.moveToPointV2(pasosEnX, pasosEnY);
    #endif
    #ifdef funcAccelRamp
    #ifdef version1Electro
    Robot.accelRamp(pasosEnX, pasosEnY,0);
    #endif

    #ifdef version4Electro
    Robot.accelRamp(pasosEnX, pasosEnY,0,0,1);
    #endif

    #endif

    #ifdef version1Electro
    activateElectromagnet();
    #endif
    #ifdef version4Electro
    activateElectromagnetV4E(1);
    #endif

    #ifdef relacionMicroSteps4
    Robot.setSpeedMotors(500);
    Robot.setSpeedRampFunction(500);
    #endif
    #ifdef relacionMicroSteps16
    Robot.setSpeedMotors(2000);
    Robot.setSpeedRampFunction(2000);
    #endif
    #ifdef relacionMicroSteps32
    Robot.setSpeedMotors(2000);
    #endif
    // Recorrrido en eje x
    while (matrizBinVerif[5][3] == 1)   //Antes [8][1]
    {
        //detectChessBoardVerif();
        detectChessBoardAlgoritmoSensores();
        // Serial.println(matrizBinVerif[1][1]);
        pasosEnY = pasosEnY + 1;

        #ifdef funcMoveTo
        Robot.moveToPointV2(pasosEnX, pasosEnY);
        #endif
        #ifdef funcAccelRamp
        #ifdef version1Electro
        Robot.accelRamp(pasosEnX, pasosEnY,0);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(pasosEnX, pasosEnY,0,0,1);
        #endif


        #endif

        if(pasosEnY > 190)
        {
            #ifdef funcMoveTo
            Robot.moveToPointV2(0,0);
            #endif
            #ifdef funcAccelRamp
            #ifdef version1Electro
            Robot.accelRamp(0,0,0);
            #endif

            #ifdef version4Electro
            Robot.accelRamp(0,0,0,0,1);
            #endif

            #endif
            Serial.println("Llego al limite del tablero (Lado Superior)");
            
            #ifdef version1Electro
            deactivateElectromagnet();
            #endif

            #ifdef version4Electro
            deactivateElectromagnetV4E(1);
            #endif
            
            while(true)
            {
                //Infinite Loop
            }
        }
    }
    Serial.println("Se detecto el sensor superior");
    Serial.println("Posicion: ");
    Serial.println(pasosEnY);
    posDerY = pasosEnY;

    pasosEnX = 25;
    pasosEnY = 0;   //Antes 125
    #ifdef funcMoveTo
    Robot.moveToPointV2(pasosEnX, pasosEnY);
    #endif
    #ifdef funcAccelRamp
    #ifdef version1Electro
    Robot.accelRamp(pasosEnX, pasosEnY,0);
    #endif

    #ifdef version4Electro
    Robot.accelRamp(pasosEnX, pasosEnY,0,0,1);
    #endif

    #endif
    //detectChessBoardVerif();
    detectChessBoardAlgoritmoSensores();
    while (matrizBinVerif[5][6] == 1)   //Antes [8][1]
    {
        //detectChessBoardVerif();
        detectChessBoardAlgoritmoSensores();
        // Serial.println(matrizBinVerif[1][1]);
        pasosEnY = pasosEnY - 1;
        #ifdef funcMoveTo
        Robot.moveToPointV2(pasosEnX, pasosEnY);
        #endif
        #ifdef funcAccelRamp
        #ifdef version1Electro
        Robot.accelRamp(pasosEnX, pasosEnY,0);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(pasosEnX, pasosEnY,0,0,1);
        #endif

        #endif

        if(pasosEnY < -190)
        {
            #ifdef funcMoveTo
            Robot.moveToPointV2(0,0);
            #endif
            #ifdef funcAccelRamp
            #ifdef version1Electro
            Robot.accelRamp(0,0,0);
            #endif

            #ifdef version4Electro
            Robot.accelRamp(0,0,0,0,1);
            #endif

            #endif
            Serial.println("Llego al limite del tablero (Lado Inferior)");

            #ifdef version1Electro
            deactivateElectromagnet();
            #endif

            #ifdef version4Electro
            deactivateElectromagnetV4E(1);
            #endif

            while(true)
            {
                //Infinite Loop
            }
        }
    }
    Serial.println("Se detecto el sensor inferior");
    Serial.println("Posicion: ");
    Serial.println(pasosEnY);
    posIzqY = pasosEnY;

    posMediaY = (posDerY - posIzqY) / 2;
    Serial.println("Pos Media en Y");
    Serial.println(posMediaY);

    #ifdef version1Electro
    deactivateElectromagnet();
    #endif

    #ifdef version4Electro
    deactivateElectromagnetV4E(1);
    #endif

//=============================================================
    difInferiorX = posIzqX + posMediaX;
    difSuperiorX = posDerX - posMediaX;

    difInferiorY = posIzqY + posMediaY;
    difSuperiorY = posDerY - posMediaY;

    Serial.println("difInferiorX");
    Serial.println(difInferiorX);
    Serial.println("difSuperiorX");
    Serial.println(difSuperiorX);
    Serial.println("difInferiorY");
    Serial.println(difInferiorY);
    Serial.println("difSuperiorY");
    Serial.println(difSuperiorY);

//=============================================================
    #ifdef funcMoveTo
    Robot.moveToPointV2(0,0);
    #endif
    #ifdef funcAccelRamp
    #ifdef version1Electro
    Robot.accelRamp(0,0,0);
    #endif

    #ifdef version4Electro
    Robot.accelRamp(0,0,0,0,1);
    #endif

    #endif
    desfaseEnX = (difInferiorX + difSuperiorX)/2;
    desfaseEnY = (difInferiorY + difSuperiorY)/2;

    Serial.println("Desfase en X");
    Serial.println(desfaseEnX);
    Serial.println("Desfase en Y");
    Serial.println(desfaseEnY);

    Robot.setSpeedMotors(defGlobalSpeed);
    Robot.setSpeedRampFunction(defGlobalSpeed);

    #ifdef version1Electro
    activateElectromagnet();
    deactivateElectromagnet();
    #endif

    #ifdef version4Electro
    activateElectromagnet();
    deactivateElectromagnet();
    #endif
}


void detectChessBoardAlgoritmoSensores()
{
    bool matrizBinTemp[10][10];
    //testPWMElectromagnetDeactivate();
    readRawChessBoard();

#ifdef oldSensors
    orderedSensorValues[0][0]=muxValues[3][4][4];
    orderedSensorValues[0][1]=muxValues[3][4][6];
    orderedSensorValues[0][2]=muxValues[3][4][7];
    orderedSensorValues[0][3]=muxValues[3][4][5];
    orderedSensorValues[0][4]=muxValues[3][4][2];
    orderedSensorValues[0][5]=muxValues[3][4][1];
    orderedSensorValues[0][6]=muxValues[3][4][0];
    orderedSensorValues[0][7]=muxValues[3][4][3];
    orderedSensorValues[0][8]=muxValues[3][6][0];
    orderedSensorValues[0][9]=muxValues[3][7][0];

    orderedSensorValues[1][0]=muxValues[3][2][4];
    orderedSensorValues[1][1]=muxValues[3][2][6];
    orderedSensorValues[1][2]=muxValues[3][2][7];
    orderedSensorValues[1][3]=muxValues[3][2][5];
    orderedSensorValues[1][4]=muxValues[3][2][2];
    orderedSensorValues[1][5]=muxValues[3][2][1];
    orderedSensorValues[1][6]=muxValues[3][2][0];
    orderedSensorValues[1][7]=muxValues[3][2][3];
    orderedSensorValues[1][8]=muxValues[3][1][0];
    orderedSensorValues[1][9]=muxValues[3][0][0];

    orderedSensorValues[2][0]=muxValues[4][2][4];
    orderedSensorValues[2][1]=muxValues[4][2][6];
    orderedSensorValues[2][2]=muxValues[4][2][7];
    orderedSensorValues[2][3]=muxValues[4][2][5];
    orderedSensorValues[2][4]=muxValues[4][2][2];
    orderedSensorValues[2][5]=muxValues[4][2][1];
    orderedSensorValues[2][6]=muxValues[4][2][0];
    orderedSensorValues[2][7]=muxValues[4][2][3];
    orderedSensorValues[2][8]=muxValues[3][5][0];
    orderedSensorValues[2][9]=muxValues[3][3][0];

    orderedSensorValues[3][0]=muxValues[4][3][4];
    orderedSensorValues[3][1]=muxValues[4][3][6];
    orderedSensorValues[3][2]=muxValues[4][3][7];
    orderedSensorValues[3][3]=muxValues[4][3][5];
    orderedSensorValues[3][4]=muxValues[4][3][2];
    orderedSensorValues[3][5]=muxValues[4][3][1];
    orderedSensorValues[3][6]=muxValues[4][3][0];
    orderedSensorValues[3][7]=muxValues[4][3][3];
    orderedSensorValues[3][8]=muxValues[4][0][0];
    orderedSensorValues[3][9]=muxValues[4][1][0];

    orderedSensorValues[4][0]=muxValues[4][5][4];
    orderedSensorValues[4][1]=muxValues[4][5][6];
    orderedSensorValues[4][2]=muxValues[4][5][7];
    orderedSensorValues[4][3]=muxValues[4][5][5];
    orderedSensorValues[4][4]=muxValues[4][5][2];
    orderedSensorValues[4][5]=muxValues[4][5][1];
    orderedSensorValues[4][6]=muxValues[4][5][0];
    orderedSensorValues[4][7]=muxValues[4][5][3];
    orderedSensorValues[4][8]=muxValues[4][7][0];
    orderedSensorValues[4][9]=muxValues[4][6][0];

    orderedSensorValues[5][0]=muxValues[2][5][4];
    orderedSensorValues[5][1]=muxValues[2][5][6];
    orderedSensorValues[5][2]=muxValues[2][5][7];
    orderedSensorValues[5][3]=muxValues[2][5][5];
    orderedSensorValues[5][4]=muxValues[2][5][2];
    orderedSensorValues[5][5]=muxValues[2][5][1];
    orderedSensorValues[5][6]=muxValues[2][5][0];
    orderedSensorValues[5][7]=muxValues[2][5][3];
    orderedSensorValues[5][8]=muxValues[2][7][0];
    orderedSensorValues[5][9]=muxValues[2][6][0];

    orderedSensorValues[6][0]=muxValues[2][0][4];
    orderedSensorValues[6][1]=muxValues[2][0][6];
    orderedSensorValues[6][2]=muxValues[2][0][7];
    orderedSensorValues[6][3]=muxValues[2][0][5];
    orderedSensorValues[6][4]=muxValues[2][0][2];
    orderedSensorValues[6][5]=muxValues[2][0][1];
    orderedSensorValues[6][6]=muxValues[2][0][0];
    orderedSensorValues[6][7]=muxValues[2][0][3];
    orderedSensorValues[6][8]=muxValues[2][1][0];
    orderedSensorValues[6][9]=muxValues[2][2][0];

    orderedSensorValues[7][0]=muxValues[1][5][4];
    orderedSensorValues[7][1]=muxValues[1][5][6];
    orderedSensorValues[7][2]=muxValues[1][5][7];
    orderedSensorValues[7][3]=muxValues[1][5][5];
    orderedSensorValues[7][4]=muxValues[1][5][2];
    orderedSensorValues[7][5]=muxValues[1][5][1];
    orderedSensorValues[7][6]=muxValues[1][5][0];
    orderedSensorValues[7][7]=muxValues[1][5][3];
    orderedSensorValues[7][8]=muxValues[1][3][0];
    orderedSensorValues[7][9]=muxValues[2][3][0];

    orderedSensorValues[8][0]=muxValues[1][7][4];
    orderedSensorValues[8][1]=muxValues[1][7][6];
    orderedSensorValues[8][2]=muxValues[1][7][7];
    orderedSensorValues[8][3]=muxValues[1][7][5];
    orderedSensorValues[8][4]=muxValues[1][7][2];
    orderedSensorValues[8][5]=muxValues[1][7][1];
    orderedSensorValues[8][6]=muxValues[1][7][0];
    orderedSensorValues[8][7]=muxValues[1][7][3];
    orderedSensorValues[8][8]=muxValues[1][6][0];
    orderedSensorValues[8][9]=muxValues[1][4][0];

    orderedSensorValues[9][0]=muxValues[1][0][4];
    orderedSensorValues[9][1]=muxValues[1][0][6];
    orderedSensorValues[9][2]=muxValues[1][0][7];
    orderedSensorValues[9][3]=muxValues[1][0][5];
    orderedSensorValues[9][4]=muxValues[1][0][2];
    orderedSensorValues[9][5]=muxValues[1][0][1];
    orderedSensorValues[9][6]=muxValues[1][0][0];
    orderedSensorValues[9][7]=muxValues[1][0][3];
    orderedSensorValues[9][8]=muxValues[1][1][0];
    orderedSensorValues[9][9]=muxValues[1][2][0];
#endif
    
#ifdef newSensors
    orderedSensorValues[0][0]=muxValues[3][4][4];
    orderedSensorValues[0][1]=muxValues[3][4][6];
    orderedSensorValues[0][2]=muxValues[3][4][7];
    orderedSensorValues[0][3]=muxValues[3][4][5];
    orderedSensorValues[0][4]=muxValues[3][4][2];
    orderedSensorValues[0][5]=muxValues[3][4][1];
    orderedSensorValues[0][6]=muxValues[3][4][0];
    orderedSensorValues[0][7]=muxValues[3][4][3];
    orderedSensorValues[0][8]=muxValues[3][7][0];
    orderedSensorValues[0][9]=muxValues[3][6][0];
    orderedSensorValues[1][0]=muxValues[3][1][4];
    orderedSensorValues[1][1]=muxValues[3][1][6];
    orderedSensorValues[1][2]=muxValues[3][1][7];
    orderedSensorValues[1][3]=muxValues[3][1][5];
    orderedSensorValues[1][4]=muxValues[3][1][2];
    orderedSensorValues[1][5]=muxValues[3][1][1];
    orderedSensorValues[1][6]=muxValues[3][1][0];
    orderedSensorValues[1][7]=muxValues[3][1][3];
    orderedSensorValues[1][8]=muxValues[3][5][0];
    orderedSensorValues[1][9]=muxValues[3][2][0];
    orderedSensorValues[2][0]=muxValues[4][0][4];
    orderedSensorValues[2][1]=muxValues[4][0][6];
    orderedSensorValues[2][2]=muxValues[4][0][7];
    orderedSensorValues[2][3]=muxValues[4][0][5];
    orderedSensorValues[2][4]=muxValues[4][0][2];
    orderedSensorValues[2][5]=muxValues[4][0][1];
    orderedSensorValues[2][6]=muxValues[4][0][0];
    orderedSensorValues[2][7]=muxValues[4][0][3];
    orderedSensorValues[2][8]=muxValues[3][0][0];
    orderedSensorValues[2][9]=muxValues[3][3][0];
    orderedSensorValues[3][0]=muxValues[4][6][4];
    orderedSensorValues[3][1]=muxValues[4][6][6];
    orderedSensorValues[3][2]=muxValues[4][6][7];
    orderedSensorValues[3][3]=muxValues[4][6][5];
    orderedSensorValues[3][4]=muxValues[4][6][2];
    orderedSensorValues[3][5]=muxValues[4][6][1];
    orderedSensorValues[3][6]=muxValues[4][6][0];
    orderedSensorValues[3][7]=muxValues[4][6][3];
    orderedSensorValues[3][8]=muxValues[4][2][0];
    orderedSensorValues[3][9]=muxValues[4][1][0];
    orderedSensorValues[4][0]=muxValues[4][3][4];
    orderedSensorValues[4][1]=muxValues[4][3][6];
    orderedSensorValues[4][2]=muxValues[4][3][7];
    orderedSensorValues[4][3]=muxValues[4][3][5];
    orderedSensorValues[4][4]=muxValues[4][3][2];
    orderedSensorValues[4][5]=muxValues[4][3][1];
    orderedSensorValues[4][6]=muxValues[4][3][0];
    orderedSensorValues[4][7]=muxValues[4][3][3];
    orderedSensorValues[4][8]=muxValues[4][5][0];
    orderedSensorValues[4][9]=muxValues[4][7][0];
    orderedSensorValues[5][0]=muxValues[2][6][4];
    orderedSensorValues[5][1]=muxValues[2][6][6];
    orderedSensorValues[5][2]=muxValues[2][6][7];
    orderedSensorValues[5][3]=muxValues[2][6][5];
    orderedSensorValues[5][4]=muxValues[2][6][2];
    orderedSensorValues[5][5]=muxValues[2][6][1];
    orderedSensorValues[5][6]=muxValues[2][6][0];
    orderedSensorValues[5][7]=muxValues[2][6][3];
    orderedSensorValues[5][8]=muxValues[2][5][0];
    orderedSensorValues[5][9]=muxValues[2][7][0];
    orderedSensorValues[6][0]=muxValues[2][0][4];
    orderedSensorValues[6][1]=muxValues[2][0][6];
    orderedSensorValues[6][2]=muxValues[2][0][7];
    orderedSensorValues[6][3]=muxValues[2][0][5];
    orderedSensorValues[6][4]=muxValues[2][0][2];
    orderedSensorValues[6][5]=muxValues[2][0][1];
    orderedSensorValues[6][6]=muxValues[2][0][0];
    orderedSensorValues[6][7]=muxValues[2][0][3];
    orderedSensorValues[6][8]=muxValues[2][1][0];
    orderedSensorValues[6][9]=muxValues[2][2][0];
    orderedSensorValues[7][0]=muxValues[1][4][4];
    orderedSensorValues[7][1]=muxValues[1][4][6];
    orderedSensorValues[7][2]=muxValues[1][4][7];
    orderedSensorValues[7][3]=muxValues[1][4][5];
    orderedSensorValues[7][4]=muxValues[1][4][2];
    orderedSensorValues[7][5]=muxValues[1][4][1];
    orderedSensorValues[7][6]=muxValues[1][4][0];
    orderedSensorValues[7][7]=muxValues[1][4][3];
    orderedSensorValues[7][8]=muxValues[2][3][0];
    orderedSensorValues[7][9]=muxValues[1][3][0];
    orderedSensorValues[8][0]=muxValues[1][6][4];
    orderedSensorValues[8][1]=muxValues[1][6][6];
    orderedSensorValues[8][2]=muxValues[1][6][7];
    orderedSensorValues[8][3]=muxValues[1][6][5];
    orderedSensorValues[8][4]=muxValues[1][6][2];
    orderedSensorValues[8][5]=muxValues[1][6][1];
    orderedSensorValues[8][6]=muxValues[1][6][0];
    orderedSensorValues[8][7]=muxValues[1][6][3];
    orderedSensorValues[8][8]=muxValues[1][5][0];
    orderedSensorValues[8][9]=muxValues[1][7][0];
    orderedSensorValues[9][0]=muxValues[1][0][4];
    orderedSensorValues[9][1]=muxValues[1][0][6];
    orderedSensorValues[9][2]=muxValues[1][0][7];
    orderedSensorValues[9][3]=muxValues[1][0][5];
    orderedSensorValues[9][4]=muxValues[1][0][2];
    orderedSensorValues[9][5]=muxValues[1][0][1];
    orderedSensorValues[9][6]=muxValues[1][0][0];
    orderedSensorValues[9][7]=muxValues[1][0][3];
    orderedSensorValues[9][8]=muxValues[1][2][0];
    orderedSensorValues[9][9]=muxValues[1][1][0];
#endif

//================  Pasar el vector a la matriz binaria Temporal=================
int a = 0;
int b = 9;

for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinTemp[j][i] = orderedSensorValues[a][b];
        b = b - 1;
    }
    a = a + 1;
    b = 9;
}

#if PosMatrizSensor == matrizPos1
//================  Matriz en posicion 1 (Original) =================
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinVerif[i][j] = matrizBinTemp[i][j];
    }
}
#endif

#if PosMatrizSensor == matrizPos2
//================  Matriz en posicion 2 (Rota 90grados en sentido horario) =================
 a = 0;
 b = 9;
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinVerif[i][j] = matrizBinTemp[a][b];
        b = b-1;
    }
    a = a + 1;
    b = 9;
}
#endif

#if PosMatrizSensor == matrizPos3
//================  Matriz en posicion 3 (Rota 180grados en sentido horario) =================
 a = 9;
 b = 9;
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinVerif[i][j] = matrizBinTemp[a][b];
        a = a-1;
    }
    a = 9;
    b = b - 1;
}
#endif

#if PosMatrizSensor == matrizPos4
//================  Matriz en posicion 4 (Rota 270grados en sentido horario) =================
 a = 9;
 b = 0;
for (int j = 0; j < 10; j++)
{
    for (int i = 0; i < 10; i++)
    {
        matrizBinVerif[i][j] = matrizBinTemp[a][b];
        b = b + 1;
    }
    a = a - 1;
    b = 0;
}
#endif

//======================================================================
}

void testVibrador()
{
    /*
    int contVibrator = 0;
    while(contVibrator < 5)
    {
        digitalWrite(22,HIGH);
        delay(1000);
        digitalWrite(22,LOW);
        delay(1000);
        contVibrator++;
    }
    */
    digitalWrite(22,HIGH);
    delay(800);
    digitalWrite(22,LOW);
    delay(800);
}

/**================================================================================================
 *                                           moveOnTheLine
 *  Esta funcion mueve piezas a traves de las lineas del tablero para evitar chocar con otras piezas.
 *  Es necesario verificar si la pocision inicial esta fuera del area de juego ya que en esta zona es
 *  necesario hacer un movimiento extra para evitar que la gondola se delice sobre la cara interior de
 * la base del ajedrez.
 *================================================================================================**/

void moveOnTheLinev2(double xIni,double yIni,double xFin,double yFin,int Electro)
{

    int tempElectro;
    //=======Puntos intermedios para trayectoria del caballo=========
    float diferenciaX;
    float diferenciaY;
    float puntoInterX;
    float puntoInterY;

    int vectInterPointsX[5] = {-1,-1,-1,-1,-1};
    int vectInterPointsY[5] = {-1,-1,-1,-1,-1};

    //int vectVelPunto[18] = {2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000};
    //int vectVelPunto[18] = {100,500,1000,1500,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,1500,1000,500,100};
    #ifdef relacionMicroSteps4 
    //int vectVelPunto[18] = {800,1000,1600,1900,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,1900,1600,1000,800};
    //int vectVelPunto[18] = {1600,2000,3200,3800,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,3800,3200,2000,1600};
    
    //int vectVelPunto[18] = {500,1000,1500,2000,2500,3000,3000,3000,3000,3000,3000,3000,3000,2500,2000,1500,1000,500};
    //int vectVelPunto[18] = {200,500,900,1400,2000,2700,3000,3000,3000,3000,3000,3000,2700,2000,1400,900,500,200};
    //int vectVelPunto[18] = {100,600,1500,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,1500,600,100};
    //int vectVelPunto[18] = {100,1000,2700,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,2700,1000,100};
    //int vectVelPunto[18] = {100,700,2000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,2000,700,100};   //Accel 6000 speed 3000
    //int vectVelPunto[18] = {100,800,1600,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,1600,800,100};   //Accel 4600 speed 2300 Version mas rapida que funciona mejor
    //int vectVelPunto[18] = {59,766,1532,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,1532,766,59};


    #ifdef curvasV1
    int vectVelPunto[24] = {59,118,766,1532,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,1532,766,118,59};
    #endif

    #ifdef curvasV2
    //int vectVelPunto[24] = {59,559,1059,1559,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,1559,1059,559};
    //int vectVelPunto[24] = {76,726,1376,2026,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,2026,1376,726};
    //int vectVelPunto[24] = {118,1118,2118,3118,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,3118,2118,1118};

    //int vectVelPunto[24] = {4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600};
    //int vectVelPunto[24] = {500,1000,1500,2000,2500,3000,3500,4000,4500,4600,4600,4600,4600,4600,4600,4500,4000,3500,3000,2500,2000,1500,1000,500};

    
    //Funciona en curvas Largas
    //int vectVelPunto[24] = {800,1600,2400,3200,4000,4800,5600,6400,7200,8000,8000,8000,8000,8000,8000,7200,6400,5600,4800,4000,3200,2400,1600,800};
    

    int vectVelPunto[24] = {600,1200,1800,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,1800,1200,600};
    //int vectVelPunto[24] = {600,1200,1800,2400,3000,3600,4200,4800,5400,6000,6000,6000,6000,6000,6000,5400,4800,4200,3600,3000,2400,1800,1200,600};
    int vectVelOut[24] = {59,559,1059,1559,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,1559,1059,559};
    int vectVelIn[24] = {59,559,1059,1559,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,1559,1059,559};

    if(xIni <= -200 || xIni >= 200 || yIni <= -200 || yIni >= 200)
    {
        for(int i = 0; i < 24; i++)
        {
            vectVelPunto[i] = vectVelIn[i];
        }
    }
    
    if(xFin <= -200 || xFin >= 200 || yFin <= -200 || yFin >= 200)
    {
        for(int i = 0; i < 24; i++)
        {
            vectVelPunto[i] = vectVelOut[i];
        }
    }



 /*                        
    int vectVelPunto[87] = {2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,
                            2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,
                            2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300};

    int vectVelOut[87] = {2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,
                            2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,
                            2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300}; 

    int vectVelIn[87] = {2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,
                            2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,
                            2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300,2300}; 

     if(xIni <= -200 || xIni >= 200 || yIni <= -200 || yIni >= 200)
    {
        for(int i = 0; i < 87; i++)
        {
            vectVelPunto[i] = vectVelIn[i];
        }
    }
    
    if(xFin <= -200 || xFin >= 200 || yFin <= -200 || yFin >= 200)
    {
        for(int i = 0; i < 87; i++)
        {
            vectVelPunto[i] = vectVelOut[i];
        }
    }
    */
    //int vectVelPunto[24] = {400,800,1200,1600,2000,2400,2800,3200,3600,4000,4000,4000,4000,4000,4000,3600,3200,2800,2400,2000,1600,1200,800,400};

    //int vectVelPunto[24] = {8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000};
/*
    int vectVelPunto[87] = {160,320,480,640,800,960,1120,1280,1440,1600,1760,1920,2080,2240,2400,2560,2720,2880,3040,3200,3360,3520,3680,3840,4000,4160,4320,4480,4600,
                            4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,4600,
                            4600,4480,4320,4160,4000,3840,3680,3520,3360,3200,3040,2880,2720,2560,2400,2240,2080,1920,1760,1600,1440,1280,1120,960,800,640,480,320,160
                            };
                            */
/*
            int vectVelPunto[273] = {300, 600, 900, 1200, 1500, 1800, 2100, 2400, 2700, 3000, 3300, 3600, 3900, 4200, 4500, 4800, 5100, 5400, 5700, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000,6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000,6000,
                             6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000,6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000,6000,
                             6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 6000, 5700, 5400, 5100, 4800, 4500, 4200, 3900, 3600,3300, 3000, 2700, 2400, 2100, 1800, 1500, 1200, 900, 600,300
    };
    */
    #endif


    //int vectVelPunto[24] = {103,206,1340,2681,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,2681,1340,206,103};
    //int vectVelPunto[24] = {75,150,1005,2010,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,2010,1005,150,75};
    #endif

    #ifdef relacionMicroSteps16 
    //int vectVelPunto[18] = {3200,4000,6400,7600,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,7600,6400,4000,3200};
    //int vectVelPunto[18] = {800,1000,1600,1900,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,1900,1600,1000,800};
    //int vectVelPunto[18] = {1600,2000,3200,3800,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,3800,3200,2000,1600};
    ////////////int vectVelPunto[18] = {2400,3000,4800,5700,6000,6000,6000,6000,6000,6000,6000,6000,6000,6000,5700,4800,3000,2400};
    //int vectVelPunto[18] = {3200,4000,6400,7600,8000,8000,8000,8000,8000,8000,8000,8000,8000,8000,7600,6400,4000,3200};
    //int vectVelPunto[18] = {4800,6000,9600,11400,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,11400,9600,6000,4800};
    //int vectVelPunto[18] = {4000,5000,8000,9500,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,9500,8000,5000,4000};
    
    int vectVelPunto[18] = {400,2800,8000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,12000,8000,2800,400};
    
    #endif

    #ifdef relacionMicroSteps32 
    int vectVelPunto[18] = {800,1000,1600,1900,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,1900,1600,1000,800};
    #endif

    //Funcion para recorrer mas area del escaque
    //==========================================
    checkAreaChess(xIni, yIni, 6);
    //==========================================
    
    diferenciaX = abs(xIni - xFin);
    diferenciaY = abs(yIni - yFin);

    vectInterPointsX[0] = xIni;
    vectInterPointsY[0] = yIni;
    
    /*================================================================================================*/
    if(diferenciaX > diferenciaY)
    {
        if(yFin < yIni) 
        {
                puntoInterX = xIni;
                puntoInterY = yIni - 25;
                #ifdef funcMoveTo
                //Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                //Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
                vectInterPointsX[1] = puntoInterX;
                vectInterPointsY[1] = puntoInterY;

        }
        //Aqui me falta la condicion en caso de que las coordenadas Y sean las mismas
        if(yFin > yIni)    
        {
                puntoInterX = xIni;
                puntoInterY = yIni + 25;
                #ifdef funcMoveTo
                //Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                //Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
                vectInterPointsX[1] = puntoInterX;
                vectInterPointsY[1] = puntoInterY;
        }
        //=================================================================================
        if(xFin < xIni)
        {
            puntoInterX = xFin + 25;
            #ifdef funcMoveTo
            //Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            //Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
            vectInterPointsX[2] = puntoInterX;
            vectInterPointsY[2] = puntoInterY;
        }
        //Aqui me falta la condicion en caso de que las coordenadas X sean las mismas
        if(xFin > xIni)
        {
            puntoInterX = xFin - 25;
            #ifdef funcMoveTo
            //Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            //Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
            vectInterPointsX[2] = puntoInterX;
            vectInterPointsY[2] = puntoInterY;
        }
        //=================================================================================
        puntoInterY = yFin;
        #ifdef funcMoveTo
        //Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        //Robot.accelRamp(puntoInterX, puntoInterY,2);
        #endif
        vectInterPointsX[3] = puntoInterX;
        vectInterPointsY[3] = puntoInterY;
    }
    else
    {
       if(xFin < xIni) 
        {
                puntoInterX = xIni - 25;
                puntoInterY = yIni;
                #ifdef funcMoveTo
                //Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                //Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
                vectInterPointsX[1] = puntoInterX;
                vectInterPointsY[1] = puntoInterY;
        }
        //Aqui me falta la condicion en caso de que las coordenadas X sean las mismas
        if(xFin > xIni)    
        {
                puntoInterX = xIni + 25;
                puntoInterY = yIni;
                #ifdef funcMoveTo
                //Robot.moveToPointV2(puntoInterX, puntoInterY);
                #endif
                #ifdef funcAccelRamp
                //Robot.accelRamp(puntoInterX, puntoInterY,2);
                #endif
                vectInterPointsX[1] = puntoInterX;
                vectInterPointsY[1] = puntoInterY;
        }
        //=================================================================================
        if(yFin < yIni)
        {
            puntoInterY = yFin + 25;
            #ifdef funcMoveTo
            //Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            //Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
            vectInterPointsX[2] = puntoInterX;
            vectInterPointsY[2] = puntoInterY;
        }
        //Aqui me falta la condicion en caso de que las coordenadas X sean las mismas
        if(yFin > yIni)
        {
            puntoInterY = yFin - 25;
            #ifdef funcMoveTo
            //Robot.moveToPointV2(puntoInterX, puntoInterY);
            #endif
            #ifdef funcAccelRamp
            //Robot.accelRamp(puntoInterX, puntoInterY,2);
            #endif
            vectInterPointsX[2] = puntoInterX;
            vectInterPointsY[2] = puntoInterY;
        }
        //=================================================================================
        puntoInterX = xFin;
        #ifdef funcMoveTo
        //Robot.moveToPointV2(puntoInterX, puntoInterY);
        #endif
        #ifdef funcAccelRamp
        //Robot.accelRamp(puntoInterX, puntoInterY,2);
        #endif 
        vectInterPointsX[3] = puntoInterX;
        vectInterPointsY[3] = puntoInterY;
    }
    //===============================================================



    #ifdef centrarElectromagnet
    double compXanterior = 0;
    double compYanterior = 0;
    
    compXanterior = xFin;
    compYanterior = yFin;

    double desfase;
    desfase = desfaseElectroiman;
    if((puntoInterX != xFin)&&(puntoInterY != yFin))
    {
        desfase = sqrt((desfaseElectroiman*desfaseElectroiman)/2);
    }

    if(puntoInterX > xFin)
    {
        compXanterior = compXanterior + desfase;
    }
    if(puntoInterX < xFin)
    {
        compXanterior = compXanterior - desfase;
    }

    if(puntoInterY > yFin)
    {
        compYanterior = compYanterior + desfase;
    }
    if(puntoInterY < yFin)
    {
        compYanterior = compYanterior - desfase;
    }
    #ifdef funcMoveTo
    //Robot.moveToPointV2(compXanterior,compYanterior,1);
    #endif

    #ifdef funcMoveTo
    testPWMElectromagnetDeactivate();
    #endif

    #endif


    #ifdef funcMoveTo
    //Robot.moveToPointV2(xFin,yFin);
    #endif
    #ifdef funcAccelRamp
    //Robot.accelRamp(xFin,yFin,2,1);
    #endif
    vectInterPointsX[4] = xFin;
    vectInterPointsY[4] = yFin;

    #ifdef centrarElectromagnet
    //activateElectromagnet();
    #endif


    if(diferenciaX < diferenciaY)
    {
        if(diferenciaX == 50)
        {
            vectInterPointsX[2] = vectInterPointsX[1];
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
        if(diferenciaX == 0)
        {
            //El caso donde estan en la misma columna del tablero
            vectInterPointsX[1] = xIni + 25;
            vectInterPointsY[1] = yIni;

            vectInterPointsX[2] = vectInterPointsX[1] ;
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
    }
    if(diferenciaX > diferenciaY)
    {
        if(diferenciaY == 50)
        {
            vectInterPointsX[2] = xFin;
            vectInterPointsY[2] = vectInterPointsY[1];

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
        if(diferenciaY == 0)
        {
            //El caso donde estan en la misma columna del tablero

            vectInterPointsX[1] = xIni;
            vectInterPointsY[1] = yIni + 25;

            vectInterPointsX[2] = xFin;
            vectInterPointsY[2] = vectInterPointsY[1];

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;

        }
    }
    if(diferenciaX == 50 && diferenciaY == 50)
    {
        if(xIni < xFin)
        {
            vectInterPointsX[1] = xIni + 25;
            vectInterPointsY[1] = yIni;

            vectInterPointsX[2] = vectInterPointsX[1];
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;

        }
        else
        {
            vectInterPointsX[1] = xIni - 25;
            vectInterPointsY[1] = yIni;

            vectInterPointsX[2] = vectInterPointsX[1];
            vectInterPointsY[2] = yFin;

            vectInterPointsX[3] = xFin;
            vectInterPointsY[3] = yFin;

            vectInterPointsX[4] = -1;
            vectInterPointsY[4] = -1;
        }
    }
/*
    Serial.println("Coordenadas del movimiento entre lineas");
    for(int i = 0; i < 5; i++)
    {
        Serial.print(vectInterPointsX[i]);
        Serial.print("\t");
        Serial.println(vectInterPointsY[i]);
    }
*/

    double xC0, yC0;
    double* apXc0 = &xC0;
    double* apYc0 = &yC0;

    double xC1, yC1;
    double* apXc1 = &xC1;
    double* apYc1 = &yC1;

    double xC2, yC2;
    double* apXc2 = &xC2;
    double* apYc2 = &yC2;

    double xC3, yC3;
    double* apXc3 = &xC3;
    double* apYc3 = &yC3;

    double xC4, yC4;
    double* apXc4 = &xC4;
    double* apYc4 = &yC4;

    double xC5, yC5;
    double* apXc5 = &xC5;
    double* apYc5 = &yC5;

    double xC6, yC6;
    double* apXc6 = &xC6;
    double* apYc6 = &yC6;

    double xC7, yC7;
    double* apXc7 = &xC7;
    double* apYc7 = &yC7;

    int totalPuntosEnCurva = 7;   // Antes 7

    double arrayPuntosX[totalPuntosEnCurva + 1] = {0};   //Se contempla un elemento mas del vector para contemplar desde el punto inicial hasta el punto final de la curva
    double arrayPuntosY[totalPuntosEnCurva + 1] = {0};

    int indexSpeed = 0;

    //Se utiliza xC1  yC1
    int sizeVectInter = 0;
    if(vectInterPointsX[4] == -1  || vectInterPointsY[4] == -1)
    {
        sizeVectInter = 6;
    }
    else
    {
        sizeVectInter = 8;
    }

    double vectFinalInterX[sizeVectInter];
    double vectFinalInterY[sizeVectInter];

    if(sizeVectInter == 6)
    {
        #ifdef version1Electro
        testPWMElectromagnetActivate();
        #endif

        #ifdef version4Electro
        testPWMElectromagnetActivateV4E(Electro);
        #endif

        vectFinalInterX[0] = vectInterPointsX[0];
        vectFinalInterY[0] = vectInterPointsY[0];
        //Robot.setSpeedMotors(vectVelPunto[0]);
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);
        #ifdef funcMoveTo
        Robot.moveToPointV2(vectFinalInterX[0],vectFinalInterY[0]);
        #endif


        #ifdef funcAccelRamp
        #ifdef version1Electro
        Robot.accelRamp(vectFinalInterX[0],vectFinalInterY[0],0);
        #endif
        #ifdef version4Electro
        Robot.accelRamp(vectFinalInterX[0],vectFinalInterY[0],0,0,Electro);
        #endif
        #endif

        #ifdef curvasV1
        puntosDeCurva(vectInterPointsX[0],vectInterPointsY[0],vectInterPointsX[1],vectInterPointsY[1],vectInterPointsX[2],vectInterPointsY[2],apXc0,apYc0,apXc1,apYc1,apXc2,apYc2,apXc3,apYc3,apXc4,apYc4,apXc5,apYc5,apXc6,apYc6,apXc7,apYc7);
        #endif

        #ifdef curvasV2
        puntosDeCurvaV2(vectInterPointsX[0],vectInterPointsY[0],vectInterPointsX[1],vectInterPointsY[1],vectInterPointsX[2],vectInterPointsY[2],arrayPuntosX,arrayPuntosY,totalPuntosEnCurva);
        #endif

        #ifdef funcMoveTo
        Robot.moveToPointV2(xC1,yC1);
        Robot.moveToPointV2(xC2,yC2);
        Robot.moveToPointV2(xC3,yC3);
        Robot.moveToPointV2(xC4,yC4);
        Robot.moveToPointV2(xC5,yC5);
        #endif
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.setSpeedMotors(vectVelPunto[1]);
        Robot.accelRamp(xC1,yC1,0);
        Robot.setSpeedMotors(vectVelPunto[2]);
        Robot.accelRamp(xC2,yC2,0);
        Robot.setSpeedMotors(vectVelPunto[3]);
        Robot.accelRamp(xC3,yC3,0);
        Robot.setSpeedMotors(vectVelPunto[4]);
        Robot.accelRamp(xC4,yC4,0);
        Robot.setSpeedMotors(vectVelPunto[5]);
        Robot.accelRamp(xC5,yC5,0);
        Robot.setSpeedMotors(vectVelPunto[6]);
        Robot.accelRamp(xC6,yC6,0);
        Robot.setSpeedMotors(vectVelPunto[7]);
        Robot.accelRamp(xC7,yC7,0);
        #endif

        #ifdef version4Electro

        #ifdef curvasV1
        Robot.setSpeedMotors(vectVelPunto[1]);
        Robot.accelRamp(xC1,yC1,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[2]);
        Robot.accelRamp(xC2,yC2,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[3]);
        Robot.accelRamp(xC3,yC3,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[4]);
        Robot.accelRamp(xC4,yC4,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[5]);
        Robot.accelRamp(xC5,yC5,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[6]);
        Robot.accelRamp(xC6,yC6,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[7]);
        Robot.accelRamp(xC7,yC7,0,0,Electro);
        #endif

        #ifdef curvasV2
        /*
        Robot.setSpeedMotors(vectVelPunto[1]);
        Robot.accelRamp(arrayPuntosX[1],arrayPuntosY[1],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[2]);
        Robot.accelRamp(arrayPuntosX[2],arrayPuntosY[2],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[3]);
        Robot.accelRamp(arrayPuntosX[3],arrayPuntosY[3],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[4]);
        Robot.accelRamp(arrayPuntosX[4],arrayPuntosY[4],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[5]);
        Robot.accelRamp(arrayPuntosX[5],arrayPuntosY[5],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[6]);
        Robot.accelRamp(arrayPuntosX[6],arrayPuntosY[6],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[7]);
        Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],0,0,Electro);
        */
       for(int i = 0; i < totalPuntosEnCurva; i++)
       {
        indexSpeed++;
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);
        Robot.accelRamp(arrayPuntosX[i+1],arrayPuntosY[i+1],0,0,Electro);
       }
        #endif

        #endif

        #endif

#ifdef version4Electro
if((xFin >= -200 &&  xFin <= 200)&&(yFin >= -200 && yFin <= 200))
{
    //Continua Normal
}
else
{
    if(xFin <= -200)       //Si va a sacar del lado izquierdo
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC7,yC7,2,0,4);
        #endif
        #ifdef curvasV2
        //Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],2,0,4);
        Robot.accelRamp(arrayPuntosX[totalPuntosEnCurva],arrayPuntosY[totalPuntosEnCurva],2,0,4);
        #endif
        activateElectromagnetV4E(4);
        Electro = 4;
    }
    if(xFin >= 200)        //Si va a sacar del lado derecho
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC7,yC7,2,0,2);
        #endif
        #ifdef curvasV2
        //Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],2,0,2);
        Robot.accelRamp(arrayPuntosX[totalPuntosEnCurva],arrayPuntosY[totalPuntosEnCurva],2,0,2);
        #endif
        activateElectromagnetV4E(2);
        Electro = 2;
    }
    if(yFin <= -200)       //Si va a sacar del lado inferior
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC7,yC7,2,0,3);
        #endif
        #ifdef curvasV2
        //Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],2,0,3);
        Robot.accelRamp(arrayPuntosX[totalPuntosEnCurva],arrayPuntosY[totalPuntosEnCurva],2,0,3);
        #endif
        activateElectromagnetV4E(3);
        Electro = 3;
    }
    if(yFin >= 200)        //Si va a sacar del lado superior
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC7,yC7,2,0,1);
        #endif
        #ifdef curvasV2
        //Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],2,0,1);
        Robot.accelRamp(arrayPuntosX[totalPuntosEnCurva],arrayPuntosY[totalPuntosEnCurva],2,0,1);
        #endif
        activateElectromagnetV4E(1);
        Electro = 1;
    }     
}
#endif


//if((xFin > -200 &&  xFin < 200)&&(yFin > -200 && yFin < 200))
//{
        #ifdef curvasV1
        puntosDeCurva(vectInterPointsX[1],vectInterPointsY[1],vectInterPointsX[2],vectInterPointsY[2],vectInterPointsX[3],vectInterPointsY[3],apXc0,apYc0,apXc1,apYc1,apXc2,apYc2,apXc3,apYc3,apXc4,apYc4,apXc5,apYc5,apXc6,apYc6,apXc7,apYc7);
        #endif

        #ifdef curvasV2
        puntosDeCurvaV2(vectInterPointsX[1],vectInterPointsY[1],vectInterPointsX[2],vectInterPointsY[2],vectInterPointsX[3],vectInterPointsY[3],arrayPuntosX,arrayPuntosY,totalPuntosEnCurva);
        #endif

        #ifdef funcMoveTo
        Robot.moveToPointV2(xC0,yC0);
        Robot.moveToPointV2(xC1,yC1);
        Robot.moveToPointV2(xC2,yC2);
        Robot.moveToPointV2(xC3,yC3);
        #endif


        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.setSpeedMotors(vectVelPunto[16]);
        Robot.accelRamp(xC0,yC0,0);
        Robot.setSpeedMotors(vectVelPunto[17]);
        Robot.accelRamp(xC1,yC1,0);
        Robot.setSpeedMotors(vectVelPunto[18]);
        Robot.accelRamp(xC2,yC2,0);
        Robot.setSpeedMotors(vectVelPunto[19]);
        Robot.accelRamp(xC3,yC3,0);
        Robot.setSpeedMotors(vectVelPunto[20]);
        Robot.accelRamp(xC4,yC4,0);
        #endif

        #ifdef version4Electro

        #ifdef curvasV1
        Robot.setSpeedMotors(vectVelPunto[16]);
        Robot.accelRamp(xC0,yC0,0,0,Electro);
        #endif
        #ifdef curvasV2
        indexSpeed = ((totalPuntosEnCurva+1)*3) - (totalPuntosEnCurva+1);
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);
        //Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],0,0,Electro);
        if ((xFin <= -200 || xFin >= 200) || (yFin <= -200 || yFin >= 200)) // Condicion, si va a sacar una pieza
        {
            if(diferenciaX < 60 || diferenciaY < 60)
            {
                Robot.accelRamp(arrayPuntosX[0], arrayPuntosY[0], 0, 0, Electro);
            }
            else
            {
                Robot.accelRamp(arrayPuntosX[0], arrayPuntosY[0], 1, 0, Electro); // En caso de sacar la pieza se dirige a este punto en aceleracion
            }
            
        }
        else
        {
            if((xIni <= -200 || xIni >= 200) || (yIni <= -200 || yIni >= 200))
            {
                if(diferenciaX < 60 || diferenciaY < 60)
                {
                    Robot.accelRamp(arrayPuntosX[0], arrayPuntosY[0], 0, 0, Electro);
                }
                else
                {
                    Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],-1,0,Electro);
                }
            }
            else
            {
            Robot.accelRamp(arrayPuntosX[0], arrayPuntosY[0], 0, 0, Electro);
            }
        }
        
        #endif

#ifdef version4Electro
if((xIni >= -200 &&  xIni <= 200)&&(yIni >= -200 && yIni <= 200))
{
    //Continua Normal
}
else
{
    if(xIni <= -200)       //Si va a sacar del lado izquierdo
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC0,yC0,2,0,mainElectro);
        #endif
        #ifdef curvasV2
        Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],2,0,mainElectro);        
        #endif
        activateElectromagnetV4E(mainElectro);
        Electro = mainElectro;
    }
    if(xIni >= 200)        //Si va a sacar del lado derecho
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC0,yC0,2,0,mainElectro);
        #endif
        #ifdef curvasV2
        Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],2,0,mainElectro);
        #endif
        activateElectromagnetV4E(mainElectro);
        Electro = mainElectro;
    }
    if(yIni <= -200)       //Si va a sacar del lado inferior
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC0,yC0,2,0,mainElectro);
        #endif
        #ifdef curvasV2
        Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],2,0,mainElectro);
        #endif
        activateElectromagnetV4E(mainElectro);
        Electro = mainElectro;
    }
    if(yIni >= 200)        //Si va a sacar del lado superior
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC0,yC0,2,0,mainElectro);
        #endif
        #ifdef curvasV2
        Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],2,0,mainElectro);
        #endif
        activateElectromagnetV4E(mainElectro);
        Electro = mainElectro;
    }     
}
#endif

        #ifdef curvasV1
        Robot.setSpeedMotors(vectVelPunto[17]);
        Robot.accelRamp(xC1,yC1,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[18]);
        Robot.accelRamp(xC2,yC2,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[19]);
        Robot.accelRamp(xC3,yC3,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[20]);
        Robot.accelRamp(xC4,yC4,0,0,Electro);
        #endif
        #ifdef curvasV2
        /*
        Robot.setSpeedMotors(vectVelPunto[17]);
        Robot.accelRamp(arrayPuntosX[1],arrayPuntosY[1],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[18]);
        Robot.accelRamp(arrayPuntosX[2],arrayPuntosY[2],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[19]);
        Robot.accelRamp(arrayPuntosX[3],arrayPuntosY[3],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[20]);
        Robot.accelRamp(arrayPuntosX[4],arrayPuntosY[4],0,0,Electro);
        */
        #endif

        #endif

        #endif

        #ifdef funcMoveTo
        Robot.moveToPointV2(xC4,yC4);
        testPWMElectromagnetDeactivate();
        #endif
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.setSpeedMotors(vectVelPunto[21]);
        Robot.accelRamp(xC5,yC5,0);
        Robot.setSpeedMotors(vectVelPunto[22]);
        Robot.accelRamp(xC6,yC6,0);
        #endif

        #ifdef version4Electro

        #ifdef curvasV1
        Robot.setSpeedMotors(vectVelPunto[21]);
        Robot.accelRamp(xC5,yC5,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[22]);
        Robot.accelRamp(xC6,yC6,0,0,Electro);
        #endif
        #ifdef curvasV2
        /*
        Robot.setSpeedMotors(vectVelPunto[21]);
        Robot.accelRamp(arrayPuntosX[5],arrayPuntosY[5],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[22]);
        Robot.accelRamp(arrayPuntosX[6],arrayPuntosY[6],0,0,Electro);
        */

       for(int i = 0; i < totalPuntosEnCurva-1; i++)
       {
        indexSpeed++;
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);
        Robot.accelRamp(arrayPuntosX[i+1],arrayPuntosY[i+1],0,0,Electro);
       }
        #endif

        #endif
        
        #ifdef version1Electro
        testPWMElectromagnetDeactivate();
        #endif

        #ifdef version4Electro
        testPWMElectromagnetDeactivateV4E(Electro);
        #endif

        #endif

        vectFinalInterX[5] = vectInterPointsX[3];
        vectFinalInterY[5] = vectInterPointsY[3];

        #ifdef funcMoveTo
        Robot.moveToPointV2(vectFinalInterX[5],vectFinalInterY[5]);
        activateElectromagnet();
        #endif

        #ifdef funcAccelRamp
        //Robot.setSpeedMotors(vectVelPunto[23]);
        indexSpeed++;
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);

        #ifdef version1Electro
        Robot.accelRamp(vectFinalInterX[5],vectFinalInterY[5],0);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(vectFinalInterX[5],vectFinalInterY[5],0,0,Electro);
        #endif

        #ifdef version1Electro
        activateElectromagnet();
        #endif

        #ifdef version4Electro
        activateElectromagnetV4E(Electro);
        #endif

        #endif

        #ifdef version1Electro
        deactivateElectromagnet();
        #endif

        #ifdef version4Electro
        deactivateElectromagnetV4E(Electro);
        #endif

#ifdef version4Electro
if((xFin >= -200 &&  xFin <= 200)&&(yFin >= -200 && yFin <= 200))
{
    //Continua Normal
}
else
{
    Electro = tempElectro;

}
#endif

#ifdef version4Electro
if((xIni >= -200 &&  xIni <= 200)&&(yIni >= -200 && yIni <= 200))
{
    //Continua Normal
}
else
{
    Electro = tempElectro;

}
#endif

    }

    if(sizeVectInter == 8)
    {
        #ifdef version1Electro
        testPWMElectromagnetActivate();
        #endif

        #ifdef version4Electro
        testPWMElectromagnetActivateV4E(Electro);
        #endif

        vectFinalInterX[0] = vectInterPointsX[0];
        vectFinalInterY[0] = vectInterPointsY[0];
        #ifdef funcMoveTo
        Robot.moveToPointV2(vectFinalInterX[0],vectFinalInterY[0]);
        #endif
        #ifdef funcAccelRamp
        Robot.setSpeedMotors(vectVelPunto[0]);
        Robot.setSpeedRampFunction(vectVelPunto[0]);
        Robot.setAccelRampFunction((vectVelPunto[0])*2);

        #ifdef version1Electro
        Robot.accelRamp(vectFinalInterX[0],vectFinalInterY[0],0);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(vectFinalInterX[0],vectFinalInterY[0],0,0,Electro);
        #endif

        #endif

        #ifdef curvasV1
        puntosDeCurva(vectInterPointsX[0],vectInterPointsY[0],vectInterPointsX[1],vectInterPointsY[1],vectInterPointsX[2],vectInterPointsY[2],apXc0,apYc0,apXc1,apYc1,apXc2,apYc2,apXc3,apYc3,apXc4,apYc4,apXc5,apYc5,apXc6,apYc6,apXc7,apYc7);
        #endif

        #ifdef curvasV2
        puntosDeCurvaV2(vectInterPointsX[0],vectInterPointsY[0],vectInterPointsX[1],vectInterPointsY[1],vectInterPointsX[2],vectInterPointsY[2],arrayPuntosX,arrayPuntosY,totalPuntosEnCurva);
        #endif
        
        #ifdef funcMoveTo
        Robot.moveToPointV2(xC1,yC1);
        Robot.moveToPointV2(xC2,yC2);
        Robot.moveToPointV2(xC3,yC3);
        Robot.moveToPointV2(xC4,yC4);
        Robot.moveToPointV2(xC5,yC5);
        #endif

        #ifdef funcAccelRamp
        
        #ifdef version1Electro
        Robot.setSpeedMotors(vectVelPunto[1]);
        Robot.accelRamp(xC1,yC1,0);
        Robot.setSpeedMotors(vectVelPunto[2]);
        Robot.accelRamp(xC2,yC2,0);
        Robot.setSpeedMotors(vectVelPunto[3]);
        Robot.accelRamp(xC3,yC3,0);
        Robot.setSpeedMotors(vectVelPunto[4]);
        Robot.accelRamp(xC4,yC4,0);
        Robot.setSpeedMotors(vectVelPunto[5]);
        Robot.accelRamp(xC5,yC5,0);
        Robot.setSpeedMotors(vectVelPunto[6]);
        Robot.accelRamp(xC6,yC6,0);
        Robot.setSpeedMotors(vectVelPunto[7]);
        Robot.accelRamp(xC7,yC7,0);
        #endif

        #ifdef version4Electro

        #ifdef curvasV1
        Robot.setSpeedMotors(vectVelPunto[1]);
        Robot.accelRamp(xC1,yC1,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[2]);
        Robot.accelRamp(xC2,yC2,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[3]);
        Robot.accelRamp(xC3,yC3,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[4]);
        Robot.accelRamp(xC4,yC4,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[5]);
        Robot.accelRamp(xC5,yC5,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[6]);
        Robot.accelRamp(xC6,yC6,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[7]);
        Robot.accelRamp(xC7,yC7,0,0,Electro);
        #endif
        #ifdef curvasV2
        /*
        Robot.setSpeedMotors(vectVelPunto[1]);
        Robot.accelRamp(arrayPuntosX[1],arrayPuntosY[1],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[2]);
        Robot.accelRamp(arrayPuntosX[2],arrayPuntosY[2],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[3]);
        Robot.accelRamp(arrayPuntosX[3],arrayPuntosY[3],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[4]);
        Robot.accelRamp(arrayPuntosX[4],arrayPuntosY[4],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[5]);
        Robot.accelRamp(arrayPuntosX[5],arrayPuntosY[5],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[6]);
        Robot.accelRamp(arrayPuntosX[6],arrayPuntosY[6],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[7]);
        Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],0,0,Electro);
        */
       for(int i = 0; i < totalPuntosEnCurva; i++)
       {
        indexSpeed++;
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);
        Robot.accelRamp(arrayPuntosX[i+1],arrayPuntosY[i+1],0,0,Electro);
       }
        #endif

        #endif

        #endif

#ifdef version4Electro
if((xFin >= -200 &&  xFin <= 200)&&(yFin >= -200 && yFin <= 200))
{
    //Continua Normal
}
else
{
    if(xFin <= -200)       //Si va a sacar del lado izquierdo
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC7,yC7,2,0,4);
        #endif
        #ifdef curvasV2
        //Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],2,0,4);
        Robot.accelRamp(arrayPuntosX[totalPuntosEnCurva],arrayPuntosY[totalPuntosEnCurva],2,0,4);
        #endif
        activateElectromagnetV4E(4);
        Electro = 4;
    }
    if(xFin >= 200)        //Si va a sacar del lado derecho
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC7,yC7,2,0,2);
        #endif
        #ifdef curvasV2
        //Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],2,0,2);
        Robot.accelRamp(arrayPuntosX[totalPuntosEnCurva],arrayPuntosY[totalPuntosEnCurva],2,0,2);
        #endif
        activateElectromagnetV4E(2);
        Electro = 2;
    }
    if(yFin <= -200)       //Si va a sacar del lado inferior
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC7,yC7,2,0,3);
        #endif
        #ifdef curvasV2
        //Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],2,0,3);
        Robot.accelRamp(arrayPuntosX[totalPuntosEnCurva],arrayPuntosY[totalPuntosEnCurva],2,0,3);
        #endif
        activateElectromagnetV4E(3);
        Electro = 3;
    }
    if(yFin >= 200)        //Si va a sacar del lado superior
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC7,yC7,2,0,1);
        #endif
        #ifdef curvasV2
        //Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],2,0,1);
        Robot.accelRamp(arrayPuntosX[totalPuntosEnCurva],arrayPuntosY[totalPuntosEnCurva],2,0,1);
        #endif
        activateElectromagnetV4E(1);
        Electro = 1;
    }     
}
#endif

        #ifdef curvasV1
        puntosDeCurva(vectInterPointsX[1],vectInterPointsY[1],vectInterPointsX[2],vectInterPointsY[2],vectInterPointsX[3],vectInterPointsY[3],apXc0,apYc0,apXc1,apYc1,apXc2,apYc2,apXc3,apYc3,apXc4,apYc4,apXc5,apYc5,apXc6,apYc6,apXc7,apYc7);
        #endif

        #ifdef curvasV2
        puntosDeCurvaV2(vectInterPointsX[1],vectInterPointsY[1],vectInterPointsX[2],vectInterPointsY[2],vectInterPointsX[3],vectInterPointsY[3],arrayPuntosX,arrayPuntosY,totalPuntosEnCurva);
        #endif
        
        #ifdef funcMoveTo
        Robot.moveToPointV2(xC0,yC0);
        Robot.moveToPointV2(xC1,yC1);
        Robot.moveToPointV2(xC2,yC2);
        Robot.moveToPointV2(xC3,yC3);
        Robot.moveToPointV2(xC4,yC4);
        Robot.moveToPointV2(xC5,yC5);
        #endif
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.setSpeedMotors(vectVelPunto[8]);
        Robot.accelRamp(xC0,yC0,0);
        Robot.setSpeedMotors(vectVelPunto[9]);
        Robot.accelRamp(xC1,yC1,0);
        Robot.setSpeedMotors(vectVelPunto[10]);
        Robot.accelRamp(xC2,yC2,0);
        Robot.setSpeedMotors(vectVelPunto[11]);
        Robot.accelRamp(xC3,yC3,0);
        Robot.setSpeedMotors(vectVelPunto[12]);
        Robot.accelRamp(xC4,yC4,0);
        Robot.setSpeedMotors(vectVelPunto[13]);
        Robot.accelRamp(xC5,yC5,0);
        Robot.setSpeedMotors(vectVelPunto[14]);
        Robot.accelRamp(xC6,yC6,0);
        Robot.setSpeedMotors(vectVelPunto[15]);
        Robot.accelRamp(xC7,yC7,0);
        #endif

        #ifdef version4Electro

        #ifdef curvasV1
        Robot.setSpeedMotors(vectVelPunto[8]);
        Robot.accelRamp(xC0,yC0,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[9]);
        Robot.accelRamp(xC1,yC1,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[10]);
        Robot.accelRamp(xC2,yC2,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[11]);
        Robot.accelRamp(xC3,yC3,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[12]);
        Robot.accelRamp(xC4,yC4,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[13]);
        Robot.accelRamp(xC5,yC5,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[14]);
        Robot.accelRamp(xC6,yC6,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[15]);
        Robot.accelRamp(xC7,yC7,0,0,Electro);
        #endif
        #ifdef curvasV2
        /*
        Robot.setSpeedMotors(vectVelPunto[8]);
        Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[9]);
        Robot.accelRamp(arrayPuntosX[1],arrayPuntosY[1],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[10]);
        Robot.accelRamp(arrayPuntosX[2],arrayPuntosY[2],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[11]);
        Robot.accelRamp(arrayPuntosX[3],arrayPuntosY[3],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[12]);
        Robot.accelRamp(arrayPuntosX[4],arrayPuntosY[4],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[13]);
        Robot.accelRamp(arrayPuntosX[5],arrayPuntosY[5],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[14]);
        Robot.accelRamp(arrayPuntosX[6],arrayPuntosY[6],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[15]);
        Robot.accelRamp(arrayPuntosX[7],arrayPuntosY[7],0,0,Electro);
        */
       for(int i = 0; i <= totalPuntosEnCurva; i++)
       {
        indexSpeed++;
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);
        if (i == 0)
        {
            if ((xFin <= -200 || xFin >= 200) || (yFin <= -200 || yFin >= 200)) // Condicion, si va a sacar una pieza
            {
                Robot.accelRamp(arrayPuntosX[i], arrayPuntosY[i], 1, 0, Electro); // En caso de sacar la pieza se dirige a este punto en aceleracion
            }
            else
            {
                Robot.accelRamp(arrayPuntosX[i], arrayPuntosY[i], 0, 0, Electro);
            }
        }
        else
        {
            Robot.accelRamp(arrayPuntosX[i],arrayPuntosY[i],0,0,Electro);
        }
        
       }
        #endif

        #endif


        #endif
//if((xFin > -200 &&  xFin < 200)&&(yFin > -200 && yFin < 200))
//{
        #ifdef curvasV1
        puntosDeCurva(vectInterPointsX[2],vectInterPointsY[2],vectInterPointsX[3],vectInterPointsY[3],vectInterPointsX[4],vectInterPointsY[4],apXc0,apYc0,apXc1,apYc1,apXc2,apYc2,apXc3,apYc3,apXc4,apYc4,apXc5,apYc5,apXc6,apYc6,apXc7,apYc7);
        #endif
        
        #ifdef curvasV2
        puntosDeCurvaV2(vectInterPointsX[2],vectInterPointsY[2],vectInterPointsX[3],vectInterPointsY[3],vectInterPointsX[4],vectInterPointsY[4],arrayPuntosX,arrayPuntosY,totalPuntosEnCurva);
        #endif

        #ifdef funcMoveTo
        Robot.moveToPointV2(xC0,yC0);
        Robot.moveToPointV2(xC1,yC1);
        Robot.moveToPointV2(xC2,yC2);
        Robot.moveToPointV2(xC3,yC3);
        #endif



#ifdef funcAccelRamp

#ifdef version1Electro
        Robot.setSpeedMotors(vectVelPunto[16]);
        Robot.accelRamp(xC0,yC0,0);
        Robot.setSpeedMotors(vectVelPunto[17]);
        Robot.accelRamp(xC1,yC1,0);
        Robot.setSpeedMotors(vectVelPunto[18]);
        Robot.accelRamp(xC2,yC2,0);
        Robot.setSpeedMotors(vectVelPunto[19]);
        Robot.accelRamp(xC3,yC3,0);
        Robot.setSpeedMotors(vectVelPunto[20]);
        Robot.accelRamp(xC4,yC4,0);
        #endif

        #ifdef version4Electro
        
        #ifdef curvasV1
        Robot.setSpeedMotors(vectVelPunto[16]);
        Robot.accelRamp(xC0,yC0,0,0,Electro);
        #endif
        #ifdef curvasV2
        //Robot.setSpeedMotors(vectVelPunto[16]);
        //Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],0,0,Electro);

        indexSpeed++;
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);
        if((xIni <= -200 || xIni >= 200) || (yIni <= -200 || yIni >= 200))
        {
            Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],-1,0,Electro);
        }
        else
        {
            Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0],0,0,Electro);
        }
        #endif

#ifdef version4Electro
        if ((xIni >= -200 && xIni <= 200) && (yIni >= -200 && yIni <= 200))
        {
    // Continua Normal
        }
        else
        {
    if (xIni <= -200) // Si va a meter desde el lado izquierdo
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC0, yC0, 2, 0, mainElectro);
        #endif
        #ifdef curvasV2
        Robot.accelRamp(arrayPuntosX[0],arrayPuntosY[0], 2, 0, mainElectro);
        #endif
        activateElectromagnetV4E(mainElectro);
        Electro = mainElectro;
    }
    if (xIni >= 200) // Si va a meter desde el lado derecho
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC0, yC0, 2, 0, mainElectro);
        #endif
        #ifdef curvasV2
        Robot.accelRamp(arrayPuntosX[0], arrayPuntosY[0], 2, 0, mainElectro);
        #endif
        activateElectromagnetV4E(mainElectro);
        Electro = mainElectro;
    }
    if (yIni <= -200) // Si va a meter desde el lado inferior
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC0, yC0, 2, 0, mainElectro);
        #endif
        #ifdef curvasV2
        Robot.accelRamp(arrayPuntosX[0], arrayPuntosY[0], 2, 0, mainElectro);
        #endif
        activateElectromagnetV4E(mainElectro);
        Electro = mainElectro;
    }
    if (yIni >= 200) // Si va a meter desde el lado superior
    {
        deactivateElectromagnetV4E(Electro);
        tempElectro = Electro;
        #ifdef curvasV1
        Robot.accelRamp(xC0, yC0, 2, 0, mainElectro);
        #endif
        #ifdef curvasV2
        Robot.accelRamp(arrayPuntosX[0], arrayPuntosY[0], 2, 0, mainElectro);
        #endif
        activateElectromagnetV4E(mainElectro);
        Electro = mainElectro;
    }
        }
#endif




        #ifdef curvasV1
        Robot.setSpeedMotors(vectVelPunto[17]);
        Robot.accelRamp(xC1,yC1,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[18]);
        Robot.accelRamp(xC2,yC2,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[19]);
        Robot.accelRamp(xC3,yC3,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[20]);
        Robot.accelRamp(xC4,yC4,0,0,Electro);
        #endif
        #ifdef curvasV2
        /*
        Robot.setSpeedMotors(vectVelPunto[17]);
        Robot.accelRamp(arrayPuntosX[1],arrayPuntosY[1],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[18]);
        Robot.accelRamp(arrayPuntosX[2],arrayPuntosY[2],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[19]);
        Robot.accelRamp(arrayPuntosX[3],arrayPuntosY[3],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[20]);
        Robot.accelRamp(arrayPuntosX[4],arrayPuntosY[4],0,0,Electro);
        */
        #endif


        #endif

        #endif

        #ifdef funcMoveTo
        Robot.moveToPointV2(xC4,yC4);
        testPWMElectromagnetDeactivate();
        #endif
        #ifdef funcAccelRamp

        #ifdef version1Electro
        Robot.setSpeedMotors(vectVelPunto[21]);
        Robot.accelRamp(xC5,yC5,0);
        Robot.setSpeedMotors(vectVelPunto[22]);
        Robot.accelRamp(xC6,yC6,0);
        #endif

        #ifdef version4Electro

        #ifdef curvasV1
        Robot.setSpeedMotors(vectVelPunto[21]);
        Robot.accelRamp(xC5,yC5,0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[22]);
        Robot.accelRamp(xC6,yC6,0,0,Electro);
        #endif
        #ifdef curvasV2
        /*
        Robot.setSpeedMotors(vectVelPunto[21]);
        Robot.accelRamp(arrayPuntosX[5],arrayPuntosY[5],0,0,Electro);
        Robot.setSpeedMotors(vectVelPunto[22]);
        Robot.accelRamp(arrayPuntosX[6],arrayPuntosY[6],0,0,Electro);
        */

       for(int i = 0; i < totalPuntosEnCurva-1; i++)
       {
        indexSpeed++;
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);
        Robot.accelRamp(arrayPuntosX[i+1],arrayPuntosY[i+1],0,0,Electro);
       }
        #endif

        #endif

        #ifdef version1Electro
        testPWMElectromagnetDeactivate();
        #endif

        #ifdef version4Electro
        testPWMElectromagnetDeactivateV4E(Electro);
        #endif

        #endif

        vectFinalInterX[7] = vectInterPointsX[4];
        vectFinalInterY[7] = vectInterPointsY[4];
        #ifdef funcMoveTo
        Robot.moveToPointV2(vectFinalInterX[7],vectFinalInterY[7]);

        activateElectromagnet();
        
        #endif
        #ifdef funcAccelRamp
        //Robot.setSpeedMotors(vectVelPunto[23]);

        indexSpeed++;
        Robot.setSpeedMotors(vectVelPunto[indexSpeed]);
        Robot.setSpeedRampFunction(vectVelPunto[indexSpeed]);
        Robot.setAccelRampFunction((vectVelPunto[indexSpeed])*2);
        
        #ifdef version1Electro
        Robot.accelRamp(vectFinalInterX[7],vectFinalInterY[7],0);
        #endif

        #ifdef version4Electro
        Robot.accelRamp(vectFinalInterX[7],vectFinalInterY[7],0,0,Electro);
        #endif

        #ifdef version1Electro
        activateElectromagnet();
        #endif

        #ifdef version4Electro
        activateElectromagnetV4E(Electro);
        #endif

        #endif

        #ifdef version1Electro
        deactivateElectromagnet();
        #endif

        #ifdef version4Electro
        deactivateElectromagnetV4E(Electro);
        #endif


    #ifdef version4Electro
    if((xFin >= -200 &&  xFin <= 200)&&(yFin >= -200 && yFin <= 200))
    {
        //Continua Normal
    }
    else
    {
        Electro = tempElectro;

    }
    #endif

    }

    #ifdef version4Electro
    if ((xIni >= -200 && xIni <= 200) && (yIni >= -200 && yIni <= 200))
    {
    // Continua Normal
    }
    else
    {
    Electro = tempElectro;
    }
    #endif

    Robot.setSpeedMotors(defGlobalSpeed);
    Robot.setSpeedRampFunction(defGlobalSpeed);
    Robot.setAccelRampFunction(defGlobalAccel);

    
}


void puntosDeCurva(double xP1,double yP1,double xP2,double yP2,double xP3,double yP3,double* xC0,double* yC0,double* xC1,double* yC1,double* xC2,double* yC2,double* xC3,double* yC3,double* xC4,double* yC4,double* xC5,double* yC5,double* xC6,double* yC6,double* xC7,double* yC7)
{
    double offsetCurvaX2 = 0;
    double offsetCurvaY2 = 0;

    double offsetCurvaX1 = 0;
    double offsetCurvaY1 = 0;

    //Para desfase de 3mm
    offsetCurvaX2 = 24;      //Antes 22
    offsetCurvaY2 = 0.0;     //Antes  offsetCurvaY = 0.2;

    offsetCurvaX1 = 23;      //Antes 22
    offsetCurvaY1 = 0.1;     //Antes  offsetCurvaY = 0.2; 

    //Para desfase de 4mm
    //offsetCurvaX = 21;
    //offsetCurvaY = 0.3;


   //Caso 1
   if(xP1 < xP2 && yP3 > yP2)
   {
        *xC0 = xP2 - 25;
        *yC0 = yP2 + 0;
        *xC1 = xP2 - offsetCurvaX2;
        *yC1 = yP2 + offsetCurvaY2;
        *xC2 = xP2 - offsetCurvaX1;
        *yC2 = yP2 + offsetCurvaY1;
        *xC3 = xP2 - 12.0;
        *yC3 = yP2 + 3.00;
        *xC4 = xP2 - 3.00;
        *yC4 = yP2 + 12.0;
        *xC5 = xP2 - offsetCurvaY1;
        *yC5 = yP2 + offsetCurvaX1;
        *xC6 = xP2 - offsetCurvaY2;
        *yC6 = yP2 + offsetCurvaX2;
        *xC7 = xP2 - 0;
        *yC7 = yP2 + 25;
   }
   //Caso 2
   if(yP1 > yP2 && xP3 < xP2)
   {
        *xC0 = xP2 - 0;
        *yC0 = yP2 + 25;
        *xC1 = xP2 - offsetCurvaY2;
        *yC1 = yP2 + offsetCurvaX2;
        *xC2 = xP2 - offsetCurvaY1;
        *yC2 = yP2 + offsetCurvaX1;
        *xC3 = xP2 - 3.00;
        *yC3 = yP2 + 12.0;
        *xC4 = xP2 - 12.0;
        *yC4 = yP2 + 3.0;
        *xC5 = xP2 - offsetCurvaX1;
        *yC5 = yP2 + offsetCurvaY1;
        *xC6 = xP2 - offsetCurvaX2;
        *yC6 = yP2 + offsetCurvaY2;
        *xC7 = xP2 - 25;
        *yC7 = yP2 + 0;
   }
   //Caso 3
   if(yP1 > yP2 && xP3 > xP2)
   {
        *xC0 = xP2 + 0;
        *yC0 = yP2 + 25;
        *xC1 = xP2 + offsetCurvaY2;
        *yC1 = yP2 + offsetCurvaX2;
        *xC2 = xP2 + offsetCurvaY1;
        *yC2 = yP2 + offsetCurvaX1;
        *xC3 = xP2 + 3.0;
        *yC3 = yP2 + 12.0;
        *xC4 = xP2 + 12.0;
        *yC4 = yP2 + 3.00;
        *xC5 = xP2 + offsetCurvaX1;
        *yC5 = yP2 + offsetCurvaY1;
        *xC6 = xP2 + offsetCurvaX2;
        *yC6 = yP2 + offsetCurvaY2;
        *xC7 = xP2 + 25;
        *yC7 = yP2 + 0;
   }
   //Caso 4
   if(xP1 > xP2 && yP3 > yP2)
   {
        *xC0 = xP2 + 25;
        *yC0 = yP2 + 0;
        *xC1 = xP2 + offsetCurvaX2;
        *yC1 = yP2 + offsetCurvaY2;
        *xC2 = xP2 + offsetCurvaX1;
        *yC2 = yP2 + offsetCurvaY1;
        *xC3 = xP2 + 12.0;
        *yC3 = yP2 + 3.0;
        *xC4 = xP2 + 3.0;
        *yC4 = yP2 + 12.0;
        *xC5 = xP2 + offsetCurvaY1;
        *yC5 = yP2 + offsetCurvaX1;
        *xC6 = xP2 + offsetCurvaY2;
        *yC6 = yP2 + offsetCurvaX2;
        *xC7 = xP2 + 0;
        *yC7 = yP2 + 25;
   }
   //Caso 5
   if(yP1 < yP2 && xP3 > xP2)
   {
        *xC0 = xP2 + 0;
        *yC0 = yP2 - 25;
        *xC1 = xP2 + offsetCurvaY2;
        *yC1 = yP2 - offsetCurvaX2;
        *xC2 = xP2 + offsetCurvaY1;
        *yC2 = yP2 - offsetCurvaX1;
        *xC3 = xP2 + 3.0;
        *yC3 = yP2 - 12.0;
        *xC4 = xP2 + 12.0;
        *yC4 = yP2 - 3.0;
        *xC5 = xP2 + offsetCurvaX1;
        *yC5 = yP2 - offsetCurvaY1;
        *xC6 = xP2 + offsetCurvaX2;
        *yC6 = yP2 - offsetCurvaY2;
        *xC7 = xP2 + 25;
        *yC7 = yP2 - 0;
   }
   //Caso 6
   if(xP1 > xP2 && yP3 < yP2)
   {
        *xC0 = xP2 + 25;
        *yC0 = yP2 - 0;
        *xC1 = xP2 + offsetCurvaX2;
        *yC1 = yP2 - offsetCurvaY2;
        *xC2 = xP2 + offsetCurvaX1;
        *yC2 = yP2 - offsetCurvaY1;
        *xC3 = xP2 + 12.0;
        *yC3 = yP2 - 3.0;
        *xC4 = xP2 + 3.0;
        *yC4 = yP2 - 12.0;
        *xC5 = xP2 + offsetCurvaY1;
        *yC5 = yP2 - offsetCurvaX1;
        *xC6 = xP2 + offsetCurvaY2;
        *yC6 = yP2 - offsetCurvaX2;
        *xC7 = xP2 + 0;
        *yC7 = yP2 - 25;
   }
   //Caso 7
   if(yP1 < yP2 && xP3 < xP2)
   {
        *xC0 = xP2 - 0;
        *yC0 = yP2 - 25;
        *xC1 = xP2 - offsetCurvaY2;
        *yC1 = yP2 - offsetCurvaX2;
        *xC2 = xP2 - offsetCurvaY1;
        *yC2 = yP2 - offsetCurvaX1;
        *xC3 = xP2 - 3.0;
        *yC3 = yP2 - 12.0;
        *xC4 = xP2 - 12.0;
        *yC4 = yP2 - 3.0;
        *xC5 = xP2 - offsetCurvaX1;
        *yC5 = yP2 - offsetCurvaY1;
        *xC6 = xP2 - offsetCurvaX2;
        *yC6 = yP2 - offsetCurvaY2;
        *xC7 = xP2 - 25;
        *yC7 = yP2 - 0;
   }
   //Caso 8
   if(xP1 < xP2 && yP3 < yP2)
   {
        *xC0 = xP2 - 25;
        *yC0 = yP2 - 0;
        *xC1 = xP2 - offsetCurvaX2;
        *yC1 = yP2 - offsetCurvaY2;
        *xC2 = xP2 - offsetCurvaX1;
        *yC2 = yP2 - offsetCurvaY1;
        *xC3 = xP2 - 12.0;
        *yC3 = yP2 - 3.0;
        *xC4 = xP2 - 3.0;
        *yC4 = yP2 - 12.0;
        *xC5 = xP2 - offsetCurvaY1;
        *yC5 = yP2 - offsetCurvaX1;
        *xC6 = xP2 - offsetCurvaY2;
        *yC6 = yP2 - offsetCurvaX2;
        *xC7 = xP2 - 0;
        *yC7 = yP2 - 25;
   }
}

void reorderAutoForzado()
{
    sensorsDir();
    detectChessBoard();
    int resultFinalPos;
    int resultContPiezas = -1;

    char matriz2[8][8]; 

    double compFinX, compFinY;
    double* ApFinX = &compFinX;
    double* ApFinY = &compFinY;

    double coordIniX, coordIniY;
    double* ApIniX = &coordIniX;
    double* ApIniY = &coordIniY;

    char cVectX[8] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    char cVectY[8] = {'8', '7', '6', '5', '4', '3', '2', '1'};
    char charPosXFinal = 'v';
    char charPosYFinal = 'v';
    bool colorActualChess = false;


    int vectMovesIniX[32] = {125,125,75,125,75,25,-175,175,-125,225,-225,225,-225,225,-225,-225,125,-125,175,-225,-225,-175,-225,-225,225,225,75,225,125,175,225,225};
    int vectMovesIniY[32] = {-125,-75,-25,-225,175,-75,225,-225,75,-175,175,-125,125,-75,75,-175,-25,25,-25,25,-25,-75,-75,-125,-25,175,25,25,75,25,75,125};

    int vectMovesFinX[32] = {25,175,125,25,-25,-175,75,-125,175,-75,25,-125,75,-75,-75,125,-25,-25,-175,-175,-125,-75,-125,-175,175,75,-25,125,25,75,125,175};
    int vectMovesFinY[32] = {-175,-125,-125,175,-175,125,-125,125,-175,125,-125,175,-175,175,-175,-175,175,-125,175,-175,-175,-125,-125,-125,175,175,125,175,125,125,125,125};

    char vectMovesPieces[32] = {'K','P','P','k','Q','p','P','p','R','p','P','n','B','b','B','N','q','P','r','R','N','P','P','P','r','b','p','n','p','p','p','p'};

    char vectCoordTableroX[32] = {'e','h','g','e','d','a','f','b','h','c','e','b','f','c','c','g','d','d','a','a','b','c','b','a','h','f','d','g','e','f','g','h'};
    char vectCoordTableroY[32] = {'1','2','2','8','1','7','2','7','1','7','2','8','1','8','1','1','8','2','8','1','1','2','2','2','8','8','7','8','7','7','7','7'};


                comerVersion3('g','5',1,true,'k');


                #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
                 //resultFinalPos;
                detectChessBoardVerif();
                resultFinalPos = checkFinalPositionGraveyard(true);
                while(resultFinalPos == 0)
                {
                    comerVersion3('g','5',1,true,'k');
                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPositionGraveyard(true);
                }
                #endif

                matriz[6][3] = '.';
                Serial.println("Matriz y piezasMuertas despues de sacar al rey");
                printMatriz();
                imprimirVectPiecesD();
                
                #ifdef activateSensors
                //compareVirtualMatrizVsSensors();
                #endif   

    for(int j = 0; j < 32; j++)
    {
                #ifdef funcMoveTo
                Robot.setSpeedMotors(maximumVelocityDirect);
                #endif
                #ifdef funcAccelRamp
                Robot.setSpeedMotors(maximunVelocityDirectRamp);
                Robot.setSpeedRampFunction(maximunVelocityDirectRamp);
                Robot.setAccelRampFunction(maximunAccelDirectRamp);
                #endif
                
                #ifdef moveDirectPosPiece
                
                #ifdef funcMoveTo
                Robot.moveToPointV2(coordIniX, coordIniY,1);
                #endif

                #ifdef funcAccelRamp
                Robot.accelRamp(vectMovesIniX[j],vectMovesIniY[j],2); //Antes desacelracion,2
                #endif
                
                #else
                moveOnTheLineIni(coordIniX, coordIniY, compFinX,compFinY);
                #endif

                #ifdef funcMoveTo
                Robot.setSpeedMotors(defGlobalSpeed);
                #endif
                #ifdef funcAccelRamp
                Robot.setSpeedMotors(defGlobalSpeed);
                Robot.setSpeedRampFunction(defGlobalSpeed);
                Robot.setAccelRampFunction(defGlobalAccel);
                #endif

                
                //moveOnTheLine(coordIniX, coordIniY, compFinX,compFinY);
                moveOnTheLinev2(vectMovesIniX[j],vectMovesIniY[j],vectMovesFinX[j],vectMovesFinY[j]);

                #ifdef activateSensors
                //Serial.println("En funcion realizaJugada");
                //resultFinalPos;
                detectChessBoardVerif();
                resultFinalPos = checkFinalPosition(vectCoordTableroX[j],vectCoordTableroY[j]);
                while(resultFinalPos == 0)
                {
                    //moveOnTheLineIni(coordIniX, coordIniY, compFinX,compFinY);

                    #ifdef funcMoveTo
                    Robot.setSpeedMotors(maximumVelocityDirect);
                    #endif
                    #ifdef funcAccelRamp
                    Robot.setSpeedMotors(maximunVelocityDirectRamp);
                    Robot.setSpeedRampFunction(maximunVelocityDirectRamp);
                    Robot.setAccelRampFunction(maximunAccelDirectRamp);
                    #endif

                    #ifdef moveDirectPosPiece
                    
                    #ifdef funcMoveTo
                    Robot.moveToPointV2(coordIniX, coordIniY,1);
                    #endif

                    #ifdef funcAccelRamp
                    Robot.accelRamp(vectMovesIniX[j],vectMovesIniY[j],2);         //Antes desacelracion,2
                    #endif
                    
                    #else
                    moveOnTheLineIni(coordIniX, coordIniY, compFinX,compFinY);
                    #endif

                    #ifdef funcMoveTo
                    Robot.setSpeedMotors(defGlobalSpeed);
                    #endif
                    #ifdef funcAccelRamp
                    Robot.setSpeedMotors(defGlobalSpeed);
                    Robot.setSpeedRampFunction(defGlobalSpeed);
                    Robot.setAccelRampFunction(defGlobalAccel);
                    #endif


                    //moveOnTheLine(coordIniX, coordIniY, compFinX,compFinY);
                    moveOnTheLinev2(vectMovesIniX[j],vectMovesIniY[j],vectMovesFinX[j],vectMovesFinY[j]);

                    detectChessBoardVerif();
                    resultFinalPos = checkFinalPosition(vectCoordTableroX[j],vectCoordTableroY[j]);
                }
                #endif
                detectChessBoard();
                resultContPiezas = contChessSensors(); //Devuelve 1 si estan todas las piezas 0 si faltan piezas en el tablero
                while(resultContPiezas == 0)
                {
                    delay(100);
                    detectChessBoard();
                    resultContPiezas = contChessSensors(); //Devuelve 1 si estan todas las piezas 0 si faltan piezas en el tablero
                }
        
    }
}

void activateElectromagnetV4E(int optionElectro)
{   
    //Serial.println("Funcion activateElectromagnetV4E ...");
    //Serial.println("SE ACTIVO ELECTROIMAN : ");
    //Serial.println(optionElectro);
    #ifdef electromagnetConf1
    //Serial.println("->> ACTIVATE ELECTROMAGNET");
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);

   ledcWrite(ledChannel, 254);
   delay(200);
   #endif

   #ifdef electromagnetConf2
   //Serial.println("->> DEACTIVATE ELECTROMAGNET");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
#ifdef normalElectro
    if(optionElectro == 1)
    {
        ledcAttachPin(22, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        //digitalWrite(22,HIGH);
        ledcWrite(ledChannel, 120); //253
        digitalWrite(23,LOW);
    }
    if(optionElectro == 2)
    {
        ledcAttachPin(12, ledChannel);
        //digitalWrite(12,HIGH);
        ledcWrite(ledChannel, 120); //253
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 3)
    {
        ledcAttachPin(13, ledChannel);
        digitalWrite(12,LOW);
        //digitalWrite(13,HIGH);
        ledcWrite(ledChannel, 120); //253
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 4)
    {
        ledcAttachPin(23, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        //digitalWrite(23,HIGH);
        ledcWrite(ledChannel, 120); //253
    }
#endif
#ifdef invertElectro
    if(optionElectro == 1)
    {
        ledcAttachPin(13, ledChannel);
        digitalWrite(12,HIGH);
        //digitalWrite(13,HIGH);
        ledcWrite(ledChannel, 0); //antes 50
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 2)
    {
        ledcAttachPin(12, ledChannel);
        //digitalWrite(12,HIGH);
        ledcWrite(ledChannel, 0); //ates 50
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 3)
    {
        ledcAttachPin(22, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        //digitalWrite(22,HIGH);
        ledcWrite(ledChannel, 0); //antes 50
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 4)
    {
        ledcAttachPin(23, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        //digitalWrite(23,HIGH);
        ledcWrite(ledChannel, 0); //antes 50
    }
#endif

    //ledcWrite(ledChannel, 0);
    delay(600); //Antes delay 1000 //Antes 800
   #endif
     
}

void deactivateElectromagnetV4E(int optionElectro)
{
    //Serial.println("SE DESACTIVO ELECTROIMAN : ");
    //Serial.println(optionElectro);
    #ifdef electromagnetConf1
    //Serial.println("->> DEACTIVATE ELECTROMAGNET");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 0);
    delay(100); //Antes delay 1000 //Antes 800
    #endif

    #ifdef electromagnetConf2
    //Serial.println("->> ACTIVATE ELECTROMAGNET");
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    //ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
#ifdef normalElectro
     if(optionElectro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
#endif
#ifdef invertElectro
     if(optionElectro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
#endif
   //ledcWrite(ledChannel, 254);
   ledcDetachPin(12);
   ledcDetachPin(13);
   ledcDetachPin(22);
   ledcDetachPin(23);  
   delay(200);
    #endif
    
}

void testPWMElectromagnetDeactivateV4E(int optionElectro)
{
    //Serial.println("SE desactivo ELECTROIMAN : ");
    //Serial.println(optionElectro);
    #ifdef electromagnetConf1
    //Serial.println("->> DEACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 0);   //ANTES 1
    #endif

    #ifdef electromagnetConf2
    //Serial.println("->> ACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;        //ANTES 5000
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    //ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
#ifdef normalElectro
     if(optionElectro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
#endif
#ifdef invertElectro
     if(optionElectro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
#endif
    //ledcWrite(ledChannel, 254); //Antes 220

    #endif
    ledcDetachPin(12);
   ledcDetachPin(13);
   ledcDetachPin(22);
   ledcDetachPin(23);
}


void testPWMElectromagnetActivateV4E(int optionElectro)
{
    //Serial.println("Funcion testPWMElectromagnetActivateV4E ...");
    //Serial.println("SE activo ELECTROIMAN : ");
    //Serial.println(optionElectro);
    #ifdef electromagnetConf1
    //Serial.println("->> ACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;        //ANTES 5000
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);

    ledcWrite(ledChannel, 254); //Antes 220
    delay(200); //Antes 1000
    #endif

    #ifdef electromagnetConf2
    //Serial.println("->> DEACTIVATE ELECTROMAGNET PWM");
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
#ifdef normalElectro
     if(optionElectro == 1)
    {
        ledcAttachPin(22, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        //digitalWrite(22,HIGH);
        ledcWrite(ledChannel, 120); //253
        digitalWrite(23,LOW);
    }
    if(optionElectro == 2)
    {
        ledcAttachPin(12, ledChannel);
        //digitalWrite(12,HIGH);
        ledcWrite(ledChannel, 120); //253 
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 3)
    {
        ledcAttachPin(13, ledChannel);
        digitalWrite(12,LOW);
        //digitalWrite(13,HIGH);
        ledcWrite(ledChannel, 120);  //253 
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(optionElectro == 4)
    {
        ledcAttachPin(23, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        //digitalWrite(23,HIGH);
        ledcWrite(ledChannel, 120); //253 
    }
#endif
#ifdef invertElectro
     if(optionElectro == 1)
    {
        ledcAttachPin(13, ledChannel);
        digitalWrite(12,HIGH);
        //digitalWrite(13,HIGH);
        ledcWrite(ledChannel, 0); //antes 50
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 2)
    {
        ledcAttachPin(12, ledChannel);
        //digitalWrite(12,HIGH);
        ledcWrite(ledChannel, 0); //antes 50
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 3)
    {
        ledcAttachPin(22, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        //digitalWrite(22,HIGH);
        ledcWrite(ledChannel, 0); //antes 50
        digitalWrite(23,HIGH);
    }
    if(optionElectro == 4)
    {
        ledcAttachPin(23, ledChannel);
        digitalWrite(12,HIGH);
        digitalWrite(13,HIGH);
        digitalWrite(22,HIGH);
        //digitalWrite(23,HIGH);
        ledcWrite(ledChannel, 0); //antes 50
    }
#endif
    //ledcWrite(ledChannel, 0);   //ANTES 1
    delay(200); //Antes 1000
    #endif
}

void puntosDeCurvaV2(double xP1,double yP1,double xP2,double yP2,double xP3,double yP3, double *arrayTestX, double *arrayTestY, int sizeArray)
{
    double offsetCurvaX2 = 0;
    double offsetCurvaY2 = 0;

    double offsetCurvaX1 = 0;
    double offsetCurvaY1 = 0;

    //Para desfase de 3mm
    offsetCurvaX2 = 24;      //Antes 22
    offsetCurvaY2 = 0.0;     //Antes  offsetCurvaY = 0.2;

    offsetCurvaX1 = 23;      //Antes 22
    offsetCurvaY1 = 0.1;     //Antes  offsetCurvaY = 0.2; 

    //Para desfase de 4mm
    //offsetCurvaX = 21;
    //offsetCurvaY = 0.3;

    double degreeInterval;
    degreeInterval = 90/((sizeArray)*1.0);
/*
    Serial.println("sizeArray");
    Serial.println(sizeArray);

    Serial.println("Longitud de Arcos en grados");
    Serial.println(degreeInterval);
*/

    double tempArrayX[sizeArray + 1] = {0};
    double tempArrayY[sizeArray + 1] = {0};

    /*
    Serial.println("arrayTest: ");
    for(int i = 0; i <= nX; i++)
    {
        Serial.println(arrayTestX[i]);

    }
    delay(60000);
    */

   double pCurvaX;
   double pCurvaXsinCent;
   
   double pCurvaY;
   double pCurvaYsinCent;

   double interInRadians;

   //Variables para quitar digitos despues de los decimales en x
   int integerN1x;
   int integerN2x;
   double extraCentX;
   //-----------------------------------------------------------

   //Variables para quitar digitos despues de los decimales en y
   int integerN1y;
   int integerN2y;
   double extraCentY;
   //-----------------------------------------------------------

//   Serial.println("Puntos Generados: ");
   for (int i = 0; i < (sizeArray + 1); i++)
   {
                interInRadians = ((i) * degreeInterval) * (PI / 180);
                pCurvaX = 25 * cos(interInRadians);
                pCurvaY = 25 * sin(interInRadians);

                // Bloque para eliminar los digitos despues de decimales en coordendas x
                integerN1x = pCurvaX;
                extraCentX = pCurvaX - integerN1x;
                extraCentX = extraCentX * 10;
                integerN2x = extraCentX;
                extraCentX = extraCentX - integerN2x;
                extraCentX = extraCentX / 10;
                pCurvaXsinCent = pCurvaX - extraCentX;
                //--------------------------------------------------------------------

                // Bloque para eliminar los digitos despues de decimales en coordendas y
                integerN1y = pCurvaY;
                extraCentY = pCurvaY - integerN1y;
                extraCentY = extraCentY * 10;
                integerN2y = extraCentY;
                extraCentY = extraCentY - integerN2y;
                extraCentY = extraCentY / 10;
                pCurvaYsinCent = pCurvaY - extraCentY;
                //--------------------------------------------------------------------
/*
                Serial.print(pCurvaXsinCent);
                Serial.print("\t");
                Serial.println(pCurvaYsinCent);
                
                
*/

                pCurvaXsinCent = abs(pCurvaXsinCent - 25);
                pCurvaYsinCent = abs(pCurvaYsinCent - 25);

                tempArrayX[i] =  pCurvaXsinCent;
                tempArrayY[i] =  pCurvaYsinCent;
   }

   //Caso 1
   if(xP1 < xP2 && yP3 > yP2)
   {
    /*
        *xC0 = xP2 - 25;
        *yC0 = yP2 + 0;
        *xC1 = xP2 - offsetCurvaX2;
        *yC1 = yP2 + offsetCurvaY2;
        *xC2 = xP2 - offsetCurvaX1;
        *yC2 = yP2 + offsetCurvaY1;
        *xC3 = xP2 - 12.0;
        *yC3 = yP2 + 3.00;
        *xC4 = xP2 - 3.00;
        *yC4 = yP2 + 12.0;
        *xC5 = xP2 - offsetCurvaY1;
        *yC5 = yP2 + offsetCurvaX1;
        *xC6 = xP2 - offsetCurvaY2;
        *yC6 = yP2 + offsetCurvaX2;
        *xC7 = xP2 - 0;
        *yC7 = yP2 + 25;
        */

        for (int i = 0; i < (sizeArray + 1); i++)
        {
            arrayTestX[i] = round(xP2 - tempArrayY[i]);
            arrayTestY[i] = round(yP2 + tempArrayX[i]);
        }
   }
   //Caso 2
   if(yP1 > yP2 && xP3 < xP2)
   {
    /*
        *xC0 = xP2 - 0;
        *yC0 = yP2 + 25;
        *xC1 = xP2 - offsetCurvaY2;
        *yC1 = yP2 + offsetCurvaX2;
        *xC2 = xP2 - offsetCurvaY1;
        *yC2 = yP2 + offsetCurvaX1;
        *xC3 = xP2 - 3.00;
        *yC3 = yP2 + 12.0;
        *xC4 = xP2 - 12.0;
        *yC4 = yP2 + 3.0;
        *xC5 = xP2 - offsetCurvaX1;
        *yC5 = yP2 + offsetCurvaY1;
        *xC6 = xP2 - offsetCurvaX2;
        *yC6 = yP2 + offsetCurvaY2;
        *xC7 = xP2 - 25;
        *yC7 = yP2 + 0;
        */

       for (int i = 0; i < (sizeArray + 1); i++)
        {
            arrayTestX[i] = round(xP2 - tempArrayX[i]);
            arrayTestY[i] = round(yP2 + tempArrayY[i]);
        }
   }
   //Caso 3
   if(yP1 > yP2 && xP3 > xP2)
   {
    /*
        *xC0 = xP2 + 0;
        *yC0 = yP2 + 25;
        *xC1 = xP2 + offsetCurvaY2;
        *yC1 = yP2 + offsetCurvaX2;
        *xC2 = xP2 + offsetCurvaY1;
        *yC2 = yP2 + offsetCurvaX1;
        *xC3 = xP2 + 3.0;
        *yC3 = yP2 + 12.0;
        *xC4 = xP2 + 12.0;
        *yC4 = yP2 + 3.00;
        *xC5 = xP2 + offsetCurvaX1;
        *yC5 = yP2 + offsetCurvaY1;
        *xC6 = xP2 + offsetCurvaX2;
        *yC6 = yP2 + offsetCurvaY2;
        *xC7 = xP2 + 25;
        *yC7 = yP2 + 0;
        */

       for (int i = 0; i < (sizeArray + 1); i++)
        {
            arrayTestX[i] = round(xP2 + tempArrayX[i]);
            arrayTestY[i] = round(yP2 + tempArrayY[i]);
        }
   }
   //Caso 4
   if(xP1 > xP2 && yP3 > yP2)
   {
    /*
        *xC0 = xP2 + 25;
        *yC0 = yP2 + 0;
        *xC1 = xP2 + offsetCurvaX2;
        *yC1 = yP2 + offsetCurvaY2;
        *xC2 = xP2 + offsetCurvaX1;
        *yC2 = yP2 + offsetCurvaY1;
        *xC3 = xP2 + 12.0;
        *yC3 = yP2 + 3.0;
        *xC4 = xP2 + 3.0;
        *yC4 = yP2 + 12.0;
        *xC5 = xP2 + offsetCurvaY1;
        *yC5 = yP2 + offsetCurvaX1;
        *xC6 = xP2 + offsetCurvaY2;
        *yC6 = yP2 + offsetCurvaX2;
        *xC7 = xP2 + 0;
        *yC7 = yP2 + 25;
        */

       for (int i = 0; i < (sizeArray + 1); i++)
        {
            arrayTestX[i] = round(xP2 + tempArrayY[i]);
            arrayTestY[i] = round(yP2 + tempArrayX[i]);
        }
   }
   //Caso 5
   if(yP1 < yP2 && xP3 > xP2)
   {
    /*
        *xC0 = xP2 + 0;
        *yC0 = yP2 - 25;
        *xC1 = xP2 + offsetCurvaY2;
        *yC1 = yP2 - offsetCurvaX2;
        *xC2 = xP2 + offsetCurvaY1;
        *yC2 = yP2 - offsetCurvaX1;
        *xC3 = xP2 + 3.0;
        *yC3 = yP2 - 12.0;
        *xC4 = xP2 + 12.0;
        *yC4 = yP2 - 3.0;
        *xC5 = xP2 + offsetCurvaX1;
        *yC5 = yP2 - offsetCurvaY1;
        *xC6 = xP2 + offsetCurvaX2;
        *yC6 = yP2 - offsetCurvaY2;
        *xC7 = xP2 + 25;
        *yC7 = yP2 - 0;
    */

   for (int i = 0; i < (sizeArray + 1); i++)
        {
            arrayTestX[i] = round(xP2 + tempArrayX[i]);
            arrayTestY[i] = round(yP2 - tempArrayY[i]);
        }
   }
   //Caso 6
   if(xP1 > xP2 && yP3 < yP2)
   {
    /*
        *xC0 = xP2 + 25;
        *yC0 = yP2 - 0;
        *xC1 = xP2 + offsetCurvaX2;
        *yC1 = yP2 - offsetCurvaY2;
        *xC2 = xP2 + offsetCurvaX1;
        *yC2 = yP2 - offsetCurvaY1;
        *xC3 = xP2 + 12.0;
        *yC3 = yP2 - 3.0;
        *xC4 = xP2 + 3.0;
        *yC4 = yP2 - 12.0;
        *xC5 = xP2 + offsetCurvaY1;
        *yC5 = yP2 - offsetCurvaX1;
        *xC6 = xP2 + offsetCurvaY2;
        *yC6 = yP2 - offsetCurvaX2;
        *xC7 = xP2 + 0;
        *yC7 = yP2 - 25;
        */

       for (int i = 0; i < (sizeArray + 1); i++)
        {
            arrayTestX[i] = round(xP2 + tempArrayY[i]);
            arrayTestY[i] = round(yP2 - tempArrayX[i]);
        }
   }
   //Caso 7
   if(yP1 < yP2 && xP3 < xP2)
   {
    /*
        *xC0 = xP2 - 0;
        *yC0 = yP2 - 25;
        *xC1 = xP2 - offsetCurvaY2;
        *yC1 = yP2 - offsetCurvaX2;
        *xC2 = xP2 - offsetCurvaY1;
        *yC2 = yP2 - offsetCurvaX1;
        *xC3 = xP2 - 3.0;
        *yC3 = yP2 - 12.0;
        *xC4 = xP2 - 12.0;
        *yC4 = yP2 - 3.0;
        *xC5 = xP2 - offsetCurvaX1;
        *yC5 = yP2 - offsetCurvaY1;
        *xC6 = xP2 - offsetCurvaX2;
        *yC6 = yP2 - offsetCurvaY2;
        *xC7 = xP2 - 25;
        *yC7 = yP2 - 0;
    */

   for (int i = 0; i < (sizeArray + 1); i++)
        {
            arrayTestX[i] = round(xP2 - tempArrayX[i]);
            arrayTestY[i] = round(yP2 - tempArrayY[i]);
        }
   }
   //Caso 8
   if(xP1 < xP2 && yP3 < yP2)
   {
    /*
        *xC0 = xP2 - 25;
        *yC0 = yP2 - 0;
        *xC1 = xP2 - offsetCurvaX2;
        *yC1 = yP2 - offsetCurvaY2;
        *xC2 = xP2 - offsetCurvaX1;
        *yC2 = yP2 - offsetCurvaY1;
        *xC3 = xP2 - 12.0;
        *yC3 = yP2 - 3.0;
        *xC4 = xP2 - 3.0;
        *yC4 = yP2 - 12.0;
        *xC5 = xP2 - offsetCurvaY1;
        *yC5 = yP2 - offsetCurvaX1;
        *xC6 = xP2 - offsetCurvaY2;
        *yC6 = yP2 - offsetCurvaX2;
        *xC7 = xP2 - 0;
        *yC7 = yP2 - 25;
        */

       for (int i = 0; i < (sizeArray + 1); i++)
        {
            arrayTestX[i] = round(xP2 - tempArrayY[i]);
            arrayTestY[i] = round(yP2 - tempArrayX[i]);
        }
   }
}

void saveGamesFirstTime()
{
/*
    const char *contentsGame;

    String nameFileIni = "";

    while(!SPIFFS.begin(true)){
    Serial.println("SPIFFS Mount Failed");
    }
    Serial.println("Saving Games ...");

    int m0 = Game0.length();
    char char_array0[m0 + 1];
    strcpy(char_array0, Game0.c_str());
    contentsGame = char_array0;
    createNewFileChess(SPIFFS, "/Game0.txt", contentsGame);

    int m1 = Game1.length();
    char char_array1[m1 + 1];
    strcpy(char_array1, Game1.c_str());
    contentsGame = char_array1;
    createNewFileChess(SPIFFS, "/Game1.txt", contentsGame);
    
    int m2 = Game2.length();
    char char_array2[m2 + 1];
    strcpy(char_array2, Game2.c_str());
    contentsGame = char_array2;
    createNewFileChess(SPIFFS, "/Game2.txt", contentsGame);

    int m3 = Game3.length();
    char char_array3[m3 + 1];
    strcpy(char_array3, Game3.c_str());
    contentsGame = char_array3;
    createNewFileChess(SPIFFS, "/Game3.txt", contentsGame);

    int m4 = Game4.length();
    char char_array4[m4 + 1];
    strcpy(char_array4, Game4.c_str());
    contentsGame = char_array4;
    createNewFileChess(SPIFFS, "/Game4.txt", contentsGame);

    int m5 = Game5.length();
    char char_array5[m5 + 1];
    strcpy(char_array5, Game5.c_str());
    contentsGame = char_array5;
    createNewFileChess(SPIFFS, "/Game5.txt", contentsGame);

    int m6 = Game6.length();
    char char_array6[m6 + 1];
    strcpy(char_array6, Game6.c_str());
    contentsGame = char_array6;
    createNewFileChess(SPIFFS, "/Game6.txt", contentsGame);

    int m7 = Game7.length();
    char char_array7[m7 + 1];
    strcpy(char_array7, Game7.c_str());
    contentsGame = char_array7;
    createNewFileChess(SPIFFS, "/Game7.txt", contentsGame);

    int m8 = Game8.length();
    char char_array8[m8 + 1];
    strcpy(char_array8, Game8.c_str());
    contentsGame = char_array8;
    createNewFileChess(SPIFFS, "/Game8.txt", contentsGame);

    int m9 = Game9.length();
    char char_array9[m9 + 1];
    strcpy(char_array9, Game9.c_str());
    contentsGame = char_array9;
    createNewFileChess(SPIFFS, "/Game9.txt", contentsGame);

    int m10 = Game10.length();
    char char_array10[m10 + 1];
    strcpy(char_array10, Game10.c_str());
    contentsGame = char_array10;
    createNewFileChess(SPIFFS, "/Game10.txt", contentsGame);

    int m11 = Game11.length();
    char char_array11[m11 + 1];
    strcpy(char_array11, Game11.c_str());
    contentsGame = char_array11;
    createNewFileChess(SPIFFS, "/Game11.txt", contentsGame);
    
    int m12 = Game12.length();
    char char_array12[m12 + 1];
    strcpy(char_array12, Game12.c_str());
    contentsGame = char_array12;
    createNewFileChess(SPIFFS, "/Game12.txt", contentsGame);

    int m13 = Game13.length();
    char char_array13[m13 + 1];
    strcpy(char_array13, Game13.c_str());
    contentsGame = char_array13;
    createNewFileChess(SPIFFS, "/Game13.txt", contentsGame);

    int m14 = Game14.length();
    char char_array14[m14 + 1];
    strcpy(char_array14, Game14.c_str());
    contentsGame = char_array14;
    createNewFileChess(SPIFFS, "/Game14.txt", contentsGame);

    int m15 = Game15.length();
    char char_array15[m15 + 1];
    strcpy(char_array15, Game15.c_str());
    contentsGame = char_array15;
    createNewFileChess(SPIFFS, "/Game15.txt", contentsGame);

    int m16 = Game16.length();
    char char_array16[m16 + 1];
    strcpy(char_array16, Game16.c_str());
    contentsGame = char_array16;
    createNewFileChess(SPIFFS, "/Game16.txt", contentsGame);

    int m17 = Game17.length();
    char char_array17[m17 + 1];
    strcpy(char_array17, Game17.c_str());
    contentsGame = char_array17;
    createNewFileChess(SPIFFS, "/Game17.txt", contentsGame);

    int m18 = Game18.length();
    char char_array18[m18 + 1];
    strcpy(char_array18, Game18.c_str());
    contentsGame = char_array18;
    createNewFileChess(SPIFFS, "/Game18.txt", contentsGame);

    int m19 = Game19.length();
    char char_array19[m19 + 1];
    strcpy(char_array19, Game19.c_str());
    contentsGame = char_array19;
    createNewFileChess(SPIFFS, "/Game19.txt", contentsGame);

    int m20 = Game20.length();
    char char_array20[m20 + 1];
    strcpy(char_array20, Game20.c_str());
    contentsGame = char_array20;
    createNewFileChess(SPIFFS, "/Game20.txt", contentsGame);

    int m21 = Game21.length();
    char char_array21[m21 + 1];
    strcpy(char_array21, Game21.c_str());
    contentsGame = char_array21;
    createNewFileChess(SPIFFS, "/Game21.txt", contentsGame);
    
    int m22 = Game22.length();
    char char_array22[m22 + 1];
    strcpy(char_array22, Game22.c_str());
    contentsGame = char_array22;
    createNewFileChess(SPIFFS, "/Game22.txt", contentsGame);

    int m23 = Game23.length();
    char char_array23[m23 + 1];
    strcpy(char_array23, Game23.c_str());
    contentsGame = char_array23;
    createNewFileChess(SPIFFS, "/Game23.txt", contentsGame);

    int m24 = Game24.length();
    char char_array24[m24 + 1];
    strcpy(char_array24, Game24.c_str());
    contentsGame = char_array24;
    createNewFileChess(SPIFFS, "/Game24.txt", contentsGame);

    int m25 = Game25.length();
    char char_array25[m25 + 1];
    strcpy(char_array25, Game25.c_str());
    contentsGame = char_array25;
    createNewFileChess(SPIFFS, "/Game25.txt", contentsGame);

    int m26 = Game26.length();
    char char_array26[m26 + 1];
    strcpy(char_array26, Game26.c_str());
    contentsGame = char_array26;
    createNewFileChess(SPIFFS, "/Game26.txt", contentsGame);

    int m27 = Game27.length();
    char char_array27[m27 + 1];
    strcpy(char_array27, Game27.c_str());
    contentsGame = char_array27;
    createNewFileChess(SPIFFS, "/Game27.txt", contentsGame);

    int m28 = Game28.length();
    char char_array28[m28 + 1];
    strcpy(char_array28, Game28.c_str());
    contentsGame = char_array28;
    createNewFileChess(SPIFFS, "/Game28.txt", contentsGame);

    int m29 = Game29.length();
    char char_array29[m29 + 1];
    strcpy(char_array29, Game29.c_str());
    contentsGame = char_array29;
    createNewFileChess(SPIFFS, "/Game29.txt", contentsGame);

    int m30 = Game30.length();
    char char_array30[m30 + 1];
    strcpy(char_array30, Game30.c_str());
    contentsGame = char_array30;
    createNewFileChess(SPIFFS, "/Game30.txt", contentsGame);

    int m31 = Game31.length();
    char char_array31[m31 + 1];
    strcpy(char_array31, Game31.c_str());
    contentsGame = char_array31;
    createNewFileChess(SPIFFS, "/Game31.txt", contentsGame);
    
    int m32 = Game32.length();
    char char_array32[m32 + 1];
    strcpy(char_array32, Game32.c_str());
    contentsGame = char_array32;
    createNewFileChess(SPIFFS, "/Game32.txt", contentsGame);

    int m33 = Game33.length();
    char char_array33[m33 + 1];
    strcpy(char_array33, Game33.c_str());
    contentsGame = char_array33;
    createNewFileChess(SPIFFS, "/Game33.txt", contentsGame);

    int m34 = Game34.length();
    char char_array34[m34 + 1];
    strcpy(char_array34, Game34.c_str());
    contentsGame = char_array34;
    createNewFileChess(SPIFFS, "/Game34.txt", contentsGame);

    int m35 = Game35.length();
    char char_array35[m35 + 1];
    strcpy(char_array35, Game35.c_str());
    contentsGame = char_array35;
    createNewFileChess(SPIFFS, "/Game35.txt", contentsGame);

    int m36 = Game36.length();
    char char_array36[m36 + 1];
    strcpy(char_array36, Game36.c_str());
    contentsGame = char_array36;
    createNewFileChess(SPIFFS, "/Game36.txt", contentsGame);

    int m37 = Game37.length();
    char char_array37[m37 + 1];
    strcpy(char_array37, Game37.c_str());
    contentsGame = char_array37;
    createNewFileChess(SPIFFS, "/Game37.txt", contentsGame);

    int m38 = Game38.length();
    char char_array38[m38 + 1];
    strcpy(char_array38, Game38.c_str());
    contentsGame = char_array38;
    createNewFileChess(SPIFFS, "/Game38.txt", contentsGame);

    int m39 = Game39.length();
    char char_array39[m39 + 1];
    strcpy(char_array39, Game39.c_str());
    contentsGame = char_array39;
    createNewFileChess(SPIFFS, "/Game39.txt", contentsGame);

    int m40 = Game40.length();
    char char_array40[m40 + 1];
    strcpy(char_array40, Game40.c_str());
    contentsGame = char_array40;
    createNewFileChess(SPIFFS, "/Game40.txt", contentsGame);

    int m41 = Game41.length();
    char char_array41[m41 + 1];
    strcpy(char_array41, Game41.c_str());
    contentsGame = char_array41;
    createNewFileChess(SPIFFS, "/Game41.txt", contentsGame);
    
    int m42 = Game42.length();
    char char_array42[m42 + 1];
    strcpy(char_array42, Game42.c_str());
    contentsGame = char_array42;
    createNewFileChess(SPIFFS, "/Game42.txt", contentsGame);

    int m43 = Game43.length();
    char char_array43[m43 + 1];
    strcpy(char_array43, Game43.c_str());
    contentsGame = char_array43;
    createNewFileChess(SPIFFS, "/Game43.txt", contentsGame);

    int m44 = Game44.length();
    char char_array44[m44 + 1];
    strcpy(char_array44, Game44.c_str());
    contentsGame = char_array44;
    createNewFileChess(SPIFFS, "/Game44.txt", contentsGame);

    int m45 = Game45.length();
    char char_array45[m45 + 1];
    strcpy(char_array45, Game45.c_str());
    contentsGame = char_array45;
    createNewFileChess(SPIFFS, "/Game45.txt", contentsGame);

    int m46 = Game46.length();
    char char_array46[m46 + 1];
    strcpy(char_array46, Game46.c_str());
    contentsGame = char_array46;
    createNewFileChess(SPIFFS, "/Game46.txt", contentsGame);

    int m47 = Game47.length();
    char char_array47[m47 + 1];
    strcpy(char_array47, Game47.c_str());
    contentsGame = char_array47;
    createNewFileChess(SPIFFS, "/Game47.txt", contentsGame);

    int m48 = Game48.length();
    char char_array48[m48 + 1];
    strcpy(char_array48, Game48.c_str());
    contentsGame = char_array48;
    createNewFileChess(SPIFFS, "/Game48.txt", contentsGame);

    int m49 = Game49.length();
    char char_array49[m49 + 1];
    strcpy(char_array49, Game49.c_str());
    contentsGame = char_array49;
    createNewFileChess(SPIFFS, "/Game49.txt", contentsGame);

    int m50 = Game50.length();
    char char_array50[m50 + 1];
    strcpy(char_array50, Game50.c_str());
    contentsGame = char_array50;
    createNewFileChess(SPIFFS, "/Game50.txt", contentsGame);

    int m51 = Game51.length();
    char char_array51[m51 + 1];
    strcpy(char_array51, Game51.c_str());
    contentsGame = char_array51;
    createNewFileChess(SPIFFS, "/Game51.txt", contentsGame);
    
    int m52 = Game52.length();
    char char_array52[m52 + 1];
    strcpy(char_array52, Game52.c_str());
    contentsGame = char_array52;
    createNewFileChess(SPIFFS, "/Game52.txt", contentsGame);

    int m53 = Game53.length();
    char char_array53[m53 + 1];
    strcpy(char_array53, Game53.c_str());
    contentsGame = char_array53;
    createNewFileChess(SPIFFS, "/Game53.txt", contentsGame);

    int m54 = Game54.length();
    char char_array54[m54 + 1];
    strcpy(char_array54, Game54.c_str());
    contentsGame = char_array54;
    createNewFileChess(SPIFFS, "/Game54.txt", contentsGame);

    int m55 = Game55.length();
    char char_array55[m55 + 1];
    strcpy(char_array55, Game55.c_str());
    contentsGame = char_array55;
    createNewFileChess(SPIFFS, "/Game55.txt", contentsGame);

    int m56 = Game56.length();
    char char_array56[m56 + 1];
    strcpy(char_array56, Game56.c_str());
    contentsGame = char_array56;
    createNewFileChess(SPIFFS, "/Game56.txt", contentsGame);

    int m57 = Game57.length();
    char char_array57[m57 + 1];
    strcpy(char_array57, Game57.c_str());
    contentsGame = char_array57;
    createNewFileChess(SPIFFS, "/Game57.txt", contentsGame);

    int m58 = Game58.length();
    char char_array58[m58 + 1];
    strcpy(char_array58, Game58.c_str());
    contentsGame = char_array58;
    createNewFileChess(SPIFFS, "/Game58.txt", contentsGame);

    int m59 = Game59.length();
    char char_array59[m59 + 1];
    strcpy(char_array59, Game59.c_str());
    contentsGame = char_array59;
    createNewFileChess(SPIFFS, "/Game59.txt", contentsGame);

    int m60 = Game60.length();
    char char_array60[m60 + 1];
    strcpy(char_array60, Game60.c_str());
    contentsGame = char_array60;
    createNewFileChess(SPIFFS, "/Game60.txt", contentsGame);

    int m61 = Game61.length();
    char char_array61[m61 + 1];
    strcpy(char_array61, Game61.c_str());
    contentsGame = char_array61;
    createNewFileChess(SPIFFS, "/Game61.txt", contentsGame);
    
    int m62 = Game62.length();
    char char_array62[m62 + 1];
    strcpy(char_array62, Game62.c_str());
    contentsGame = char_array62;
    createNewFileChess(SPIFFS, "/Game62.txt", contentsGame);

    int m63 = Game63.length();
    char char_array63[m63 + 1];
    strcpy(char_array63, Game63.c_str());
    contentsGame = char_array63;
    createNewFileChess(SPIFFS, "/Game63.txt", contentsGame);

    int m64 = Game64.length();
    char char_array64[m64 + 1];
    strcpy(char_array64, Game64.c_str());
    contentsGame = char_array64;
    createNewFileChess(SPIFFS, "/Game64.txt", contentsGame);

    int m65 = Game65.length();
    char char_array65[m65 + 1];
    strcpy(char_array65, Game65.c_str());
    contentsGame = char_array65;
    createNewFileChess(SPIFFS, "/Game65.txt", contentsGame);

    int m66 = Game66.length();
    char char_array66[m66 + 1];
    strcpy(char_array66, Game66.c_str());
    contentsGame = char_array66;
    createNewFileChess(SPIFFS, "/Game66.txt", contentsGame);

    int m67 = Game67.length();
    char char_array67[m67 + 1];
    strcpy(char_array67, Game67.c_str());
    contentsGame = char_array67;
    createNewFileChess(SPIFFS, "/Game67.txt", contentsGame);

    int m68 = Game68.length();
    char char_array68[m68 + 1];
    strcpy(char_array68, Game68.c_str());
    contentsGame = char_array68;
    createNewFileChess(SPIFFS, "/Game68.txt", contentsGame);

    int m69 = Game69.length();
    char char_array69[m69 + 1];
    strcpy(char_array69, Game69.c_str());
    contentsGame = char_array69;
    createNewFileChess(SPIFFS, "/Game69.txt", contentsGame);

    int m70 = Game70.length();
    char char_array70[m70 + 1];
    strcpy(char_array70, Game70.c_str());
    contentsGame = char_array70;
    createNewFileChess(SPIFFS, "/Game70.txt", contentsGame);

    int m71 = Game71.length();
    char char_array71[m71 + 1];
    strcpy(char_array71, Game71.c_str());
    contentsGame = char_array71;
    createNewFileChess(SPIFFS, "/Game71.txt", contentsGame);
    
    int m72 = Game72.length();
    char char_array72[m72 + 1];
    strcpy(char_array72, Game72.c_str());
    contentsGame = char_array72;
    createNewFileChess(SPIFFS, "/Game72.txt", contentsGame);

    int m73 = Game73.length();
    char char_array73[m73 + 1];
    strcpy(char_array73, Game73.c_str());
    contentsGame = char_array73;
    createNewFileChess(SPIFFS, "/Game73.txt", contentsGame);

    int m74 = Game74.length();
    char char_array74[m74 + 1];
    strcpy(char_array74, Game74.c_str());
    contentsGame = char_array74;
    createNewFileChess(SPIFFS, "/Game74.txt", contentsGame);

    int m75 = Game75.length();
    char char_array75[m75 + 1];
    strcpy(char_array75, Game75.c_str());
    contentsGame = char_array75;
    createNewFileChess(SPIFFS, "/Game75.txt", contentsGame);

    int m76 = Game76.length();
    char char_array76[m76 + 1];
    strcpy(char_array76, Game76.c_str());
    contentsGame = char_array76;
    createNewFileChess(SPIFFS, "/Game76.txt", contentsGame);

    int m77 = Game77.length();
    char char_array77[m77 + 1];
    strcpy(char_array77, Game77.c_str());
    contentsGame = char_array77;
    createNewFileChess(SPIFFS, "/Game77.txt", contentsGame);

    int m78 = Game78.length();
    char char_array78[m78 + 1];
    strcpy(char_array78, Game78.c_str());
    contentsGame = char_array78;
    createNewFileChess(SPIFFS, "/Game78.txt", contentsGame);

    int m79 = Game79.length();
    char char_array79[m79 + 1];
    strcpy(char_array79, Game79.c_str());
    contentsGame = char_array79;
    createNewFileChess(SPIFFS, "/Game79.txt", contentsGame);

    int m80 = Game80.length();
    char char_array80[m80 + 1];
    strcpy(char_array80, Game80.c_str());
    contentsGame = char_array80;
    createNewFileChess(SPIFFS, "/Game80.txt", contentsGame);

    int m81 = Game81.length();
    char char_array81[m81 + 1];
    strcpy(char_array81, Game81.c_str());
    contentsGame = char_array81;
    createNewFileChess(SPIFFS, "/Game81.txt", contentsGame);
    
    int m82 = Game82.length();
    char char_array82[m82 + 1];
    strcpy(char_array82, Game82.c_str());
    contentsGame = char_array82;
    createNewFileChess(SPIFFS, "/Game82.txt", contentsGame);

    int m83 = Game83.length();
    char char_array83[m83 + 1];
    strcpy(char_array83, Game83.c_str());
    contentsGame = char_array83;
    createNewFileChess(SPIFFS, "/Game83.txt", contentsGame);

    int m84 = Game84.length();
    char char_array84[m84 + 1];
    strcpy(char_array84, Game84.c_str());
    contentsGame = char_array84;
    createNewFileChess(SPIFFS, "/Game84.txt", contentsGame);

    int m85 = Game85.length();
    char char_array85[m85 + 1];
    strcpy(char_array85, Game85.c_str());
    contentsGame = char_array85;
    createNewFileChess(SPIFFS, "/Game85.txt", contentsGame);

    int m86 = Game86.length();
    char char_array86[m86 + 1];
    strcpy(char_array86, Game86.c_str());
    contentsGame = char_array86;
    createNewFileChess(SPIFFS, "/Game86.txt", contentsGame);

    int m87 = Game87.length();
    char char_array87[m87 + 1];
    strcpy(char_array87, Game87.c_str());
    contentsGame = char_array87;
    createNewFileChess(SPIFFS, "/Game87.txt", contentsGame);

    int m88 = Game88.length();
    char char_array88[m88 + 1];
    strcpy(char_array88, Game88.c_str());
    contentsGame = char_array88;
    createNewFileChess(SPIFFS, "/Game88.txt", contentsGame);

    int m89 = Game89.length();
    char char_array89[m89 + 1];
    strcpy(char_array89, Game89.c_str());
    contentsGame = char_array89;
    createNewFileChess(SPIFFS, "/Game89.txt", contentsGame);

    int m90 = Game90.length();
    char char_array90[m90 + 1];
    strcpy(char_array90, Game90.c_str());
    contentsGame = char_array90;
    createNewFileChess(SPIFFS, "/Game90.txt", contentsGame);

    int m91 = Game91.length();
    char char_array91[m91 + 1];
    strcpy(char_array91, Game91.c_str());
    contentsGame = char_array91;
    createNewFileChess(SPIFFS, "/Game91.txt", contentsGame);
    
    int m92 = Game92.length();
    char char_array92[m92 + 1];
    strcpy(char_array92, Game92.c_str());
    contentsGame = char_array92;
    createNewFileChess(SPIFFS, "/Game92.txt", contentsGame);

    int m93 = Game93.length();
    char char_array93[m93 + 1];
    strcpy(char_array93, Game93.c_str());
    contentsGame = char_array93;
    createNewFileChess(SPIFFS, "/Game93.txt", contentsGame);

    int m94 = Game94.length();
    char char_array94[m94 + 1];
    strcpy(char_array94, Game94.c_str());
    contentsGame = char_array94;
    createNewFileChess(SPIFFS, "/Game94.txt", contentsGame);

    int m95 = Game95.length();
    char char_array95[m95 + 1];
    strcpy(char_array95, Game95.c_str());
    contentsGame = char_array95;
    createNewFileChess(SPIFFS, "/Game95.txt", contentsGame);

    int m96 = Game96.length();
    char char_array96[m96 + 1];
    strcpy(char_array96, Game96.c_str());
    contentsGame = char_array96;
    createNewFileChess(SPIFFS, "/Game96.txt", contentsGame);

    int m97 = Game97.length();
    char char_array97[m97 + 1];
    strcpy(char_array97, Game97.c_str());
    contentsGame = char_array97;
    createNewFileChess(SPIFFS, "/Game97.txt", contentsGame);

    int m98 = Game98.length();
    char char_array98[m98 + 1];
    strcpy(char_array98, Game98.c_str());
    contentsGame = char_array98;
    createNewFileChess(SPIFFS, "/Game98.txt", contentsGame);

    int m99 = Game99.length();
    char char_array99[m99 + 1];
    strcpy(char_array99, Game99.c_str());
    contentsGame = char_array99;
    createNewFileChess(SPIFFS, "/Game99.txt", contentsGame);
*/
}