#include <MechanismH.h>
#if MACHINE_STYLE == ROBOT_H

extern TMC2209Stepper driver;
extern TMC2209Stepper driver2;

AccelStepper stepper1(AccelStepper::DRIVER, MOTOR_0_STEP_PIN, MOTOR_0_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR_1_STEP_PIN, MOTOR_1_DIR_PIN);
MultiStepper steppers;

float valueA = 0;
float valueB = 0;
byte L;
byte H;
float xPrev = 0;
float yPrev = 0;

float xActual = 0;
float yActual = 0;

double stepsPrevM1 = 0;
double stepsPrevM2 = 0;

extern int desfaseEnX;
extern int desfaseEnY;

double globalSpeed = defGlobalSpeed;
double globalAccel = defGlobalAccel;

void deactivateElectromag(void);
void deactivateElectromagV4E(int Electro);


MechanismH::MechanismH()
{
}
// -------------------------
// moveToPoint()                 (Computes Steps)     
//--------------------------
/*
this function compute the steps needed for the two motors to move using the H-FRAME
*/
void MechanismH::moveToPoint(double xlocal, double ylocal, int bCompens){

    //Aplica la compensacion del nuevo algoritmo para centrar electroiman con los sensores
    #ifdef activateSensors
    xlocal = xlocal + desfaseEnX;
    ylocal = ylocal + desfaseEnY; 
    #endif
    //=====================================================================================


  
    
  

#ifdef debug
Serial.println("Coordinate in moveToPoint");
Serial.print("Coordinate X:  ");
Serial.println(xlocal);
Serial.print("Coordinate Y:  ");
Serial.println(ylocal);
#endif
    //===========================================================================

    //--------------------------------------------------------------
    //Local Variables
    //--------------------------------------------------------------

    double Tetha1 = 0;
    double Tetha2 = 0;

    double numberofSteps1 = 0;
    double numberofSteps2 = 0;

    double twoTimesPi = 2 * MyPI;

    //------------------------------------------------------------------------------------------//
    //Para calcular sin usar Pi, consideando que la vuelta de la polea es de 40mm
    /* double revolutionpermm1 = 0;
  double revolutionpermm2 = 0;
*/
  double numberofSteps3 = 0;
  double numberofSteps4 = 0; 
    //------------------------------------------------------------------------------------------//

    long positions[2];

    //Computing the angles in rad using the reverse kinematics 
    Tetha1 = (-xlocal+ylocal)/pulleyRadius;
    Tetha2 = (-xlocal-ylocal)/pulleyRadius;


//Making the convertion from angles to steps USANDO PI
    numberofSteps1 = (Tetha1 / (twoTimesPi))*neededStepsfor1Turn * -1;
    numberofSteps2 = (Tetha2 / (twoTimesPi))*neededStepsfor1Turn * -1;

  //------------------------------------------------------------------------------------------//
//Pruebas USANDO POLEA DE 40MNM

    /*revolutionpermm1 = (-(xlocal/40)+(ylocal/40));
    revolutionpermm2 = (-(xlocal/40)-(ylocal/40));
    */

    numberofSteps3 = ((-(xlocal*20)+(ylocal*20)))* -1;
    numberofSteps4 = ((-(xlocal*20)-(ylocal*20)))* -1;
  //------------------------------------------------------------------------------------------//

  
//------------------------------
//Asigning positions to our Matrix mulsteppers mover. USANDO POLEA DE 40MM 
    positions[0] = numberofSteps3; // Pasos_totales1;
    positions[1] = numberofSteps4;
//------------------------------
//Librarys moves steppers until tehy are on position.
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();

    //Alamacena la posicion actual para usarla en el siguiente movimiento como dato anterior
    valueA = xlocal;
    xActual = xlocal;
    

    valueB = ylocal;
    yActual = ylocal;
    
    //======================================================================================

  /*
  Serial.println("Connection Motor 1");
  uint8_t result_m1 = driver.test_connection();
	if (result_m1 == 0)
	{
		Serial.println(F("M1_OK,"));
	}
	else
	{
		Serial.println(F("M1_Fail,"));
	}
  Serial.println("Actual microsteps M1");
  Serial.println(driver.microsteps());
  Serial.println("Actual current M1");
  Serial.println(driver.rms_current());

  Serial.println("Connection Motor 2");
  uint8_t result_m2 = driver2.test_connection();
	if (result_m2 == 0)
	{
		Serial.println(F("M2_OK,"));
	}
	else
	{
		Serial.println(F("M2_Fail,"));
	}
  Serial.println("Actual microsteps M2");
  Serial.println(driver2.microsteps());
  Serial.println("Actual current M2");
  Serial.println(driver2.rms_current());

  */
}


void MechanismH::init()
{
	    // -------------------------
      // Doing Setup for Stepper motors. 
      //--------------------------
      pinMode(ENABLE_PIN, OUTPUT);                      //this initialize enable pin as output

      #ifdef CambiosPCBTavo
      digitalWrite(ENABLE_PIN, LOW);                    //WE put the enable pin to low, LOW means that TMC2208 is working.
      #endif

      #ifdef CambiosPCBLalo
      digitalWrite(ENABLE_PIN, HIGH);                    //WE put the enable pin to low, LOW means that TMC2208 is working.
      #endif

      stepper1.setCurrentPosition(0);                           //Axcell Stepper Library works with abosult positioning, we put this line to assure that when the system is started up we set the position 0,0.
      stepper2.setCurrentPosition(0);

      stepper1.setMaxSpeed(defGlobalSpeed);    //This is the maximum speed the Steppers can reach. 
      stepper2.setMaxSpeed(defGlobalSpeed);

      #ifdef relacionMicroSteps1
      stepper1.setMaxSpeed(defGlobalSpeed);    //This is the maximum speed the Steppers can reach. 
      stepper2.setMaxSpeed(defGlobalSpeed);
      #endif

      #ifdef relacionMicroSteps4
      stepper1.setMaxSpeed(defGlobalSpeed);    //This is the maximum speed the Steppers can reach. 
      stepper2.setMaxSpeed(defGlobalSpeed);
      #endif

      #ifdef relacionMicroSteps16
      stepper1.setMaxSpeed(defGlobalSpeed);    //This is the maximum speed the Steppers can reach. 
      stepper2.setMaxSpeed(defGlobalSpeed);
      #endif

       #ifdef relacionMicroSteps32
      stepper1.setMaxSpeed(defGlobalSpeed);    //This is the maximum speed the Steppers can reach. 
      stepper2.setMaxSpeed(defGlobalSpeed);
      #endif

       #ifdef relacionMicroSteps64
      stepper1.setMaxSpeed(defGlobalSpeed);    //This is the maximum speed the Steppers can reach. 
      stepper2.setMaxSpeed(defGlobalSpeed);
      #endif

      #ifdef relacionMicroSteps128
      stepper1.setMaxSpeed(defGlobalSpeed);    //This is the maximum speed the Steppers can reach. 
      stepper2.setMaxSpeed(defGlobalSpeed);
      #endif

      steppers.addStepper(stepper1);                            //We also use a library called Multistteper that allow us to manipulate our robot in a less complex way using Matrix concepts.
      steppers.addStepper(stepper2);



/*/
//Test de recorrer varias posiciones a la misma velocidad
//======================================================= 
long positions[2]; // Array of desired stepper positions
for (int i = 0; i < 75; i++)
{
  positions[0] = vect1[i];
  positions[1] = vect2[i];
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
}
*/

//=======================================================
     //=====Agregado para lo de la calibracion intermedia===
      xActual = 0;
      yActual = 0;     
      //====================================================
}

void MechanismH::reInitVariables()
{
  stepper1.setCurrentPosition(0);                           //Axcell Stepper Library works with abosult positioning, we put this line to assure that when the system is started up we set the position 0,0.
  stepper2.setCurrentPosition(0);

  xActual = 0;
  yActual = 0;

  stepsPrevM1 = 0;
  stepsPrevM2 = 0;
  
  desfaseEnX = 0;
  desfaseEnY = 0;

}

void MechanismH::getActualPosition(double *actualPosX, double *actualPosY){
  *actualPosX = xActual;
  *actualPosY = yActual;
}


void MechanismH::setSpeedMotors(int actualSpeedMotors)
{
      stepper1.setMaxSpeed(actualSpeedMotors);    //This is the maximum speed the Steppers can reach. 
      stepper2.setMaxSpeed(actualSpeedMotors);
}

void MechanismH::setSpeedRampFunction(double SpeedMotors)
{
      globalSpeed = SpeedMotors;
}

void MechanismH::setAccelRampFunction(double AccelMotors)
{
      globalAccel = AccelMotors;
}


void MechanismH::accelRamp(double xlocal, double ylocal, int typeMove, int bandElectro, int numElectro)
{

//Desfases para la rotacion Original
/*
  if(numElectro == 1)
  {
   ylocal = ylocal - 20;   //
  }
  if(numElectro == 2)
  {
    //xlocal = xlocal - 37;
     xlocal = xlocal - 40; //-
    ylocal = ylocal - 10; //-
  }
  if(numElectro == 3)
  {
   ylocal = ylocal +   25;
  }
  if(numElectro == 4)
  {
    xlocal = xlocal + 40; //+
    ylocal = ylocal - 10;//-
  }
  */

  //Desfases para la rotacion a 90 grados
  if(numElectro == 1)
  {
    xlocal = xlocal + 20;   //
  }
  if(numElectro == 2)
  {
    //xlocal = xlocal - 37;
    ylocal = ylocal - 40; //-
    xlocal = xlocal + 10; //-
  }
  if(numElectro == 3)
  {
   xlocal = xlocal -   25;
  }
  if(numElectro == 4)
  {
    ylocal = ylocal + 40; //+
    xlocal = xlocal + 10;//-
  }

      int bandElectromagnet = 0;
      double accelStepsM1;
      double accelStepsM2;
      double speedM1;
      double speedM2;
      double speedLimitM1;
      double speedLimitM2;
      double accelM1;
      double accelM2;
      double aumentoVelCada10msM1;
      double aumentoVelCada10msM2;

      float relacionDeDistancias;

      double aceleracionDef;
      double velocidadDef;
      double aumentoVelCada10ms;
      // double tiempoEnAceleracion;
      double avanceEnAceleracion;

      double minSpeed;
      //--------------------------------------------------------------
      // Local Variables
      //--------------------------------------------------------------

      double Tetha1 = 0;
      double Tetha2 = 0;

      double numberofSteps1 = 0;
      double numberofSteps2 = 0;

      double twoTimesPi = 2 * MyPI;

      //------------------------------------------------------------------------------------------//
      // Para calcular sin usar Pi, consideando que la vuelta de la polea es de 40mm
      // double revolutionpermm1 = 0;
    //double revolutionpermm2 = 0;
    //
      double numberofSteps3 = 0;
      double numberofSteps4 = 0;
      //------------------------------------------------------------------------------------------//

      long positions[2];

      // Aplica la compensacion del nuevo algoritmo para centrar electroiman con los sensores
#ifdef activateSensors
      xlocal = xlocal + desfaseEnX;
      ylocal = ylocal + desfaseEnY;
#endif
      //=====================================================================================

      // Computing the angles in rad using the reverse kinematics
      Tetha1 = (-xlocal + ylocal) / pulleyRadius;
      Tetha2 = (-xlocal - ylocal) / pulleyRadius;

      // Making the convertion from angles to steps USANDO PI
      numberofSteps1 = (Tetha1 / (twoTimesPi)) * neededStepsfor1Turn * -1;
      numberofSteps2 = (Tetha2 / (twoTimesPi)) * neededStepsfor1Turn * -1;

      //------------------------------------------------------------------------------------------//
      // Pruebas USANDO POLEA DE 40MNM

      //revolutionpermm1 = (-(xlocal/40)+(ylocal/40));
      //revolutionpermm2 = (-(xlocal/40)-(ylocal/40));
      //
#ifdef relacionMicroSteps1
#ifdef CambiosPCBTavo
  numberofSteps3 = ((-(xlocal * 5) - (ylocal * 5))) * -1;    //Para mi configuracion numberofSteps3 = ((-(xlocal * 20) + (ylocal * 20))) * -1; 
  numberofSteps4 = ((-(xlocal * 5) + (ylocal * 5))) * -1;    //Para mi configuracion numberofSteps4 = ((-(xlocal * 20) - (ylocal * 20))) * -1;
  #endif
#ifdef CambiosPCBLalo
  numberofSteps3 = ((-(xlocal * 5) - (ylocal * 5))) * -1;
  numberofSteps4 = ((-(xlocal * 5) + (ylocal * 5))) * -1;
#endif
  #endif


#ifdef relacionMicroSteps4
#ifdef CambiosPCBTavo
  numberofSteps3 = ((-(xlocal * 20) - (ylocal * 20))) * -1;    //Para mi configuracion numberofSteps3 = ((-(xlocal * 20) + (ylocal * 20))) * -1; 
  numberofSteps4 = ((-(xlocal * 20) + (ylocal * 20))) * -1;    //Para mi configuracion numberofSteps4 = ((-(xlocal * 20) - (ylocal * 20))) * -1;
  #endif
#ifdef CambiosPCBLalo
  numberofSteps3 = ((-(xlocal * 20) - (ylocal * 20))) * -1;
  numberofSteps4 = ((-(xlocal * 20) + (ylocal * 20))) * -1;
#endif
  #endif

  #ifdef relacionMicroSteps16
  numberofSteps3 = ((-(xlocal * 80) - (ylocal * 80))) * -1;    
  numberofSteps4 = ((-(xlocal * 80) + (ylocal * 80))) * -1; 
#endif

#ifdef relacionMicroSteps32
  numberofSteps3 = ((-(xlocal * 160) - (ylocal * 160))) * -1;    
  numberofSteps4 = ((-(xlocal * 160) + (ylocal * 160))) * -1; 
#endif

#ifdef relacionMicroSteps64
#ifdef CambiosPCBTavo
  numberofSteps3 = ((-(xlocal * 320) + (ylocal * 320))) * 1;    //Para mi configuracion numberofSteps3 = ((-(xlocal * 20) + (ylocal * 20))) * -1; 
  numberofSteps4 = ((-(xlocal * 320) - (ylocal * 320))) * 1;    //Para mi configuracion numberofSteps4 = ((-(xlocal * 20) - (ylocal * 20))) * -1;
  #endif
#ifdef CambiosPCBLalo
  numberofSteps3 = ((-(xlocal * 320) + (ylocal * 320))) * 1;
  numberofSteps4 = ((-(xlocal * 320) - (ylocal * 320))) * 1;
#endif
  #endif


#ifdef relacionMicroSteps128
#ifdef CambiosPCBTavo
  numberofSteps3 = ((-(xlocal * 640) + (ylocal * 640))) * 1;    //Para mi configuracion numberofSteps3 = ((-(xlocal * 20) + (ylocal * 20))) * -1; 
  numberofSteps4 = ((-(xlocal * 640) - (ylocal * 640))) * 1;    //Para mi configuracion numberofSteps4 = ((-(xlocal * 20) - (ylocal * 20))) * -1;
  #endif
#ifdef CambiosPCBLalo
  numberofSteps3 = ((-(xlocal * 640) + (ylocal * 640))) * 1;
  numberofSteps4 = ((-(xlocal * 640) - (ylocal * 640))) * 1;
#endif
  #endif

/*
Serial.println("Coordenadas destino");
  Serial.println("xActual");
  Serial.println(xlocal);
  Serial.println("yActual");
  Serial.println(ylocal);
  */
  //------------------------------------------------------------------------------------------//
 /* 
  Serial.println("Coordenadas actuales");
  Serial.println("xActual");
  Serial.println(xActual);
  Serial.println("yActual");
  Serial.println(yActual);

  Serial.println("Coordenadas destino");
  Serial.println("xActual");
  Serial.println(xlocal);
  Serial.println("yActual");
  Serial.println(ylocal);
  */
#ifdef accelRampDebug //Ultimo mensaje que se comento en esta funcion, para ver solo las impresiones de los tableros virtuales y las jugadas
  Serial.println("DATOS PARA CALCULO DE POSICION ELECTROIMAN");
  Serial.println("============================================");

  Serial.println("Coordenadas actuales");
  Serial.println("xActual");
  Serial.println(xActual);
  Serial.println("yActual");
  Serial.println(yActual);

  Serial.println("Coordenadas destino");
  Serial.println("xActual");
  Serial.println(xlocal);
  Serial.println("yActual");
  Serial.println(ylocal);
#endif

#ifdef accelRampDebug
  //=====================================================================
  Serial.println("DATOS PARA CALCULO DE POSICION ELECTROIMAN");
  Serial.println("============================================");

  Serial.println("Coordenadas actuales");
  Serial.println("xActual");
  Serial.println(xActual);
  Serial.println("yActual");
  Serial.println(yActual);

  Serial.println("Coordenadas destino");
  Serial.println("xActual");
  Serial.println(xlocal);
  Serial.println("yActual");
  Serial.println(ylocal);
#endif


  //Serial.println("Pasos calculados por la ecuaciones de movimiento: ");
  //Serial.println("Steps Motor 1");
  //Serial.println(numberofSteps3);
  //Serial.println("Steps Motor 2");
  //Serial.println(numberofSteps4);

  //Serial.println("============================================");
  

   //=================================================================
     //Se van a calcular las coordenadas objetivo previas a la posicion final 
     //con el desfase definido enn la variable "desfaseElectroiman" dada en mm
    
    double compXanterior = 0;
    double compYanterior = 0;
    
    compXanterior = xlocal;
    compYanterior = ylocal;

    double desfase;
    desfase = desfaseElectroiman;
    if((xActual != xlocal)&&(yActual != ylocal))
    {
        desfase = sqrt((desfaseElectroiman*desfaseElectroiman)/2);
    }

    if(xActual > xlocal)
    {
        compXanterior = compXanterior + desfase;
    }
    if(xActual < xlocal)
    {
        compXanterior = compXanterior - desfase;
    }

    if(yActual > ylocal)
    {
        compYanterior = compYanterior + desfase;
    }
    if(yActual < ylocal)
    {
        compYanterior = compYanterior - desfase;
    }
    //#ifdef funcMoveTo
    //Robot.moveToPoint(compXanterior,compYanterior,1);
    //#endif


#ifdef accelRampDebug
    Serial.println("Posicion Previa al punto final (Donde se apaga el electroiman)");
    
    Serial.println("Coordenada Previa X");
    Serial.println(compXanterior);

    Serial.println("Coordenada Previa Y");
    Serial.println(compYanterior);
#endif


    //--------------------------------------------------------------
  // Local Variables
  //--------------------------------------------------------------

  double numberofStepsM3 = 0;
  double numberofStepsM4 = 0;
  //------------------------------------------------------------------------------------------//

   //Aplica la compensacion del nuevo algoritmo para centrar electroiman con los sensores
    #ifdef activateSensors
    //xlocal = xlocal + desfaseEnX;
    //ylocal = ylocal + desfaseEnY;
    #endif
    //=====================================================================================
  #ifdef relacionMicroSteps1
  numberofStepsM3 = ((-(compXanterior * 5) - (compYanterior * 5))) * -1;
  numberofStepsM4 = ((-(compXanterior * 5) + (compYanterior * 5))) * -1;
  #endif

  #ifdef relacionMicroSteps4
  numberofStepsM3 = ((-(compXanterior * 20) - (compYanterior * 20))) * -1;
  numberofStepsM4 = ((-(compXanterior * 20) + (compYanterior * 20))) * -1;
  #endif

  #ifdef relacionMicroSteps16
  numberofStepsM3 = ((-(compXanterior * 80) - (compYanterior * 80))) * -1;
  numberofStepsM4 = ((-(compXanterior * 80) + (compYanterior * 80))) * -1;
  #endif

  #ifdef relacionMicroSteps32
  numberofStepsM3 = ((-(compXanterior * 160) - (compYanterior * 160))) * -1;
  numberofStepsM4 = ((-(compXanterior * 160) + (compYanterior * 160))) * -1;
  #endif

  #ifdef relacionMicroSteps64
  numberofStepsM3 = ((-(compXanterior * 320) + (compYanterior * 320))) * 1;
  numberofStepsM4 = ((-(compXanterior * 320) - (compYanterior * 320))) * 1;
  #endif

  #ifdef relacionMicroSteps128
  numberofStepsM3 = ((-(compXanterior * 640) + (compYanterior * 640))) * 1;
  numberofStepsM4 = ((-(compXanterior * 640) - (compYanterior * 640))) * 1;
  #endif
  //------------------------------------------------------------------------------------------//

#ifdef accelRampDebug
  Serial.println("Pasos calculados para la posicion previa al punto final");
  //Serial.println("Pasos prev X ");
  //Serial.println(numberofStepsM3);
  //Serial.println("Pasos prev Y ");
  //Serial.println(numberofStepsM4);
#endif

  numberofStepsM3 = numberofStepsM3 - stepsPrevM1;
  numberofStepsM4 = numberofStepsM4 - stepsPrevM2;

  numberofStepsM3 = abs(numberofStepsM3);   
  numberofStepsM4 = abs(numberofStepsM4);

  numberofSteps3 = round(numberofSteps3);
  numberofSteps4 = round(numberofSteps4);

#ifdef accelRampDebug
  Serial.println("Pasos prev X ");
  Serial.println(numberofStepsM3);
  Serial.println("Pasos prev Y ");
  Serial.println(numberofStepsM4);
#endif

    
    //=====================================================================



  #ifdef accelRampDebug
  //===================================================================
  Serial.println("Pasos calculados por la ecuaciones de movimiento: ");
  Serial.println("Steps Motor 1");
  Serial.println(numberofSteps3);
  Serial.println("Steps Motor 2");
  Serial.println(numberofSteps4);
  //=====================================================================
  #endif

  double tempStepsPrev1;
  double tempStepsPrev2;
  tempStepsPrev1 = numberofSteps3;
  tempStepsPrev2 = numberofSteps4;

  
  velocidadDef = globalSpeed;    //32000 8000 Velocidad en steps/segundo     //Maxima velocidad 8,000      -Minima 300 (un 5 porciento de la aceleracion definida)
  aceleracionDef = globalAccel;  //32000 16000 Aceleracion en steps/segundo^2   //Maxima aceleracion 32,000
  
  #ifdef relacionMicroSteps1
  minSpeed = (aceleracionDef/10)*2;
  #endif

  #ifdef relacionMicroSteps4
  minSpeed = (aceleracionDef/100)*2;
  #endif

  #ifdef relacionMicroSteps16
  minSpeed = (aceleracionDef/100)*2;
  #endif

  #ifdef relacionMicroSteps32
  minSpeed = (aceleracionDef/100)*2;
  #endif

  #ifdef relacionMicroSteps64
  minSpeed = (aceleracionDef/10)*2;
  #endif

  #ifdef relacionMicroSteps128
  minSpeed = (aceleracionDef/10)*2;
  #endif
  
  //Serial.println("Numero de pasos anteriores: ");
  //Serial.println("Steps Motor 1");
  //Serial.println(stepsPrevM1);
  //Serial.println("Steps Motor 2");
  //Serial.println(stepsPrevM2);
  

  numberofSteps3 = numberofSteps3 - stepsPrevM1;
  numberofSteps4 = numberofSteps4 - stepsPrevM2;
  
  numberofSteps3 = round(numberofSteps3);
  numberofSteps4 = round(numberofSteps4);

  stepsPrevM1 = tempStepsPrev1;
  stepsPrevM2 = tempStepsPrev2;
  
  /*
  Serial.println("Pasos recalculados de forma absoluta: ");
  Serial.println("Steps Motor 1");
  Serial.println(numberofSteps3);
  Serial.println("Steps Motor 2");
  Serial.println(numberofSteps4);
  */
  
#ifdef pinoutv1
  if(numberofSteps3 >= 0)
  {
    stepper1.setPinsInverted(false);
  }
  else
  {
    stepper1.setPinsInverted(true);
  }
  if(numberofSteps4 >= 0)
  {
    stepper2.setPinsInverted(false);
  }
  else
  {
    stepper2.setPinsInverted(true);
  }
  #endif

  #ifdef pinoutv2
  if(numberofSteps3 >= 0)
  {
    stepper1.setPinsInverted(true);   //Para mi configuracion true
  }
  else
  {
    stepper1.setPinsInverted(false);  // Para mi configuracion false
  }
  if(numberofSteps4 >= 0)
  {
    stepper2.setPinsInverted(false);   //Para mi configuracion true
  }
  else
  {
    stepper2.setPinsInverted(true);  //Para mi configuracion false
  }
  #endif

  #ifdef pinoutv3
  if(numberofSteps3 >= 0)
  {
    stepper1.setPinsInverted(true); //true
  }
  else
  {
    stepper1.setPinsInverted(false); //false
  }
  if(numberofSteps4 >= 0)
  {
    stepper2.setPinsInverted(true); //true
  }
  else
  {
    stepper2.setPinsInverted(false); //false
  }
  #endif

  accelStepsM1 = abs(numberofSteps3);   //Pasos que debe dar el motor 1 para llegar a la coordenada objetivo
  accelStepsM2 = abs(numberofSteps4);   //Pasos que debe dar el motor 2 para llegar a la coordenada objetivo

  accelStepsM1 = round(accelStepsM1);
  accelStepsM2 = round(accelStepsM2);

  //tiempoEnAceleracion = (velocidadDef/aceleracionDef);    //calculamos cuanto tiempo le toma alcanzar la velocidad definida
  
  float speedTest;

  unsigned long tiempo1;
  unsigned long tiempo2;
  unsigned long lapseTime;

  accelM1 = aceleracionDef;
  accelM2 = aceleracionDef;

  speedM1 = velocidadDef;

  
  //Serial.println("Aumento de velocidad cada 10ms");
  //Serial.println(aumentoVelCada10ms);
  //Serial.println("Tiempo en aceleracion");
  //Serial.println(tiempoEnAceleracion);

  //speedTest = speedTest + aumentoVelCada10ms;
  //stepper1.setSpeed(speedTest);
  //stepper2.setSpeed(speedTest);
  //tiempo1 = millis();
  

if(typeMove == 1)
{

  accelM1 = aceleracionDef;
  accelM2 = aceleracionDef;

  speedM1 = velocidadDef;
  speedM2 = velocidadDef;

  //speedTest = 0;
  speedM1 = 0;
  speedM2 = 0;
  
  speedLimitM1 = velocidadDef;
  speedLimitM2 = velocidadDef;
  
  //stepper1.setSpeed(speedTest);
  //stepper2.setSpeed(speedTest);
  stepper1.setSpeed(speedM1);
  stepper2.setSpeed(speedM2);

  if (accelStepsM1 > accelStepsM2)
  {
    // La mayor distancia la recorre el motor 1
    relacionDeDistancias = abs(accelStepsM2 / accelStepsM1); // Me da la relacion para saber que tan lento debe ir el motor 2 respecto del 1
    // Serial.print("Relacion de distancias: ");
    // Serial.println(relacionDeDistancias);
    speedLimitM2 = velocidadDef * relacionDeDistancias;
    // tiempoAccelM1 = (speedMotor1)/(accelMotor1);
    accelM2 = aceleracionDef * relacionDeDistancias;
  }
  else
  {
    // La mayor distancia la recorre el motor 2
    relacionDeDistancias = abs(accelStepsM1 / accelStepsM2);

    speedLimitM1 = velocidadDef * relacionDeDistancias;
    accelM1 = aceleracionDef * relacionDeDistancias;

   #ifdef accelRampDebug
   Serial.println("speedLimitM1");
   Serial.println(speedLimitM1);
   Serial.println("accelM1");
   Serial.println(accelM1);
   Serial.println("Relacion de distancias");
   Serial.println(relacionDeDistancias); 
    #endif
  }

  //aumentoVelCada10ms = (aceleracionDef/100);              //calculamos el aumento de la velocidad cada 10ms
  #ifdef relacionMicroSteps1
   aumentoVelCada10msM1 = (accelM1/10);               
   aumentoVelCada10msM2 = (accelM2/10);
  #endif
  #ifdef relacionMicroSteps4
   aumentoVelCada10msM1 = (accelM1/100);               
   aumentoVelCada10msM2 = (accelM2/100);
  #endif

  #ifdef relacionMicroSteps16
   aumentoVelCada10msM1 = (accelM1/100);               
   aumentoVelCada10msM2 = (accelM2/100);
  #endif

  #ifdef relacionMicroSteps32
   aumentoVelCada10msM1 = (accelM1/100);               
   aumentoVelCada10msM2 = (accelM2/100);
  #endif

  #ifdef relacionMicroSteps64
   aumentoVelCada10msM1 = (accelM1/10);               
   aumentoVelCada10msM2 = (accelM2/10);
  #endif

  #ifdef relacionMicroSteps128
   aumentoVelCada10msM1 = (accelM1/10);               
   aumentoVelCada10msM2 = (accelM2/10);
  #endif

   #ifdef accelRampDebug
   Serial.println("aumentoVelCada10msM1");
   Serial.println(aumentoVelCada10msM1);
   Serial.println("aumentoVelCada10msM2");
   Serial.println(aumentoVelCada10msM2);   
   #endif 
//============= Movimiento en aceleracion hasta alcanzar la velocidad definida o la posicion ebjetivo ========================

  while (true)
  {
    if(stepper1.currentPosition() == accelStepsM1)
    {
      stepper1.stop();
    }
    else
    {
      stepper1.runSpeed();
    }

    if(stepper2.currentPosition() == accelStepsM2)
    {
      stepper2.stop();
    }
    else
    {
      stepper2.runSpeed();
    }
    //Serial.println(stepper1.currentPosition());
    //Serial.println(speedTest);
    
    tiempo2 = millis();
    lapseTime = tiempo2 - tiempo1;
    if(lapseTime >= 10)
    {
      //speedTest = speedTest + aumentoVelCada10ms;
      speedM1 = speedM1 + aumentoVelCada10msM1;
      speedM2 = speedM2 + aumentoVelCada10msM2;
      
      stepper1.setSpeed(speedM1);
      stepper2.setSpeed(speedM2);
      tiempo1 = millis();
    }
    
    if(((speedM1 == speedLimitM1) && (speedM2 == speedLimitM2)) || ((stepper1.currentPosition() == accelStepsM1) && (stepper2.currentPosition() == accelStepsM2)))
    {
      break;
    }
  }



  #ifdef accelRampDebug
  Serial.println("Posicion despues de aceleracion M1");
  Serial.println(stepper1.currentPosition());
  Serial.println("Posicion despues de aceleracion M2");
  Serial.println(stepper2.currentPosition());

  Serial.println("Velocidad despues de aceleracion M1");
  Serial.println(speedM1);

  Serial.println("Velocidad despues de aceleracion M2");
  Serial.println(speedM2);
  #endif

  //======================  Movimiento en aceleracion constante  ====================================
  if (stepper1.currentPosition() != accelStepsM1 || stepper2.currentPosition() != accelStepsM2)
  {
    while (true)
    {
      //Serial.println(stepper1.currentPosition());
      //Serial.println(stepper2.currentPosition());
      // Serial.println(speedTest);
      if (stepper1.currentPosition() == accelStepsM1)
      {
        stepper1.stop();
      }
      else
      {
        stepper1.runSpeed();
      }

      if (stepper2.currentPosition() == accelStepsM2)
      {
        stepper2.stop();
      }
      else
      {
        stepper2.runSpeed();
      }
      if (((stepper1.currentPosition() == accelStepsM1) && (stepper2.currentPosition() == accelStepsM2))) // checar condicion, buscar que estas condicionnes dependan de ambos motores
      {
        break;
      }
    }
  }

  #ifdef accelRampDebug
  Serial.println("Velocidad final M1");
  Serial.println(speedM1);

  Serial.println("Velocidad final M2");
  Serial.println(speedM2);
  #endif
}
//====================================================================================================

if(typeMove == 0)
{
  accelM1 = aceleracionDef;
  accelM2 = aceleracionDef;

  speedM1 = velocidadDef;
  speedM2 = velocidadDef;

  if (accelStepsM1 > accelStepsM2)
  {
    // La mayor distancia la recorre el motor 1
    relacionDeDistancias = abs(accelStepsM2 / accelStepsM1);
    speedM2 = speedM2 * relacionDeDistancias;
  }
  else
  {
    // La mayor distancia la recorre el motor 2
    relacionDeDistancias = abs(accelStepsM1 / accelStepsM2);
    speedM1 = speedM1 * relacionDeDistancias;
  }
  stepper1.setSpeed(speedM1);
  stepper2.setSpeed(speedM2);

  //========  Movimiento en aceleracion constante  ========
  if (stepper1.currentPosition() != accelStepsM1 || stepper2.currentPosition() != accelStepsM2)
  {
    while (true)
    {
      // Serial.println(stepper1.currentPosition());
      // Serial.println(speedTest);
      if (stepper1.currentPosition() == accelStepsM1)
      {
        stepper1.stop();
      }
      else
      {
        stepper1.runSpeed();
      }

      if (stepper2.currentPosition() == accelStepsM2)
      {
        stepper2.stop();
      }
      else
      {
        stepper2.runSpeed();
      }
      if (((stepper1.currentPosition() == accelStepsM1) && (stepper2.currentPosition() == accelStepsM2))) // checar condicion, buscar que estas condicionnes dependan de ambos motores
      {
        break;
      }
    }
  }
}
//====================================================================================================

if(typeMove == -1)
{
  double stepsEnDesaceleracionM1;
  double stepsEnDesaceleracionM2;
  double stepsEnAcelCteM1;
  double stepsEnAcelCteM2;

  double lapseTime = 0.01;

  double tiempoEnAceleracionM1;
  double tiempoEnAceleracionM2;
  double totalLapseTimesM1;
  double totalLapseTimesM2;

  double aInicial;
  double aFinal;

  accelM1 = aceleracionDef;
  accelM2 = aceleracionDef;

  speedM1 = velocidadDef;
  speedM2 = velocidadDef;

  // se calcula la relacion de velocidades para que ambos motores
  if (accelStepsM1 > accelStepsM2)
  {
    // La mayor distancia la recorre el motor 1
    relacionDeDistancias = abs(accelStepsM2 / accelStepsM1);
    speedM2 = speedM2 * relacionDeDistancias;
  }
  else
  {
    // La mayor distancia la recorre el motor 2
    relacionDeDistancias = abs(accelStepsM1 / accelStepsM2);
    speedM1 = speedM1 * relacionDeDistancias;
  }

  // Se calculan los pasos que debera recorrer cada motor en desaceleracion.
  tiempoEnAceleracionM1 = velocidadDef / aceleracionDef;
  totalLapseTimesM1 = tiempoEnAceleracionM1 / lapseTime;
  aumentoVelCada10msM1 = speedM1 / totalLapseTimesM1;

  aInicial = aumentoVelCada10msM1 * (lapseTime);
  aFinal = aumentoVelCada10msM1 * (tiempoEnAceleracionM1);
  stepsEnDesaceleracionM1 = (totalLapseTimesM1) * ((aInicial + aFinal) / (2));

  stepsEnAcelCteM1 = accelStepsM1 - stepsEnDesaceleracionM1;

  tiempoEnAceleracionM2 = velocidadDef / aceleracionDef;
  totalLapseTimesM2 = tiempoEnAceleracionM2 / lapseTime;
  aumentoVelCada10msM2 = speedM2 / totalLapseTimesM2;

  aInicial = aumentoVelCada10msM2 * (lapseTime);
  aFinal = aumentoVelCada10msM2 * (tiempoEnAceleracionM2);
  stepsEnDesaceleracionM2 = (totalLapseTimesM2) * ((aInicial + aFinal) / (2));

  stepsEnAcelCteM2 = accelStepsM2 - stepsEnDesaceleracionM2;

#ifdef accelRampDebug
  Serial.println("Pasos calculados en acelereacion cte M1: ");
  Serial.println(stepsEnAcelCteM1);
  Serial.println("Pasos calculados en acelereacion cte M2: ");
  Serial.println(stepsEnAcelCteM2);

  Serial.println("Pasos calculados en desaceleracionn M1: ");
  Serial.println(stepsEnDesaceleracionM1);
  Serial.println("Pasos calculados en desaceleracion M2: ");
  Serial.println(stepsEnDesaceleracionM2);

  Serial.println("Velocidad inicial M1: ");
  Serial.println(speedM1);
  Serial.println("Velocidad inicial M2: ");
  Serial.println(speedM2);

  Serial.println("AumentoVelCada10msM1");
  Serial.println(aumentoVelCada10msM1);

  Serial.println("AumentoVelCada10msM2");
  Serial.println(aumentoVelCada10msM2);
#endif

  //===========================================================
  stepper1.setSpeed(speedM1);
  stepper2.setSpeed(speedM2);
  //========  Movimiento en aceleracion constante  ========
  if (stepper1.currentPosition() != stepsEnAcelCteM1 || stepper2.currentPosition() != stepsEnAcelCteM2)
  {
    while (true)
    {
      // Serial.println(stepper1.currentPosition());
      // Serial.println(speedTest);
      if (stepper1.currentPosition() >= stepsEnAcelCteM1)
      {
        stepper1.stop();
      }
      else
      {
        stepper1.runSpeed();
      }

      if (stepper2.currentPosition() >= stepsEnAcelCteM2)
      {
        stepper2.stop();
      }
      else
      {
        stepper2.runSpeed();
      }
      if (((stepper1.currentPosition() >= stepsEnAcelCteM1) && (stepper2.currentPosition() >= stepsEnAcelCteM2))) // checar condicion, buscar que estas condicionnes dependan de ambos motores
      {
        break;
      }
    }
  }
#ifdef accelRampDebug
  Serial.println("Posicion despues de aceleracion cte M1");
  Serial.println(stepper1.currentPosition());
  Serial.println("Posicion despues de aceleracion cte M2");
  Serial.println(stepper2.currentPosition());
#endif

  //============= Movimiento en desaceleracion hasta alcanzar la velocidad definida o la posicion ebjetivo ========================

  while (true)
  {
    if (stepper1.currentPosition() == accelStepsM1)
    {
      stepper1.stop();
    }
    else
    {
      stepper1.runSpeed();
    }

    if (stepper2.currentPosition() == accelStepsM2)
    {
      stepper2.stop();
    }
    else
    {
      stepper2.runSpeed();
    }
    // Serial.println(stepper1.currentPosition());
    // Serial.println(speedTest);

    tiempo2 = millis();
    lapseTime = tiempo2 - tiempo1;
    if (lapseTime >= 10)
    {
      // speedTest = speedTest + aumentoVelCada10ms;
      speedM1 = speedM1 - aumentoVelCada10msM1;
      speedM2 = speedM2 - aumentoVelCada10msM2;

      stepper1.setSpeed(speedM1);
      stepper2.setSpeed(speedM2);
      tiempo1 = millis();
    }

    if (speedM1 < minSpeed)
    {
      speedM1 = minSpeed;
    }
    if (speedM2 < minSpeed)
    {
      speedM2 = minSpeed;
    }

    if ((stepper1.currentPosition() == accelStepsM1) && (stepper2.currentPosition() == accelStepsM2))
    {
      break;
    }
  }
}

//==============================================================================================================================
if(typeMove == 2)
{
  double puntoMedioTrayectoriaM1;
  double pasosEnAcelereacionM1;
  double puntoMedioTrayectoriaM2;
  double pasosEnAcelereacionM2;

  double limiteAccelCteM1;
  double limiteAccelCteM2;

  puntoMedioTrayectoriaM1 = accelStepsM1 /2;
  puntoMedioTrayectoriaM2 = accelStepsM2 /2;

  accelM1 = aceleracionDef;
  accelM2 = aceleracionDef;

  speedM1 = velocidadDef;
  speedM2 = velocidadDef;

  //speedTest = 0;
  speedM1 = 0;
  speedM2 = 0;
  
  speedLimitM1 = velocidadDef;
  speedLimitM2 = velocidadDef;
  
  //stepper1.setSpeed(speedTest);
  //stepper2.setSpeed(speedTest);
  stepper1.setSpeed(speedM1);
  stepper2.setSpeed(speedM2);

  if (accelStepsM1 > accelStepsM2)
  {
    // La mayor distancia la recorre el motor 1
    relacionDeDistancias = abs(accelStepsM2 / accelStepsM1); // Me da la relacion para saber que tan lento debe ir el motor 2 respecto del 1
    // Serial.print("Relacion de distancias: ");
    // Serial.println(relacionDeDistancias);
    speedLimitM2 = velocidadDef * relacionDeDistancias;
    // tiempoAccelM1 = (speedMotor1)/(accelMotor1);
    accelM2 = aceleracionDef * relacionDeDistancias;
  }
  else
  {
    // La mayor distancia la recorre el motor 2
    relacionDeDistancias = abs(accelStepsM1 / accelStepsM2);

    speedLimitM1 = velocidadDef * relacionDeDistancias;
    accelM1 = aceleracionDef * relacionDeDistancias;

   #ifdef accelRampDebug
   Serial.println("speedLimitM1");
   Serial.println(speedLimitM1);
   Serial.println("accelM1");
   Serial.println(accelM1);
   Serial.println("speedLimitM2");
   Serial.println(speedLimitM2);
   Serial.println("accelM2");
   Serial.println(accelM2);
   Serial.println("Relacion de distancias");
   Serial.println(relacionDeDistancias); 
    #endif
  }

  //aumentoVelCada10ms = (aceleracionDef/100);              //calculamos el aumento de la velocidad cada 10ms
   #ifdef relacionMicroSteps1
   aumentoVelCada10msM1 = (accelM1/10);               
   aumentoVelCada10msM2 = (accelM2/10);
   #endif
  #ifdef relacionMicroSteps4
   aumentoVelCada10msM1 = (accelM1/100);               
   aumentoVelCada10msM2 = (accelM2/100);
   #endif

   #ifdef relacionMicroSteps16
   aumentoVelCada10msM1 = (accelM1/100);               
   aumentoVelCada10msM2 = (accelM2/100);
   #endif

   #ifdef relacionMicroSteps32
   aumentoVelCada10msM1 = (accelM1/100);               
   aumentoVelCada10msM2 = (accelM2/100);
   #endif

   #ifdef relacionMicroSteps64
   aumentoVelCada10msM1 = (accelM1/10);               
   aumentoVelCada10msM2 = (accelM2/10);
   #endif

   #ifdef relacionMicroSteps128
   aumentoVelCada10msM1 = (accelM1/10);               
   aumentoVelCada10msM2 = (accelM2/10);
   #endif

   #ifdef accelRampDebug
   Serial.println("aumentoVelCada10msM1");
   Serial.println(aumentoVelCada10msM1);
   Serial.println("aumentoVelCada10msM2");
   Serial.println(aumentoVelCada10msM2);   
   #endif 
//============= Movimiento en aceleracion hasta alcanzar la velocidad definida o la posicion ebjetivo ========================

  while (true)
  {
    //------------------Condiciones para activar electroiman-----------
    if (bandElectro == 1)
    {
      if (bandElectromagnet == 0)
      {
        if (stepper1.currentPosition() > numberofStepsM3 || stepper2.currentPosition() > numberofStepsM4)
        {
          #ifdef version4Electro
          deactivateElectromagV4E(numElectro);
          #endif
          bandElectromagnet = 1;
        }
      }
    }
    //----------------------------------------------------------------

    if(stepper1.currentPosition() == puntoMedioTrayectoriaM1)
    {
      stepper1.stop();
    }
    else
    {
      stepper1.runSpeed();
    }

    if(stepper2.currentPosition() == puntoMedioTrayectoriaM2)
    {
      stepper2.stop();
    }
    else
    {
      stepper2.runSpeed();
    }
    //Serial.println(stepper1.currentPosition());
    //Serial.println(speedTest);
    
    tiempo2 = millis();
    lapseTime = tiempo2 - tiempo1;
    if(lapseTime >= 10)
    {
      //speedTest = speedTest + aumentoVelCada10ms;
      speedM1 = speedM1 + aumentoVelCada10msM1;
      speedM2 = speedM2 + aumentoVelCada10msM2;
      
      stepper1.setSpeed(speedM1);
      stepper2.setSpeed(speedM2);
      tiempo1 = millis();
    }
    
    if(((speedM1 == speedLimitM1) && (speedM2 == speedLimitM2)) || ((stepper1.currentPosition() == puntoMedioTrayectoriaM1) && (stepper2.currentPosition() == puntoMedioTrayectoriaM2)))
    {
      break;
    }
  }
//==============================================================================================================================


  #ifdef accelRampDebug
  Serial.println("Posicion despues de aceleracion M1");
  Serial.println(stepper1.currentPosition());
  Serial.println("Posicion despues de aceleracion M2");
  Serial.println(stepper2.currentPosition());


  Serial.println("Velocidad despues de aceleracion M1");
  Serial.println(speedM1);

  Serial.println("Velocidad despues de aceleracion M2");
  Serial.println(speedM2);
  #endif
  pasosEnAcelereacionM1 = stepper1.currentPosition();
  pasosEnAcelereacionM2 = stepper2.currentPosition();

  //======================  Movimiento en aceleracion constante  ====================================
  if (stepper1.currentPosition() < puntoMedioTrayectoriaM1 || stepper2.currentPosition() < puntoMedioTrayectoriaM2)
  {
    while (true)
    {
      //------------------Condiciones para activar electroiman-----------
      if (bandElectro == 1)
      {
        if (bandElectromagnet == 0)
        {
          if (stepper1.currentPosition() > numberofStepsM3 || stepper2.currentPosition() > numberofStepsM4)
          {
            #ifdef version4Electro
            deactivateElectromagV4E(numElectro);
            #endif
            bandElectromagnet = 1;
          }
        }
      }
      //----------------------------------------------------------------

      //Serial.println(stepper1.currentPosition());
      //Serial.println(stepper2.currentPosition());
      // Serial.println(speedTest);
      if (stepper1.currentPosition() >= puntoMedioTrayectoriaM1)
      {
        stepper1.stop();
      }
      else
      {
        stepper1.runSpeed();
      }

      if (stepper2.currentPosition() >= puntoMedioTrayectoriaM2)
      {
        stepper2.stop();
      }
      else
      {
        stepper2.runSpeed();
      }
      if (((stepper1.currentPosition() >= puntoMedioTrayectoriaM1) && (stepper2.currentPosition() >= puntoMedioTrayectoriaM2))) // checar condicion, buscar que estas condicionnes dependan de ambos motores
      {
        break;
      }
    }
  }

  limiteAccelCteM1 = ((stepper1.currentPosition() - pasosEnAcelereacionM1) * 2) + pasosEnAcelereacionM1;
  limiteAccelCteM2 = ((stepper2.currentPosition() - pasosEnAcelereacionM2) * 2) + pasosEnAcelereacionM2;

  #ifdef accelRampDebug
  Serial.println("Posicion Media M1");
  Serial.println(stepper1.currentPosition());
  Serial.println("Posicion Media M2");
  Serial.println(stepper2.currentPosition());

  Serial.println("Limite Accel Cte M1");
  Serial.println(limiteAccelCteM1);
  Serial.println("Limite Accel Cte M2");
  Serial.println(limiteAccelCteM2);
  #endif


  stepper1.setSpeed(speedM1);
  stepper2.setSpeed(speedM2);

  //======================  Movimiento en aceleracion constante segunda mitad ====================================
  if (stepper1.currentPosition() < limiteAccelCteM1 || stepper2.currentPosition() < limiteAccelCteM2)
  {
    while (true)
    {
      //------------------Condiciones para activar electroiman-----------
      if (bandElectro == 1)
      {
        if (bandElectromagnet == 0)
        {
          if (stepper1.currentPosition() > numberofStepsM3 || stepper2.currentPosition() > numberofStepsM4)
          {
            #ifdef version4Electro
            deactivateElectromagV4E(numElectro);
            #endif
            bandElectromagnet = 1;
          }
        }
      }
      //----------------------------------------------------------------

      //Serial.println(stepper1.currentPosition());
      //Serial.println(stepper2.currentPosition());
      // Serial.println(speedTest);
      if (stepper1.currentPosition() >= limiteAccelCteM1)
      {
        stepper1.stop();
      }
      else
      {
        stepper1.runSpeed();
      }

      if (stepper2.currentPosition() >= limiteAccelCteM2)
      {
        stepper2.stop();
      }
      else
      {
        stepper2.runSpeed();
      }
      if (((stepper1.currentPosition() >= limiteAccelCteM1) && (stepper2.currentPosition() >= limiteAccelCteM2))) // checar condicion, buscar que estas condicionnes dependan de ambos motores
      {
        break;
      }
    }
  }
  #ifdef accelRampDebug
  Serial.println("Posicion final en acel Cte M1");
  Serial.println(stepper1.currentPosition());
  Serial.println("Posicion final en acel Cte M2");
  Serial.println(stepper2.currentPosition());
  #endif

  if (stepper1.currentPosition() < accelStepsM1 || stepper2.currentPosition() < accelStepsM2)
  {
    while (true)
    {

      //------------------Condiciones para activar electroiman-----------
      if (bandElectro == 1)
      {
        if (bandElectromagnet == 0)
        {
          if (stepper1.currentPosition() > numberofStepsM3 || stepper2.currentPosition() > numberofStepsM4)
          {
            #ifdef version4Electro
            deactivateElectromagV4E(numElectro);
            #endif
            bandElectromagnet = 1;
          }
        }
      }
      //----------------------------------------------------------------

      if (stepper1.currentPosition() >= accelStepsM1)
      {
        stepper1.stop();
      }
      else
      {
        stepper1.runSpeed();
      }

      if (stepper2.currentPosition() >= accelStepsM2)
      {
        stepper2.stop();
      }
      else
      {
        stepper2.runSpeed();
      }

      //Serial.println(stepper1.currentPosition());
      //Serial.println(speedM1);
      //Serial.println(stepper2.currentPosition());
      //Serial.println(speedM2);

      tiempo2 = millis();
      lapseTime = tiempo2 - tiempo1;
      if (lapseTime >= 10)
      {
        // speedTest = speedTest + aumentoVelCada10ms;
        speedM1 = speedM1 - aumentoVelCada10msM1;
        speedM2 = speedM2 - aumentoVelCada10msM2;

        stepper1.setSpeed(speedM1);
        stepper2.setSpeed(speedM2);
        tiempo1 = millis();
      }
      if (speedM1 < minSpeed)
      {
        speedM1 = minSpeed;
      }
      if (speedM2 < minSpeed)
      {
        speedM2 = minSpeed;
      }

      if (((stepper1.currentPosition() >= accelStepsM1) && (stepper2.currentPosition() >= accelStepsM2)))
      {
        break;
      }
    }
  }
}

//====================================================================================================

  //valueA = xlocal;
    xActual = xlocal;

    //valueB = ylocal;
    yActual = ylocal;
  
#ifdef accelRampDebug  
  Serial.println("Pasos ejecutados por M1");
  Serial.println(stepper1.currentPosition());
  Serial.println("Pasos ejecutados por M2");
  Serial.println(stepper2.currentPosition());
#endif

  stepper1.setCurrentPosition(0);                          
  stepper2.setCurrentPosition(0);
#ifdef accelRampDebug  
  Serial.println("============================================");
#endif  


}


void deactivateElectromag()
{
   #ifdef pinoutv1
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 1);   //ANTES 1
    #endif
    #ifdef pinoutv2
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 253);   //ANTES 1
    #endif
}


void deactivateElectromagV4E(int Electro)
{
   #ifdef pinoutv1
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    ledcAttachPin(LED_PIN, ledChannel);
    
    ledcWrite(ledChannel, 1);   //ANTES 1
    #endif
    #ifdef pinoutv2
    // setting PWM properties
    const int freq = 5000;
    const int ledChannel = 0;
    const int resolution = 8;

    // configure LED PWM functionalitites
    //ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    #ifdef normalElectro
     if(Electro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(Electro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(Electro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    if(Electro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
        digitalWrite(22,LOW);
        digitalWrite(23,LOW);
    }
    #endif


        #ifdef invertElectro
     if(Electro == 1)
    {
        //ledcAttachPin(12, ledChannel);
        digitalWrite(magnet1,HIGH);
        digitalWrite(magnet2,HIGH);
        digitalWrite(magnet3,HIGH);
        digitalWrite(magnet4,HIGH);
    }
    if(Electro == 2)
    {
        //ledcAttachPin(13, ledChannel);
        digitalWrite(magnet1,HIGH);
        digitalWrite(magnet2,HIGH);
        digitalWrite(magnet3,HIGH);
        digitalWrite(magnet4,HIGH);
    }
    if(Electro == 3)
    {
        //ledcAttachPin(22, ledChannel);
        digitalWrite(magnet1,HIGH);
        digitalWrite(magnet2,HIGH);
        digitalWrite(magnet3,HIGH);
        digitalWrite(magnet4,HIGH);
    }
    if(Electro == 4)
    {
        //ledcAttachPin(23, ledChannel);
        digitalWrite(magnet1,HIGH);
        digitalWrite(magnet2,HIGH);
        digitalWrite(magnet3,HIGH);
        digitalWrite(magnet4,HIGH);
    }
    #endif
    
    //ledcWrite(ledChannel, 253);   //ANTES 1
    #endif

    ledcDetachPin(magnet1);
   ledcDetachPin(magnet2);
   ledcDetachPin(magnet3);
   ledcDetachPin(magnet4);
}


void MechanismH::moveToPointV2(double xlocal, double ylocal,int bCompens)
{

// Aplica la compensacion del nuevo algoritmo para centrar electroiman con los sensores
#ifdef activateSensors
  xlocal = xlocal + desfaseEnX;
  ylocal = ylocal + desfaseEnY;
#endif
  //=====================================================================================
#ifdef debug
/*
  Serial.println("Coordinate in moveToPoint");
  Serial.print("Coordinate X:  ");
  Serial.println(xlocal);
  Serial.print("Coordinate Y:  ");
  Serial.println(ylocal);
  */
  Serial.println("Posiciones iniciales en pasos");
  Serial.print("Motor 1:  ");
  Serial.println(stepper1.currentPosition());
  Serial.print("Motor 2:  ");
  Serial.println(stepper2.currentPosition());
#endif
  //===========================================================================

  //--------------------------------------------------------------
  // Local Variables
  //--------------------------------------------------------------

  double Tetha1 = 0;
  double Tetha2 = 0;

  double numberofSteps1 = 0;
  double numberofSteps2 = 0;

  double twoTimesPi = 2 * MyPI;

  //------------------------------------------------------------------------------------------//
  // Para calcular sin usar Pi, consideando que la vuelta de la polea es de 40mm
  /* double revolutionpermm1 = 0;
  double revolutionpermm2 = 0;
  */
  double numberofSteps3 = 0;
  double numberofSteps4 = 0;
  //------------------------------------------------------------------------------------------//

  long positions[2];

  // Computing the angles in rad using the reverse kinematics
  Tetha1 = (-xlocal + ylocal) / pulleyRadius;
  Tetha2 = (-xlocal - ylocal) / pulleyRadius;

  // Making the convertion from angles to steps USANDO PI
  numberofSteps1 = (Tetha1 / (twoTimesPi)) * neededStepsfor1Turn * -1;
  numberofSteps2 = (Tetha2 / (twoTimesPi)) * neededStepsfor1Turn * -1;

#ifdef relacionMicroSteps1
  numberofSteps3 = ((-(xlocal * 5) + (ylocal * 5))) * -1;
  numberofSteps4 = ((-(xlocal * 5) - (ylocal * 5))) * -1;
#endif

#ifdef relacionMicroSteps4
  numberofSteps3 = ((-(xlocal * 20) + (ylocal * 20))) * -1;
  numberofSteps4 = ((-(xlocal * 20) - (ylocal * 20))) * -1;
#endif

#ifdef relacionMicroSteps16
  numberofSteps3 = ((-(xlocal * 80) + (ylocal * 80))) * -1;
  numberofSteps4 = ((-(xlocal * 80) - (ylocal * 80))) * -1;
#endif

#ifdef relacionMicroSteps32
  numberofSteps3 = ((-(xlocal * 160) + (ylocal * 160))) * -1;
  numberofSteps4 = ((-(xlocal * 160) - (ylocal * 160))) * -1;
#endif

#ifdef relacionMicroSteps64
  numberofSteps3 = ((-(xlocal * 320) + (ylocal * 320))) * -1;
  numberofSteps4 = ((-(xlocal * 320) - (ylocal * 320))) * -1;
#endif

#ifdef relacionMicroSteps128
  numberofSteps3 = ((-(xlocal * 640) + (ylocal * 640))) * -1;
  numberofSteps4 = ((-(xlocal * 640) - (ylocal * 640))) * -1;
#endif

  //------------------------------------------------------------------------------------------//
  #ifdef debug
/*
  Serial.println("Coordinate in moveToPoint");
  Serial.print("Coordinate X:  ");
  Serial.println(xlocal);
  Serial.print("Coordinate Y:  ");
  Serial.println(ylocal);
  */
  Serial.println("Posiciones destino en pasos");
  Serial.print("Motor 1:  ");
  Serial.println(numberofSteps3);
  Serial.print("Motor 2:  ");
  Serial.println(numberofSteps4);
#endif

  positions[0] = numberofSteps3; // Pasos_totales1;
  positions[1] = numberofSteps4;
  
  //------------------------------
  // Librarys moves steppers until tehy are on position.
  //steppers.moveTo(positions);
  //steppers.runSpeedToPosition();
  //================================================
// C++ program for Bresenhams Line Generation

	int x1 = 0, y1 = 0, x2 = 0, y2 = 0, dx, dy, pk;
  x1 = stepper1.currentPosition();
  y1 = stepper2.currentPosition();

  x2 = numberofSteps3;
  y2 = numberofSteps4;

	dx = abs(x2 - x1);
	dy = abs(y2 - y1);

//================================================

  // Alamacena la posicion actual para usarla en el siguiente movimiento como dato anterior
  valueA = xlocal;
  xActual = xlocal;

  valueB = ylocal;
  yActual = ylocal;

  //=====================================================================================
}

#endif