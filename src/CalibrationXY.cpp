#include <CalibrationXY.h>
#if MACHINE_STYLE == ROBOT_H

hw_timer_t *timer1H = NULL;
int flag_timerH = 0;
int stall_outH = 0;
int *Ap_stallH = &stall_outH;
volatile int contPasosInterrup = 0;

#ifdef relacionMicroSteps1
int velInterruptions =  1250;    //Para un paso completo
#endif

#ifdef relacionMicroSteps4
int velInterruptions =  5000;
#endif

#ifdef relacionMicroSteps16
int velInterruptions =  2000;
#endif

#ifdef relacionMicroSteps32
int velInterruptions =  4000;
#endif

#ifdef relacionMicroSteps64
int velInterruptions =  8000;   //Para 1/64 de paso
#endif


#ifdef relacionMicroSteps128
int velInterruptions =  16000;   //Para 1/64 de paso
#endif

TMC2209Stepper driver(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS1);
TMC2209Stepper driver2(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS2);

extern AccelStepper stepper1;
extern AccelStepper stepper2;
extern MultiStepper steppers;

void normal_turnH();
void check_stallH(int, int *);
void IRAM_ATTR onTimerH();
void testMotorsH(void);
void configDriversH(void);

void calibraEjeY(void);
void calibraEjeX(void);

void calibraEjeXv2(void);
void calibraEjeYv2(void);


void move(int, int);

CalibrationXY::CalibrationXY()
{
}

/**================================================================================================
 *                                           Funcion Calibration::init()
 *  
 *  Realiza las configuraciones necesarias para el uso de los motores a pasos, asi como las confi-
 *  guraciones iniciales para los pines digitales.
 * 
 *  Tambien inicializa la funcion normal_turnH() para el movimiento de los motores mediante interrupciones.
 *================================================================================================**/

void CalibrationXY::initCalibration()
{
	SERIAL_PORT2.begin(115200);
	
        //===========Motor 1 y 2============
	pinMode(MOTOR_0_STEP_PIN, OUTPUT);
	pinMode(MOTOR_0_DIR_PIN, OUTPUT);
        pinMode(MOTOR_1_STEP_PIN, OUTPUT);
	pinMode(MOTOR_1_DIR_PIN, OUTPUT);
        digitalWrite(MOTOR_0_DIR_PIN, LOW);
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
	
        configDriversH();

        digitalWrite(MOTOR_0_DIR_PIN, HIGH);
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        pinMode(ENABLE_PIN, OUTPUT);
        #ifdef CambiosPCBTavo
        digitalWrite(ENABLE_PIN, LOW);
        #endif
        #ifdef CambiosPCBLalo
        digitalWrite(ENABLE_PIN, HIGH);
        #endif
        testMotorsH();
        driver.rms_current(CURRENT_IN_CALIBRATION);
        driver.microsteps(MICROSTEPPING);
        delay(100);

        driver2.rms_current(CURRENT_IN_CALIBRATION);
        driver2.microsteps(MICROSTEPPING);
        delay(100);

        flag_timerH = 0;                                 //flag_timerH controla las interrupciones para el movimiento de los motores
        normal_turnH();

        #ifdef tempTest
        flag_timerH = 3;   //
        #endif

}
/*================================================================================================*/


/**================================================================================================
 *                                           Funcion Calibration::start()
 *  
 *  Esta es la funcion principal del proceso de calibracion para el mecanismo tipo H.
 *  Se realizan dos secuencias de calibracion, una sobre el eje x y otra sobre el eje y para encontar
 *  el centro del sistema
 *================================================================================================**/
int CalibrationXY::startCalibration()
{       
        driver.en_spreadCycle(false);
        driver2.en_spreadCycle(false);

        #ifdef debug
        Serial.println("MicroSteps");
        Serial.println(driver.microsteps());
        Serial.println(driver2.microsteps());
        #endif
        delay(500);

        //calibraEjeY();          //Funcion para la calibracion del eje y
        calibraEjeYv2();
        #ifdef pinoutv1
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);                             //Configuracion de direccion -> Movimiento hacia abajo sobre eje y
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        #endif
        #ifdef pinoutv2
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);     //LOW                           //Configuracion de direccion -> Movimiento hacia abajo sobre eje y
        digitalWrite(MOTOR_1_DIR_PIN, LOW);    //HIGH
        #endif

        #ifdef pinoutv3
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);    //HIGH                         //Configuracion de direccion -> Movimiento hacia abajo sobre eje y
        digitalWrite(MOTOR_1_DIR_PIN, LOW);     //LOW
        #endif

        #ifdef relacionMicroSteps1
        move(1055,1000);          //Para un paso completo
        #endif

        #ifdef relacionMicroSteps4
        move(4220,500);          //4180 antes delay 200 (18370)    Se mueve para compensar desfase mecanico de una pieza (Temporal)  move(18350,500); 
        #endif

        #ifdef relacionMicroSteps16
        move(16880,100);
        #endif
        
        #ifdef relacionMicroSteps32
        move(36760,100);
        #endif

        #ifdef relacionMicroSteps64
        move(67520,15);          //Para 1/64 de paso
        #endif

        #ifdef relacionMicroSteps128
        move(135040,7);          //Para 1/128 de paso
        #endif

        //calibraEjeX();          //Funcion para la calibracion del eje X
        calibraEjeXv2();
        #ifdef pinoutv1
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);                            //Configuracion de direccion -> Movimiento hacia la izquierda sobre eje x
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        #endif
        #ifdef pinoutv2
        digitalWrite(MOTOR_0_DIR_PIN, LOW); //LOW                           //Configuracion de direccion -> Movimiento hacia la izquierda sobre eje x
        digitalWrite(MOTOR_1_DIR_PIN, LOW); //LOW
        #endif

        #ifdef pinoutv3
        digitalWrite(MOTOR_0_DIR_PIN, LOW);  //LOW                          //Configuracion de direccion -> Movimiento hacia la izquierda sobre eje x
        digitalWrite(MOTOR_1_DIR_PIN, LOW);  //LOW
        #endif

        #ifdef relacionMicroSteps1
        #ifdef CambiosPCBTavo
        move(1010,1000);          //Para pasos completos 
        #endif
        #ifdef CambiosPCBLalo
        move(999,1000);          //Para pasos completos     
        #endif
        #endif
        
        #ifdef relacionMicroSteps4
        #ifdef CambiosPCBTavo
        move(4040,500);          //3930 antes delay 200 (18450)    Se mueve para compensar el ancho de la gondola (Temporal) move(18350,500);      
        #endif
        #ifdef CambiosPCBLalo
        move(3996,500);          //3930 antes delay 200 (18450)    Se mueve para compensar el ancho de la gondola (Temporal) move(18350,500);      
        #endif
        #endif

        #ifdef relacionMicroSteps16
        #ifdef CambiosPCBTavo
        move(16160,100);
        #endif
        #ifdef CambiosPCBLalo
        move(15984,100);
        #endif
        #endif

        #ifdef relacionMicroSteps32
        #ifdef CambiosPCBTavo
        move(32320,100);
        #endif
        #ifdef CambiosPCBLalo
        move(31968,100);
        #endif
        #endif

        #ifdef relacionMicroSteps64
        #ifdef CambiosPCBTavo
        move(64640,15);          //Para 1/64 de paso    
        #endif
        #ifdef CambiosPCBLalo   
        move(63936,15);          //Para 1/64 de paso    
        #endif
        #endif

        #ifdef relacionMicroSteps128
        #ifdef CambiosPCBTavo
        move(129280,7);          //Para 1/128 de paso    
        #endif
        #ifdef CambiosPCBLalo   
        move(127872,7);          //Para 1/128 de paso    
        #endif
        #endif

        driver.en_spreadCycle(false);    //true
        driver2.en_spreadCycle(false);   //true
        
        stepper1.setCurrentPosition(0);                           
        stepper2.setCurrentPosition(0);
        return 0;
}
/*================================================================================================*/

/**
 * @brief Esta funcion perimite mover los motores de manera controlada mediante numero de pasos. 
 * @param pasos Esta variable indica el numero de pasos que se desea avanzar.
 * @param Speed Indica la velocidad del giro.
 */
void move(int pasos, int Speed)
{
        for (int i = 0; i < pasos; i++)
        {
                digitalWrite(MOTOR_0_STEP_PIN, LOW);
                delayMicroseconds(Speed);
                digitalWrite(MOTOR_0_STEP_PIN, HIGH);
                delayMicroseconds(Speed);

                digitalWrite(MOTOR_1_STEP_PIN, LOW);
                delayMicroseconds(Speed);
                digitalWrite(MOTOR_1_STEP_PIN, HIGH);
                delayMicroseconds(Speed);
        }
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion check_stallH
 *?  Verifica el valor actual del StallGuard y devuelve "1"  si el stalguard se ha activado
 *?  y devuelve 0 en caso contrario 
 *@param result_stall           Devuelve el resultado del StallGuard  
 *@param flag_timerH             Detiene las interrupciones cuando se ha detectado un cambio de estado   
 *@return void
 *================================================================================================**/
void check_stallH(int driver_m, int *result_stall)
{
	int stall_data = 0;
	if (driver_m == 1)
	{
                stall_data = driver.SG_RESULT();                //Solicita al driver el valor de StallGuard
                #ifdef relacionMicroSteps1
                #ifdef CambiosPCBTavo
		if (stall_data < 250)   //Para pasos completos
                #endif
                #ifdef CambiosPCBLalo
		if (stall_data < 200)   //Para pasos completos
                #endif
                #endif 
                #ifdef relacionMicroSteps4
		if (stall_data < 125)   //120
                #endif                            //A menor valor menor sensibilidad
                #ifdef relacionMicroSteps16
                if (stall_data < 140)
                #endif
                #ifdef relacionMicroSteps32
                if (stall_data < 15)
                #endif
                #ifdef relacionMicroSteps64
                #ifdef CambiosPCBTavo
                if (stall_data < 110)     //Para 1/64 de paso
                #endif
                #ifdef CambiosPCBLalo
                if (stall_data < 100)     //Para 1/64 de paso
                #endif
                #endif 
                #ifdef relacionMicroSteps128
                #ifdef CambiosPCBTavo
                if (stall_data < 110)     //Para 1/128 de paso
                #endif
                #ifdef CambiosPCBLalo
                if (stall_data < 100)     //Para 1/128 de paso
                #endif
                #endif
        	{
			flag_timerH = 0;
                        #ifdef debug
			Serial.println("Se activo stall guard M1");
                        #endif
			delay(1000);
			*result_stall = 1;
		}
		else
		{
			*result_stall = 0;
		}
	}

	if (driver_m == 2)
	{
                stall_data = driver2.SG_RESULT();                //Solicita al driver el valor de StallGuard


                #ifdef relacionMicroSteps1
		if (stall_data < 250)       //Para pasos completos
                #endif   
		#ifdef relacionMicroSteps4
		if (stall_data < 125)       //120
                #endif                            //A menor valor menor sensibilidad    Antes 25   //Antes 35
                #ifdef relacionMicroSteps16
                if (stall_data < 140)
                #endif
                #ifdef relacionMicroSteps32
                if (stall_data < 15)
                #endif
                #ifdef relacionMicroSteps64
                if (stall_data < 110)         //Para 1/64 de paso
                #endif 
                #ifdef relacionMicroSteps128
                if (stall_data < 110)         //Para 1/128 de paso
                #endif
		{
			flag_timerH = 0;
                        #ifdef debug
			Serial.println("Se activo stall guard M2");
                        #endif
			delay(1000);
			*result_stall = 1;
		}
		else
		{
			*result_stall = 0;
		}
	}
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion testMotorsH()
 *?  Esta funcion realiza configuraciones de corriente y microsteps, y verifica que el driver se haya 
 *?  configurado con los valores correctos
 *@param name 
 *@param name  
 *@return void
 *================================================================================================**/
void testMotorsH()
{
    /**==============================================
     *                Test Motor 1
     *=============================================**/

    /*======= Prueba comunicacion con el driver =======*/
    #ifdef debug
    Serial.println(" ");
    Serial.println("Test Motor 1");
    #endif
    uint8_t result_1 = driver.test_connection();
	if (result_1 == 0)
	{
                #ifdef debug
		Serial.println(F("M1_OK,"));
                #endif
	}
	else
	{
                #ifdef debug
		Serial.println(F("M1_Fail,"));
                #endif
	}
       Serial.println(" "); 
    /*======= Prueba configuracion a 700mA y 32 microsteps =======*/   
    Serial.println("M1 configuracion a 700mA y 32 microsteps"); 
    driver.rms_current(700);
    driver.microsteps(32);
    #ifdef debug
    Serial.println(driver.rms_current());
    Serial.println(driver.microsteps());
    #endif
    delay(100);
    /*======= Prueba configuracion a 900mA y 64 microsteps =======*/
    Serial.println("M1 configuracion a 900mA y 64 microsteps");
    driver.rms_current(900);
    driver.microsteps(64);

    #ifdef debug
    Serial.println(driver.rms_current());
    Serial.println(driver.microsteps());
    #endif
    delay(100);
    /*======= Prueba configuracion a 500mA y 0 microsteps =======*/
    Serial.println("M1 configuracion a 500mA y 0 microsteps");
    driver.rms_current(CURRENT_IN_CALIBRATION);
    driver.microsteps(MICROSTEPPING);
    #ifdef debug
    Serial.println(driver.rms_current());
    Serial.println(driver.microsteps());
    #endif
    /*=============================================*/
    
    /**==============================================
     *                Test Motor 2
     *=============================================**/
Serial.println(" ");
    /*======= Prueba comunicacion con el driver =======*/
    #ifdef debug
    Serial.println("Test Motor 2");
    #endif
    uint8_t result_2 = driver2.test_connection();
	if (result_2 == 0)
	{
                #ifdef debug
		Serial.println(F("M2_OK,"));
                #endif
	}
	else
	{
                #ifdef debug
		Serial.println(F("M2_Fail,"));
                #endif
	}
        Serial.println(" ");
    /*======= Prueba configuracion a 700mA y 32 microsteps =======*/ 
    Serial.println("M2 configuracion a 700mA y 32 microsteps");
    driver2.rms_current(700);
    driver2.microsteps(32);
    #ifdef debug
    Serial.println(driver2.rms_current());
    Serial.println(driver2.microsteps());
    #endif
    delay(100);
    /*======= Prueba configuracion a 900mA y 64 microsteps =======*/
    Serial.println("M2 configuracion a 900mA y 64 microsteps");
    driver2.rms_current(900);
    driver2.microsteps(64);
    #ifdef debug
    Serial.println(driver2.rms_current());
    Serial.println(driver2.microsteps());
    #endif
    delay(100);
    /*======= Prueba configuracion a 500mA y 0 microsteps =======*/
    Serial.println("M2 configuracion a 500mA y 0 microsteps");
    driver2.rms_current(CURRENT_IN_CALIBRATION);
    driver2.microsteps(MICROSTEPPING);
    #ifdef debug
    Serial.println(driver2.rms_current());
    Serial.println(driver2.microsteps());
    #endif
    /*=============================================*/
    /*
        Serial.println(" ");
    Serial.println("Test de Buzzer");
    cute.init(BUZZER_PIN);
    ledcSetup(5, 2000, 16);   // channel, max frequency, resolution
    ledcAttachPin(BUZZER_PIN, 5);

    cute._tone(NOTE_C6,50,30); /// 1046.5  //C6
delay(500);
cute._tone(NOTE_Db6,50,30); /// 1108.73  //C#6/Db6
delay(500);
cute._tone(NOTE_D6,50,30); /// 1174.66  //D6
delay(500);
cute._tone(NOTE_Eb6,50,30); /// 1244.51  //D#6/Eb6
delay(500);
cute._tone(NOTE_E6,50,30); /// 1318.51  //E6
delay(500);
cute._tone(NOTE_F6,50,30); /// 1396.91  //F6
delay(500);
cute._tone(NOTE_Gb6,50,30); /// 1479.98  //F#6/Gb6
delay(500);
cute._tone(NOTE_G6,50,30); /// 1567.98  //G6
delay(500);
cute._tone(NOTE_Ab6,50,30); /// 1661.22  //G#6/Ab6
delay(500);
cute._tone(NOTE_A6,50,30); /// 1760  //A6
delay(500);
cute._tone(NOTE_Bb6,50,30); /// 1864.66  //A#6/Bb6
delay(500);
cute._tone(NOTE_B6,50,30); /// 1975.53  //B6
delay(500);


cute._tone(NOTE_C7,50,30); /// 2093  //C7
delay(500);
cute._tone(NOTE_Db7,50,30); /// 2217.46  //C#7/Db7
delay(500);
cute._tone(NOTE_D7,50,30); /// 2349.32  //D7
delay(500);
cute._tone(NOTE_Eb7,50,30); /// 2489.02  //D#7/Eb7
delay(500);
cute._tone(NOTE_E7,50,30); /// 2637.02  //E7
delay(500);
cute._tone(NOTE_F7,50,30); /// 2793.83  //F7
delay(500);
cute._tone(NOTE_Gb7,50,30); /// 2959.96  //F#7/Gb7
delay(500);
cute._tone(NOTE_G7,50,30); /// 3135.96  //G7
delay(500);
cute._tone(NOTE_Ab7,50,30); /// 3322.44  //G#7/Ab7
delay(500);
cute._tone(NOTE_A7,50,30); /// 3520  //A7
delay(500);
cute._tone(NOTE_Bb7,50,30); /// 3729.31  //A#7/Bb7
delay(500);
cute._tone(NOTE_B7,50,30); /// 3951.07  //B7
delay(500);
Serial.println(" ");
*/

}
/**================================================================================================
 **                                      Funcion normal_turnH()
 *?  En esta funcion se realizan las configuracione necesarias de timers y contadores para generar 
 *?  las interrupciones que controlaran el movimiento de los motores 
 *@return void
 *================================================================================================**/
void normal_turnH()
{

        cli();                                        //stop interrupts
        timer1H = timerBegin(3, 8, true);              // Se configura el timer, en este caso uso el timer 4 de los 4 disponibles en el ESP(0,1,2,3)
                                                      // el prescaler es 8, y true es una Flagera que indica si la interrupcion se realiza en borde o en level
        timerAttachInterrupt(timer1H, &onTimerH, true); //Se vincula el timer con la funcion AttachInterrup
                                                      //la cual se ejecuta cuando se genera la interrupcion
        timerAlarmWrite(timer1H, velInterruptions, true);          //En esta funcion se define el valor del contador en el cual se genera la interrupción del timer original 10000
        timerAlarmEnable(timer1H);                     //Función para habilitar el temporizador.
        sei();                                        //allow interrupts
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion IRAM_ATTR onTimerH()
 *?  En esta funcion se realizan la activacion de los pines STEP cada que ocurre una interrupcion 
 *?  la bandera flag_timerH ayuda a indicar que motores son los que reciben la senal de control
 *@return void
 *================================================================================================**/
void IRAM_ATTR onTimerH()
{
	if (flag_timerH == 1)                                                            //Controla el giro del motor 1
	{
		digitalWrite(MOTOR_0_STEP_PIN, !digitalRead(MOTOR_0_STEP_PIN));
	}
        if (flag_timerH == 2)                                                            //Controla el giro del motor 2
	{
		digitalWrite(MOTOR_1_STEP_PIN, !digitalRead(MOTOR_1_STEP_PIN));
	}
	if (flag_timerH == 3)                                                            //Controla el giro de ambos motores
	{
		digitalWrite(MOTOR_1_STEP_PIN, !digitalRead(MOTOR_1_STEP_PIN));
                digitalWrite(MOTOR_0_STEP_PIN, !digitalRead(MOTOR_0_STEP_PIN));
                contPasosInterrup++;
	}
}
/*================================================================================================*/

/**================================================================================================
 **                                      configDriversH()
 *?  En esta funcion se realiza las configuraciones necesarias de los drivers 
 *@return void
 *================================================================================================**/
void configDriversH(void)
{
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

	driver.ihold(I_HOLD);
	driver2.ihold(I_HOLD);

	
	//EEPROM.begin(EEPROM_SIZE);
	//delay(1000);
	int cont = 0;
	while (driver.microsteps() != MICROSTEPPING)
	{
		driver.microsteps(MICROSTEPPING);
		delay(100);
		cont++;
		if (cont == 5)
		{
			break;
		}
	}
	cont = 0;
	while (driver2.microsteps() != MICROSTEPPING)
	{
		driver2.microsteps(MICROSTEPPING);
		delay(100);
		cont++;
		if (cont == 5)
		{
			break;
		}
	}
        EEPROM.begin(EEPROM_SIZE);

        //Borrar dato en eeprom
        float val;
        val = 0.0;
        EEPROM.writeFloat(401, val);
        EEPROM.commit();

        EEPROM.writeFloat(405, val);
        EEPROM.commit();
        //======================================================================================
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion calibraEjeY()
 *?  Realiza una rutina de calibracion que consiste en econtrar los limites del mecanismo sobre el eje
 *?  Y con ayuda del StallGuard, posteriormente se posiciona en uno de los extremos y se mueve hacia la
 *? posicion central del eje. 
 *@param name type    
 *@return void
 *================================================================================================**/
void calibraEjeY()
{
        int sumaInterrup = 0;
        /*================== Primer recorrido hacia la derecha =================*/

        digitalWrite(MOTOR_0_DIR_PIN, LOW);                            //Configuracion de direccion -> Movimiento hacia arriba sobre eje y
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);

        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checa el valor del StallGuard hasta que detecta un obstaculo
        }
        stall_outH = 0;
        flag_timerH = 0;                                                 //Desactiva las interrupcions para ambos motores
        contPasosInterrup = 0;
        delay(500);
        /*================== Segundo recorrido hacia la izquierda =================*/
/*
        digitalWrite(MOTOR_0_DIR_PIN, LOW);                             //Configuracion de direccion -> Movimiento hacia abajo sobre eje y
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);

        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        delay(500);
        contPasosInterrup = 0;
        while (stall_outH == 0)
        {
                Serial.println(driver.SG_RESULT(), DEC);
                check_stallH(2, Ap_stallH);                               //Checa el valor del StallGuard hasta que detecta un obstaculo
        }
        Serial.println("Total de pasos");
        Serial.println(contPasosInterrup);
        sumaInterrup = contPasosInterrup;

        stall_outH = 0;
        flag_timerH = 0;                                                 //Desactiva las interrupcions para ambos motores
        contPasosInterrup = 0;
        delay(500);

        sumaInterrup = sumaInterrup / 2;                                //Obtiene la mitad del numero de interrupciones para el siguiente movimiento
        Serial.println("Pasos de regreso");
        Serial.println(sumaInterrup);
        delay(500);
  */      
        /*================== Tercer recorrido hacia la derecha =================*/
 /*       digitalWrite(MOTOR_0_DIR_PIN, HIGH);                            //Configuracion de direccion -> Movimiento hacia arriba sobre eje y
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        
        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        contPasosInterrup = 0;
        while (contPasosInterrup < sumaInterrup)                        //Avanza hasta completar el numero de interrupciones para llegara al centro
        {
                Serial.println(contPasosInterrup);
        }
        Serial.println("Total de pasos");
        Serial.println(contPasosInterrup);
        flag_timerH = 0;
        contPasosInterrup = 0;
        delay(500);
        */
}
/*================================================================================================*/

/**================================================================================================
 **                                      Funcion calibraEjeX()
 *?  Realiza una rutina de calibracion que consiste en econtrar los limites del mecanismo sobre el eje
 *?  x con ayuda del StallGuard, posteriormente se posiciona en uno de los extremos y se mueve hacia la
 *? posicion central del eje. 
 *@param name type    
 *@return void
 *================================================================================================**/
void calibraEjeX()
{
        int sumaInterrup = 0;
        /*================== Primer recorrido hacia arriba =================*/
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);                             //Configuracion de direccion -> Movimiento hacia la derecha eje X
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        
        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checa el valor del StallGuard hasta que detecta un obstaculo
        }
        stall_outH = 0;

        flag_timerH = 0;
        contPasosInterrup = 0;
        delay(500);
        
        /*================== Segundo recorrido hacia la izquierda =================*/
 /*       digitalWrite(MOTOR_0_DIR_PIN, HIGH);                            //Configuracion de direccion -> Movimiento hacia la izquierda sobre eje x
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);

        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        delay(500);
        contPasosInterrup = 0;
        while (stall_outH == 0)
        {
                Serial.println(driver.SG_RESULT(), DEC);
                check_stallH(1, Ap_stallH);                               //Checa el valor del StallGuard hasta que detecta un obstaculo
        }
        Serial.println("Total de pasos");
        Serial.println(contPasosInterrup);
        sumaInterrup = contPasosInterrup;

        stall_outH = 0;

        flag_timerH = 0;
        contPasosInterrup = 0;
        delay(500);

        sumaInterrup = sumaInterrup / 2;                                //Obtiene la mitad del numero de interrupciones para el siguiente movimiento
        Serial.println("Pasos de regreso");
        Serial.println(sumaInterrup);
        delay(500);
        */     
        /*================== Segundo recorrido hacia arriba =================*/
 /*      digitalWrite(MOTOR_0_DIR_PIN, LOW);                             //Configuracion de direccion -> Movimiento hacia arriba sobre eje y
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        contPasosInterrup = 0;
        while (contPasosInterrup < sumaInterrup)                        //Avanza hasta completar el numero de interrupciones para llegara al centro
        {
                Serial.println(contPasosInterrup);
        }
        Serial.println("Total de pasos");
        Serial.println(contPasosInterrup);
        flag_timerH = 0;
        contPasosInterrup = 0;
        delay(500);

        */
}




void calibraEjeXv2()
{
        int sumaInterrup = 0;
/*
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);                             //Configuracion de direccion -> Movimiento hacia la derecha eje X
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        
        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checa el valor del StallGuard hasta que detecta un obstaculo
        }
        stall_outH = 0;
        flag_timerH = 0;
        delay(500);
        digitalWrite(MOTOR_0_DIR_PIN, LOW);
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        flag_timerH = 3;
        contPasosInterrup = 0;
        while(contPasosInterrup < 5000)
        {
                flag_timerH = 3;
        }
        flag_timerH = 0;
*/
        #ifdef pinoutv1
        digitalWrite(MOTOR_0_DIR_PIN, LOW);                             //Configuracion de direccion -> Movimiento hacia la derecha eje X
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        #endif

        #ifdef pinoutv2
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);     //HIGH                        //Configuracion de direccion -> Movimiento hacia la derecha eje X
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);     //HIGH
        #endif

        #ifdef pinoutv3
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);      //HIGH                       //Configuracion de direccion -> Movimiento hacia la derecha eje X
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);      //HIGH
        #endif

        #ifdef relacionMicroSteps1
        velInterruptions = 12000;       //Para pasos completos
        #endif
        
        #ifdef relacionMicroSteps4
        velInterruptions = 7000;
        #endif

        #ifdef relacionMicroSteps16
        velInterruptions = 2000;
        #endif

        #ifdef relacionMicroSteps32
        velInterruptions = 1000;
        #endif

        #ifdef relacionMicroSteps64
        velInterruptions = 500;         //Para 1/64 de paso
        #endif

        #ifdef relacionMicroSteps128
        velInterruptions = 250;         //Para 1/128 de paso
        #endif

        normal_turnH();
        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver2.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checa el valor del StallGuard hasta que detecta un obstaculo
        }
        stall_outH = 0;
        flag_timerH = 0;

        #ifdef relacionMicroSteps1
        velInterruptions = 12000;       //Para pasos completos
        #endif

        #ifdef relacionMicroSteps4
        velInterruptions = 6000;
        #endif

        #ifdef relacionMicroSteps16
        velInterruptions = 2000;
        #endif

        #ifdef relacionMicroSteps32
        velInterruptions = 1000;
        #endif


        #ifdef relacionMicroSteps64
        velInterruptions = 500;         //Para 1/64 de paso
        #endif

        #ifdef relacionMicroSteps128
        velInterruptions = 250;         //Para 1/128 de paso
        #endif


        contPasosInterrup = 0;
        delay(500);
        

}



void calibraEjeYv2()
{
        int sumaInterrup = 0;
        /*================== Primer recorrido hacia la derecha =================*/
/*
        digitalWrite(MOTOR_0_DIR_PIN, LOW);                            //Configuracion de direccion -> Movimiento hacia arriba sobre eje y
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);

        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checa el valor del StallGuard hasta que detecta un obstaculo
        }
        stall_outH = 0;
        flag_timerH = 0;                                                 //Desactiva las interrupcions para ambos motores
        

        delay(500);
        digitalWrite(MOTOR_0_DIR_PIN, HIGH);
        digitalWrite(MOTOR_1_DIR_PIN, LOW);
        flag_timerH = 3;
        contPasosInterrup = 0;
        while(contPasosInterrup < 5000)
        {
                flag_timerH = 3;
        }
        flag_timerH = 0;
*/
        #ifdef pinoutv1
        digitalWrite(MOTOR_0_DIR_PIN, LOW);                             //Configuracion de direccion -> Movimiento hacia la derecha eje X
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);
        #endif

        #ifdef pinoutv2
        digitalWrite(MOTOR_0_DIR_PIN, LOW);   //HIGH                          //Configuracion de direccion -> Movimiento hacia la derecha eje X
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);    //LOW
        #endif

        #ifdef pinoutv3
        digitalWrite(MOTOR_0_DIR_PIN, LOW);      //LOW                       //Configuracion de direccion -> Movimiento hacia la derecha eje X
        digitalWrite(MOTOR_1_DIR_PIN, HIGH);     //HIGH
        #endif

        #ifdef relacionMicroSteps1
        velInterruptions = 12000;       //Para pasos completos
        #endif

        #ifdef relacionMicroSteps4
        velInterruptions = 7000;
        #endif

        #ifdef relacionMicroSteps16
        velInterruptions = 2000;
        #endif

        #ifdef relacionMicroSteps32
        velInterruptions = 1000;
        #endif

        #ifdef relacionMicroSteps64
        velInterruptions = 500;           //Para 1/64 de paso
        #endif

        #ifdef relacionMicroSteps128
        velInterruptions = 250;           //Para 1/128 de paso
        #endif

        normal_turnH();
        flag_timerH = 3;                                                 //Activa las interrupcions para ambos motores
        delay(500);
        while (stall_outH == 0)
        {
                #ifdef debug
                Serial.println(driver2.SG_RESULT(), DEC);
                #endif
                check_stallH(2, Ap_stallH);                               //Checa el valor del StallGuard hasta que detecta un obstaculo
        }
        stall_outH = 0;
        flag_timerH = 0;

        #ifdef relacionMicroSteps1
        velInterruptions = 12000;       //Para pasos completos
        #endif

        #ifdef relacionMicroSteps4
        velInterruptions = 6000;
        #endif

        #ifdef relacionMicroSteps16
        velInterruptions = 2000;
        #endif

        #ifdef relacionMicroSteps32
        velInterruptions = 1000;
        #endif

        #ifdef relacionMicroSteps64
        velInterruptions = 500;           //Para 1/64 de paso
        #endif

        #ifdef relacionMicroSteps128
        velInterruptions = 250;           //Para 1/128 de paso
        #endif

}





#endif  