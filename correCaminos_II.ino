/* Autor : Santiago Ventura Gomez . 2017
Curso Experto Universitario en Robótica , Programación e Impresión 3D
Asignatura : Robótica
Profesor : Alberto Valero
Actividad 5 : Robot CorreCaminos II
*/

/*
Objetivo : 

Se debe construir un robot que recorra un circuito delimitado por paredes

Acerca del circuito:

Debe ser un circuito cerrado, como los de Fórmula 1, donde una vez dentro no haya salida.
El circuito debe tener al menos 1 metro de longitud
El circuito debe estar delimitado por paredes (ver video de ejemplo https://www.youtube.com/watch?v=tYlN4ppaMQ4)
Las paredes se pueden hacer con cartulina, cajas de cartón, etc. 
Acerca del robot:

La construcción del robot es totalmente libre
El robot debe recorrer el circuito sin salirse y sin chocarse contra las paredes.

*/

/* Para definir los lados Izquierda y Derecha del Robot se hace visto desde la parte trasera y mirando en la dirección
    de avance del mismo */
    
/* Elementos de construcción del Robot :

    * Para las piezas del Chasis , me he descargado los archivos STL del Printbot Evolution , y he utilizado la base de 3 mm
      Todas las piezas están impresas en una impresora Hephestos 2 
    * Movimiento de las Ruedas : Robot diferencial de tres puntos de apoyo , que cuenta con dos servomotores de movimiento continuo para las ruedas
      y un apoyo fijo trasero
    * La detección de las paredes se realiza mediante un sensor de ultrasonidos montado sobre un mini servomotor 0 - 180 º , que le permite bascular
      girando a izquierda ( 135 º ) y derecha ( 45 º )
    * Boton pulsador que nos permitirá controlar el momento de inicio del movimiento de los motores de las ruedas
 */
 
 /* Asignación de pines :
 
   * Servo RuedaDrcha     :  Pin 4
   * Servo RuedaIzqda     :  Pin 5
   * Servo ControlUS      :  Pin 7
   * PulsadorInicio       :  Pin 8
   * Sensor ultrasonidos  : TRIG = PIN 11 , ECHO = PIN 12

*/
/*  Valores de velocidades y giros :

   * Avance Robot :
       Rueda Derecha : Va_drcha
       Rueda Izquierda : Va_izqda
  
   * Giro Derecha Robot :
       Rueda Derecha : Va_drcha - Reduccion de velocidad
       Rueda Izquierda : Va_izqda
   
   * Giro Izquierda Robot :
       Rueda Derecha : Va_drcha
       Rueda Izquierda : Va_izqda + Reduccion de velocidad
   
   * Parar :
       Rueda Derecha : 90º
       Rueda Izquierda : 90º
  */

/********************************* DEFINICION DE LIBRERIAS Y VARIABLES GLOBALES **********************************************/

#include <Servo.h>                            // Libreria Arduino que controla funciones especiales de los servomotores
#include <BitbloqUS.h>                        // Libreria Bitbloq que sirve `para controlar un sensor de ultrasonidos

Servo RuedaIzqda;                             // Declaracion de variable tipo Servo para el motor que mueve la rueda Izquierda
Servo RuedaDrcha;                             // Declaracion de variable tipo Servo para el motor que mueve la rueda Derecha

Servo ControlUS;                              // Declaracion de variable tipo Servo para el servomotor que controla el giro del sensor US

US ultrasonidos(11,12);                       // Declaracion de variable tipo US para el sensor de ultrasonidos y pines TRIG y ECHO donde
                                              // está conectado

int PulsadorInicio = 8;                       // Declaracion del pin asignado al boton que iniciará el programa

float d_limite = 12;                          // Distancia minima limite en cms desde el sensor a cualquiera de las paredes del circuito
float distancia;                              // Variable que guarda el valor de distancia en cm recibido desde el sensor US
int Va_derecha = 130;                         // Velocidad de avance de la rueda derecha , en grados
int Va_izquierda = 50;                         // Velocidad de avance de la rueda izquierda , en grados
int Reduccion_vel = 30;                       // Reduccion en la velocidad de una cualquiera de las ruedas respecto al máximo
int torsion = 45;                             // Angulo que debe girar el sensor de ultrasonido a ambos lados de la linea central de avance (90º)
                                              // 90º + torsion ---> Mirar a la izquierda
                                              // 90º - torsion ---> Mirar a la derecha

float T_inic = 0;                             // Valor inicial de tiempo de un periodo entre 2 movimientos/mediciones seguidos del sensor US                                              
float T_final = 0;                            // Valor final de tiempo del mismo periodo entre 2 movimientos/mediciones seguidos del sensor US
float periodo = 250;                          // Valor del tiempo que transcurre entre dos mediciones del sensor US o entre dos movimientos del servo



/***********************    DEFINICION DE FUNCIONES LIGADAS Al MOVIMIENTO DE LAS RUEDAS DEL ROBOT    **********************************/


                                                                             
void giroDrcha(Servo Rueda_D,Servo Rueda_I,int Va_D,int Va_I,int Red_vel){   // Funcion que hace girar al robot a la derecha                                   
        Rueda_D.write(Va_D - Red_vel);                                       // No paramos del todo la rueda drcha , solo la ralentizamos
        Rueda_I.write(Va_I);                   
}

void giroIzqda(Servo Rueda_D,Servo Rueda_I,int Va_D,int Va_I,int Red_vel){   // Funcion que hace girar al robot a la izquierda
        Rueda_D.write(Va_D);
        Rueda_I.write(Va_I + Red_vel);                                       // No paramos del todo la rueda izda , solo la ralentizamos                
}

void avanzar(Servo Rueda_D,Servo Rueda_I,int Va_D,int Va_I){                 // Funcion que hace avanzar al robot
        Rueda_D.write(Va_D);
        Rueda_I.write(Va_I);        
}

void parar(Servo Rueda_D,Servo Rueda_I){                                     // Funcion que hace pararse al robot
        Rueda_D.write(90);
        Rueda_I.write(90);        
}

/*************************  FUNCION QUE CONTROLA EL GIRO DEL SERVO QUE MUEVE EL SENSOR DE ULTRASONIDOS  **************************/

int mover_sensor(int ang,Servo torretaUS ){
        torretaUS.write(90+ang);                  // Movemos el servo que control la posicion del sensor US   a 90 mas el valor ang
        ang = -ang;                               // Cambiamos de signo la variable ang
        return ang;                               // Devolvemos dicho valor
}

/******************  FUNCION QUE CONTROLA QUE ACCION REALIZAMOS SOBRE LAS RUEDAS SEGUN LECTURA DEL SENSOR US *********************/

void decidirMovimiento(int angulo,float dist,float dist_min){
    
    if (angulo == 45 && dist < dist_min){                                                // Si mirando a la izquierda detectamos pared demasiado proxima ....
             giroDrcha(RuedaDrcha,RuedaIzqda,Va_derecha,Va_izquierda,Reduccion_vel);     // Giramos a la derecha
    }
    else if (angulo == -45 && dist < dist_min){                                          // Si mirando a la derecha detectamos pared demasiado proxima ...
            giroIzqda(RuedaDrcha,RuedaIzqda,Va_derecha,Va_izquierda,Reduccion_vel);      // Giramos a la izquierda
    }
    else {                                                                               // En cualquier otra situación ...
            avanzar(RuedaDrcha,RuedaIzqda,Va_derecha,Va_izquierda);                      // Avanzamos en linea recta
    }

}

void setup() {

  
  RuedaDrcha.attach(4);                       // Declaracion del pin al que está conectado el servomotor de la rueda derecha
  RuedaIzqda.attach(5);                       // Declaracion del pin al que está conectado el servomotor de la rueda izquierda
  
  ControlUS.attach(7);                        // Declaracion del pin al que está conectado el micro servo que gira el sensor de ultrasonidos
    
  pinMode(PulsadorInicio,INPUT);              // Declaramos el pin del pulsador como entrada
  
 
  parar(RuedaDrcha,RuedaIzqda);               // Inicialmente las dos ruedas paradas
  
  while(digitalRead(PulsadorInicio) == 0){    // Bucle que espera a que se pulse el boton PulsadorInicio
           delay(10);
  }
  while(digitalRead(PulsadorInicio) == 1){    // Bucle que espera a que soltemos el boton PulsadorInicio
           delay(10); 
  } 
  
 
  /* Una vez pulsado el boton el robot comienza el circuito AVANZANDO RECTO y se mantiene así salvo que exista
      una interrupcion desde uno de los dos sensores IR */
  
  avanzar(RuedaDrcha,RuedaIzqda,Va_derecha,Va_izquierda); 
  
  T_inic = millis();                           // Tomamos la primera medida del valor de tiempo para saber cuando tenemos que girar el sensor
  
}



void loop() {

  /****************************************                  INICIO PROGRAMA PRINCIPAL              ********************************/
  
  
 T_final = millis();                                         // Tomamos el valor del tiempo en estos momentos
 if (T_final >= T_inic + periodo){                           // En el caso de que el tiempo pasado desde la ultima lectura sea mayor que "periodo" entonces...
 torsion = mover_sensor(torsion,ControlUS);                  // Movemos el cabezal alternativamente a la izquierda o a la derecha segun proceda
 distancia = ultrasonidos.read();                            // Leemos distancia en cms al obstaculo mas cercano con el sensor de US
 decidirMovimiento(torsion,distancia,d_limite);              // Decidimos hacia donde movemos las ruedas del robot
 T_inic = T_final;                                           // A la salida de este condicional asignamos el valor de T_final como inicial del siguiente periodo de medida
 }
 
  
  /****************************************                       FIN DEL PROGRAMA                   ********************************/

}
