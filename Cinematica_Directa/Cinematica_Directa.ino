//Programa para calcular la cinemática directa de un manipulador planar
//de dos grados de libertad

//Librerias
#include <Servo.h>

Servo motor1;
Servo motor2;
Servo motor3;
//variables
float nv = 0;
// enlaces
float L1 = 0, L2 = 0, L3 = 0;
// angulos
float angle1;
float angle2;
float angle3;
// coordenadas
float x;
float y;
float rad_angle1;
float rad_angle2;
float rad_angle3;
volatile float pi = 3.14159265359;

void setup() {
  Serial.begin(9600);
  // Configuracion de servos
  motor1.attach(9);
  motor2.attach(10);
  motor3.attach(11);
  // Configuracion del LED
  pinMode(2, OUTPUT);
  // envia a los servos a su posición inicial
  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  Serial.println("---------------------------------------------------");
  Serial.println("| Programa para calcular la cinematica directa de  |");
  Serial.println("| un manipulador planar de tres grados de libertad |");
  Serial.println("|  * No se aceptan valores negativos para las      |");
  Serial.println("|    longitudes y los grados.                      |");
  Serial.println("|  * No se aceptan valores mayores a 180 para los  |");
  Serial.println("|    grados                                        |");
  Serial.println("---------------------------------------------------");
  //captura las longitudes de los brazos
  Serial.print("  Longitud del primer enlace: ");
  L1 = leeLongitud();
  Serial.print(L1); Serial.println(" cm");
  Serial.print("  Longitud del segundo enlace: ");
  L2 = leeLongitud();
  Serial.print(L2); Serial.println(" cm");
  Serial.print("  Longitud del tercer enlace: ");
  L3 = leeLongitud();
  Serial.print(L3); Serial.println(" cm");
  Serial.println();
}

//Rutina para evitar el valor cero que envia la función parseFloat
void no_valor() {
  while (Serial.available() == 0) {}
  nv = Serial.parseFloat();
}

//Programa principal
void loop() {
  Serial.print("  Angulo 1: ");
  angle1 = leeGrados();
  Serial.print(angle1); Serial.println(" °"[2]);

  Serial.print("  Angulo 2: ");
  angle2 = leeGrados();
  Serial.print(angle2); Serial.println(" °"[2]);

  Serial.print("  Angulo 3: ");
  angle3 = leeGrados();
  Serial.print(angle3); Serial.println(" °"[2]);

  delay(1000);
  //calculo de angulos en radianes
  rad_angle1 = (angle1 * pi) / 180;
  rad_angle2 = (angle2 * pi) / 180;
  rad_angle3 = (angle3 * pi) / 180;
  //envio de posición de servos
  posicionMotores(rad_angle1, rad_angle2, rad_angle3);
  delay(1000);

  x = L1 * cos(rad_angle1) + L2 * cos(rad_angle1 + rad_angle2) + L3 * cos(rad_angle1 + rad_angle2 + rad_angle3);
  y = L1 * sin(rad_angle1) + L2 * sin(rad_angle1 + rad_angle2) + L3 * sin(rad_angle1 + rad_angle2 + rad_angle3);
  delay(1000);
  
  //despliega los resultados
  //Serial.println("--->");
  Serial.println();
  Serial.print("Posicion del elemento final: ");
  Serial.print("(x, y) = (");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.println(")");
  digitalWrite(2, HIGH);
  delay(5000);

  // envia a los servos a su posición inicial
  posicionMotores(0, 0, 0);
  digitalWrite(2, LOW);
  delay(5000);
  Serial.println();
  Serial.println("--- Nuevos angulos ---");
}

void posicionMotores(int p1, int p2, int p3){
  motor1.write(p1);
  motor2.write(p2);
  motor3.write(p3);
}

float leeLongitud(){
  float l = -1;
  while(l <= 0){
    while(Serial.available() == 0) {}
    l = Serial.parseFloat();
    no_valor();  //función para evitar el cero de la captura de teclado
  }
  return l;
}

float leeGrados(){
  float g = -1;
  while(g < 0 || g > 180){
    while(Serial.available() == 0) {}
    g = Serial.parseFloat();
    no_valor();  //función para evitar el cero de la captura de teclado
  }
  return g;
}