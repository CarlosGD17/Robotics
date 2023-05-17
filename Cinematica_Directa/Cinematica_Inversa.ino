#include <Servo.h>
Servo motor1, motor2, motor3;
 // longitud de cada enlace (brazo)  
float L1, L2, L3;
float pi = 3.14159265359;

// angulos entre los ennlaces de los brazos
float angle1, angle2, angle3;
float angleTotal;      

float radAngle1, radAngle2, radAngle3;
float radAngleTotal;

// para calcular el punto final
float x, y;
float x1, y1;
float x2, y2;

float nv = 0;

void setup() {
  motor1.attach(9);
  motor2.attach(10);
  motor3.attach(11);
  Serial.begin(9600);
  posicionarMotores(0, 0, 0);
  
  Serial.println("---------------------------------------------------");
  Serial.println("| Programa para calcular la cinematica inversa de  |");
  Serial.println("| un manipulador planar de tres grados de libertad |");
  Serial.println("|  * No se aceptan valores negativos para las      |");
  Serial.println("|    longitudes.                      		   |");
  Serial.println("---------------------------------------------------");
  
  Serial.println("Digite las longitudes del:");
  Serial.print("   - Primer brazo: ");
  L1 = leerLongitud();
  Serial.print(L1); Serial.println("cm");
   
  Serial.print("   - Segundo brazo: ");
  L2 = leerLongitud();
  Serial.print(L2); Serial.println("cm");

  Serial.print("   - Tercer brazo: ");
  L3 = leerLongitud();
  Serial.print(L3); Serial.println("cm");
  Serial.println();
}

void loop(){
  Serial.println("Digite las coordenadas: ");
  Serial.print("   - x: ");
  x = leerLongitud();
  Serial.println(x);
      
  Serial.print("   - y: ");
  y = leerLongitud();
  Serial.println(y);
  
  Serial.println();
  
  if((sqrt(x*x + y*y) > L1 + L2 + L3) ||
    	(x > 0 && y < 0)){
    Serial.println("**El punto no esta dentro del espacio de trabajo**");
    loop();
  }
  
  probarAngulos();
  
  delay(1000);    

  posicionarMotores(angle1, angle2, angle3);
   
  x1 = L1 * cos(radAngle1);
  y1 = L1 * sin(radAngle1);
  
  Serial.println("-->");
  Serial.println("Los motores deben girar:");
  Serial.print("   Angulo 1 = "); Serial.println(angle1);
  Serial.print("   Angulo 2 = "); Serial.println(angle2);
  Serial.print("   Angulo 3 = "); Serial.println(angle3);
  Serial.println("-->");
  Serial.println();
  
  delay(5000);
  digitalWrite(2, LOW);
  posicionarMotores(0, 0, 0);
  delay(500);
 }

void posicionarMotores(int p1, int p2, int p3){
  motor1.write(p1);
  delay(200);
  motor2.write(p2);
  delay(200);
  motor3.write(p3);
}

float leerLongitud(){
  float l = -1;
  while(l <= 0){
    while(Serial.available() == 0) {}
    l = Serial.parseFloat();
    no_valor();  //función para evitar el cero de la captura de teclado
  }
  return l;
}

float validarCoordenadas(char c){
  int temp = 21;
  while(temp > 20){
    while(Serial.available() == 0) {}
    temp = Serial.parseFloat();
    no_valor();
  }
  return temp;
}

//Rutina para evitar el valor cero que envia la función parseFloat
void no_valor() {
  while (Serial.available() == 0) {}
  nv = Serial.parseFloat();
}

void probarAngulos(){
  float tempX, tempY;
  for(int i = 0; i <= 540; i=i+5){
    radAngleTotal = (i * pi) / 180;
  	x2 = x - L3 * cos(radAngleTotal);
  	y2 = y - L3 * sin(radAngleTotal);   
  	radAngle2 = acos((sq(x2) + sq(y2) - sq(L1) - sq(L2)) / (2 * L1 * L2));
  	radAngle1 = acos(((L1 + L2 * cos(radAngle2)) * x2 + (L2 * y2 * sin(radAngle2))) / (sq(x2) + sq(y2)));
  
  	angle1 = (radAngle1*180) / pi;
  	angle2 = (radAngle2*180) / pi;
  	angle3 = i - angle1 - angle2;
    
    if((angle1 >= 0 && angle1 <= 180) && 
       (angle2 >= 0 && angle2 <= 180) &&
       (angle3 >= 0 && angle3 <= 180)){
      Serial.println(i);
      break;
    }
  }
}
