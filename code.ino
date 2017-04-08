// Tecsup
// Autores: Rosario Huanca Gonza
//          Junior Usca Huacasi

const int infra0=A7;
const int infra1=A6;
const int infra2=A5;
const int infra3=A4;
const int infra4=A3;
const int infra5=A2;
const int infra6=A1;
const int infra7=A0;

const int speed_left_engine = 5; // PWM se conecta al pin 1 del puente-H

const int dir_left_engine_1 = 3; // Entrada 2 del puente-H
const int dir_left_engine_2 = 4; // Entrada 7 del puente-H

const int speed_right_engine = 2; // PWM se conecta al pin 9 del puente-H

const int dir_right_engine_1 = 6; // Entrada 10 del puente-H
const int dir_right_engine_2 = 9; // Entrada 14 del puente-H


int * sensorInfrarojoVal;
#define size 8 /// number of sensors
#define rules 12 /// number of rules

float gradoPertenencia[size][4];
float implicancia[rules];
float avgCrisp[rules];

float last_angle = 0;

float radio = 0.165; /// distance bettewen wheels in meters

float distance = 346.97; /// getter from a linear function

const int m_speed = 165; /// velocity in PWM [0-255]

#define ra 0 
#define rb 610 // 3 V
#define rc 660 // 3.6 V
#define rd 700 // 4.3 V
#define re 760 // 4.85 V
#define rf 1023   // 5 V
/*
#define ra 0 
#define rb 640 // 3 V
#define rc 717 // 3.6 V
#define rd 730 // 4.3 V
#define re 780 // 4.85 V
#define rf 1023   // 5 V
*/

/*#define rb 614.4 // 3 V
#define rc 737.28 // 3.6 V
#define rd 880.64 // 4.3 V
#define re 993.28 // 4.85 V
#define rf 1023   // 5 V*/

#define ani -90 
#define aa -75 
#define ab -60
#define ac1 -45
#define ac -30
#define ad -15
#define ae 0
#define af 15
#define ag 30
#define ag1 45
#define ah 60
#define ai 75
#define anf 90

#define lejos 0
#define borde 1
#define medio 2
#define dentro 3

#define defrente 0
#define ligeroIzq 1
#define ligeroDer 2
#define izquierda 3
#define derecha 4
#define medioIzq 5
#define medioDer 6
#define fuerteIzq 7
#define fuerteDer 8


void setup() {
  Serial.begin(9600);
  sensorInfrarojoVal = new int[size];
  last_angle = 0.0;

  pinMode(dir_left_engine_1, OUTPUT);
  pinMode(dir_left_engine_2, OUTPUT);

  pinMode(dir_right_engine_1, OUTPUT);
  pinMode(dir_right_engine_2, OUTPUT);

}

// the loop routine runs over and over again forever:
void loop() {
  
  /// prueba1: mover hacia adelante
  //move(255,255);
  /// prueba2: mover hacia adelante con velocidad promedio
  //move(m_speed, m_speed);
  /// pruebafinal: mover con angulos
  
  leerInfrarojo();
  matGradoPertenencia();  
  motorInferencia();
  //movement();
  

  imprimir();

  //delay(100);
}

void leerInfrarojo(){
  sensorInfrarojoVal[0]=constrain(analogRead(infra0),0,1023);  
  sensorInfrarojoVal[1]=constrain(analogRead(infra1),0,1023);
  sensorInfrarojoVal[2]=constrain(analogRead(infra2),0,1023);
  sensorInfrarojoVal[3]=constrain(analogRead(infra3),0,1023);
  sensorInfrarojoVal[4]=constrain(analogRead(infra4),0,1023);
  sensorInfrarojoVal[5]=constrain(analogRead(infra5),0,1023);
  sensorInfrarojoVal[6]=constrain(analogRead(infra6),0,1023);
  sensorInfrarojoVal[7]=constrain(analogRead(infra7),0,1023);
  //sensorInfrarojoVal[6]=0;
  //sensorInfrarojoVal[7]=0;
}

float fuzyTrapezoidal(float x, float a1, float b1, float c1, float d1){
  float g1 = 1.0; float g2 = 1.0;
  bool flag1 = true; bool flag2 = true;
  if ( b1-a1 != 0.0 ) { g1 = (x-a1)/(b1-a1); flag1 = false; }
  if ( d1-c1 != 0.0 ) { g2 = (d1-x)/(d1-c1); flag2 = false; }
  if (flag1) if(x<a1) return false;
  if (flag2) if(x>c1) return false;
  if (flag1 && flag2) if(x>=a1 && x<=c1) return true; else return false;
  return my_max( my_min( my_min( g1, 1.0 ), g2 ), 0 );
}

float invFuzyTrapezoidal(float g, float a1, float b1, float c1, float d1){
  /// sacamos el valor x del lado izquierdo del trapezoide
  float x1 = a1+(g*(b1-a1));
  /// sacamos el valor x del lado derecho del trapezoide
  float x2 = d1-(g*(d1-c1));
  /// sacamos el promedio
  return ((x2-x1)/2.0)+x1;
}

void matGradoPertenencia(){
  for(int i=0;i<size;++i){
    gradoPertenencia[i][lejos] = fuzyTrapezoidal(sensorInfrarojoVal[i],ra,ra,rb,rc);
    gradoPertenencia[i][borde] = fuzyTrapezoidal(sensorInfrarojoVal[i],rb,rc,rc,rd);
    gradoPertenencia[i][medio] = fuzyTrapezoidal(sensorInfrarojoVal[i],rc,rd,rd,re);
    gradoPertenencia[i][dentro] = fuzyTrapezoidal(sensorInfrarojoVal[i],rd,re,rf,rf);
  }
}

float motorInferencia(){
  /// calculamos la implicancia
  h0();h1();h2();h3();h4();h5();h6();h7();h8();h9();h10();h11();

  /// DESFUZIFICACION sumatorias
  float dividendo = .0;
  float divisor = .0;
  for(byte i = 0; i < rules; ++i ) {
    dividendo += (implicancia[i] * avgCrisp[i]);
    divisor += implicancia[i];
  }
  if (divisor == .0) return last_angle;
  //if (divisor == .0) return 0;

  last_angle = dividendo / divisor;

  return last_angle;
}

void imprimir(){
  Serial.println(" ==================================================== ");
  for(int i=0;i<size;i++){
    Serial.print(sensorInfrarojoVal[i]);
    Serial.print(" || ");
  }
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" Lejos | Borde | Medio | Dentro |");
  for(int i=0;i<size;i++){
    for(int j=0;j<4;j++){
      Serial.print(gradoPertenencia[i][j]);
      Serial.print(" | ");
    }
    Serial.println(" ");
  }
  for(int i=0;i<rules;i++){
    Serial.print(implicancia[i]);
    Serial.print(" || ");
  }
  Serial.println(" ");
  for(int i=0;i<rules;i++){
    Serial.print(avgCrisp[i]);
    Serial.print(" || ");
  }
  Serial.println(" ANGULO: ");

  Serial.println(last_angle);
  
  
}

float my_min(float x, float y) {
  if(x<y) return x; else return y;
}

float my_max(float x, float y) {
  if(x>y) return x; else return y;
}

void movement() {
  /// variacion_de_espacio/2*PI*R = angulo/360
  float velocidad_rueda = ((last_angle * PI * radio) / 180.0) * distance;
  velocidad_rueda = velocidad_rueda / 2.0;
  velocidad_rueda = abs(velocidad_rueda);
  if (last_angle < 0.0) {
    move (m_speed - velocidad_rueda, m_speed + velocidad_rueda);
  } else if (last_angle > 0.0) {
    move (m_speed + velocidad_rueda, m_speed - velocidad_rueda);
  } else { /// if angle is 0
    move (255,255);
  }
}


void move (byte speed_left, byte speed_right) {

  Serial.print(speed_right);
  Serial.print("  ||  ");
  Serial.println(speed_left);
  
  digitalWrite(dir_left_engine_1,HIGH);
  digitalWrite(dir_left_engine_2,LOW);
    
  digitalWrite(dir_right_engine_1,HIGH);
  digitalWrite(dir_right_engine_2,LOW);

  analogWrite(speed_left_engine, speed_right);
  analogWrite(speed_right_engine, speed_left);
}




/// RULES
void h0() {
  implicancia[0] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        gradoPertenencia[1][lejos]), 
        gradoPertenencia[2][lejos]), 
        gradoPertenencia[3][dentro]), 
        gradoPertenencia[4][dentro]),
        gradoPertenencia[5][lejos]),
        gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// defrente
  avgCrisp[0] = invFuzyTrapezoidal(implicancia[0], ad,ae,ae,af);
}
void h1() {
  implicancia[1] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        gradoPertenencia[1][lejos]), 
        gradoPertenencia[2][lejos]), 
        my_max(gradoPertenencia[3][borde], gradoPertenencia[3][medio])), 
        gradoPertenencia[4][dentro]),
        gradoPertenencia[5][lejos]),
        gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// ligero a la izquierda
  avgCrisp[1] = invFuzyTrapezoidal(implicancia[1], ac,ad,ad,ae);
}
void h2() {
  implicancia[2] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        gradoPertenencia[1][lejos]), 
        gradoPertenencia[2][lejos]), 
        gradoPertenencia[3][dentro]),
        my_max(gradoPertenencia[4][borde], gradoPertenencia[4][medio])), 
        gradoPertenencia[5][lejos]),
        gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// ligero a la derecha
  avgCrisp[2] = invFuzyTrapezoidal(implicancia[2], ae,af,af,ag);
}
void h3() {
  implicancia[3] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        gradoPertenencia[1][lejos]), 
        gradoPertenencia[2][lejos]), 
        1.0-gradoPertenencia[3][lejos]),
        gradoPertenencia[4][lejos]), 
        gradoPertenencia[5][lejos]),
        gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// ligero izquierda
  avgCrisp[3] = invFuzyTrapezoidal(implicancia[3], ac1,ad,ad,ae);
}
void h4() {
  implicancia[4] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        gradoPertenencia[1][lejos]), 
        gradoPertenencia[2][lejos]), 
        gradoPertenencia[3][lejos]), 
        1.0-gradoPertenencia[4][lejos]),
        gradoPertenencia[5][lejos]),
        gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// ligero derecha
  avgCrisp[4] = invFuzyTrapezoidal(implicancia[4], ae,af,af,ag1);
}
void h5() {
  implicancia[5] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        1.0-gradoPertenencia[1][lejos]), 
        1.0-gradoPertenencia[2][lejos]), 
        gradoPertenencia[3][lejos]), 
        gradoPertenencia[4][lejos]),
        gradoPertenencia[5][lejos]),
        gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// medio a la izquierda
  avgCrisp[5] = invFuzyTrapezoidal(implicancia[5], aa,ab,ab,ac);
}
void h6() {
  implicancia[6] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        gradoPertenencia[1][lejos]), 
        gradoPertenencia[2][lejos]), 
        gradoPertenencia[3][lejos]), 
        gradoPertenencia[4][lejos]),
        1.0-gradoPertenencia[5][lejos]),
        1.0-gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// medio a la derecha
  avgCrisp[6] = invFuzyTrapezoidal(implicancia[6], ag,ah,ah,ai);
}
void h7() {
  implicancia[7] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        1.0-gradoPertenencia[0][lejos], 
        1.0-gradoPertenencia[1][lejos]), 
        gradoPertenencia[2][lejos]), 
        gradoPertenencia[3][lejos]), 
        gradoPertenencia[4][lejos]),
        gradoPertenencia[5][lejos]),
        gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// fuerte a la izquierda
  avgCrisp[7] = invFuzyTrapezoidal(implicancia[7], ani,ani,aa,ab);
}
void h8() {
  implicancia[8] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        gradoPertenencia[1][lejos]), 
        gradoPertenencia[2][lejos]), 
        gradoPertenencia[3][lejos]), 
        gradoPertenencia[4][lejos]),
        gradoPertenencia[5][lejos]),
        1.0-gradoPertenencia[6][lejos]),
        1.0-gradoPertenencia[7][lejos]);
  /// fuerte a la derecha
  avgCrisp[8] = invFuzyTrapezoidal(implicancia[8], ah,ai,anf,anf);
}
void h9() {
  implicancia[9] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        gradoPertenencia[1][lejos]), 
        1.0-gradoPertenencia[2][lejos]), 
        1.0-gradoPertenencia[3][lejos]), 
        gradoPertenencia[4][lejos]),
        gradoPertenencia[5][lejos]),
        gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// izquierda
  avgCrisp[9] = invFuzyTrapezoidal(implicancia[9], ab,ac,ac,ad);
}
void h10() {
  implicancia[10] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][lejos], 
        gradoPertenencia[1][lejos]), 
        gradoPertenencia[2][lejos]), 
        gradoPertenencia[3][lejos]), 
        1.0-gradoPertenencia[4][lejos]),
        1.0-gradoPertenencia[5][lejos]),
        gradoPertenencia[6][lejos]),
        gradoPertenencia[7][lejos]);
  /// derecha
  avgCrisp[10] = invFuzyTrapezoidal(implicancia[10], af,ag,ag,ah);
}

void h11() {
  implicancia[11] = my_min( my_min( my_min( my_min( my_min( my_min( my_min(
        gradoPertenencia[0][dentro], 
        gradoPertenencia[1][dentro]), 
        gradoPertenencia[2][dentro]), 
        gradoPertenencia[3][dentro]), 
        gradoPertenencia[4][dentro]),
        gradoPertenencia[5][dentro]),
        gradoPertenencia[6][dentro]),
        gradoPertenencia[7][dentro]);
  /// defrente
  avgCrisp[11] = invFuzyTrapezoidal(implicancia[11], ad,ae,ae,af);
}
