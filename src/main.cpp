#include <Arduino.h>
#include <IntervalTimer.h>
#include <iostream>
#include <cmath>

/*

초기 각도?

*/

 
// const double PI = 3.14159265358979323846;
const double StepAngle = 1.8;

inline double radian(double degree){
    return degree * PI / 180.0;
}

inline double degree(double radian){
    return radian *  180.0/PI;
}

class MOTER{

private:

  int step_pin;
  int direction_pin;
  int delay;
  
  double current_angle;
  
  bool step_state;
  bool direction_state;
  
  inline void OneStep(){

    digitalWrite(this->step_pin,this->step_state);
    this -> step_state = !this -> step_state;
    digitalWrite(this->step_pin,this->step_state);
    this -> step_state = !this -> step_state;   
    
  }

  inline void Direction_Set(int _direction){
    
    digitalWrite(this->direction_pin,_direction);

  }

public:

  MOTER(int step_pin_num,int direction_pin_num ){
    
    this -> step_pin = step_pin_num;
    this -> direction_pin = direction_pin_num;

    this -> current_angle = 0.0;
    this -> step_state = 0;
    this -> direction_state = 0;
  }

  ~MOTER(){
  }

  void MoterSetup(){
    pinMode(this->step_pin,OUTPUT);
    pinMode(this->direction_pin,OUTPUT);
    
  }

  void SetDelay(int _delay){
    this -> delay = _delay;
  }

  void Rotate(double _degree){
    int _step;

    if(_degree >= 0){
      _step =  (int)(_degree/StepAngle);

      this -> Direction_Set(1);

      for(int i = 0; i<_step; i++){

        this -> OneStep();  
        this -> current_angle += StepAngle * (double)this-> direction_state; 

      }
    }
    else{

      _step = (int)(_degree*-1.0/StepAngle);
      
      this -> Direction_Set(0);


      for(int i = 0; i<_step; i++){

        this -> OneStep();  
        this -> current_angle -= StepAngle;
      }

    
    }

  }
  
  double Get_Current_Angel(){
    return this -> current_angle;
  }


};

class DELTA_ROBOT {
private:
    double BASE_SIDE_LEN; // B
    double PLATFORM_SIDE_LEN; //sp
    double UPPER_LINK_LEN; // L
    double LOWER_LINK_LEN; // l
    double BASE_U_LEN; // U_b
    double BASE_W_LEN; // W_b
    double PLATFORM_U_LEN; // U_p
    double PLATFORM_W_LEN; // W_p
    double X_OFFSET; // a;
    double X_SHIFT; // b;
    double Y_SHIFT; // c;

    MOTER& moter1;
    MOTER& moter2;
    MOTER& moter3;

public:
    
    DELTA_ROBOT(MOTER& _moter1,MOTER& _moter2,MOTER& _moter3)
    :moter1(_moter1),moter2(_moter2),moter3(_moter3) {}

    
    void setup_length(double B_LEN, double P_LEN, double U_LNE, double L_LEN) {
        BASE_SIDE_LEN = B_LEN;
        PLATFORM_SIDE_LEN = P_LEN;
        UPPER_LINK_LEN = U_LNE;
        LOWER_LINK_LEN = L_LEN;
        BASE_U_LEN = B_LEN*sqrt(3)/3; // U_b
        BASE_W_LEN = B_LEN*sqrt(3)/6; // W_b
        PLATFORM_U_LEN = P_LEN*sqrt(3)/3; // U_p
        PLATFORM_W_LEN = P_LEN*sqrt(3)/6; // W_p
        X_OFFSET = BASE_W_LEN - PLATFORM_U_LEN; //a
        X_SHIFT = PLATFORM_SIDE_LEN*cos(radian(60)) - BASE_W_LEN*cos(radian(30)); //b
        Y_SHIFT = PLATFORM_W_LEN - BASE_W_LEN*sin(radian(30)); //c

    
    }

    //
    double Moter1_Angle(double x,double y,double z){

        double C = X_OFFSET*X_OFFSET + UPPER_LINK_LEN*UPPER_LINK_LEN - LOWER_LINK_LEN*LOWER_LINK_LEN; 
        
        double A = -2*UPPER_LINK_LEN*X_OFFSET;
        
        double B = 2*z*UPPER_LINK_LEN;

        C += x*x + y*y + z*z + 2*y*X_OFFSET;
        A += -2*UPPER_LINK_LEN*y;

        return degree(2*atan( (-A - sqrt( A*A + B*B - C*C  ))/(C-B)   ));
    }
    double Moter2_Angle(double x,double y,double z){

        double C = X_SHIFT*X_SHIFT + Y_SHIFT*Y_SHIFT + UPPER_LINK_LEN*UPPER_LINK_LEN - LOWER_LINK_LEN*LOWER_LINK_LEN; 
        
        double A = 2*UPPER_LINK_LEN*( X_SHIFT*cos(radian(30)) + Y_SHIFT*sin(radian(30))   );
        
        double B = 2*z*UPPER_LINK_LEN;

        C += x*x + y*y + z*z + 2*y*Y_SHIFT + 2*x*X_SHIFT;
        A += 2*UPPER_LINK_LEN*( x*cos(radian(30)) + y*sin(radian(30)) );

        return degree(2*atan( (-A - sqrt( A*A + B*B - C*C  ))/(C-B)   ));
    }

    double Moter3_Angle(double x,double y,double z){

        double C =  X_SHIFT*X_SHIFT + Y_SHIFT*Y_SHIFT + UPPER_LINK_LEN*UPPER_LINK_LEN - LOWER_LINK_LEN*LOWER_LINK_LEN; 
        
        double A = 2*UPPER_LINK_LEN*( X_SHIFT*cos(radian(30)) + Y_SHIFT*sin(radian(30)) );
        
        double B = 2*z*UPPER_LINK_LEN;


        C += x*x + y*y + z*z + 2*y*Y_SHIFT - 2*x*X_SHIFT;
        A += 2*UPPER_LINK_LEN*( x*cos(radian(30)) + y*sin(radian(30)) );

        return degree(2*atan( (-A - sqrt( A*A + B*B - C*C  ))/(C-B)   ));
    }
    

    void move(double x,double y, double z){
      //이동
      double moter1_angel =  Moter1_Angle(x,y,z);
      double moter2_angel =  Moter2_Angle(x,y,z);
      double moter3_angel =  Moter3_Angle(x,y,z);
      
      this -> moter1.Rotate(moter1_angel);
      this -> moter2.Rotate(moter2_angel);
      this -> moter3.Rotate(moter3_angel);

      //원위치
      moter1_angel = -  moter1.Get_Current_Angel();
      moter2_angel = -  moter2.Get_Current_Angel();
      moter3_angel = -  moter3.Get_Current_Angel();
      
      this -> moter1.Rotate(moter1_angel);
      this -> moter2.Rotate(moter2_angel);
      this -> moter3.Rotate(moter3_angel);

    }
   
};





MOTER moter1(2,3);
MOTER moter2(4,5);
MOTER moter3(6,7);
DELTA_ROBOT delta_robot(moter1,moter2,moter3);



void setup() {
  moter1.MoterSetup();
  moter2.MoterSetup();
  moter3.MoterSetup();
  delta_robot.setup_length(150,75,200,460);

  Serial.begin(9600);
}

void loop() {

  char buf = Serial.read();

  if(buf == 'a'){
    delta_robot.move(50.0,50.0,-400.0);
  }
  else if(buf == 'b'){
    delta_robot.move(-50.0,50.0,-400.0);
  }
  else if(buf == 'c'){
    delta_robot.move(50.0,-50.0,-400.0);    
  }
  else if(buf == 'd'){
    delta_robot.move(-50.0,-50.0,-400.0);
  }

}
