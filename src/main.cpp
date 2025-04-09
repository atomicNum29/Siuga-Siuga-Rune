#include <Arduino.h>
// #include <IntervalTimer.h>
// #include <cmath>

/*

초기 각도  = 0
베이스와 수평 = 90


*/

 
// const double PI = 3.14159265358979323846;

int buffer_sequence;

const double StepAngle = 1.8;




class MOTER{

private:

  int step_pin;
  int direction_pin;
  
  
  double current_angle; // 현재 모터 각도
  double target_angle; // 이동해야할 모터 각도
  bool move_state;  // 현재각도와 타겟각도가 다르다면 움직여야하므로 1 , 같다면 0 

  bool direction_state;  
  ////////////////// private - 변수선언 END



public: 


  MOTER(int step_pin_num,int direction_pin_num ){
    
    this -> step_pin = step_pin_num;
    this -> direction_pin = direction_pin_num;


    this -> current_angle = 0.0; // 초기 각도는 상단링크+하단링크와 베이스+플랫폼이 수직인 상태
    this -> move_state = false; 
    
    this -> direction_state = 0;
  }//////// 생성자 END



  void MoterSetup(){
    pinMode(this->step_pin,OUTPUT);
    pinMode(this->direction_pin,OUTPUT);
    
  }///////// Moter set up   END 


  inline void Direction_Set(int _direction){
    
    digitalWrite(this->direction_pin,_direction);

  }///////// Direction_set END


  void Target_Angel_Set(double angle){
    
    this -> target_angle = angle;
    this -> move_state = true; // 타겟앵글이 변경되면 일단 움직여야 하는 상태로.

  }////////// target_angle_set END

  



  inline void OneStep(){

    digitalWrite(this->step_pin,1);
    delayMicroseconds(200);
    
    digitalWrite(this->step_pin,0);
    delayMicroseconds(200);

    
  } /////// oneStep ENd

  void Move_Target(){
   
    /*
    
    cur  -45  
    tar  -90

    -90 - (-45)
    -45
    

    cur -45
    tar 0
    
    0 - (-45)
    +45

    */
    if(this ->move_state){

      if( (target_angle - current_angle)*(target_angle - current_angle) <= 1.8*1.8  ){ // 절대값이 1.8이하로 차이난다면
        this ->move_state = false; // 움직일 수 있는 각도까지 움직였으면 회전이 필요없는 상태로 변경
        return;
      }
      else if(target_angle - current_angle < 0){
        this -> Direction_Set(false); // -각도로 ( = 상단링크가 위쪽,펼쳐지는 방향으로)
        current_angle -= StepAngle;
      }
      else{
        this -> Direction_Set(true); // +각도로 ( = 상단링크가 아래쪽, 오므려지는 방향으로)
        current_angle += StepAngle;
      }

      OneStep(); //설정된 방향대로 1.8도 이동 
    
    }



  }////////// Move_Target END
  
  bool Move_State_Get(){
    return this -> move_state;
  }///////// move_state_get END


  double Get_Current_Angel(){
    return this -> current_angle;
  }////////// Get_Current_Angel END


}; /////////모터 클래스정의 끝

class DELTA_ROBOT {
private:

  MOTER& moter1;
  MOTER& moter2;
  MOTER& moter3;


public:
    
  DELTA_ROBOT(MOTER& _moter1,MOTER& _moter2,MOTER& _moter3)
    :moter1(_moter1),moter2(_moter2),moter3(_moter3) {}
  ////////////생성자 END

    
  void dleta_target_angle_set(double angle, int sequecne){

    if(sequecne == 0){
      this -> moter1.Target_Angel_Set(angle);
    }
    else if(sequecne == 1){
      this -> moter2.Target_Angel_Set(angle);
    }
    else if( sequecne == 2){
      this -> moter3.Target_Angel_Set(angle);
    }

  }///////// dleta_target_anlge END
     


  bool check_move_state(){

    
    if( moter1.Move_State_Get() + moter2.Move_State_Get() + moter3.Move_State_Get() == 0  ){

      return false; // 3개 모두 움직임이 끝난 상태(0 + 0 + 0)인 경우 0 

    } 

    return true; // 하나라도 움직여야 하는경우 1


  }//////// check_move_state END



  void move(){

    while(  this -> check_move_state() ){

        this -> moter1.Move_Target();
        this -> moter2.Move_Target();
        this -> moter3.Move_Target();

    }
  }////////// move END
  
  

}; ///////DELTA_ROBOT END 





MOTER moter1(4,5);
MOTER moter2(6,7);
MOTER moter3(8,9);
DELTA_ROBOT delta_robot(moter1,moter2,moter3);



void setup() {
  
  moter1.MoterSetup();
  moter2.MoterSetup();
  moter3.MoterSetup();
  buffer_sequence  = 0;
  Serial.begin(9600);

}


void loop() {}



void serialEvent()
{
	static char buffer[100] = {0}; // 시리얼로 입력된 데이터를 저장하기 위한 공간
	static int index = -1;		   // buffer의 사용 상태를 알기 위한 변수
	
    if (Serial.available()) // 버퍼에 읽을 수 있는 문자가 있다면 실행합니다.
	{
		index++;
		buffer[index] = Serial.read();
    Serial.println(buffer); 

	}


	if (buffer[index] == '\n'){   

    String input(buffer); // 버퍼에 받은 정보를 문자열로 바꾸는 함수를 호출합니다.

    delta_robot.dleta_target_angle_set(input.toFloat(),buffer_sequence); // 순서대로 0,1,2 0(3%3) 1,2,3번 모터에 입력된 각도를 타켓앵글로 설정


    buffer_sequence++;
    

    if(buffer_sequence == 3){ // 각도를 3번 받았으면 이동시작.
      delta_robot.move(); //TASK로?
    }


    buffer_sequence = (buffer_sequence+1) %3;
		index = -1; // 한 명령어에 대한 처리가 끝났으므로 buffer 사용 정보 초기화.
  }

}



