import processing.serial.*; 
int val; 
Serial port;
int i;
int k;
int I;
int K;
float steeringOutput;
int cx;
int cy;
int Cx;
int Cy;
int o=255;
int s; 
String myString=null;

void setup() {
  size(600, 1000); 
  ///println(Serial.list());

  port=new Serial(this, "COM7", 9600);
}

void draw() {///아두이노에서 loop랑 같다고 생각하면 됨
  background(100);//창의 색깔을 검은색으로(0이 검은색 뜻함)

  //fill(o); //밑의 사각형을 흰색으로 채운다.


  //rect(130, 500, 50, -i);//값에 해당하는 사각형 그림, I는 아두이노의 첫번째 휨센서 값임,밑에서 정의할 거임, 저 사각형의 의미는 값의 변화량만큼 사각형의 길이가 변하는 것
  //fill(o);
  //rect(200, 500, 50, -k);//동일, K는 두번째 휨센서 값
  //fill(o);
  
  text(i, 130, 100);
  rect(130, 500, 50, -I);//값에 해당하는 사각형 그림, I는 아두이노의 첫번째 휨센서 값임,밑에서 정의할 거임, 저 사각형의 의미는 값의 변화량만큼 사각형의 길이가 변하는 것
  fill(o);
  
  text(k, 200, 100);
  rect(200, 500, 50, -K);//동일, K는 두번째 휨센서 값
  fill(o);
  
  text(steeringOutput/10, 300, 100);
  delay(10);
}



void serialEvent(Serial p) { //시리얼 이벤트 함수 호출(시리얼이벤트는 다른 만드는 함수들과 다르게 프로세싱에서 시리얼 통신에 관해 정의된 함수, 이 함수 안에 시리얼통신에 관한 내용을 넣으면 자동으로 draw안에서 실행됨)
  try { 
    myString=p.readStringUntil(';');//mystring은 시리얼에서 "."까지 읽은 문자열이다.  
    String[] list=split(myString, ',');//String 배열은 myString값을 ,를 기준으로 쪼갠 것임
    i=int(list[0])*1;//첫번째 값을 i로
    k=int(list[1])*1;//두번째갑을 k로
    steeringOutput = int(list[2]);
    
    I=i*400/750;
    K=k*400/750;

    print(i);
    print(k); 
    print(" ");
  }
  catch(Exception e) {//try에서 시행해야 할 것을 시행할 동안 에러가나면 catch 부분으로 에러를 돌려 결과적을 로 에러가 나도 통신이 계속될 수 있도록 한다.
  }
}