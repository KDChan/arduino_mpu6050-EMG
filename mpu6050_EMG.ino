//최종 결과만 전달 a[0] = 0x0F, a[1] = 총 카운트, a[2] = 유효카운트, a[3] = 평균점수(정수형)

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include <SoftwareSerial.h>

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

int val;
int count = 0;
int x;
int y;
int z = 0;
int result;
int count2 = 0;
int avr = 0;

int TX = 2;
int RX = 3;

int rec;

SoftwareSerial mySerial(2, 3);

byte buffer[1024];
int bufferPosition;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; //가속도 x, y값
double temp; // Temperature
double gyroXangle, gyroYangle; //자이로센서의 x, y 축 값
double compAngleX, compAngleY; //상보필터
double kalAngleX, kalAngleY; // 칼만필터 적용 x, y

uint32_t timer;
uint8_t i2cData[14]; //i2c통신 버퍼 (14바이트)
////////////////////////////

void setup() {
  Serial.begin(230400);
  mySerial.begin(115200);
  bufferPosition = 0;

  Wire.begin();
  i2cData[0] = 7; // 샘플링속도 1000Hz - 8Hz(7+1) = 1000Hz 설정
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // 4개의 레지스터에 동시에 기록
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register

    while (1);

  }

  delay(100); // 센서 안정화를 위한 딜레이

  /* Set kalman and gyro starting angle */
  /* 칼만과 자이로의 시작 각도 설정 */
  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);

  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  //-파이 ~ +파이  =>  0 ~ 2파이값으로 변환
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;

  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;

  timer = micros();


}

void loop() {

  byte a[4]; //실시간 전달 (텍스트1, 카운트2, 평균3)
  a[0] = 0x0F;

  rec = mySerial.read();

  if (rec == 5) {
    count = 0;
    count2 = 0;
    avr = 0;
    z = 0;

    int i = 1;

    while (i == 1) {
      rec = mySerial.read();

      delay(10);

      val = analogRead(A0);

      angleValue();

      Serial.print(kalAngleY);
      Serial.print("  ");
      Serial.println(val);


      //카운트
      x = kalAngleY;
      val = analogRead(A0);
      if (x - y < -1.5 && result == 0) {  //현재 값 - 이전 값 = 음수  -> 내려감
        val = analogRead(A0);

        if (x > 270 || x < 0) {
          result = 1;
        }

        if ( x >= 210 && x <= 270) {
          if (val > 600) {
            Serial.println("4단계 정상운동, 힘o");
            count += 1;
            z += 100;
          }
          else {
            Serial.println("4단계 정상운동, 힘x");
            result = 1;
            z += 50;
          }

          count2 += 1;
          result = 1;
        }

        else if ( x < 210 && x >= 170) {
          if (val > 600) {
            Serial.println("3단계 운동, 힘o");
            z += 80;
          }
          else {
            Serial.println("3단계 운동, 힘x");
            z += 40;
          }

          result = 1;
          count2 += 1;
        }

        else if ( x < 170 && x >= 130) {
          if (val > 600) {
            Serial.println("2단계 운동, 힘o");
            z += 70;
          }
          else {
            Serial.println("2단계 운동, 힘x");
            z += 30;
          }

          result = 1;
          count2 += 1;
        }

        else if ( x < 130 && x >= 90) {
          if (val > 600) {
            Serial.println("1단계 운동, 힘o");
            z += 60;
          }
          else {
            Serial.println("힘과 움직임 x");
            z += 0;
          }
          result = 1;
          count2 += 1;

        }
      }
      if (x - y > 3) {  //올라감
        result = 0;
      }


      y = x;
      avr = int(z / count2);
      if (avr == 255)
        avr = 0;


      a[1] = count2; //총 횟수
      a[2] = count;//유효카운트
      a[3] = avr;//평균

      //실시간 : 텍스트, 총 카운트수 | 최종 : 평균

      if (rec == 2) { //종료 2를 입력받았을 때
        Serial.println("종료");
        i = 2;
      }

      Serial.println(a[1]); //총횟수
      Serial.println(a[2]); //유효 카운트
      Serial.println(a[3]); //평균
      Serial.println(count2);
      mySerial.write(a, 4);
      val = analogRead(A0);
      delay(10);
    }//while

  }//if mySerial == 5
} //loop

void angleValue() {
  /* Update all the values */
  /* 측정한 데이터를 받아옴 */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);

  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  //atan(높이,밑변) 으로 -파이~+파이의 라디안 값 반환 후 각도로 바꾸는 계산 (위 링크)
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

  double gyroXrate = (double)gyroX / 131.0; //자이로 x축 각도 변화량
  double gyroYrate = -((double)gyroY / 131.0);  //y축
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // 필터보정이 없는 자이로 x각도
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000); // 필터보정이 없는 자이로 y각도

  //상보필터를 이용한 각도 계산
  //값의 계수 (0.93)는 실험으로 얻을 수 있는 값 -> 필요한 만큼 변화 가능
  compAngleX = (0.97 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.03 * accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.97 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.03 * accYangle);

  //칼만 필터를 이용한 각도 계산(칼만은 kalman.h 라이브러리를 통해 계산함)
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);



  timer = micros();

  temp = ((double)tempRaw + 12412.0) / 340.0;

}
