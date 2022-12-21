#include "IRremote.h"
#include "Oscillator.h"
#include <Servo.h>

#define BTN_UP 16736925
#define BTN_DOWN 16754775
#define BTN_LEFT 16761405
#define BTN_RIGHT 16720605
#define BTN_RESET 16712445
#define BTN_DANCE 16718055
#define BTN_MUSIC 16724175
#define BTN_MODE 16743045
#define BTN_ADD 16716015
#define BTN_SUB 16726215
#define BTN_IDLE 16734885
#define BTN_KEEP 4294967295
/*
         ---------------
        |     O   O     |
        |---------------|
YR 3==> |               | <== YL 2
         ---------------
            ||     ||
            ||     ||
RR 5==>   -----   ------  <== RL 4
         |-----   ------|
*/
#define TRIM_YL 4
#define TRIM_YR -11
#define TRIM_RL -2
#define TRIM_RR 2

#define YL_PIN 10 // 3
#define YR_PIN 9  // 2
#define RL_PIN 12 // 1
#define RR_PIN 6  // 0
#define RECV_PIN 3
#define ECHO_PIN 4
#define TRIG_PIN 5
#define ST188_R_PIN A1
#define ST188_L_PIN A0
#define MY1690_PIN 8
#define HT6871_PIN 7
#define N_SERVOS 4
#define INTERVALTIME 10.0
#define POS 90
#define AMPL 30.0
#define AMPL1 20.0
#define TIME 20

int distance;
int st188Val_L;
int st188Val_R;
bool adjustFlag = true;
int ST188Threshold = 40;
int ST188RightDataMin = 200;
int ST188LeftDataMin = 200;
int UltraThresholdMin = 7;
int UltraThresholdMax = 20;
float rate = AMPL1 / AMPL;
Oscillator servo[N_SERVOS];
IRrecv irrecv(RECV_PIN);
decode_results results;
enum MODE {
  IDLE,
  IRREMOTE,
  OBSTACLE,
  FOLLOW,
  MUSIC,
  DANCE,
} mode = IDLE;
enum IRMODE {
  FORWARD,
  BACKWAED,
  TURNRIGHT,
  TURNLIFT,
  STOP,
} IRmode = STOP;
int musicIndex = 2;
int danceIndex = 2;
bool danceFlag = false;
unsigned long preIrValue;
unsigned long preIrMillis;
unsigned long preMp3Millis;
int t = 495;
double pause = 0;
class MY1690_16S {
public:
  int volume = 20; //设置音量大小
                   // STOP PLAYING PAUSE FF FR
  String playStatus[5] = {"0", "1", "2", "3", "4"};
  void playSong(unsigned char num, unsigned char vol) {
    setVolume(vol);         //音量大小
    setPlayMode(4);         //播放模式:不循环
    CMD_SongSelet[4] = num; //选择播放的歌曲
    checkCode(CMD_SongSelet);
    Serial.write(CMD_SongSelet, 7);
    delay(10);
  };
  String getPlayStatus() {
    Serial.write(CMD_getPlayStatus, 5);
    delay(10);
    return getStatus();
  }
  String getStatus() {
    String statusMp3 = "";
    while (Serial.available()) {
      statusMp3 += (char)Serial.read();
    }
    return statusMp3;
  };
  void stopPlay() {
    setPlayMode(4);
    Serial.write(CMD_MusicStop, 5);
    delay(10);
  };
  void setVolume(unsigned char vol) {
    CMD_VolumeSet[3] = vol;
    checkCode(CMD_VolumeSet);
    Serial.write(CMD_VolumeSet, 6);
    delay(10);
  };
  void setPlayMode(unsigned char mode) {
    CMD_PlayMode[3] = mode;
    checkCode(CMD_PlayMode);
    Serial.write(CMD_PlayMode, 6);
    delay(10);
  };
  void checkCode(unsigned char *vs) {
    int val = vs[1];
    int i;
    for (i = 2; i < vs[1]; i++) {
      val = val ^ vs[i];
    }
    vs[i] = val;
  };
  void ampMode(int p, bool m) {
    pinMode(p, OUTPUT);
    if (m) {
      digitalWrite(p, HIGH);
    } else {
      digitalWrite(p, LOW);
    }
  };
  void init() {
    ampMode(HT6871_PIN, HIGH);
    stopPlay();
  }

private:
  byte CMD_MusicPlay[5] = {0x7E, 0x03, 0x11, 0x12, 0xEF};
  byte CMD_MusicStop[5] = {0x7E, 0x03, 0x1E, 0x1D, 0xEF};
  byte CMD_MusicNext[5] = {0x7E, 0x03, 0x13, 0x10, 0xEF};
  byte CMD_MusicPrev[5] = {0x7E, 0x03, 0x14, 0x17, 0xEF};
  byte CMD_VolumePlus[5] = {0x7E, 0x03, 0x15, 0x16, 0xEF};
  byte CMD_VolumeDown[5] = {0x7E, 0x03, 0x16, 0x15, 0xEF};
  byte CMD_VolumeSet[6] = {0x7E, 0x04, 0x31, 0x00, 0x00, 0xEF};
  byte CMD_PlayMode[6] = {0x7E, 0x04, 0x33, 0x00, 0x00, 0xEF};
  byte CMD_SongSelet[7] = {0x7E, 0x05, 0x41, 0x00, 0x00, 0x00, 0xEF};
  byte CMD_getPlayStatus[5] = {0x7E, 0x03, 0x20, 0x23, 0xEF};
} MP3;
void oscillate(int A[N_SERVOS], int O[N_SERVOS], int T,
               double phase_diff[N_SERVOS]) {
  for (int i = 0; i < 4; i++) {
    servo[i].SetO(O[i]);
    servo[i].SetA(A[i]);
    servo[i].SetT(T);
    servo[i].SetPh(phase_diff[i]);
  }
  double ref = millis();
  for (double x = ref; x < T + ref; x = millis()) {
    for (int i = 0; i < 4; i++) {
      servo[i].refresh();
    }
  }
}
unsigned long final_time;
unsigned long interval_time;
int oneTime;
int iteration;
float increment[N_SERVOS];
int oldPosition[] = {POS, POS, POS, POS};
void home() {
  int move1[] = {90, 90, 90, 90};
  moveNServos(500, move1);
}
void moveNServos(int time, int newPosition[]) {
  for (int i = 0; i < N_SERVOS; i++)
    increment[i] = ((newPosition[i]) - oldPosition[i]) / (time / INTERVALTIME);

  final_time = millis() + time;

  iteration = 1;
  while (millis() < final_time) { // Javi del futuro cambia esto
    interval_time = millis() + INTERVALTIME;

    oneTime = 0;
    while (millis() < interval_time) {
      if (oneTime < 1) {
        for (int i = 0; i < N_SERVOS; i++) {
          servo[i].SetPosition(oldPosition[i] + (iteration * increment[i]));
        }
        iteration++;
        oneTime++;
      }
    }
  }

  for (int i = 0; i < N_SERVOS; i++) {
    oldPosition[i] = newPosition[i];
  }
}
void walk(int steps, int T, int dir) {
  int A[4] = {30, 30, 30, 30};
  int O[4] = {0, 0, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(dir * 90),
                          DEG2RAD(dir * 90)};
  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void turn(int steps, int T, int dir) {
  int A[4] = {30, 30, 0, 0};
  if (dir == 1) {
    A[2] = 30;
    A[3] = 10;
  } else {
    A[2] = 10;
    A[3] = 30;
  }
  int O[4] = {0, 0, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};
  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void moonWalkRight(int steps, int T) {
  int A[4] = {25, 25, 0, 0};
  int O[4] = {-15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90),
                          DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void moonWalkLeft(int steps, int T) {
  int A[4] = {25, 25, 0, 0};
  int O[4] = {-15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 - 120), DEG2RAD(90),
                          DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void crusaito(int steps, int T) {
  int A[4] = {25, 25, 30, 30};
  int O[4] = {-15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180 + 120), DEG2RAD(90),
                          DEG2RAD(90)};
  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void swing(int steps, int T) {
  int A[4] = {25, 25, 0, 0};
  int O[4] = {-15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void upDown(int steps, int T) {
  int A[4] = {25, 25, 0, 0};
  int O[4] = {-15, 15, 0, 0};
  double phase_diff[4] = {DEG2RAD(180), DEG2RAD(0), DEG2RAD(270), DEG2RAD(270)};
  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
  home();
}
void flapping(int steps, int T) {
  int A[4] = {15, 15, 8, 8};
  int O[4] = {-A[0], A[1], 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(180), DEG2RAD(-90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void run(int steps, int T) {
  int A[4] = {10, 10, 10, 10};
  int O[4] = {0, 0, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(90), DEG2RAD(90)};

  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void backyard(int steps, int T) {
  int A[4] = {15, 15, 30, 30};
  int O[4] = {0, 0, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90)};

  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void backyardSlow(int steps, int T) {
  int A[4] = {15, 15, 30, 30};
  int O[4] = {0, 0, 0, 0};
  double phase_diff[4] = {DEG2RAD(0), DEG2RAD(0), DEG2RAD(-90), DEG2RAD(-90)};

  for (int i = 0; i < steps; i++)
    oscillate(A, O, T, phase_diff);
}
void goingUp(int tempo) {
  int p = 55;
  for (int i = 0; i <= p; i++) {
    servo[0].SetPosition(POS - i);
    servo[1].SetPosition(POS + i);
    delay(tempo);
  }
  delay(500);
  for (int i = 0; i <= p; i++) {
    servo[0].SetPosition(POS - p + i);
    servo[1].SetPosition(POS + p - i);
    delay(tempo);
  }
}
void drunk(int tempo) {
  int move1[] = {60, 70, 90, 90};
  int move2[] = {110, 120, 90, 90};
  int move3[] = {60, 70, 90, 90};
  int move4[] = {110, 120, 90, 90};
  int move5[] = {90, 90, 90, 90};
  moveNServos(tempo * 0.5, move1);
  moveNServos(tempo * 0.5, move2);
  moveNServos(tempo * 0.5, move3);
  moveNServos(tempo * 0.5, move4);
  moveNServos(tempo * 0.5, move5);
}
//舞蹈1_13
void noGravity(int tempo) {
  int move1[] = {120, 140, 90, 90};
  int move2[] = {120, 30, 90, 90};
  int move3[] = {120, 120, 90, 90};
  int move4[] = {120, 30, 120, 120};
  int move5[] = {120, 30, 60, 60};
  int move6[] = {90, 90, 90, 90};
  moveNServos(tempo * 0.235, move1);
  delay(tempo * 2);
  moveNServos(tempo * 0.5, move2);
  moveNServos(tempo * 0.5, move3);
  moveNServos(tempo * 0.5, move2);
  delay(tempo);
  moveNServos(tempo * 0.5, move4);
  delay(tempo / 10);
  moveNServos(tempo * 0.5, move5);
  delay(tempo / 10);
  moveNServos(tempo * 0.5, move4);
  delay(tempo / 10);
  moveNServos(tempo, move6);
}
void kickLeft(int tempo) {
  servo[1].SetPosition(POS + AMPL + AMPL1);
  servo[0].SetPosition(POS + AMPL);
  delay(tempo);
  servo[1].SetPosition(POS);
  servo[0].SetPosition(POS + AMPL);
  delay(tempo / 4);
  servo[1].SetPosition(POS + AMPL);
  servo[0].SetPosition(POS + AMPL);
  delay(tempo / 4);
  servo[1].SetPosition(POS);
  servo[0].SetPosition(POS + AMPL);
  delay(tempo / 4);
  servo[1].SetPosition(POS + AMPL);
  servo[0].SetPosition(POS + AMPL);
  delay(tempo / 4);
  for (int i = 0; i <= AMPL; i++) {
    servo[1].SetPosition(POS + AMPL - i);
    servo[0].SetPosition(POS + AMPL - i);
    delay(10);
  }
  delay(tempo);
}
void kickRight(int tempo) {
  servo[1].SetPosition(POS - AMPL);
  servo[0].SetPosition(POS - AMPL - AMPL1);
  delay(tempo);
  servo[1].SetPosition(POS - AMPL);
  servo[0].SetPosition(POS);
  delay(tempo / 4);
  servo[1].SetPosition(POS - AMPL);
  servo[0].SetPosition(POS - AMPL);
  delay(tempo / 4);
  servo[1].SetPosition(POS - AMPL);
  servo[0].SetPosition(POS);
  delay(tempo / 4);
  servo[1].SetPosition(POS - AMPL);
  servo[0].SetPosition(POS - AMPL);
  delay(tempo / 4);
  for (int i = 0; i <= AMPL; i++) {
    servo[0].SetPosition(POS - AMPL + i);
    servo[1].SetPosition(POS - AMPL + i);
    delay(10);
  }
  delay(tempo);
}
void legRaise(int tempo, int dir) {
  if (dir) {
    int move1[] = {60, 60, 60, 60};
    int move2[] = {90, 90, 90, 90};
    moveNServos(tempo * 0.5, move1);
    delay(tempo);
    moveNServos(tempo * 0.5, move2);
    delay(tempo);
  } else {
    int move1[] = {120, 120, 120, 120};
    int move2[] = {90, 90, 90, 90};
    moveNServos(tempo * 0.5, move1);
    delay(tempo);
    moveNServos(tempo * 0.5, move2);
    delay(tempo);
  }
}
//舞蹈1_12
void legRaise1(int tempo, int dir) {
  if (dir) {
    int move1[] = {50, 60, 90, 90};
    int move2[] = {60, 60, 120, 90};
    int move3[] = {60, 60, 60, 90};
    int move4[] = {90, 90, 90, 90};
    moveNServos(tempo * 0.235, move1);
    delay(tempo);
    moveNServos(tempo * 0.235, move2);
    delay(tempo / 4);
    moveNServos(tempo * 0.235, move3);
    delay(tempo / 4);
    moveNServos(tempo * 0.235, move2);
    delay(tempo / 4);
    moveNServos(tempo * 0.235, move3);
    delay(tempo / 4);
    moveNServos(tempo * 0.5, move4);
    delay(tempo);
  } else {
    int move1[] = {120, 130, 90, 90};
    int move2[] = {120, 120, 90, 60};
    int move3[] = {120, 120, 90, 120};
    int move4[] = {90, 90, 90, 90};
    moveNServos(tempo * 0.235, move1);
    delay(tempo);
    moveNServos(tempo * 0.235, move2);
    delay(tempo / 4);
    moveNServos(tempo * 0.235, move3);
    delay(tempo / 4);
    moveNServos(tempo * 0.235, move2);
    delay(tempo / 4);
    moveNServos(tempo * 0.235, move3);
    delay(tempo / 4);
    moveNServos(tempo * 0.5, move4);
    delay(tempo);
  }
}
//舞蹈1_11
void legRaise2(int steps, int tempo, int dir) {
  if (dir) {
    int move1[] = {20, 100, 90, 90};
    int move2[] = {20, 80, 120, 90};
    for (int i = 0; i < steps; i++) {
      moveNServos(tempo * 0.5, move1);
      delay(tempo / 4);
      moveNServos(tempo * 0.5, move2);
      delay(tempo / 4);
    }
  } else {
    int move1[] = {80, 160, 90, 90};
    int move2[] = {110, 160, 90, 60};
    for (int i = 0; i < steps; i++) {
      moveNServos(tempo * 0.5, move1);
      delay(tempo / 4);
      moveNServos(tempo * 0.5, move2);
      delay(tempo / 4);
    }
  }
  home();
}
//舞蹈1_10
void legRaise3(int steps, int tempo, int dir) {
  if (dir) {
    int move1[] = {0, 60, 90, 90};
    int move2[] = {0, 100, 90, 90};
    for (int i = 0; i < steps; i++) {
      moveNServos(tempo * 0.5, move1);
      delay(tempo / 4);
      moveNServos(tempo * 0.5, move2);
      delay(tempo / 4);
    }
  } else {
    int move1[] = {120, 180, 90, 90};
    int move2[] = {80, 180, 90, 90};
    for (int i = 0; i < steps; i++) {
      moveNServos(tempo * 0.5, move1);
      delay(tempo / 4);
      moveNServos(tempo * 0.5, move2);
      delay(tempo / 4);
    }
  }
  home();
}
//舞蹈1_10
void legRaise4(int tempo, int dir) {
  if (dir) {
    int move1[] = {0, 60, 90, 90};
    int move2[] = {0, 100, 90, 90};

    moveNServos(tempo * 0.5, move1);
    delay(tempo / 4);
    moveNServos(tempo * 0.5, move2);
    delay(tempo / 4);

  } else {
    int move1[] = {120, 180, 90, 90};
    int move2[] = {80, 180, 90, 90};
    moveNServos(tempo * 0.5, move1);
    delay(tempo / 4);
    moveNServos(tempo * 0.5, move2);
    delay(tempo / 4);
  }
  home();
}
void sitdown() {
  int move1[] = {180, 0, 90, 90};
  moveNServos(500, move1);
  delay(1000);
  home();
  delay(100);
}
void lateral_fuerte(boolean dir, int tempo) {
  if (dir) {
    int move1[] = {40, 60, 90, 90};
    int move2[] = {120, 60, 90, 90};
    int move3[] = {40, 60, 90, 90};
    moveNServos(tempo * 0.1, move1);
    delay(tempo);
    moveNServos(tempo * 0.1, move2);
    delay(tempo / 2);
    moveNServos(tempo * 0.1, move3);
    delay(tempo / 2);
  } else {
    int move1[] = {120, 140, 90, 90};
    int move2[] = {120, 60, 90, 90};
    int move3[] = {120, 140, 90, 90};
    moveNServos(tempo * 0.1, move1);
    delay(tempo);
    moveNServos(tempo * 0.1, move2);
    delay(tempo / 2);
    moveNServos(tempo * 0.1, move3);
    delay(tempo / 2);
  }
  home();
}
void primera_parte() {
  int move1[4] = {60, 120, 90, 90};
  int move2[4] = {90, 90, 90, 90};
  int move3[4] = {40, 140, 90, 90};
  lateral_fuerte(1, t);
  moveNServos(t * 0.5, move1);
  moveNServos(t * 0.5, move2);
  lateral_fuerte(0, t);
  moveNServos(t * 0.5, move1);
  moveNServos(t * 0.5, move2);
  lateral_fuerte(1, t);
  moveNServos(t * 0.5, move1);
  moveNServos(t * 0.5, move2);
  lateral_fuerte(0, t);
  crusaito(1, t * 1.4);
  moveNServos(t * 1, move3);
  home();
}
void segunda_parte() {

  int move1[4] = {90, 90, 80, 100};
  int move2[4] = {90, 90, 100, 80};
  int move3[4] = {90, 90, 80, 100};
  int move4[4] = {90, 90, 100, 80};

  int move5[4] = {40, 140, 80, 100};
  int move6[4] = {40, 140, 100, 80};
  int move7[4] = {90, 90, 80, 100};
  int move8[4] = {90, 90, 100, 80};

  int move9[4] = {40, 140, 80, 100};
  int move10[4] = {40, 140, 100, 80};
  int move11[4] = {90, 90, 80, 100};
  int move12[4] = {90, 90, 100, 80};

  for (int x = 0; x < 3; x++) {
    for (int i = 0; i < 3; i++) {
      pause = millis();
      moveNServos(t * 0.15, move1);
      moveNServos(t * 0.15, move2);
      moveNServos(t * 0.15, move3);
      moveNServos(t * 0.15, move4);
      while (millis() < (pause + t))
        ;
    }
    pause = millis();
    moveNServos(t * 0.15, move5);
    moveNServos(t * 0.15, move6);
    moveNServos(t * 0.15, move7);
    moveNServos(t * 0.15, move8);
    while (millis() < (pause + t))
      ;
  }

  for (int i = 0; i < 3; i++) {
    pause = millis();
    moveNServos(t * 0.15, move9);
    moveNServos(t * 0.15, move10);
    moveNServos(t * 0.15, move11);
    moveNServos(t * 0.15, move12);
    while (millis() < (pause + t))
      ;
  }
  home();
}
void dance() {
  primera_parte();
  segunda_parte();
  moonWalkLeft(4, t * 2);
  moonWalkRight(4, t * 2);
  moonWalkLeft(4, t * 2);
  moonWalkRight(4, t * 2);
  primera_parte();
  crusaito(1, t * 8);
  crusaito(1, t * 7);

  for (int i = 0; i < 16; i++) {
    flapping(1, t / 4);
    delay(3 * t / 4);
  }

  moonWalkRight(4, t * 2);
  moonWalkLeft(4, t * 2);
  moonWalkRight(4, t * 2);
  moonWalkLeft(4, t * 2);

  drunk(t * 4);
  drunk(t * 4);
  drunk(t * 4);
  drunk(t * 4);
  kickLeft(t);
  kickRight(t);
  drunk(t * 8);
  drunk(t * 4);
  drunk(t / 2);
  delay(t * 4);

  drunk(t / 2);

  delay(t * 4);
  walk(2, t * 2, 1);
  backyard(2, t * 2);
  goingUp(t * 2);
  goingUp(t * 1);
  noGravity(t * 2);
  crusaito(1, t * 2);
  crusaito(1, t * 8);
  crusaito(1, t * 2);
  crusaito(1, t * 8);
  crusaito(1, t * 2);
  crusaito(1, t * 3);

  delay(t);
  primera_parte();
  for (int i = 0; i < 32; i++) {
    flapping(1, t / 2);
    delay(t / 2);
  }

  for (int i = 0; i < 4; i++)
    servo[i].SetPosition(90);
}
void dance2() {
  primera_parte();
  segunda_parte();
  moonWalkLeft(2, t / 2);
  moonWalkRight(2, t / 2);
  crusaito(1, t / 2);
  crusaito(1, t * 8);
  for (int i = 0; i < 3; i++) {
    flapping(1, t);
    delay(3 * t / 4);
  }
  drunk(t / 4);
  drunk(t / 2);
  drunk(t);
  drunk(t * 2);
  drunk(t * 4);
  kickLeft(t);
  kickRight(t);
  walk(2, t * 2, 1);
  backyard(2, t * 2);
  goingUp(5);
  noGravity(t);
  crusaito(1, t * 2);
  crusaito(1, t * 8);
  crusaito(1, t * 3);
  lateral_fuerte(1, 500);
  lateral_fuerte(0, 500);
  segunda_parte();
  upDown(5, 500);
}
void dance3() {
  // unsigned long per = millis();
  sitdown();
  legRaise(t, 1);
  swing(10, t);
  legRaise1(t, 1);
  walk(2, t * 2, 1);
  noGravity(t);
  legRaise2(4, t, 0);
  kickRight(t);
  goingUp(5);
  legRaise3(4, t, 1);
  kickLeft(t);
  legRaise4(t, 1);
  crusaito(1, t * 2);
  swing(10, t);
  backyard(2, t * 2);
  goingUp(5);
  noGravity(t);
  drunk(t);
  lateral_fuerte(1, 500);
  lateral_fuerte(0, 500);
  upDown(2, t / 2);
  goingUp(5);
  crusaito(1, t * 8);
  noGravity(t);
  sitdown();
  segunda_parte();
}
void dance4() {
  flapping(1, t);
  moonWalkLeft(2, t);
  moonWalkRight(2, t);
  drunk(t);
  kickLeft(t);
  walk(2, t * 2, 1);
  crusaito(1, t * 8);
  lateral_fuerte(0, 500);
  sitdown();
  legRaise(t, 1);
  swing(5, t);
  backyard(2, t * 2);
  goingUp(5);
  noGravity(t);
  primera_parte();
  upDown(5, t / 2);
  crusaito(1, t * 8);
  legRaise1(t, 1);
  legRaise2(4, t, 0);
  kickRight(t);
  goingUp(5);
  legRaise3(4, t, 1);
  kickLeft(t);
  legRaise4(t, 1);
  segunda_parte();
  sitdown();
}
void start() {
  MP3.stopPlay();
  MP3.playSong(1, MP3.volume);
  startDance();
  MP3.stopPlay();
}
void startDance() {
  drunk(t);
  lateral_fuerte(1, t);
  lateral_fuerte(0, t);
  goingUp(1);
}
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return (int)pulseIn(ECHO_PIN, HIGH) / 58;
}
void motorTest() {
  while (1) {
    servo[0].SetPosition(45);
    delay(1000);
    for (int i = 45; i <= 135; i++) {
      servo[0].SetPosition(i);
      delay(10);
    }
  }
}
void obstacleMode() {
  distance = getDistance();
  if (distance > 20) {
    walk(1, t * 2, 1);
  } else {
    turn(1, t * 2, 1);
  }
}
void followMode() {
  distance = getDistance();
  st188Val_L = analogRead(ST188_L_PIN);
  st188Val_R = analogRead(ST188_R_PIN);
  Serial.print(st188Val_L);
  Serial.print("\t");
  Serial.print(distance);
  Serial.print("\t");
  Serial.println(st188Val_R);
  // delay(1000);
  if (distance > UltraThresholdMax) {
    home();
  } else if (distance < UltraThresholdMin) {
    int st188DVal = st188Val_L - st188Val_R;
    if (st188DVal > 0) {
      if (st188Val_L > (st188Val_R + ST188Threshold)) {
        turn(1, t * 2, 1);
      } else {
        turn(1, t * 2, -1);
      }
    } else {
      if (st188Val_R > (st188Val_L + ST188Threshold)) {
        turn(1, t * 2, -1);
      } else {
        turn(1, t * 2, 1);
      }
    }
  } else {
    walk(1, t * 2, 1);
  }
}
void st188Adjust() {
  if (adjustFlag == true && getDistance() > 50) {
    delay(100);
    if (getDistance() > 50) {
      long int st188RightData = 0;
      long int st188LeftData = 0;
      for (int n = 0; n < 10; n++) {
        st188LeftData += analogRead(ST188_L_PIN);
        st188RightData += analogRead(ST188_R_PIN);
      }
      ST188LeftDataMin = st188LeftData / 10;
      ST188RightDataMin = st188RightData / 10;
      adjustFlag = false;
      Serial.print("ST188LeftDataMin: ");
      Serial.print(ST188LeftDataMin);
      Serial.print("\tST188RightDataMin: ");
      Serial.println(ST188RightDataMin);
    }
  }
}
void setup() {
  Serial.begin(9600);
  pinMode(ST188_L_PIN, INPUT);
  pinMode(ST188_R_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  MP3.init();
  irrecv.enableIRIn();
  servo[0].attach(RR_PIN);
  servo[1].attach(RL_PIN);
  servo[2].attach(YR_PIN);
  servo[3].attach(YL_PIN);
  servo[0].SetTrim(TRIM_RR);
  servo[1].SetTrim(TRIM_RL);
  servo[2].SetTrim(TRIM_YR);
  servo[3].SetTrim(TRIM_YL);
  home();
  delay(2000);
  // motorTest(); //测试舵机
  start();
}
void loop() {
  st188Adjust();
  if (irrecv.decode(&results)) {
    unsigned long irValue = results.value;
    Serial.println(irValue);
    irrecv.resume();
    switch (irValue) {
    case BTN_UP:
      MP3.stopPlay();
      mode = IRREMOTE;
      IRmode = FORWARD;
      break;
    case BTN_DOWN:
      MP3.stopPlay();
      mode = IRREMOTE;
      IRmode = BACKWAED;
      break;
    case BTN_LEFT:
      MP3.stopPlay();
      mode = IRREMOTE;
      IRmode = TURNLIFT;
      break;
    case BTN_RIGHT:
      MP3.stopPlay();
      mode = IRREMOTE;
      IRmode = TURNRIGHT;
      break;
    case BTN_MODE:
      MP3.stopPlay();
      if (mode == FOLLOW) {
        mode = OBSTACLE;
      } else if (mode == OBSTACLE) {
        mode = FOLLOW;
      } else {
        mode = FOLLOW;
      }
      break;
    case BTN_IDLE:
      MP3.stopPlay();
      mode = IDLE;
      home();
      break;
    case BTN_MUSIC:
      MP3.stopPlay();
      mode = MUSIC;
      musicIndex = 2;
      MP3.playSong(musicIndex, MP3.volume);
      preMp3Millis = millis();
      break;
    case BTN_DANCE:
      MP3.stopPlay();
      mode = DANCE;
      danceFlag = true;
      break;
    case BTN_ADD:
      MP3.stopPlay();
      if (mode == MUSIC) {
        musicIndex++;
        if (musicIndex > 4) {
          musicIndex = 2;
        }
        MP3.playSong(musicIndex, MP3.volume);
      }
      if (mode == DANCE) {
        danceFlag = true;
        danceIndex++;
        if (danceIndex > 4) {
          danceIndex = 2;
        }
      }
      break;
    case BTN_SUB:
      MP3.stopPlay();
      if (mode == MUSIC) {
        musicIndex--;
        if (musicIndex < 2) {
          musicIndex = 4;
        }
        MP3.playSong(musicIndex, MP3.volume);
      }
      if (mode == DANCE) {
        danceFlag = true;
        danceIndex--;
        if (danceIndex < 2) {
          danceIndex = 4;
        }
      }
      break;
    default:
      break;
    }
  }
  switch (mode) {
  case IDLE:
    break;
  case IRREMOTE:
    switch (IRmode) {
    case FORWARD:
      walk(1, t * 2, 1);
      break;
    case BACKWAED:
      walk(1, t * 2, -1);
      break;
    case TURNRIGHT:
      turn(1, t * 2, 1);
      break;
    case TURNLIFT:
      turn(1, t * 2, -1);
      break;
    default:
      break;
    }
    break;
  case OBSTACLE:
    obstacleMode();
    break;
  case FOLLOW:
    followMode();
    break;
  case MUSIC:
    if (millis() - preMp3Millis > 1000) {
      preMp3Millis = millis();
      if (MP3.getPlayStatus() == MP3.playStatus[0]) {
        musicIndex++;
        if (musicIndex > 4) {
          musicIndex = 2;
        }
        MP3.playSong(musicIndex, MP3.volume);
      }
    }
    break;
  case DANCE:
    if (danceFlag == true) {
      MP3.stopPlay();
      MP3.playSong(danceIndex, MP3.volume);
      switch (danceIndex) {
      case 2:
        dance2();
        break;
      case 3:
        dance3();
        break;
      case 4:
        dance4();
        break;
      default:
        break;
      }
      danceFlag = false;
      MP3.stopPlay();
    }
    break;
  default:
    break;
  }
}
