#include <Servo.h>
#include <SoftwareSerial.h>

// ===== 引脚（主控 A）=====
#define PIN_BASE 9
#define PIN_L    8
#define PIN_R    7
#define PIN_CLAW 6
#define JOY_BASE A0
#define JOY_L    A1
#define JOY_R    A2
#define JOY_CLAW A3

// 软串口：RX=D2 接按键板D3(TX)，TX=D3 可接按键板D2（回传）
SoftwareSerial keypad(2, 3);

Servo srvBase, srvL, srvR, srvClaw;

// ===== 运动学参数（按实物标定）=====
const float L1 = 80.0, L2 = 80.0, L3 = 75.0, BASE_H = 55.0;
const int OFF_BASE = 90, OFF_SHOULDER = 90, OFF_ELBOW = 90;

// ===== 当前/目标状态 =====
int curB = 90, curL = 90, curR = 90, curG = 90;
int tgtB = 90, tgtL = 90, tgtR = 90, tgtG = 90;
int stepDelay = 15;

// ===== 任务三：录制/播放 =====
const int MAX_REC = 60;
int recB[MAX_REC], recL[MAX_REC], recR[MAX_REC], recG[MAX_REC];
int recCount = 0;
int playIdx = 0;
bool playing = false;
unsigned long playTimer = 0;

// ===== 任务二：夹取状态机 =====
const int poses[3][2][3] = {
  {{90, 60, 30}, {90, 120, 150}},
  {{90, 50, 40}, {90, 110, 140}},
  {{90, 70, 50}, {90, 100, 130}},
};
int pickObj = -1, pickPhase = 0, grabTarget = 0;
unsigned long pickTimer = 0;

// ===== 任务五：绘图状态机 =====
float PEN_Z = 110.0;
const float PEN_ATT_DEG = -90.0;
const int DRAW_SEG = 40, SMOOTH_SEG = 30;
int drawState = 0, drawPaused = 0;   // 0空闲 1直线 2图形 3折线 4样条
float dX0, dY0, dX1, dY1;
int dSeg = 0, dSegIdx = 0, shapeN = 0;
float shapeX[8], shapeY[8];
const int MAX_TCH = 5;
float tchX[MAX_TCH], tchY[MAX_TCH];
int tchCount = 0;

void setup() {
  Serial.begin(9600);      // 电脑（串口监视器）
  keypad.begin(9600);      // 按键板
  srvBase.attach(PIN_BASE); srvL.attach(PIN_L);
  srvR.attach(PIN_R);  srvClaw.attach(PIN_CLAW);
  home();
  Serial.println("READY");
}

void loop() {
  while (Serial.available()) {           // 电脑指令
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) handle(cmd);
  }
  while (keypad.available()) {           // 按键板指令
    String cmd = keypad.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial.print("KEY> "); Serial.println(cmd);   // 回显到电脑，方便确认
      handle(cmd);
    }
  }
  joyControl();
  stepMove();
  updatePickMachine();
  updatePlay();
  updateDraw();
}

void setTarget(int b, int l, int r, int g) {
  tgtB = constrain(b, 0, 180); tgtL = constrain(l, 0, 180);
  tgtR = constrain(r, 0, 180); tgtG = constrain(g, 0, 180);
}
void home() { setTarget(90, 90, 90, 90); }

void stepMove() {
  if (curB < tgtB) curB++; else if (curB > tgtB) curB--;
  if (curL < tgtL) curL++; else if (curL > tgtL) curL--;
  if (curR < tgtR) curR++; else if (curR > tgtR) curR--;
  if (curG < tgtG) curG++; else if (curG > tgtG) curG--;
  srvBase.write(curB); srvL.write(curL);
  srvR.write(curR); srvClaw.write(curG);
  delay(stepDelay);
}

void handle(String cmd) {
  if (cmd.indexOf(' ') > 0) {            // 复合指令：b90 x45 y45 z45
    int b = curB, l = curL, r = curR, g = curG;
    while (cmd.length() > 0) {
      int sp = cmd.indexOf(' ');
      String seg = (sp > 0) ? cmd.substring(0, sp) : cmd;
      cmd = (sp > 0) ? cmd.substring(sp + 1) : "";
      seg.trim();
      if (seg.length() >= 2) {
        char ax = seg.charAt(0);
        int v = seg.substring(1).toInt();
        if (ax == 'b') b = v; else if (ax == 'x') l = v;
        else if (ax == 'y') r = v; else if (ax == 'z') g = v;
      }
    }
    setTarget(b, l, r, g);
    return;
  }

  if      (cmd == "0") setTarget(curB, curL, curR, 170);
  else if (cmd == "S") setTarget(curB, curL, curR, 80);
  else if (cmd == "H") stepDelay = max(5, stepDelay - 5);
  else if (cmd == "L") stepDelay = min(200, stepDelay + 5);
  else if (cmd == "M") { home(); if (drawState) drawState = 0; if (playing) playing = false; }
  else if (cmd == "T") printXYZ();
  else if (cmd.length() > 1 && cmd.startsWith("x")) setTarget(curB, cmd.substring(1).toInt(), curR, curG);
  else if (cmd.length() > 1 && cmd.startsWith("y")) setTarget(curB, curL, cmd.substring(1).toInt(), curG);
  else if (cmd.length() > 1 && cmd.startsWith("z")) setTarget(curB, curL, curR, cmd.substring(1).toInt());
  else if (cmd.length() > 1 && cmd.startsWith("b")) setTarget(cmd.substring(1).toInt(), curL, curR, curG);
  else if (cmd == "A") startPick(0);
  else if (cmd == "B") startPick(1);
  else if (cmd == "C") startPick(2);
  else if (cmd == "G") { startPick(grabTarget); grabTarget = (grabTarget + 1) % 3; }
  else if (cmd == "REC" || cmd == "R") record();
  else if (cmd == "PLY" || cmd == "P") startPlay();
  else if (cmd == "DL") startDrawLine(60, 0, 120, 0);
  else if (cmd == "N") startDrawShapeN();
  else if (cmd == "TRI") startDrawShapeTri();
  else if (cmd == "Z") startDrawShapeZ();
  else if (cmd == "V") startDrawShapeV();
  else if (cmd == "TCH") teachPoint();
  else if (cmd == "PLN") startDrawPolyline();
  else if (cmd == "SMT") startDrawSmooth();
  else if (cmd == "PAUSE") { drawPaused = 1; Serial.println("PAUSED"); }
  else if (cmd == "RESUME") { drawPaused = 0; Serial.println("RESUMED"); }
  else if (cmd == "CANCEL") { drawState = 0; drawPaused = 0; Serial.println("CANCELLED"); }
}

void joyControl() {
  if (analogRead(JOY_BASE) > 700) tgtB = constrain(tgtB + 1, 0, 180);
  else if (analogRead(JOY_BASE) < 300) tgtB = constrain(tgtB - 1, 0, 180);
  if (analogRead(JOY_L) > 700) tgtL = constrain(tgtL + 1, 0, 180);
  else if (analogRead(JOY_L) < 300) tgtL = constrain(tgtL - 1, 0, 180);
  if (analogRead(JOY_R) > 700) tgtR = constrain(tgtR + 1, 0, 180);
  else if (analogRead(JOY_R) < 300) tgtR = constrain(tgtR - 1, 0, 180);
  if (analogRead(JOY_CLAW) > 700) tgtG = constrain(tgtG + 1, 0, 180);
  else if (analogRead(JOY_CLAW) < 300) tgtG = constrain(tgtG - 1, 0, 180);
}

// ===== 任务二：夹取 =====
void startPick(int obj) {
  if (pickObj >= 0) return;
  pickObj = obj; pickPhase = 0; pickTimer = millis();
  setTarget(90, poses[obj][0][0], poses[obj][0][1], 170);
}
void updatePickMachine() {
  if (pickObj < 0) return;
  unsigned long now = millis();
  int obj = pickObj;
  if (curB != tgtB || curL != tgtL || curR != tgtR || curG != tgtG) { pickTimer = now; return; }
  if (now - pickTimer < 300) return;
  switch (pickPhase) {
    case 0: setTarget(curB, curL, curR, 80); pickPhase = 1; pickTimer = now; break;
    case 1: setTarget(90, poses[obj][0][0] + 15, poses[obj][0][1], 80); pickPhase = 2; pickTimer = now; break;
    case 2: setTarget(90, poses[obj][1][0], poses[obj][1][1], 80); pickPhase = 3; pickTimer = now; break;
    case 3: setTarget(curB, curL, curR, 170); pickPhase = 4; pickTimer = now; break;
    case 4: pickObj = -1; Serial.print("PICK "); Serial.println((char)('A' + obj)); break;
  }
}

// ===== 任务三：录制/播放 =====
void record() {
  if (recCount < MAX_REC) {
    recB[recCount] = curB; recL[recCount] = curL;
    recR[recCount] = curR; recG[recCount] = curG;
    recCount++;
    Serial.print("REC "); Serial.println(recCount);
  } else Serial.println("REC FULL");
}
void startPlay() {
  if (recCount == 0) { Serial.println("NO REC"); return; }
  playIdx = 0; playing = true; playTimer = millis();
  setTarget(recB[0], recL[0], recR[0], recG[0]);
}
void updatePlay() {
  if (!playing) return;
  unsigned long now = millis();
  if (curB != tgtB || curL != tgtL || curR != tgtR || curG != tgtG) { playTimer = now; return; }
  if (now - playTimer < 300) return;
  if (playIdx >= recCount - 1) { playing = false; Serial.println("PLAY END"); return; }
  playIdx++;
  setTarget(recB[playIdx], recL[playIdx], recR[playIdx], recG[playIdx]);
}

// ===== 任务四：运动学 =====
void fk(float th1, float th2, float th3, float &X, float &Y, float &Z) {
  float a1 = radians(th1), a2 = radians(th2), a3 = radians(th3);
  float r = L1 * cos(a2) + L2 * cos(a2 + a3);
  X = r * cos(a1); Y = r * sin(a1);
  Z = BASE_H + L1 * sin(a2) + L2 * sin(a2 + a3);
}
bool ik(float X, float Y, float Z, float attDeg, int &sb, int &sl, int &sr) {
  float th0 = atan2(Y, X);
  float r = sqrt(X * X + Y * Y);
  float att = radians(attDeg);
  float Wx = r - L3 * cos(att);
  float Wz = Z - BASE_H - L3 * sin(att);
  float d = sqrt(Wx * Wx + Wz * Wz);
  if (d > L1 + L2 + 0.5 || d < fabs(L1 - L2) - 0.5) return false;
  float c3 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  c3 = constrain(c3, -1.0, 1.0);
  float th2 = acos(c3);
  float beta = atan2(Wz, Wx);
  float gamma = atan2(L2 * sin(th2), L1 + L2 * cos(th2));
  float th1 = beta - gamma;
  sb = constrain((int)round(degrees(th0) + OFF_BASE), 0, 180);
  sl = constrain((int)round(degrees(th1) + OFF_SHOULDER), 0, 180);
  sr = constrain((int)round(OFF_ELBOW - degrees(th2)), 0, 180);
  return true;
}
void printXYZ() {
  float X, Y, Z;
  fk(curB - OFF_BASE, curL - OFF_SHOULDER, OFF_ELBOW - curR, X, Y, Z);
  Serial.print("XYZ: "); Serial.print(X); Serial.print(", ");
  Serial.print(Y); Serial.print(", "); Serial.println(Z);
}
void fkCurrent(float &X, float &Y, float &Z) {
  float th0 = radians(curB - OFF_BASE);
  float th1 = radians(curL - OFF_SHOULDER);
  float th2 = radians(OFF_ELBOW - curR);
  float att = radians(PEN_ATT_DEG);
  float x3 = L1 * cos(th1) + L2 * cos(th1 + th2) + L3 * cos(att);
  float z3 = L1 * sin(th1) + L2 * sin(th1 + th2) + L3 * sin(att);
  X = x3 * cos(th0); Y = x3 * sin(th0); Z = BASE_H + z3;
}

// ===== 任务五：绘图 =====
void drawTo(float X, float Y) {
  int sb, sl, sr;
  if (!ik(X, Y, PEN_Z, PEN_ATT_DEG, sb, sl, sr)) {
    Serial.print("IK FAIL X="); Serial.print(X); Serial.print(" Y="); Serial.println(Y);
    drawState = 0; return;
  }
  setTarget(sb, sl, sr, curG);
}
void startDrawLine(float X0, float Y0, float X1, float Y1) {
  dX0 = X0; dY0 = Y0; dX1 = X1; dY1 = Y1;
  dSeg = 0; dSegIdx = 0; drawState = 1; drawPaused = 0;
  drawTo(X0, Y0);
}
void startShape(int n, const float x[], const float y[]) {
  shapeN = n;
  for (int i = 0; i < n; i++) { shapeX[i] = x[i]; shapeY[i] = y[i]; }
  dSeg = 0; dSegIdx = 0; drawState = 2; drawPaused = 0;
  drawTo(shapeX[0], shapeY[0]);
}
void startDrawShapeN() { float x[4] = {65,65,115,115}, y[4] = {-20,20,-20,20}; startShape(4, x, y); }
void startDrawShapeTri() { float x[4] = {90,60,120,90}, y[4] = {25,-15,-15,25}; startShape(4, x, y); }
void startDrawShapeZ() { float x[4] = {60,120,60,120}, y[4] = {20,20,-20,-20}; startShape(4, x, y); }
void startDrawShapeV() { float x[3] = {60,90,120}, y[3] = {20,-20,20}; startShape(3, x, y); }
void startDrawPolyline() {
  if (tchCount < 2) { Serial.println("NEED TCH>=2"); return; }
  for (int i = 0; i < tchCount; i++) { shapeX[i] = tchX[i]; shapeY[i] = tchY[i]; }
  shapeN = tchCount; dSeg = 0; dSegIdx = 0; drawState = 3; drawPaused = 0;
  drawTo(shapeX[0], shapeY[0]);
}
void startDrawSmooth() {
  if (tchCount < 2) { Serial.println("NEED TCH>=2"); return; }
  for (int i = 0; i < tchCount; i++) { shapeX[i] = tchX[i]; shapeY[i] = tchY[i]; }
  shapeN = tchCount; dSeg = 0; dSegIdx = 0; drawState = 4; drawPaused = 0;
  drawTo(shapeX[0], shapeY[0]);
}
void updateDraw() {
  if (drawState == 0 || drawPaused) return;
  if (drawState == 1) {                 // 直线
    dSegIdx++;
    if (dSegIdx > DRAW_SEG) { drawState = 0; Serial.println("LINE OK"); return; }
    float t = (float)dSegIdx / DRAW_SEG;
    drawTo(dX0 + (dX1 - dX0) * t, dY0 + (dY1 - dY0) * t);
    return;
  }
  int n = (drawState == 4) ? SMOOTH_SEG : DRAW_SEG;
  if (dSeg > shapeN - 2) { drawState = 0; Serial.println("DRAW OK"); return; }
  dSegIdx++;
  if (dSegIdx > n) {
    dSegIdx = 0; dSeg++;
    if (dSeg > shapeN - 2) { drawState = 0; Serial.println("DRAW OK"); return; }
  }
  float X, Y;
  if (drawState == 4) {                 // 样条平滑
    int i = dSeg;
    float p0x = (i == 0) ? shapeX[0] : shapeX[i-1];
    float p0y = (i == 0) ? shapeY[0] : shapeY[i-1];
    float p1x = shapeX[i], p1y = shapeY[i];
    float p2x = shapeX[i+1], p2y = shapeY[i+1];
    float p3x = (i + 2 < shapeN) ? shapeX[i+2] : shapeX[i+1];
    float p3y = (i + 2 < shapeN) ? shapeY[i+2] : shapeY[i+1];
    float t = (float)dSegIdx / n, t2 = t * t, t3 = t2 * t;
    X = 0.5f * ((2*p1x) + (-p0x+p2x)*t + (2*p0x-5*p1x+4*p2x-p3x)*t2 + (-p0x+3*p1x-3*p2x+p3x)*t3);
    Y = 0.5f * ((2*p1y) + (-p0y+p2y)*t + (2*p0y-5*p1y+4*p2y-p3y)*t2 + (-p0y+3*p1y-3*p2y+p3y)*t3);
  } else {                              // 图形/折线：直线插值
    float t = (float)dSegIdx / n;
    X = shapeX[dSeg] + (shapeX[dSeg+1] - shapeX[dSeg]) * t;
    Y = shapeY[dSeg] + (shapeY[dSeg+1] - shapeY[dSeg]) * t;
  }
  drawTo(X, Y);
}
void teachPoint() {
  if (tchCount >= MAX_TCH) { Serial.println("TCH FULL"); return; }
  float X, Y, Z;
  fkCurrent(X, Y, Z);
  tchX[tchCount] = X; tchY[tchCount] = Y;
  if (tchCount == 0) PEN_Z = Z;         // 第一个点记录绘图高度
  tchCount++;
  Serial.print("TCH "); Serial.print(tchCount); Serial.print("/5 X=");
  Serial.print(X); Serial.print(" Y="); Serial.println(Y);
}
