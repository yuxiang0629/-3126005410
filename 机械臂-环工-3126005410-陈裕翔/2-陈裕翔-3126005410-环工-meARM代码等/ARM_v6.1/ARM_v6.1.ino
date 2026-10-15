/* ============================================================
   机械臂综合控制  v6.1 最终整合版（含画图纸面接触优化）
   （任务一 ~ 任务五 全部功能合并成单文件）
   ------------------------------------------------------------
   板卡  : Arduino UNO
   舵机  : 底座=D9  左臂(肩)=D8  右臂(肘)=D7  爪子=D6
   摇杆  : 左X=A0(底座)  左Y=A1(左臂)  右X=A2(爪子)  右Y=A3(右臂)
   按键  : 面包板自制按键 D10 D11 D12 D13，内部上拉，按下=LOW
           绿1=A(D10)  蓝=B(D11)  绿2=C(D12)  红=D(D13)
   串口  : 9600

   ==== 整合历史 ====
   v5.1 : 解决 UNO 内存超限，路径/录制数组改 uint8_t，MAX_REC=40、MAX_PATH=120
   v5.2 : 任务二改为"底座主导"——夹取/放置时手臂基本不动，只转底座
   v5.3 : 新增标定辅助 PB / GB（底座角度实时打印，方便抄标定值）
   v5.4 : 任务二补回"抬起 -> 平移 -> 落下"过程（不再贴桌横滑）
   v6.0 : 以上全部合并为最终单文件
   v6.1 : 绘图区域整体前移，避开底座附近"够不到纸"的区域；
          加贴纸检测，画图前自动检查每个点笔尖实际高度并警告

   ==== 功能总览 ====
   任务一 : 串口单/复合角度指令、双摇杆操控、爪开合、速度调节、回中
   任务二 : A/B/C 物体自动夹取 -> 抬起 -> 平移 -> 落下 -> 放置
            （底座主导水平位移，左臂负责抬起/落下）
   任务三 : 录制 / 播放动作、循环夹取、回中（4 键）
   任务四 : T 指令打印各关节角度 + 末端世界坐标（正解验证）
   任务五 : 末端铅笔绘制 直线 / 三角形 / 示教折线 / 平滑曲线，
            可暂停 / 继续 / 取消

   ==== 串口指令 ====
     x45 y45 z45 g30      空格分隔：底座/左臂/右臂/爪子 一次设定
     O=爪张开  S=爪闭合  M=回中  T=打印舵机角+末端坐标
     R=录制  P=播放  G=按序夹取A/B/C  A/B/C=直接夹取对应物
     DL=画直线  N/TRI=画三角形  TCH=示教点  PLN=折线  SMT=平滑曲线
     STOP=中止   H=加快   L=放慢
     PB=打印底座角   GB=底座角实时打印 开关
   按键 (空闲)：绿 1=G  蓝=R  绿 2=P  红=M
     组合键：绿 1+ 蓝=TCH  绿 1+ 绿 2=DL  绿 1+ 红=N
             蓝 + 绿 2=PLN  蓝 + 红=SMT  绿 2+ 红=TRI
   按键 (画图中/自动中)：绿 1=暂停/继续  蓝/绿 2=继续  红=取消
   ============================================================ */
#include <Servo.h>

/* ---------------- 引脚 ---------------- */
#define PIN_CLAW 6
#define PIN_R    7
#define PIN_L    8
#define PIN_BASE 9

#define JOY_BASE A0
#define JOY_L    A1
#define JOY_CLAW A2
#define JOY_R    A3

#define BTN_A 10   // 绿 1
#define BTN_B 11   // 蓝
#define BTN_C 12   // 绿 2
#define BTN_D 13   // 红

/* ---------------- 机械参数 (mm) ---------------- */
const float L1 = 80.0;    // 大臂  肩->肘
const float L2 = 80.0;    // 小臂  肘->腕
const float L3 = 50.0;    // 腕 -> 笔尖
const float BASE_H = 55.0;

/* ---------------- 夹爪角度 ---------------- */
const int GRIP_OPEN  = 0;
const int GRIP_CLOSE = 140;

/* ---------------- 任务二：抬起量 ---------------- */
const int LIFT_AMT = 15;      // 抬臂时左臂增加的度数（越大抬得越高）

/* ---------------- 绘图参数（v6.1 前伸优化）---------------- */
const float PEN_Z   = 5.0;     // 落笔高度 (压笔后实测)
const float XY_LINE = 120.0;   // 直线长度 mm
const float DRAW_Y  = 105.0;   // 画线前沿距离：从 70 加大到 105，离底座更远
const float DRAW_X0 = 110.0;   // 直线起点 X：从 60 前移到 110
const float DRAW_CX = 150.0;   // 三角形中心 X：从 95 前移到 150
const float DRAW_R  = 50.0;    // 三角形半径（保持不变）

/* ---------------- 舵机对象 ---------------- */
Servo srvClaw, srvR, srvL, srvBase;

/* ---------------- 当前 / 目标角度 ---------------- */
int curB = 90, curL = 90, curR = 90, curG = 90;
int tgtB = 90, tgtL = 90, tgtR = 90, tgtG = 90;
int stepDelay = 15;
unsigned long stepT0 = 0;

/* ---------------- 摇杆锁存 ---------------- */
int lockB = 0, lockL = 0, lockR = 0, lockG = 0;

/* ---------------- 标定辅助：底座角度自动打印 ---------------- */
bool calBase = false;
int  lastBase = -1;

/* ---------------- 任务二：底座主导 + 抬起/落下 ----------------
   每个物体一组：{ 左臂，右臂，夹取底座角，放置底座角 }
   流程：夹取点合爪 -> 左臂抬 LIFT_AMT -> 底座转到放置点 -> 左臂落回 -> 放下
   左臂在"抬起/落下"两段各动一次，其余时间不动；底座负责主要水平位移。*/
int st = 0, stObj = 0, grabTarget = 0;
unsigned long stTimer = 0;
const uint8_t poses2[3][4] = {
  // {L 左臂，R 右臂，夹取底座角，放置底座角}
  {  2, 118,  46, 130},  // 物体 A
  {  8, 168,  16, 145},  // 物体 B
  {  4, 150, 120,  20},  // 物体 C
};

/* ---------------- 任务三：录制 / 播放 ---------------- */
const int MAX_REC = 40;
uint8_t recB[MAX_REC], recL[MAX_REC], recR[MAX_REC], recG[MAX_REC];
int recCount = 0, playIdx = 0, playState = 0;

/* ---------------- 任务五：绘图 ---------------- */
int drawState = 0;       // 0 空闲 1 直线 2 多边形 3 折线 4 样条
int drawPaused = 0;
int drawIdx = 0, drawLen = 0;
const int MAX_PATH = 120;
uint8_t pathB[MAX_PATH], pathL[MAX_PATH], pathR[MAX_PATH], pathG[MAX_PATH];
const int MAX_TCH = 6;
float tchX[MAX_TCH], tchY[MAX_TCH];
int tchCount = 0;

/* ---------------- 串口 & 按键 ---------------- */
char rxBuf[33];
int  rxLen = 0;
int  btnRaw = 0;
bool btnFired = false;
unsigned long btnT0 = 0;

/* ============================================================
   SETUP / LOOP
   ============================================================ */
void setup() {
  Serial.begin(9600);
  srvBase.attach(PIN_BASE);
  srvL.attach(PIN_L);
  srvR.attach(PIN_R);
  srvClaw.attach(PIN_CLAW);

  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
  pinMode(BTN_D, INPUT_PULLUP);

  setTarget(90, 90, 90, 90);   // 上电回中
}

void loop() {
  pollSerial();    // ① 串口指令（非阻塞）
  scanButtons();   // ② 面包板按键（非阻塞）
  stepTick();      // ③ 摇杆 / 状态机 / 播放 / 绘图 统一步进节拍
}

/* ============================================================
   舵机基础
   ============================================================ */
void writeServos() {
  srvBase.write(curB);
  srvL.write(curL);
  srvR.write(curR);
  srvClaw.write(curG);
  if (calBase && curB != lastBase) {   // 标定模式：底座角度一变就打印
    lastBase = curB;
    Serial.print("BASE="); Serial.println(curB);
  }
}

void setTarget(int b, int l, int r, int g) {
  tgtB = constrain(b, 0, 170);
  tgtL = constrain(l, 0, 170);
  tgtR = constrain(r, 0, 170);
  tgtG = constrain(g, 0, GRIP_CLOSE);
}

// 向目标推进一格；返回是否全部到位
bool stepMove() {
  bool done = true;
  if (curB < tgtB) { curB++; done = false; } else if (curB > tgtB) { curB--; done = false; }
  if (curL < tgtL) { curL++; done = false; } else if (curL > tgtL) { curL--; done = false; }
  if (curR < tgtR) { curR++; done = false; } else if (curR > tgtR) { curR--; done = false; }
  if (curG < tgtG) { curG++; done = false; } else if (curG > tgtG) { curG--; done = false; }
  writeServos();
  return done;
}

// 统一节拍：每 stepDelay 毫秒推进一次
void stepTick() {
  unsigned long now = millis();
  if (now - stepT0 < (unsigned long)stepDelay) return;
  stepT0 = now;

  int dB = latchRead(JOY_BASE, lockB);
  int dL = latchRead(JOY_L,    lockL);
  int dC = latchRead(JOY_CLAW, lockG);
  int dR = latchRead(JOY_R,    lockR);
  if (dB || dL || dC || dR) {
    abortAuto();
    curB = constrain(curB + dB, 0, 180);
    curL = constrain(curL + dL, 0, 180);
    curR = constrain(curR + dR, 0, 180);
    curG = constrain(curG + dC, 0, GRIP_CLOSE);
    tgtB = curB; tgtL = curL; tgtR = curR; tgtG = curG;
    writeServos();
    return;
  }

  if (st != 0)        { updatePickMachine(); return; }
  if (playState != 0) { updatePlay();        return; }
  if (drawState != 0) { if (!drawPaused) updateDraw(); return; }

  stepMove();
}

/* ============================================================
   摇杆：锁存式读取（兼容开关量输出的摇杆板）
   ============================================================ */
int latchRead(int pin, int &lock) {
  int v = analogRead(pin);
  if (lock == 0) {
    if (v > 900)      lock = 1;
    else if (v < 100) lock = -1;
  } else {
    if (v > 250 && v < 750) lock = 0;
  }
  return lock;
}

/* ============================================================
   串口：非阻塞逐字符收行
   ============================================================ */
void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      rxBuf[rxLen] = 0;
      if (rxLen > 0) handle(String(rxBuf));
      rxLen = 0;
    } else if (rxLen < 32) {
      rxBuf[rxLen++] = c;
    }
  }
}

/* ============================================================
   串口指令分发
   ============================================================ */
void handle(String cmd) {
  Serial.print("CMD> "); Serial.println(cmd);

  // —— 空格分隔的多路坐标：x45 y45 z45 g30 ——
  if (cmd.indexOf(' ') > 0) {
    int b = curB, l = curL, r = curR, g = curG;
    int start = 0;
    while (start < (int)cmd.length()) {
      int sp = cmd.indexOf(' ', start);
      String seg = (sp < 0) ? cmd.substring(start) : cmd.substring(start, sp);
      seg.trim();
      if (seg.startsWith("x"))      b = seg.substring(1).toInt();
      else if (seg.startsWith("y")) l = seg.substring(1).toInt();
      else if (seg.startsWith("z")) r = seg.substring(1).toInt();
      else if (seg.startsWith("g")) g = seg.substring(1).toInt();
      if (sp < 0) break;
      start = sp + 1;
    }
    abortAuto();
    setTarget(b, l, r, g);
    return;
  }

  // —— 单词指令 ——
  if      (cmd == "O")    { abortAuto(); setTarget(curB, curL, curR, GRIP_OPEN);  }
  else if (cmd == "S")    { abortAuto(); setTarget(curB, curL, curR, GRIP_CLOSE); }
  else if (cmd == "M")    { abortAuto(); setTarget(90, 90, 90, 90); }
  else if (cmd == "H")    { stepDelay = max(2, stepDelay - 2); }
  else if (cmd == "L")    { stepDelay = min(40, stepDelay + 2); }
  else if (cmd == "R")    { record(); }
  else if (cmd == "P")    { startPlay(); }
  else if (cmd == "G")    { startPick(grabTarget); grabTarget = (grabTarget + 1) % 3; }
  else if (cmd == "A")    { startPick(0); grabTarget = 1; }
  else if (cmd == "B")    { startPick(1); grabTarget = 2; }
  else if (cmd == "C")    { startPick(2); grabTarget = 0; }
  else if (cmd == "T")    { printPosition(); }
  else if (cmd == "DL")   { startDraw(1); }
  else if (cmd == "N")    { startDraw(2); }
  else if (cmd == "TRI")  { startDraw(2); }
  else if (cmd == "PLN")  { startDraw(3); }
  else if (cmd == "SMT")  { startDraw(4); }
  else if (cmd == "TCH")  { teachPoint(); }
  else if (cmd == "STOP") { abortAuto(); }
  else if (cmd == "PB")   { Serial.print("BASE="); Serial.println(curB); }
  else if (cmd == "GB")   { calBase = !calBase;
                            Serial.println(calBase ? "CAL BASE ON" : "CAL BASE OFF"); }
  else if (cmd.startsWith("x")) { abortAuto(); setTarget(cmd.substring(1).toInt(), curL, curR, curG); }
  else if (cmd.startsWith("y")) { abortAuto(); setTarget(curB, cmd.substring(1).toInt(), curR, curG); }
  else if (cmd.startsWith("z")) { abortAuto(); setTarget(curB, curL, cmd.substring(1).toInt(), curG); }
  else if (cmd.startsWith("g")) { abortAuto(); setTarget(curB, curL, curR, cmd.substring(1).toInt()); }
}

void abortAuto() {
  st = 0; playState = 0; drawState = 0; drawPaused = 0;
}

/* ============================================================
   按键：非阻塞扫描 + 消抖 + 一次按下只触发一次 + 组合键
   ============================================================ */
void scanButtons() {
  unsigned long now = millis();
  int mask = 0;
  if (!digitalRead(BTN_A)) mask |= 1;
  if (!digitalRead(BTN_B)) mask |= 2;
  if (!digitalRead(BTN_C)) mask |= 4;
  if (!digitalRead(BTN_D)) mask |= 8;

  if (mask != btnRaw) { btnRaw = mask; btnT0 = now; return; }  // 变化则重新计时
  if (now - btnT0 < 30) return;                                // 未稳定
  if (btnRaw == 0) { btnFired = false; return; }               // 松开解锁
  if (btnFired) return;                                        // 本次已触发
  btnFired = true;
  onButton(btnRaw);
}

void onButton(int mask) {
  // A. 画图中：过程控制
  if (drawState != 0) {
    switch (mask) {
      case 1: drawPaused = !drawPaused; Serial.println(drawPaused ? "PAUSE" : "RESUME"); break;
      case 2: drawPaused = 0; Serial.println("RESUME"); break;
      case 4: drawPaused = 0; Serial.println("RESUME"); break;
      case 8: abortAuto();  Serial.println("CANCEL"); break;
    }
    return;
  }

  // B. 自动运行中：红色取消
  if (st != 0 || playState != 0) {
    if (mask == 8) { abortAuto(); Serial.println("CANCEL"); }
    return;
  }

  // C. 空闲：单键=任务三，组合键=任务五
  switch (mask) {
    case 1:  handle("G");   break;   // 绿 1
    case 2:  handle("R");   break;   // 蓝
    case 4:  handle("P");   break;   // 绿 2
    case 8:  handle("M");   break;   // 红
    case 3:  handle("TCH"); break;   // 绿 1+ 蓝
    case 5:  handle("DL");  break;   // 绿 1+ 绿 2
    case 9:  handle("N");   break;   // 绿 1+ 红
    case 6:  handle("PLN"); break;   // 蓝 + 绿 2
    case 10: handle("SMT"); break;   // 蓝 + 红
    case 12: handle("TRI"); break;   // 绿 2+ 红
  }
}

/* ============================================================
   任务二：A/B/C 夹取 -> 放置（底座主导 + 抬起/落下）
   ①底座到夹取点 (爪张) -> ②合爪 -> ③抬臂 -> ④底座转放置点 -> ⑤落臂 -> ⑥张爪
   ============================================================ */
void startPick(int obj) {
  stObj = obj;
  st = 1;
  // 底座先到夹取点，左臂/右臂到固定姿态，爪子张开
  setTarget(poses2[obj][2], poses2[obj][0], poses2[obj][1], GRIP_OPEN);
  Serial.print("PICK ");
  Serial.println(obj == 0 ? "A" : (obj == 1 ? "B" : "C"));
}

void updatePickMachine() {
  switch (st) {
    case 1:  // 转到夹取点（爪张开）
      if (stepMove()) { st = 2; stTimer = millis(); }
      break;
    case 2:  // 合爪夹稳
      tgtG = GRIP_CLOSE;
      if (stepMove() && millis() - stTimer > 150) { st = 3; stTimer = millis(); }
      break;
    case 3:  // 抬臂 —— 抬起（左臂 +LIFT_AMT，底座不动）
      setTarget(curB, poses2[stObj][0] + LIFT_AMT, curR, GRIP_CLOSE);
      if (stepMove()) { st = 4; stTimer = millis(); }
      break;
    case 4:  // 底座转到放置点 —— 平移（左臂保持抬起）
      setTarget(poses2[stObj][3], poses2[stObj][0] + LIFT_AMT, curR, GRIP_CLOSE);
      if (stepMove()) { st = 5; stTimer = millis(); }
      break;
    case 5:  // 左臂落回 —— 落下（底座不动）
      setTarget(curB, poses2[stObj][0], curR, GRIP_CLOSE);
      if (stepMove()) { st = 6; stTimer = millis(); }
      break;
    case 6:  // 张开放下
      tgtG = GRIP_OPEN;
      if (stepMove() && millis() - stTimer > 150) { st = 0; Serial.println("PICK DONE"); }
      break;
    default:
      st = 0;
      break;
  }
}

/* ============================================================
   任务三：录制 / 播放
   ============================================================ */
void record() {
  if (recCount < MAX_REC) {
    recB[recCount] = (uint8_t)curB; recL[recCount] = (uint8_t)curL;
    recR[recCount] = (uint8_t)curR; recG[recCount] = (uint8_t)curG;
    recCount++;
    Serial.print("REC "); Serial.println(recCount);
  } else {
    Serial.println("REC FULL");
  }
}

void startPlay() {
  if (recCount == 0) { Serial.println("NO REC"); return; }
  playIdx = 0; playState = 1;
  setTarget(recB[0], recL[0], recR[0], recG[0]);
}

void updatePlay() {
  if (stepMove()) {
    playIdx++;
    if (playIdx >= recCount) { playState = 0; Serial.println("PLAY DONE"); }
    else setTarget(recB[playIdx], recL[playIdx], recR[playIdx], recG[playIdx]);
  }
}

/* ============================================================
   任务五：运动学 + 绘图（v6.1 含贴纸检测）
   ============================================================ */
// 逆解：给定笔尖世界坐标 (X,Y,Z)，笔尖竖直朝下，求 底座/左臂/右臂 角度
void ikTo(float X, float Y, float Z, int &b, int &l, int &r) {
  float base = degrees(atan2(Y, X)) + 90.0;     // 底座
  float rad  = sqrt(X * X + Y * Y);
  float xp   = rad;                             // 平面内水平距离
  float zp   = (Z + L3) - BASE_H;               // 肩关节为原点的竖直距离
  float d2   = xp * xp + zp * zp;
  float D    = (d2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  D = constrain(D, -1.0, 1.0);
  float th2  = atan2(-sqrt(1.0 - D * D), D);    // 肘
  float th1  = atan2(zp, xp) - atan2(L2 * sin(th2), L1 + L2 * cos(th2)); // 肩

  b = (int)round(base);
  l = (int)round(degrees(th1) + 90.0);
  r = (int)round(90.0 - degrees(th2));
  b = constrain(b, 0, 180);
  l = constrain(l, 0, 180);
  r = constrain(r, 0, 180);
}

// 正解：由舵机角反算笔尖实际 Z 高度（用于贴纸检测）
float calcPenZ(int l_now, int r_now) {
  float th1 = radians(l_now - 90);
  float th2 = radians(90 - r_now);
  return BASE_H + L1 * sin(th1) + L2 * sin(th1 + th2) - L3;
}

void addPath(float X, float Y, float Z, int grip) {
  if (drawLen >= MAX_PATH) return;
  int b, l, r;
  ikTo(X, Y, Z, b, l, r);
  // 贴纸检测：用正解回算笔尖实际高度，若明显高于 PEN_Z 则警告
  float penZ = calcPenZ(l, r);
  if (penZ > PEN_Z + 3.0) {
    Serial.print("WARN LOW-REACH X="); Serial.print(X);
    Serial.print(" Y="); Serial.print(Y);
    Serial.print(" penZ="); Serial.println(penZ);
  }
  pathB[drawLen] = (uint8_t)b;
  pathL[drawLen] = (uint8_t)l;
  pathR[drawLen] = (uint8_t)r;
  pathG[drawLen] = (uint8_t)constrain(grip, 0, GRIP_CLOSE);
  drawLen++;
}

float liftZ() { return PEN_Z + 25.0; }   // 抬笔高度

// 直线（v6.1 前伸区）
void buildLine() {
  drawLen = 0;
  addPath(DRAW_X0, DRAW_Y, liftZ(), GRIP_CLOSE);        // 抬笔到起点
  addPath(DRAW_X0, DRAW_Y, PEN_Z,  GRIP_CLOSE);         // 落笔
  int seg = 50;
  for (int i = 1; i <= seg; i++) {
    float t = i / (float)seg;
    addPath(DRAW_X0 + XY_LINE * t, DRAW_Y, PEN_Z, GRIP_CLOSE);
  }
  addPath(DRAW_X0 + XY_LINE, DRAW_Y, liftZ(), GRIP_CLOSE);   // 抬笔收尾
}

// 三角形（v6.1 前伸区）
void buildTriangle() {
  drawLen = 0;
  float cx = DRAW_CX, cy = DRAW_Y, R = DRAW_R;
  float px[3], py[3];
  for (int i = 0; i < 3; i++) {
    float a = radians(90 + i * 120);
    px[i] = cx + R * cos(a);
    py[i] = cy + R * sin(a);
  }
  addPath(px[0], py[0], liftZ(), GRIP_CLOSE);
  addPath(px[0], py[0], PEN_Z,  GRIP_CLOSE);
  for (int e = 0; e < 3; e++) {
    int n = (e + 1) % 3;
    int seg = 25;
    for (int s = 1; s <= seg; s++) {
      float t = s / (float)seg;
      addPath(px[e] + (px[n] - px[e]) * t, py[e] + (py[n] - py[e]) * t, PEN_Z, GRIP_CLOSE);
    }
  }
  addPath(px[0], py[0], liftZ(), GRIP_CLOSE);
}

// 折线（示教点）
void buildPolyline() {
  drawLen = 0;
  addPath(tchX[0], tchY[0], liftZ(), GRIP_CLOSE);
  addPath(tchX[0], tchY[0], PEN_Z,  GRIP_CLOSE);
  for (int i = 0; i < tchCount - 1; i++) {
    int seg = 20;
    for (int s = 1; s <= seg; s++) {
      float t = s / (float)seg;
      addPath(tchX[i] + (tchX[i + 1] - tchX[i]) * t,
              tchY[i] + (tchY[i + 1] - tchY[i]) * t, PEN_Z, GRIP_CLOSE);
    }
  }
  addPath(tchX[tchCount - 1], tchY[tchCount - 1], liftZ(), GRIP_CLOSE);
}

// 三次样条插值（Catmull-Rom）
float cr(float a, float b, float c, float d, float t) {
  return 0.5 * ((2 * b) + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t * t +
                (-a + 3 * b - 3 * c + d) * t * t * t);
}

void buildSpline() {
  drawLen = 0;
  addPath(tchX[0], tchY[0], liftZ(), GRIP_CLOSE);
  addPath(tchX[0], tchY[0], PEN_Z,  GRIP_CLOSE);
  for (int i = 0; i < tchCount - 1; i++) {
    float p0x = tchX[i == 0 ? 0 : i - 1],           p0y = tchY[i == 0 ? 0 : i - 1];
    float p1x = tchX[i],                            p1y = tchY[i];
    float p2x = tchX[i + 1],                        p2y = tchY[i + 1];
    float p3x = tchX[(i + 2 < tchCount) ? i + 2 : tchCount - 1];
    float p3y = tchY[(i + 2 < tchCount) ? i + 2 : tchCount - 1];
    int seg = 16;
    for (int s = 1; s <= seg; s++) {
      float t = s / (float)seg;
      addPath(cr(p0x, p1x, p2x, p3x, t), cr(p0y, p1y, p2y, p3y, t), PEN_Z, GRIP_CLOSE);
    }
  }
  addPath(tchX[tchCount - 1], tchY[tchCount - 1], liftZ(), GRIP_CLOSE);
}

void startDraw(int s) {
  if ((s == 3 || s == 4) && tchCount < 2) { Serial.println("NEED TCH"); return; }
  switch (s) {
    case 1: buildLine();     break;
    case 2: buildTriangle(); break;
    case 3: buildPolyline(); break;
    case 4: buildSpline();   break;
  }
  if (drawLen == 0) { Serial.println("NO PATH"); return; }
  drawIdx = 0; drawState = s; drawPaused = 0;
  Serial.print("DRAW #"); Serial.print(s);
  Serial.print(" pts=");  Serial.println(drawLen);
}

void updateDraw() {
  setTarget(pathB[drawIdx], pathL[drawIdx], pathR[drawIdx], pathG[drawIdx]);
  if (stepMove()) {
    drawIdx++;
    if (drawIdx >= drawLen) { drawState = 0; Serial.println("DRAW DONE"); }
  }
}

// 示教：记录当前笔尖 XY
void teachPoint() {
  if (tchCount >= MAX_TCH) { Serial.println("TCH FULL"); return; }
  float th0 = radians(curB - 90);
  float th1 = radians(curL - 90);
  float th2 = radians(90 - curR);
  float x   = L1 * cos(th1) + L2 * cos(th1 + th2);
  tchX[tchCount] = x * cos(th0);
  tchY[tchCount] = x * sin(th0);
  tchCount++;
  Serial.print("TCH "); Serial.print(tchCount);
  Serial.print(" X="); Serial.print(tchX[tchCount - 1]);
  Serial.print(" Y="); Serial.println(tchY[tchCount - 1]);
}

/* ============================================================
   T 指令：打印舵机角度 + 各关节/笔尖世界坐标
   ============================================================ */
void printPosition() {
  Serial.print("SERVO   Base="); Serial.print(curB);
  Serial.print("  L="); Serial.print(curL);
  Serial.print("  R="); Serial.print(curR);
  Serial.print("  G="); Serial.println(curG);

  float th0 = radians(curB - 90);
  float th1 = radians(curL - 90);
  float th2 = radians(90 - curR);
  float th3 = -radians(90) - (th1 + th2);

  float x1 = L1 * cos(th1),               z1 = L1 * sin(th1);
  float x2 = x1 + L2 * cos(th1 + th2),    z2 = z1 + L2 * sin(th1 + th2);
  float x3 = x2 + L3 * cos(th1 + th2 + th3), z3 = z2 + L3 * sin(th1 + th2 + th3);

  Serial.print("SHOULDER X="); Serial.print(x1 * cos(th0)); Serial.print(" Y="); Serial.print(x1 * sin(th0)); Serial.print(" Z="); Serial.println(BASE_H + z1);
  Serial.print("ELBOW    X="); Serial.print(x2 * cos(th0)); Serial.print(" Y="); Serial.print(x2 * sin(th0)); Serial.print(" Z="); Serial.println(BASE_H + z2);
  Serial.print("PEN TIP  X="); Serial.print(x3 * cos(th0)); Serial.print(" Y="); Serial.print(x3 * sin(th0)); Serial.print(" Z="); Serial.println(BASE_H + z3);
  Serial.println("----------------------------");
}
