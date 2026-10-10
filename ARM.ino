#include <Servo.h>

// 按你板子丝印定义：Claw=D6 R=D7 L=D8 Base=D9
#define PIN_CLAW 6
#define PIN_R    7
#define PIN_L    8
#define PIN_BASE 9

// 摇杆模拟输入：接 Arduino 的 A0~A3
#define JOY1X A0   // 左摇杆 X → Base 底座
#define JOY1Y A1   // 左摇杆 Y → L 左臂
#define JOY2X A2   // 右摇杆 X → Claw 爪子
#define JOY2Y A3   // 右摇杆 Y → R 右臂

Servo srvClaw, srvR, srvL, srvBase;
const float L1 = 80.0;     // 大臂长度（肩关节到肘关节）
const float L2 = 80.0;     // 小臂长度（肘关节到腕关节）
const float L3 = 75.0;     // 腕关节到夹爪中心
const float BASE_H = 55.0; // 底座中心到肩关节的竖直高度

// ===== 爪子开合角度（实测标定）=====
const int GRIP_OPEN  = 0;    // 爪子完全张开（实测显示0°）
const int GRIP_CLOSE = 150;   // 爪子闭合

// ===== 任务三：目标切换 + 非阻塞夹取状态机 =====
int grabTarget = 0;              // 0=A, 1=B, 2=C，第一次按 G 抓 A
int st = 0;                      // 夹取状态：0=空闲,1=去夹取点,2=合爪,3=抬起,4=去放置点,5=张开
int stObj = 0;                   // 当前夹取目标
unsigned long stTimer = 0;       // 状态停留计时

// ===== 任务三：动作播放状态机 =====
int playIdx = 0;       // 当前播到第几个记录点
int playState = 0;     // 0=空闲, 1=播放中

// 目标角度（非阻塞移动用）
int tgtB = 90, tgtL = 90, tgtR = 90, tgtC = 90;
unsigned long lastStep = 0;      // 移动节拍计时

int curBase = 90, curL = 90, curR = 90, curClaw = 90;
int stepDelay = 20;   // 每步延时(ms)，数值越小越快

// 任务三：录制/播放数组
const int MAX_REC = 50;
int recBase[MAX_REC], recL[MAX_REC], recR[MAX_REC], recClaw[MAX_REC];
int recCount = 0;

void setup() {
  Serial.begin(9600);
  srvBase.attach(PIN_BASE);
  srvL.attach(PIN_L);
  srvR.attach(PIN_R);
  srvClaw.attach(PIN_CLAW);
  setTarget(90,90,90,90);                     // 上电回中
}

void loop() {
  // ① 串口指令：每轮都检查
  if (Serial.available()) {
    String cmd = Serial.readStringUntil(10);
    cmd.trim();
    handle(cmd);
  }

  // ② 摇杆读数
  int dBase = joyDir(analogRead(JOY1X));   // 左摇杆 X → 底座
  int dL    = joyDir(analogRead(JOY1Y));   // 左摇杆 Y → 左臂
  int dClaw = joyDir(analogRead(JOY2X));   // 右摇杆 X → 爪子
  int dR    = joyDir(analogRead(JOY2Y));   // 右摇杆 Y → 右臂

  // ③ 节拍驱动
  if (millis() - lastStep >= (unsigned long)stepDelay) {
    lastStep = millis();

    if (dBase || dL || dClaw || dR) {
      st = 0; playState = 0;
      curBase = constrain(curBase + dBase, 0, 180);
      curL    = constrain(curL + dL, 0, 180);
      curR    = constrain(curR + dR, 0, 180);
      curClaw = constrain(curClaw + dClaw, 0, 170);
      srvBase.write(curBase); srvL.write(curL);
      srvR.write(curR); srvClaw.write(curClaw);

      // ★关键：把目标同步成当前位置，防止松手后被拉回
       tgtB = curBase; tgtL = curL;
      tgtR = curR; tgtC = curClaw;
    } else if (st != 0) {
      updatePickMachine();
    } else if (playState != 0) {
      updatePlay();
    } else {
      stepMove();
    }
  }
}

// 摇杆死区：中位约512，>700 正方向，<300 负方向，中间不动
int joyDir(int v) {
  if (v > 700) return 1;
  if (v < 300) return -1;
  return 0;
}

// ---- 指令分发 ----

void handle(String cmd) {
  
  Serial.print("收到:[");
  Serial.print(cmd);
  Serial.println("]");
  // ...原判断不变
   if (cmd.indexOf(' ') > 0) {
    int b = curBase, l = curL, r = curR, c = curClaw;
    int start = 0;
    while (start < cmd.length()) {
      int sp = cmd.indexOf(' ', start);
      String seg = (sp < 0) ? cmd.substring(start) : cmd.substring(start, sp);
      seg.trim();
      if (seg.startsWith("x")) b = seg.substring(1).toInt();
      else if (seg.startsWith("y")) l = seg.substring(1).toInt();
      else if (seg.startsWith("z")) r = seg.substring(1).toInt();
      if (sp < 0) break;
      start = sp + 1;
    }
    st = 0;          // 打断夹取
    playState = 0;   // 打断播放
    setTarget(b, l, r, c);   // 一次调用，四个舵机同时平滑移动
    return;               // 处理完直接返回，不再走下面单指令判断
  }


   // ★任何手动指令都先打断进行中的夹取
  st = 0;

  if (cmd == "O")      { setTarget(curBase, curL, curR, GRIP_OPEN); }   // 爪张开
  else if (cmd == "S") { setTarget(curBase, curL, curR, GRIP_CLOSE); }    // 爪闭合
  else if (cmd == "H") { stepDelay = max(5, stepDelay - 5); }
  else if (cmd == "L") { stepDelay = min(200, stepDelay + 5); }
  else if (cmd == "M") { setTarget(90, 90, 90, 90); }             // 回中（可打断一切）
  else if (cmd == "R") { record(); }          // ★录制当前动作
  else if (cmd == "P") { startPlay(); }       // ★开始播放录制动作
  // ★G：按一次只夹一次，抓完切换目标（不再循环3次）
  else if (cmd == "G") {
    Serial.print("夹取: ");
    Serial.println(grabTarget == 0 ? "A" : (grabTarget == 1 ? "B" : "C"));
    startPick(grabTarget);                  // 开始夹当前目标
    grabTarget = (grabTarget + 1) % 3;      // 抓完切换：0→1→2→0
  }
  else if (cmd == "A") { startPick(0); grabTarget = 1; }   // 直接夹A
  else if (cmd == "B") { startPick(1); grabTarget = 2; }
  else if (cmd == "C") { startPick(2); grabTarget = 0; }
  else if (cmd == "T") { printPosition(); }
  else if (cmd.startsWith("x")) { setTarget(cmd.substring(1).toInt(), curL, curR, curClaw); }
  else if (cmd.startsWith("y")) { setTarget(curBase, cmd.substring(1).toInt(), curR, curClaw); }
  else if (cmd.startsWith("z")) { setTarget(curBase, curL, cmd.substring(1).toInt(), curClaw); }
}

// 平滑插值移动（四个舵机逐度逼近，避免抖动）
// 设置移动目标（自动限幅 0~180）
void setTarget(int b, int l, int r, int c) {
  tgtB = constrain(b, 0, 180); tgtL = constrain(l, 0, 180);
  tgtR = constrain(r, 0, 180); tgtC = constrain(c, 0, 180);
}

// 推进一格：每轮调用一次，返回是否全部到位
bool stepMove() {
  bool done = true;
  if (curBase < tgtB)
   { curBase++; done = false; }
  else if (curBase > tgtB)
   { curBase--; done = false; }
  if (curL < tgtL) 
  { curL++; done = false; }
  else if (curL > tgtL) 
  { curL--; done = false; }
  if (curR < tgtR)
   { curR++; done = false; }
  else if (curR > tgtR)
   { curR--; done = false; }
  if (curClaw < tgtC)
   { curClaw++; done = false; }
  else if (curClaw > tgtC) 
  { curClaw--; done = false; }
  srvBase.write(curBase); srvL.write(curL);
  srvR.write(curR); srvClaw.write(curClaw);
  return done;
}
// 角度转弧度
float deg2rad(float d) { return d * PI / 180.0; }

// 打印当前位置：各关节角度 + 各部位XYZ坐标
void printPosition() {
  // ★ 角度转换：下面三行按你的机械臂安装方向调整（详见说明）
  float th0 = deg2rad(curBase - 90);   // 底座：90°=朝前
  float th1 = deg2rad(curL - 90);      // 肩：90°=大臂水平
  float th2 = deg2rad(90 - curR);      // 肘：方向不对就改成 curR - 90
  float th3 = -(th1 + th2);            // 腕：保持夹爪水平

  // 竖直平面内各关节位置（肩关节为原点）
  float x0 = 0,                          z0 = 0;
  float x1 = L1 * cos(th1),              z1 = L1 * sin(th1);
  float x2 = x1 + L2 * cos(th1 + th2),   z2 = z1 + L2 * sin(th1 + th2);
  float x3 = x2 + L3 * cos(th1 + th2 + th3), z3 = z2 + L3 * sin(th1 + th2 + th3);

  // 换算到世界坐标（底座中心为原点，X=朝前，Y=左右，Z=高度）
  float w0x = 0,                w0y = 0,                w0z = BASE_H + z0;
  float w1x = x1 * cos(th0),    w1y = x1 * sin(th0),    w1z = BASE_H + z1;
  float w2x = x2 * cos(th0),    w2y = x2 * sin(th0),    w2z = BASE_H + z2;
  float w3x = x3 * cos(th0),    w3y = x3 * sin(th0),    w3z = BASE_H + z3;

  Serial.println("======== 当前位置 ========");
  Serial.print("舵机角度: Base="); Serial.print(curBase);
  Serial.print("  L="); Serial.print(curL);
  Serial.print("  R="); Serial.print(curR);
  Serial.print("  Claw="); Serial.println(curClaw);
  Serial.print("底座关节: X="); Serial.print(w0x); Serial.print(" Y="); Serial.print(w0y); Serial.print(" Z="); Serial.println(w0z);
  Serial.print("肘关节  : X="); Serial.print(w1x); Serial.print(" Y="); Serial.print(w1y); Serial.print(" Z="); Serial.println(w1z);
  Serial.print("腕关节  : X="); Serial.print(w2x); Serial.print(" Y="); Serial.print(w2y); Serial.print(" Z="); Serial.println(w2z);
  Serial.print("夹爪末端: X="); Serial.print(w3x); Serial.print(" Y="); Serial.print(w3y); Serial.print(" Z="); Serial.println(w3z);
  Serial.println("==========================");
}


// ---- 任务二：A/B/C 物体夹取→放置（角度按实际联调改）----
const int poses[3][2][4] = {
  {{90, 60, 30, GRIP_OPEN}, {90, 120, 150, GRIP_OPEN}},   // 物体A：夹取点 / 放置点
  {{90, 50, 40, GRIP_OPEN}, {90, 110, 140, GRIP_OPEN}},   // 物体B
  {{90, 70, 50, GRIP_OPEN}, {90, 100, 130, GRIP_OPEN}},   // 物体C
};

// 开始夹取目标 obj（只设置状态，不阻塞，随时可被打断）
void startPick(int obj) {
  stObj = obj;
  st = 1;
  setTarget(poses[obj][0][0], poses[obj][0][1], poses[obj][0][2], GRIP_OPEN);  // 去夹取点，爪张开
}

// 每轮调用：当前目标到位后，推进下一阶段
void updatePickMachine() {
  switch (st) {
    case 1:   // 到夹取点
      if (stepMove()) { st = 2; stTimer = millis(); }
      break;
    case 2:   // 合爪夹住（到位后再等150ms夹稳）
      setTarget(curBase, curL, curR, GRIP_CLOSE);
      if (stepMove() && millis() - stTimer > 150) { st = 3; stTimer = millis(); }
      break;
    case 3:   // 抬起
      setTarget(poses[stObj][0][0], poses[stObj][0][1] + 15, poses[stObj][0][2], GRIP_CLOSE);
      if (stepMove()) { st = 4; }
      break;
    case 4:   // 去放置点
      setTarget(poses[stObj][1][0], poses[stObj][1][1], poses[stObj][1][2], GRIP_CLOSE);
      if (stepMove()) { st = 5; stTimer = millis(); }
      break;
    case 5:   // 张开放下
      setTarget(curBase, curL, curR, GRIP_OPEN);
      if (stepMove() && millis() - stTimer > 150) {
        st = 0;
        Serial.println("本次夹取完成");
      }
      break;
    default:
      st = 0;
      break;
  }
}

// ---- 任务三：录制 / 播放 / 循环抓取 ----
void record() {
  if (recCount < MAX_REC) {
    recBase[recCount] = curBase; recL[recCount] = curL;
    recR[recCount] = curR;        recClaw[recCount] = curClaw;
    recCount++;
  }
}

// 开始播放录制的动作（非阻塞）
void startPlay() {
  if (recCount == 0) { Serial.println("没有录制动作"); return; }
  playIdx = 0;
  playState = 1;
  setTarget(recBase[0], recL[0], recR[0], recClaw[0]);
}

// 每轮推进播放：当前点到位后播下一个
void updatePlay() {
  if (playState != 1) return;
  if (stepMove()) {                       // 当前点走完了
    playIdx++;
    if (playIdx >= recCount) {
      playState = 0;
      Serial.println("播放完成");
    } else {
      setTarget(recBase[playIdx], recL[playIdx], recR[playIdx], recClaw[playIdx]);
    }
  }
}


