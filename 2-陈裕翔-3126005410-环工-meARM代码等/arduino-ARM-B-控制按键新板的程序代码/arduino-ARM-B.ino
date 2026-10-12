#include <SoftwareSerial.h>

// 按键引脚：D10~D13，外部下拉电阻（未按=0，按下=1）
const int KEY_PINS[4] = {10, 11, 12, 13};
const char* KEY_CMDS[4] = {"G", "REC", "PLY", "M"};

int last[4] = {LOW, LOW, LOW, LOW};
unsigned long debounce[4] = {0, 0, 0, 0};

// 软串口：RX=D2（可选，收主控回传），TX=D3 → 主控D2
SoftwareSerial link(2, 3);

void setup() {
  Serial.begin(9600);     // 本板调试口（可不开）
  link.begin(9600);       // 与主控通信，波特率必须一致
  for (int i = 0; i < 4; i++) {
    pinMode(KEY_PINS[i], INPUT);   // ★ 外部已有下拉，禁止用 INPUT_PULLUP
  }
}

void loop() {
  for (int i = 0; i < 4; i++) {
    int v = digitalRead(KEY_PINS[i]);
    // 上升沿触发（LOW→HIGH）+ 防抖
    if (v == HIGH && last[i] == LOW && millis() - debounce[i] > 50) {
      debounce[i] = millis();
      link.println(KEY_CMDS[i]);            // 发给主控板
      Serial.print("SEND "); Serial.println(KEY_CMDS[i]);  // 按键板自己的调试回显
    }
    last[i] = v;
  }
}
