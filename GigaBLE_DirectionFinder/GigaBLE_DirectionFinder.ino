/*
 * BLE Direction Finder — Arduino Giga R1 WiFi + Giga Display
 *
 * Hardware:
 *   - Arduino Giga R1 WiFi
 *   - Arduino Giga Display Shield
 *   - Modulino Movement  →  SDA=pin20, SCL=pin21, 3.3V, GND
 *   - Buzzer             →  pin 2
 *
 * Librerie (Library Manager):
 *   ArduinoBLE · Arduino_GigaDisplay_GFX
 *   Arduino_GigaDisplayTouch · Arduino_Modulino
 */

#include <ArduinoBLE.h>
#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"
#include <Arduino_Modulino.h>

// ══════════════════════════════════════════════
//  HARDWARE
// ══════════════════════════════════════════════
GigaDisplay_GFX          display;
Arduino_GigaDisplayTouch touch;
ModulinoMovement         imu;
#define BUZZER_PIN 2

// ══════════════════════════════════════════════
//  ORIENTAMENTO (Landscape o Portrait)
// ══════════════════════════════════════════════
bool usePortrait = false;  // Cambia a true per Portrait mode
int DISP_W = 800, DISP_H = 480;  // Width e Height dinamici

// ══════════════════════════════════════════════
//  COLORI RGB565
// ══════════════════════════════════════════════
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_GRAY    0x8410
#define C_DGRAY   0x2104
#define C_BLUE    0x001F
#define C_DBLUE   0x000D
#define C_CYAN    0x07FF
#define C_GREEN   0x07E0
#define C_DGREEN  0x0300
#define C_YELLOW  0xFFE0
#define C_ORANGE  0xFD20
#define C_RED     0xF800
#define C_HDR     0x000D
#define C_ROW_A   0x1082
#define C_ROW_B   0x18C3
#define C_SEL_BG  0x0248
#define C_BTN_G   0x03E0
#define C_BTN_B   0x001F
#define C_BTN_R   0xA000
#define C_FTR     0x0010
#define C_PANEL   0x0820

// ══════════════════════════════════════════════
//  LAYOUT (Landscape: 800 × 480)
// ══════════════════════════════════════════════
int SW, SH;              // Width e Height del display
int HDR_H   = 60;        // Header height
int ROW_H   = 56;        // Row height
int MAX_VIS = 6;         // Max visible rows
int LIST_Y;              // = HDR_H
int FTR_Y;               // = HDR_H + ROW_H * MAX_VIS
int FTR_H;               // = SH - FTR_Y

void updateLayoutDimensions() {
  if (usePortrait) {
    SW = 480;
    SH = 800;
    MAX_VIS = 8;
    ROW_H = 46;
  } else {
    SW = 800;
    SH = 480;
    MAX_VIS = 6;
    ROW_H = 56;
  }
  HDR_H = 60;
  LIST_Y = HDR_H;
  FTR_Y = HDR_H + ROW_H * MAX_VIS;
  FTR_H = SH - FTR_Y;
}

// ══════════════════════════════════════════════
//  CONFIGURAZIONE
// ══════════════════════════════════════════════
const int  RSSI_FOUND_ENTER = -40;   // dBm ingresso FOUND
const int  RSSI_FOUND_EXIT  = -48;   // dBm uscita  FOUND
const int  FOUND_CONSEC     =  5;    // letture consecutive
const bool INVERT_DIR       = true;  // inverte frecce se errate
const int  SECTORS          = 36;    // settori 10° cad.
const int  MIN_SECTORS      =  8;    // copertura minima sweep
const int  SCAN_MS_SOFT     = 2500;  // refresh lista scan
const int  SIGNAL_TIMEOUT   = 6000;  // ms reset mappa

// ══════════════════════════════════════════════
//  STATO APP
// ══════════════════════════════════════════════
enum AppState { SCAN_SCR, SWEEP_SCR, TRACK_SCR, FOUND_SCR };
AppState appState = SCAN_SCR;

// ══════════════════════════════════════════════
//  DISPOSITIVI BLE
// ══════════════════════════════════════════════
struct BLEDev { String address, name; int rssi; };
const int MAX_DEV = 30;
BLEDev   devList[MAX_DEV];
int      devCount = 0, selIdx = -1, scrollOff = 0;
String   targetMAC = "", targetName = "";

void upsertDevice(const String& addr, const String& name, int rssi) {
  for (int i = 0; i < devCount; i++) {
    if (devList[i].address == addr) {
      devList[i].rssi = rssi;
      if (name.length() > devList[i].name.length()) devList[i].name = name;
      return;
    }
  }
  if (devCount < MAX_DEV) devList[devCount++] = {addr, name, rssi};
}

void sortByRSSI() {
  for (int i = 0; i < devCount - 1; i++)
    for (int j = 0; j < devCount - i - 1; j++)
      if (devList[j].rssi < devList[j+1].rssi) {
        BLEDev t = devList[j]; devList[j] = devList[j+1]; devList[j+1] = t;
      }
}

// ══════════════════════════════════════════════
//  KALMAN ASIMMETRICO
// ══════════════════════════════════════════════
struct Kalman {
  float q=2, p=10, x=-70;
  float update(float z) {
    p += q;
    float r = (z < x) ? 2.0f : 12.0f;
    float K = p / (p + r);
    x += K * (z - x);
    p *= (1 - K);
    return x;
  }
  void reset() { p=10; x=-70; }
} kalman;

float filteredRSSI = -999;

// ══════════════════════════════════════════════
//  YAW
// ══════════════════════════════════════════════
float yaw = 0;
void updateYaw() {
  imu.update();
  yaw = imu.getYaw();
  yaw = fmodf(yaw, 360.0f);
  if (yaw < 0) yaw += 360.0f;
}

// ══════════════════════════════════════════════
//  MAPPA RSSI × ANGOLO
// ══════════════════════════════════════════════
struct Sector { float avg = -999; int count = 0; };
Sector rssiMap[SECTORS];
int coveredSectors = 0;
unsigned long lastSignalMs = 0;

void resetMap() {
  for (int i = 0; i < SECTORS; i++) rssiMap[i] = {-999, 0};
  coveredSectors = 0;
  kalman.reset();
  filteredRSSI = -999;
}

void updateMap(float angle, float rssi) {
  int s = ((int)(angle / 10.0f) + SECTORS) % SECTORS;
  if (rssiMap[s].count == 0) coveredSectors++;
  rssiMap[s].avg = (rssiMap[s].avg * rssiMap[s].count + rssi) / (rssiMap[s].count + 1);
  rssiMap[s].count++;
}

float getBestAngle() {
  int best = -1; float bR = -999;
  for (int i = 0; i < SECTORS; i++)
    if (rssiMap[i].count > 0 && rssiMap[i].avg > bR) { bR = rssiMap[i].avg; best = i; }
  return best >= 0 ? best * 10.0f + 5.0f : -1.0f;
}

// ══════════════════════════════════════════════
//  ISTERESI FOUND
// ══════════════════════════════════════════════
int foundCount = 0;
void updateFoundCount(float rssi) {
  if      (rssi >= RSSI_FOUND_ENTER) foundCount = min(foundCount + 1, FOUND_CONSEC);
  else if (rssi <  RSSI_FOUND_EXIT)  foundCount = 0;
  else                               foundCount = max(foundCount - 1, 0);
}

// ══════════════════════════════════════════════
//  BUZZER NON-BLOCKING
// ══════════════════════════════════════════════
unsigned long lastBeepAt = 0;
bool beepActive = false;
void updateBuzzer(float rssi) {
  unsigned long now = millis();
  if (rssi < -95) { noTone(BUZZER_PIN); beepActive = false; return; }
  if (rssi >= RSSI_FOUND_ENTER) { tone(BUZZER_PIN, 1800); return; }
  int iv = map((int)constrain(rssi,-90,-52), -90,-52, 2000,150);
  if (beepActive  && now-lastBeepAt >= 60)               { noTone(BUZZER_PIN); beepActive=false; }
  else if (!beepActive && now-lastBeepAt >= (unsigned long)iv) { tone(BUZZER_PIN,1100); lastBeepAt=now; beepActive=true; }
}

// ══════════════════════════════════════════════
//  UTILITY GRAFICHE
// ══════════════════════════════════════════════
uint16_t rssiColor(float r) {
  return r >= -60 ? C_GREEN : r >= -75 ? C_YELLOW : C_RED;
}

void drawRSSIBar(int x, int y, int w, int h, float rssi) {
  display.fillRect(x, y, w, h, C_DGRAY);
  int bw = map((int)constrain(rssi,-95,-35), -95,-35, 0, w);
  if (bw > 0) display.fillRect(x, y, bw, h, rssiColor(rssi));
  display.drawRect(x, y, w, h, C_GRAY);
}

void drawBtn(int x, int y, int w, int h, uint16_t col, const char* txt, int sz=2) {
  display.fillRoundRect(x, y, w, h, 8, col);
  display.setTextColor(C_WHITE); display.setTextSize(sz);
  int tw = strlen(txt)*6*sz, th = 8*sz;
  display.setCursor(x+(w-tw)/2, y+(h-th)/2);
  display.print(txt);
}

// ══════════════════════════════════════════════
//  SCHERMATA 1 — SCAN
// ══════════════════════════════════════════════
void drawScanHeader() {
  display.fillRect(0, 0, SW, HDR_H, C_HDR);
  display.setTextColor(C_WHITE); display.setTextSize(3);
  display.setCursor(12, 15); display.print("BLE Scanner");
  display.setTextColor(C_YELLOW); display.setTextSize(2);
  if (usePortrait) {
    display.setCursor(12, 38); display.print(devCount); display.print(" disp.");
    display.fillRoundRect(340, 12, 44, 36, 6, 0x2945);
    display.setTextColor(C_WHITE); display.setTextSize(2);
    display.setCursor(348, 20); display.print("UP");
    display.fillRoundRect(388, 12, 44, 36, 6, 0x2945);
    display.setCursor(396, 20); display.print("DN");
    drawBtn(210, 12, 260, 36, C_BTN_G, "SCANSIONA");
  } else {
    display.setCursor(310, 22); display.print(devCount); display.print(" dispositivi");
    display.fillRoundRect(548, 12, 44, 36, 6, 0x2945);
    display.setTextColor(C_WHITE); display.setTextSize(2);
    display.setCursor(556, 20); display.print("UP");
    display.fillRoundRect(598, 12, 44, 36, 6, 0x2945);
    display.setCursor(606, 20); display.print("DN");
    drawBtn(650, 12, 138, 36, C_BTN_G, "SCANSIONA");
  }
}

void drawScanRow(int rowPos, int dIdx) {
  int y = LIST_Y + rowPos * ROW_H;
  bool sel = (dIdx == selIdx);
  display.fillRect(0, y, SW, ROW_H-1, sel ? C_SEL_BG : (rowPos%2==0 ? C_ROW_A : C_ROW_B));
  if (dIdx < 0 || dIdx >= devCount) return;
  BLEDev& d = devList[dIdx];
  if (sel) display.fillCircle(18, y+ROW_H/2, 8, C_BTN_G);
  else     display.drawCircle(18, y+ROW_H/2, 8, C_GRAY);
  String lbl = d.name.length()>0 ? d.name : "(no name)";

  if (usePortrait) {
    if ((int)lbl.length()>18) lbl = lbl.substring(0,18)+"..";
    display.setTextColor(d.name.length()>0 ? C_CYAN : C_GRAY);
    display.setTextSize(1); display.setCursor(34, y+6); display.print(lbl);
    display.setTextColor(C_GRAY); display.setTextSize(1);
    display.setCursor(34, y+20); display.print(d.address);
    drawRSSIBar(34, y+30, 300, 10, d.rssi);
    char buf[12]; sprintf(buf, "%d dBm", d.rssi);
    display.setTextColor(rssiColor(d.rssi)); display.setTextSize(1);
    display.setCursor(340, y+31); display.print(buf);
  } else {
    if ((int)lbl.length()>24) lbl = lbl.substring(0,24)+"..";
    display.setTextColor(d.name.length()>0 ? C_CYAN : C_GRAY);
    display.setTextSize(2); display.setCursor(34, y+6); display.print(lbl);
    display.setTextColor(C_GRAY); display.setTextSize(1);
    display.setCursor(34, y+38); display.print(d.address);
    drawRSSIBar(520, y+14, 160, 14, d.rssi);
    char buf[12]; sprintf(buf, "%d dBm", d.rssi);
    display.setTextColor(rssiColor(d.rssi)); display.setTextSize(2);
    display.setCursor(680, y+18); display.print(buf);  // Spostato a sinistra per evitare overflow
  }
}

void drawScanList() {
  for (int r=0; r<MAX_VIS; r++) { int d=scrollOff+r; drawScanRow(r, d<devCount?d:-1); }
}

void drawScanFooter() {
  display.fillRect(0, FTR_Y, SW, FTR_H, C_FTR);
  display.drawLine(0, FTR_Y, SW, FTR_Y, 0x2945);
  display.setTextSize(2);

  int btnX, btnY, btnW, btnH;
  int msgX, msgY;

  if (usePortrait) {
    btnX = 20;
    btnY = FTR_Y + 10;
    btnW = 440;
    btnH = 50;
    msgX = 10;
    msgY = FTR_Y + 70;
  } else {
    btnX = 498;
    btnY = FTR_Y + 14;
    btnW = 290;
    btnH = 54;
    msgX = 10;
    msgY = FTR_Y + 28;
  }

  if (selIdx>=0 && selIdx<devCount) {
    String n = devList[selIdx].name.length()>0 ? devList[selIdx].name : devList[selIdx].address;
    if ((int)n.length()>28) n=n.substring(0,28)+"..";
    display.setTextColor(C_YELLOW); display.setCursor(msgX, msgY);
    display.print("> "); display.print(n);
    drawBtn(btnX, btnY, btnW, btnH, C_BTN_B, "INIZIA RICERCA  >");
  } else {
    display.setTextColor(C_GRAY); display.setCursor(msgX, msgY);
    if (usePortrait) {
      display.setTextSize(1);
      display.print("Tocca un dispositivo");
    } else {
      display.print("Tocca un dispositivo per selezionarlo");
    }
    drawBtn(btnX, btnY, btnW, btnH, C_GRAY, "INIZIA RICERCA  >");
  }
}

void drawScanScreen() {
  drawScanHeader(); drawScanList(); drawScanFooter();
}

// ══════════════════════════════════════════════
//  SCHERMATA 2 — SWEEP
// ══════════════════════════════════════════════
void drawSweepScreen() {
  display.fillRect(0, 0, SW, HDR_H, C_HDR);
  display.setTextColor(C_WHITE); display.setTextSize(3);
  display.setCursor(12, 15); display.print("Fase 1 — Mappatura");
  drawBtn(SW-142, 12, 136, 36, C_BTN_R, "<- TORNA");
  display.fillRect(0, HDR_H, SW, SH-HDR_H, C_BLACK);

  display.setTextColor(C_WHITE); display.setTextSize(2);
  display.setCursor(30, 100); display.print("Ruota lentamente il dispositivo di ~180 gradi");

  int pctW = map(coveredSectors, 0, MIN_SECTORS, 0, SW-200);
  display.fillRect(100, 180, SW-200, 44, C_DGRAY);
  display.fillRect(100, 180, constrain(pctW,0,SW-200), 44, C_DGREEN);
  display.drawRect(100, 180, SW-200, 44, C_GRAY);
  int pct = constrain(map(coveredSectors, 0, MIN_SECTORS, 0, 100), 0, 100);
  char pb[8]; sprintf(pb, "%d%%", pct);
  display.setTextColor(C_WHITE); display.setTextSize(3);
  display.setCursor(SW/2-20, 187); display.print(pb);

  String tgt = targetName.length()>0 ? targetName : targetMAC;
  if ((int)tgt.length()>30) tgt = tgt.substring(0,30)+"..";
  display.setTextColor(C_CYAN); display.setTextSize(2);
  display.setCursor(30, 260); display.print("Target: "); display.print(tgt);

  char rb[24]; sprintf(rb, "RSSI: %.0f dBm", filteredRSSI);
  display.setTextColor(rssiColor(filteredRSSI));
  display.setCursor(30, 295); display.print(rb);

  drawRSSIBar(30, 325, SW-60, 20, filteredRSSI);

  char yb[24]; sprintf(yb, "Angolo corrente: %.1f deg", yaw);
  display.setTextColor(C_GRAY); display.setCursor(30, 360); display.print(yb);
}

// ══════════════════════════════════════════════
//  SCHERMATA 3 — TRACK
// ══════════════════════════════════════════════

void drawArrowArea(float delta) {
  int arrowW = usePortrait ? 300 : 556;
  display.fillRect(0, HDR_H, arrowW, SH-HDR_H, C_BLACK);
  uint16_t col = rssiColor(filteredRSSI);
  int cx=arrowW/2, cy=SH/2-20;

  if (fabsf(delta) < 25) {
    // Cerchio bersaglio
    display.drawCircle(cx, cy, 115, col);
    display.drawCircle(cx, cy,  75, col);
    display.drawCircle(cx, cy,  35, col);
    display.fillCircle(cx, cy,  12, col);
    display.drawLine(cx-125,cy,cx-120,cy,col); display.drawLine(cx+120,cy,cx+125,cy,col);
    display.drawLine(cx,cy-125,cx,cy-120,col); display.drawLine(cx,cy+120,cx,cy+125,col);
    display.setTextColor(col); display.setTextSize(2);
    display.setCursor(cx-100, cy+135); display.print("DIREZIONE CORRETTA");
  } else if (delta > 0) {
    // Freccia destra
    display.fillRect(60, cy-22, arrowW-120, 44, col);
    display.fillTriangle(arrowW-80,cy-100, arrowW-10,cy, arrowW-80,cy+100, col);
    display.setTextColor(col); display.setTextSize(2);
    display.setCursor(cx-80, cy+130); display.print("GIRA A DESTRA");
  } else {
    // Freccia sinistra
    display.fillRect(60, cy-22, arrowW-120, 44, col);
    display.fillTriangle(60,cy-100, -10,cy, 60,cy+100, col);
    display.setTextColor(col); display.setTextSize(2);
    display.setCursor(cx-80, cy+130); display.print("GIRA A SINISTRA");
  }
}

void drawInfoPanel(float delta, float bestAngle) {
  int panelX = usePortrait ? 0 : 560;
  int panelW = usePortrait ? SW : SW-560;
  display.fillRect(panelX, HDR_H, panelW, SH-HDR_H, C_PANEL);
  display.drawLine(panelX, HDR_H, panelX, SH, 0x2945);

  int ix=panelX+14, iy=HDR_H+18;

  String tgt = targetName.length()>0 ? targetName : targetMAC;
  if ((int)tgt.length()>14) tgt = tgt.substring(0,14)+"..";
  display.setTextColor(C_CYAN); display.setTextSize(2);
  display.setCursor(ix,iy); display.print(tgt); iy+=38;

  char rb[20]; sprintf(rb,"RSSI: %d dBm",(int)filteredRSSI);
  display.setTextColor(rssiColor(filteredRSSI));
  display.setCursor(ix,iy); display.print(rb); iy+=28;
  drawRSSIBar(ix, iy, panelW-28, 18, filteredRSSI); iy+=38;

  display.setTextColor(C_GRAY); display.setTextSize(2);
  char yb[20]; sprintf(yb,"Yaw:  %.1f",yaw);
  display.setCursor(ix,iy); display.print(yb); iy+=28;
  sprintf(yb,"Best: %.1f",bestAngle);
  display.setCursor(ix,iy); display.print(yb); iy+=28;
  sprintf(yb,"Delta:%.1f",delta);
  display.setTextColor(fabsf(delta)<25 ? C_GREEN : C_YELLOW);
  display.setCursor(ix,iy); display.print(yb); iy+=28;

  char cb[20]; sprintf(cb,"Cop: %d/36",coveredSectors);
  display.setTextColor(C_GRAY);
  display.setCursor(ix,iy); display.print(cb); iy+=50;

  int btnX = panelX+14;
  int btnY = iy;
  int btnW = panelW-28;
  drawBtn(btnX, btnY, btnW, 44, C_BTN_R, "<- TORNA");
}

void drawTrackHeader() {
  display.fillRect(0, 0, SW, HDR_H, C_HDR);
  display.setTextColor(C_WHITE); display.setTextSize(3);
  display.setCursor(12,15); display.print("Fase 2 — Tracking");
  display.setTextColor(C_YELLOW); display.setTextSize(2);
  display.setCursor(SW-150,22); display.print("Cov: "); display.print(coveredSectors); display.print("/36");
}

// ══════════════════════════════════════════════
//  SCHERMATA 4 — FOUND
// ══════════════════════════════════════════════
void drawFoundScreen(bool blink) {
  uint16_t bg = blink ? 0x03E0 : C_BLACK;
  uint16_t fg = blink ? C_WHITE : C_GREEN;
  display.fillScreen(bg);
  int cx=SW/2, cy=SH/2;
  display.drawCircle(cx,cy,140,fg); display.drawCircle(cx,cy,95,fg);
  display.drawCircle(cx,cy,48,fg);  display.fillCircle(cx,cy,16,fg);
  display.drawLine(cx-155,cy,cx-150,cy,fg); display.drawLine(cx+150,cy,cx+155,cy,fg);
  display.drawLine(cx,cy-155,cx,cy-150,fg); display.drawLine(cx,cy+150,cx,cy+155,fg);
  display.setTextColor(fg); display.setTextSize(5);
  display.setCursor(SW/2-100,55); display.print("TROVATO!");
  String tgt = targetName.length()>0 ? targetName : targetMAC;
  display.setTextSize(2); display.setCursor(SW/2-80,SH-60); display.print(tgt);
  drawBtn(10, SH-54, 160, 44, C_BTN_R, "<- TORNA");
}

// ══════════════════════════════════════════════
//  TOUCH HANDLER (Callback-based)
// ══════════════════════════════════════════════
unsigned long lastTouchMs = 0;
volatile bool touchPending = false;
volatile int touchX = 0, touchY = 0;

void touchCB(uint8_t contacts, GDTpoint_t* points) {
  if (contacts > 0) {
    touchX = (int)points[0].x;
    touchY = (int)points[0].y;
    touchPending = true;
    Serial.print("[TOUCH] X="); Serial.print(touchX);
    Serial.print(" Y="); Serial.print(touchY);
    Serial.print(" AppState="); Serial.println(appState);
  }
}

void goToScan() {
  noTone(BUZZER_PIN); beepActive=false;
  resetMap(); foundCount=0;
  BLE.stopScan();
  appState = SCAN_SCR;
  display.fillScreen(C_BLACK);
  BLE.scan(true);
  drawScanScreen();
}

void handleTouch(int tx, int ty) {
  if (millis() - lastTouchMs < 160) return;
  lastTouchMs = millis();

  // ── SCAN ──────────────────────────────────
  if (appState == SCAN_SCR) {
    // SCANSIONA button
    int scanBtnX = usePortrait ? 210 : 650;
    int scanBtnY = 12;
    int scanBtnW = usePortrait ? 260 : 138;
    int scanBtnH = 36;

    if (ty>=scanBtnY && ty<scanBtnY+scanBtnH && tx>=scanBtnX && tx<scanBtnX+scanBtnW) {
      devCount=0; selIdx=-1; scrollOff=0;
      BLE.stopScan(); BLE.scan(true); drawScanScreen(); return;
    }

    // UP scroll button
    int upBtnX = usePortrait ? 340 : 548;
    if (ty<HDR_H && tx>=upBtnX && tx<upBtnX+44) {
      if (scrollOff>0) { scrollOff--; drawScanList(); } return;
    }

    // DN scroll button
    int dnBtnX = usePortrait ? 388 : 598;
    if (ty<HDR_H && tx>=dnBtnX && tx<dnBtnX+44) {
      if (scrollOff+MAX_VIS<devCount) { scrollOff++; drawScanList(); } return;
    }

    // Device row selection
    if (ty>=LIST_Y && ty<FTR_Y) {
      int d = scrollOff+(ty-LIST_Y)/ROW_H;
      if (d<devCount) { selIdx=d; drawScanList(); drawScanFooter(); } return;
    }

    // INIZIA RICERCA button — Fixed coordinates
    int btnX = usePortrait ? 20 : 498;
    int btnY = FTR_Y + 14;
    int btnW = usePortrait ? 440 : 290;
    int btnH = 54;

    Serial.print("[BUTTON CHECK] ty="); Serial.print(ty); Serial.print(" btnY="); Serial.print(btnY);
    Serial.print(" btnY+H="); Serial.print(btnY+btnH); Serial.print(" tx="); Serial.print(tx);
    Serial.print(" btnX="); Serial.print(btnX); Serial.print(" btnX+W="); Serial.print(btnX+btnW);
    Serial.print(" selIdx="); Serial.println(selIdx);

    if (ty>=btnY && ty<btnY+btnH && tx>=btnX && tx<btnX+btnW && selIdx>=0) {
      Serial.println("[ACTION] INIZIA RICERCA clicked!");
      targetMAC  = devList[selIdx].address;
      targetName = devList[selIdx].name;
      resetMap(); BLE.stopScan();
      appState = SWEEP_SCR;
      display.fillScreen(C_BLACK);
      BLE.scan(true);
      drawSweepScreen(); return;
    }
  }

  // ── TORNA (da qualsiasi altra schermata) ──
  bool back = false;

  // SWEEP: "<- TORNA" button at top-right
  if (appState==SWEEP_SCR && ty>=12 && ty<48 && tx>=SW-142 && tx<SW-6) back=true;

  // TRACK: "<- TORNA" button in info panel
  if (appState==TRACK_SCR && ty>SH-60 && ty<SH-14 && tx>=574 && tx<574+212) back=true;

  // FOUND: "<- TORNA" button at bottom-left
  if (appState==FOUND_SCR && ty>=SH-54 && ty<SH-10 && tx>=10 && tx<170) back=true;

  if (back) goToScan();
}

// ══════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);

  display.begin();
  touch.begin();
  touch.onDetect(touchCB);

  updateLayoutDimensions();

  display.fillScreen(C_BLACK);
  display.setTextColor(C_WHITE); display.setTextSize(2);
  display.setCursor(260,230); display.print("Avvio in corso...");

  Modulino.begin();
  if (!imu.begin()) {
    display.setTextColor(C_ORANGE); display.setCursor(180,270);
    display.print("Modulino Movement non trovato");
    display.setCursor(180,300); display.print("Continuo senza giroscopio...");
    delay(2000);
  }

  display.setCursor(260,270); display.setTextColor(C_YELLOW);
  display.print("Avvio BLE...");

  int attempt=0;
  while (!BLE.begin()) {
    attempt++;
    display.fillRect(0,300,SW,30,C_BLACK);
    display.setTextColor(C_YELLOW); display.setTextSize(2);
    display.setCursor(260,305);
    display.print("Tentativo "); display.print(attempt); display.print("...");
    if (attempt>=8) {
      display.setTextColor(C_RED); display.setCursor(180,340);
      display.print("BLE FALLITO — riavvia la scheda");
      while(1);
    }
    delay(2000);
  }

  display.fillScreen(C_BLACK);
  BLE.scan(true);
  drawScanScreen();
  Serial.println("Giga Direction Finder pronto");
}

// ══════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════
unsigned long lastScanRedraw  = 0;
unsigned long lastSweepRedraw = 0;
unsigned long lastTrackRedraw = 0;
unsigned long lastFoundBlink  = 0;
bool          foundBlink      = false;
int           lastDeltaBin    = 99;   // 0=centro, 1=destra, -1=sinistra

void loop() {
  // ── IMU ──────────────────────────────────
  updateYaw();

  // ── Touch (callback-based) ───────────────
  if (touchPending) {
    handleTouch(touchX, touchY);
    touchPending = false;
  }

  // ── BLE ──────────────────────────────────
  BLEDevice dev = BLE.available();
  if (dev) {
    String addr = String(dev.address());
    String name = String(dev.localName());
    int    rssi = dev.rssi();

    if (appState == SCAN_SCR) {
      upsertDevice(addr, name, rssi);
    } else {
      bool match = targetMAC.length()>0
                   ? addr.equalsIgnoreCase(targetMAC)
                   : true;
      if (match) {
        if (name.length()>0 && targetName.length()==0) targetName=name;
        filteredRSSI = kalman.update((float)rssi);
        updateMap(yaw, filteredRSSI);
        updateBuzzer(filteredRSSI);
        updateFoundCount(filteredRSSI);
        lastSignalMs = millis();
      }
    }
  }

  unsigned long now = millis();

  // ── Reset mappa se segnale assente ───────
  if (appState!=SCAN_SCR && now-lastSignalMs>SIGNAL_TIMEOUT) {
    resetMap(); foundCount=0;
    Serial.println("[RESET] Segnale perso");
  }

  // ════════════════════════════════════════
  //  LOGICA SCHERMATE
  // ════════════════════════════════════════

  if (appState == SCAN_SCR) {
    if (now - lastScanRedraw > SCAN_MS_SOFT) {
      sortByRSSI(); drawScanScreen(); lastScanRedraw=now;
    }
    return;
  }

  // ── FOUND ha priorità su tutto ───────────
  if (foundCount >= FOUND_CONSEC) {
    if (appState != FOUND_SCR) {
      appState = FOUND_SCR;
      display.fillScreen(C_BLACK);
      lastFoundBlink = 0;
    }
    if (now - lastFoundBlink > 500) {
      foundBlink = !foundBlink;
      drawFoundScreen(foundBlink);
      lastFoundBlink = now;
    }
    return;
  }

  // Se si esce da FOUND, torna a TRACK o SWEEP
  if (appState == FOUND_SCR) {
    appState = coveredSectors>=MIN_SECTORS ? TRACK_SCR : SWEEP_SCR;
    display.fillScreen(C_BLACK);
    if (appState==TRACK_SCR) drawTrackHeader();
    lastDeltaBin = 99;
  }

  // ── SWEEP ────────────────────────────────
  if (appState == SWEEP_SCR) {
    if (now - lastSweepRedraw > 500) {
      drawSweepScreen(); lastSweepRedraw=now;
    }
    if (coveredSectors >= MIN_SECTORS) {
      appState = TRACK_SCR;
      display.fillScreen(C_BLACK);
      drawTrackHeader();
      lastDeltaBin = 99;
    }
    return;
  }

  // ── TRACK ────────────────────────────────
  if (appState == TRACK_SCR) {
    float bestAngle = getBestAngle();
    if (bestAngle < 0) return;

    float delta = bestAngle - yaw;
    if (delta >  180) delta -= 360;
    if (delta < -180) delta += 360;
    float d = INVERT_DIR ? -delta : delta;

    // Determina bin direzione (evita ridisegno freccia inutile)
    int bin = fabsf(d)<25 ? 0 : d>0 ? 1 : -1;

    if (now - lastTrackRedraw > 200) {
      if (bin != lastDeltaBin) {          // freccia cambiata → ridisegna
        drawArrowArea(d);
        lastDeltaBin = bin;
      }
      drawInfoPanel(d, bestAngle);        // info panel sempre aggiornato
      lastTrackRedraw = now;
    }
  }
}