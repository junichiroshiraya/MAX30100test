#include <Arduino.h> // Arduinoライブラリをインクルード
#include <M5unified.h> // M5Stackライブラリをインクルード
#include <Wire.h> // I2C通信ライブラリをインクルード
#include <WiFi.h> // WiFiライブラリをインクルード
#include <HTTPClient.h> // HTTPクライアントライブラリをインクルード
#include "MAX30100.h" // MAX30100センサライブラリをインクルード

// Sampling is tightly related to the dynamic range of the ADC.
// refer to the datasheet for further info
#define SAMPLING_RATE MAX30100_SAMPRATE_100HZ // サンプリングレートを100Hzに設定

// The LEDs currents must be set to a level that avoids clipping and maximises the
// dynamic range
#define IR_LED_CURRENT MAX30100_LED_CURR_50MA // 赤外線LEDの電流を50mAに設定
#define RED_LED_CURRENT MAX30100_LED_CURR_27_1MA // 赤色LEDの電流を27.1mAに設定

// The pulse width of the LEDs driving determines the resolution of
// the ADC (which is a Sigma-Delta).
// set HIGHRES_MODE to true only when setting PULSE_WIDTH to MAX30100_SPC_PW_1600US_16BITS
#define PULSE_WIDTH MAX30100_SPC_PW_1600US_16BITS // パルス幅を1600usに設定
#define HIGHRES_MODE true // 高解像度モードを有効に設定

// Instantiate a MAX30100 sensor class
MAX30100 sensor; // MAX30100センサのインスタンスを作成

// display: 240 x 135
#define X 240 // ディスプレイの幅を240に設定
#define Y 135 // ディスプレイの高さを135に設定
#define N 256 // データ配列のサイズを256に設定
#define Nbit 8 // ビット数を8に設定

uint16_t val_red[N], val_ir[N]; // 赤色と赤外線のデータ配列を定義
uint8_t px = 0; // データ配列のインデックスを初期化

#define Navg_base 200 // 基準値の平均化に使用するデータ数を200に設定
#define Navg_data 10 // データの平均化に使用するデータ数を10に設定
#define Navg_slope 10 // 傾きの平均化に使用するデータ数を10に設定
int data0 = 0; // 前回のデータを初期化
uint8_t fRamp = 0, fRamp0 = 0; // 傾きのフラグを初期化
int slope[Navg_slope]; // 傾きのデータ配列を定義
uint8_t pSlope = 0; // 傾きのデータ配列のインデックスを初期化
uint16_t tNP = 0, tPN = 0, tPN0 = 0, periodPN = 0, periodNP = 0; // 時間と周期を初期化
uint8_t fValid = 0; // 有効データのフラグを初期化
uint32_t sum_base = 0, sum_data = 0; // 基準値とデータの合計を初期化
long sum_slope = 0; // 傾きの合計を初期化

uint8_t f = 0; // 描画フラグを初期化

bool isOn = false; // センサの状態を初期化

// Google Sheets
char *ssid = "ifdl"; // WiFiのSSIDを設定
char *password = "hogeupip5"; // WiFiのパスワードを設定
const char* published_url = "https://script.google.com/macros/s/AKfycbzapF-EfOt_K34wxz9O-4fD5xoaAAFShaLQ6UTJSYmJVDDi3t-Qz9awCJgXJoMLVrOm/exec"; // Google SheetsのURLを設定
#define pubPeriod 300 // データ送信の周期を300に設定
int pubData[pubPeriod]; // データ送信用の配列を定義
String pubMessage; // データ送信用のメッセージを定義
int pubPx = 0; // データ送信用のインデックスを初期化

// Difine for calculate differencial
#define AVE_WIDTH 10
float sum1, sum2, sum3;
float original[pubPeriod];
float average1[pubPeriod], diff1[pubPeriod];
float average2[pubPeriod], diff2[pubPeriod];
float average3[pubPeriod];

void sensorSetup(); // センサのセットアップ関数を宣言
void connectWifi(); // WiFi接続関数を宣言
void sendToGoogleSheets(); // Google Sheets送信関数を宣言

void setup()
{
  M5.begin(); // M5Stackの初期化
  connectWifi(); // WiFiに接続
  sensorSetup(); // センサのセットアップ
  M5.Lcd.clear(); // ディスプレイをクリア
  for (uint16_t x = 0; x < N; x++)
  {
    val_red[x] = 0; // 赤色データ配列を初期化
    val_ir[x] = 0; // 赤外線データ配列を初期化
  }
}

void loop()
{
  M5.update(); // M5Stackの状態を更新
  uint16_t ir, red; // 赤外線と赤色のデータを定義
  
  if (M5.BtnA.wasPressed())
  {
    isOn = !isOn; // センサの状態を切り替え
    if (isOn)
    {
      M5.Lcd.clear(); // ディスプレイをクリア
      px = 0; // データ配列のインデックスを初期化
      pubPx = 0; // データ送信用のインデックスを初期化
      sensorSetup(); // センサのセットアップ
    } else {
      Serial.println("Stopped"); // シリアルモニタに停止メッセージを表示
      sendToGoogleSheets(); // Google Sheetsにデータを送信
    }
  }

  sensor.update(); // センサの状態を更新
  while (isOn && sensor.getRawValues(&ir, &red))
  {
    if(pubPx++ == pubPeriod)
    {
      Serial.println("Collected 3000 data"); // シリアルモニタにデータ収集完了メッセージを表示
      sendToGoogleSheets(); // Google Sheetsにデータを送信
      isOn = false; // センサの状態をオフに設定
      break; // ループを抜ける
    }

    val_red[px] = red; // 赤色データ配列にデータを追加
    val_ir[px] = ir; // 赤外線データ配列にデータを追加

    /////////////////////////////
    original[px] = static_cast<float>(red);
    if(px >= AVE_WIDTH) {		//px-AVE_WIDTH
      sum1=0;
      for(int i = px-AVE_WIDTH; i<= px; i++) {
        sum1 += original[i];
      }
      average1[px] = sum1 / (AVE_WIDTH+1);
    }
    if(px >=  AVE_WIDTH+1) {
      diff1[px] = average1[px] - average1[px-1];
    }
    if(px >= AVE_WIDTH*2+1) {		//px-AVE_WIDTH
      sum2=0;
      for(int i = px-AVE_WIDTH; i<= px; i++) {
        sum2 += diff1[i];
      }
      average2[px] = sum2 / (AVE_WIDTH+1);
    }
    if(px >=  AVE_WIDTH*2+2) {
      diff2[px] = average2[px] - average2[px-1];
    }
    if(px >= AVE_WIDTH*3+2) {		//px-AVE_WIDTH
      sum3=0;
      for(int i = px-AVE_WIDTH; i<= px; i++) {
        sum3 += diff2[i];
      }
      average3[px] = sum3 / (AVE_WIDTH+1);
    }
    /////////////////////////////

    uint16_t px0;
    sum_base = 0; // 基準値の合計を初期化
    px0 = (X + px - Navg_base - 1) % X; // 基準値のインデックスを計算
    for (uint8_t i = 0; i < Navg_base; i++)
      sum_base += val_red[(px0 + i) % X]; // 基準値の合計を計算
    sum_data = 0; // データの合計を初期化
    px0 = (X + px - Navg_data - 1) % X; // データのインデックスを計算
    for (uint8_t i = 0; i < Navg_data; i++)
      sum_data += val_red[(px0 + i) % X]; // データの合計を計算
    uint16_t base = sum_base / Navg_base; // 基準値の平均を計算
    int data = sum_data / Navg_data - base; // データの平均から基準値を引く
    slope[pSlope] = data - data0; // 傾きを計算
    pSlope = (pSlope + 1) % Navg_slope; // 傾きのインデックスを更新
    sum_slope = 0; // 傾きの合計を初期化
    for (uint8_t i = 0; i < Navg_slope; i++)
      sum_slope += slope[i]; // 傾きの合計を計算
    int avg_slope = sum_slope / Navg_slope; // 傾きの平均を計算
    if (avg_slope > 0) fRamp = 1; else fRamp = 0; // 傾きのフラグを更新

    if (fRamp0 == 0 && fRamp == 1)
      tNP = millis(); // 傾きが正から負に変わった時の時間を記録
    else if (fRamp0 == 1 && fRamp == 0){
        tPN = millis(); // 傾きが負から正に変わった時の時間を記録
        periodPN = tPN - tNP; // 正から負への周期を計算
        periodNP = tNP - tPN0; // 負から正への周期を計算
        tPN0 = tPN; // 時間を更新
    }
    fRamp0 = fRamp; // 傾きのフラグを更新
    data0 = data; // データを更新
    if (periodPN > 400 && periodPN < 800 && periodNP > 100 && periodNP < 300)
      fValid = 1; // データが有効かどうかを判定
    else
      fValid = 0; // データが無効
    //Serial.printf("%d %d %d %d %d %d %d\n", pubPx, millis(), red, base, data, periodPN, periodNP); // シリアルモニタにデータを表示
    //    printf(">red:%d\n", val_red[px]);
    //    printf(">max:%d\n", max_red);
    //    printf(">min:%d\n", min_red);

    // Start of drawing
    f++;
    if (f == 4) // draw every 4 samples
    {
      f = 0; // 描画フラグを初期化
      uint16_t min_red = 65535, min_ir = 65535, max_red = 0, max_ir = 0; // 最小値と最大値を初期化
      for (uint8_t x = 0; x < X; x++)
      {
        if (val_red[x] > max_red)
          max_red = val_red[x]; // 赤色データの最大値を更新
        if (val_red[x] < min_red)
          min_red = val_red[x]; // 赤色データの最小値を更新
        if (val_ir[x] > max_ir)
          max_ir = val_ir[x]; // 赤外線データの最大値を更新
        if (val_ir[x] < min_ir)
          min_ir = val_ir[x]; // 赤外線データの最小値を更新
      }
      uint16_t mag_red, mag_ir;
      if ((max_red - min_red) < Y)
        mag_red = 1; // 赤色データの倍率を設定
      else
        mag_red = (max_red - min_red) / Y; // 赤色データの倍率を計算
      if ((max_ir - min_ir) < Y)
        mag_ir = 1; // 赤外線データの倍率を設定
      else
        mag_ir = (max_ir - min_ir) / Y; // 赤外線データの倍率を計算
      uint8_t dx = px; // データ配列のインデックスを設定
      int y_red, y_ir;
      for (uint8_t x = 0; x < X; x++)
      {
        y_red = (val_red[dx] - min_red) / mag_red; // 赤色データのY座標を計算
        if (y_red > Y - 1)
          y_red = Y - 1; // Y座標の上限を設定
        if (y_red < 0)
          y_red = 0; // Y座標の下限を設定
        y_ir = (val_ir[dx] - min_ir) / mag_ir; // 赤外線データのY座標を計算
        if (y_ir > Y - 1)
          y_ir = Y - 1; // Y座標の上限を設定
        if (y_ir < 0)
          y_ir = 0; // Y座標の下限を設定
        M5.Lcd.drawFastHLine(0, x, Y, BLACK); // ディスプレイに水平線を描画
        if (fValid == 1)
          M5.Lcd.drawPixel(Y - y_red, x, WHITE); // データが有効な場合、白色でピクセルを描画
        else
          M5.Lcd.drawPixel(Y - y_red, x, RED); // データが無効な場合、赤色でピクセルを描画
        dx = (dx + 1) % X; // データ配列のインデックスを更新
      }
    }
    // End of drawing
    Serial.printf("%d %d %d %d %f %f %f %f %f\n", px, pubPx, millis(), red, average1[px], diff1[px], average2[px], diff2[px], average3[px]); // シリアルモニタにデータを表示
    pubData[pubPx] = average3[px]; // データ送信用の配列にデータを追加

    px = (px + 1) % X; // データ配列のインデックスを更新
  }
}

void sensorSetup()
{  
  if (!sensor.begin())
  {
    printf("fail\n"); // センサの初期化に失敗した場合、メッセージを表示
    M5.Lcd.print("MAX30100 not found."); // ディスプレイにエラーメッセージを表示
  }
  else
  {
    printf("ok\n"); // センサの初期化に成功した場合、メッセージを表示
  }

  // Set up the wanted parameters
  sensor.setMode(MAX30100_MODE_SPO2_HR); // センサのモードを設定
  sensor.setLedsCurrent(IR_LED_CURRENT, RED_LED_CURRENT); // LEDの電流を設定
  sensor.setLedsPulseWidth(PULSE_WIDTH); // LEDのパルス幅を設定
  sensor.setSamplingRate(SAMPLING_RATE); // サンプリングレートを設定
  sensor.setHighresModeEnabled(HIGHRES_MODE); // 高解像度モードを有効に設定
  sensor.resume(); // センサを再開
}

void connectWifi()
{
  int cnt = 0; // カウンタを初期化
  WiFi.begin(ssid, password); // WiFiに接続
  Serial.println("Connecting to "+String(ssid)); // シリアルモニタに接続メッセージを表示
  while (WiFi.status() != WL_CONNECTED){
    delay(500); // 接続が完了するまで待機
    if(cnt++ > 20) break; // カウンタが20を超えた場合、ループを抜ける
  }
  Serial.println("Connected"); // シリアルモニタに接続完了メッセージを表示
}

void sendToGoogleSheets()
{
  pubMessage = ""; // メッセージを初期化
  for (uint16_t i = 0; i < pubPeriod; i++){
    pubMessage += String(pubData[i]) + ","; // データをメッセージに追加
  }
  Serial.print("HTTP start\n"); // シリアルモニタにHTTP開始メッセージを表示
  HTTPClient http; // HTTPクライアントを作成
  http.begin(published_url); // URLを設定
  int httpCode = http.POST(pubMessage); // データをPOST
  if(httpCode > 0){
    Serial.printf("HTTP Response:%d\n", httpCode); // シリアルモニタにHTTPレスポンスを表示
  }else{
    Serial.printf("HTTP failed, error: %s\n", http.errorToString(httpCode).c_str()); // シリアルモニタにエラーメッセージを表示
  }
  http.end(); // HTTP接続を終了
  WiFi.disconnect(true, true); // WiFi接続を切断
}
