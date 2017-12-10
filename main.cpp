#include <stdio.h>
#include <iostream>
#include "ILI9341.h"
#include <opencv2/opencv.hpp>
#include <ctime>

using namespace std;
using namespace cv;

int main(int argc, char **argv) 
{
 double t0,t1,t2;
 bool bPress = false; // 記錄快門按下狀態
 int result = LCD_Inital(); 

 switch(result){ // 顯示初始化錯誤訊息
   case 0:   
          printf("LCD inital OK.\n");
          break;
   case 1:
          printf("BCM2835 inital failed. Are you running as root??\n");
          break;
   case 2:
          printf("GPIO inital failed. Are you running as root??\n");
          break;
   case 3:
          printf("SPI inital failed. Are you running as root??\n");
          break;
   case 4:
          printf("ILI9341 inital failed. Please check PCB connection is OK.\n");
          break;
   default:
          printf("LCD inital failed.\n");
 }
 
 if(result != 0)  
   return 0;

  VideoCapture cap(0); // 啟動攝像頭連續取像

  if (!cap.isOpened()) { // 若無法開啟則結束
    cerr << "ERROR: Unable to open the camera" << endl;
    return 0;
  }
 
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 320); // 設定攝像頭輸入解析度為320*240
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

  Mat frame;

  cout << "Start grabbing !" << endl;
  LCD_SetHorizontalDisplay(); // 設定LCD為橫式顯示

  char strFps[10]; // 儲存速度字串
  t1 = (double)getTickCount(); // 取得目前時間

  while(1) {
    t0 = t1; // 儲存舊時間
    t1 = (double)getTickCount(); // 儲存目前時間

    // 若快門鍵(SW1)按下則閃光燈(LED1)點亮
    if(bcm2835_gpio_lev(PIN_GPIO_SW1)!=0){
      bPress = true;
      bcm2835_gpio_write(PIN_GPIO_LED_R, 1); // 開啟閃光燈
      delay(30); // 等待取到有打光的影像
    }
  
    cap >> frame; // 將取得的影像複製到frame

    // 計算兩次執行時間差 取倒數即為每秒幀數 顯示在LCD左上角 
    // 若不需顯示則註解掉下面兩行
    sprintf(strFps,"%2.1f FPS", 1.0 / ((t1-t0) / getTickFrequency()));     
    putText(frame, strFps, Point(20,40), FONT_HERSHEY_DUPLEX, 0.8, Scalar(0,255,0),1);

//    // 測試按鍵SW1 SW2是否正常 並顯示在LCD上
//    if(bcm2835_gpio_lev(PIN_GPIO_SW1)!=0)
//      putText(frame, "SW1 ON ", Point (20,70), FONT_HERSHEY_DUPLEX,1, Scalar(0,255,0),1);
//    else
//      putText(frame, "SW1 OFF", Point (20,70), FONT_HERSHEY_DUPLEX,1, Scalar(0,255,0),1);

//    if(bcm2835_gpio_lev(PIN_GPIO_SW2)!=0)
//      putText(frame, "SW2 ON ", Point (20,100), FONT_HERSHEY_DUPLEX,1, Scalar(0,255,0),1);
//    else
//      putText(frame, "SW2 OFF", Point (20,100), FONT_HERSHEY_DUPLEX,1, Scalar(0,255,0),1);

    // 將取得影像逐行顯示在LCD上
    for(int i = 0; i < frame.rows; i++){  
      char *ptrS = frame.ptr<char>(i);

      ILI9341_WriteLineBGR2RGB565(ptrS, frame.cols);
    } 

    // 若無法取得影像則結束程式
    if (frame.empty()) {
        cerr << "ERROR: Unable to grab from the camera" << endl;
        break;
    }
    
    // 若快門鍵有按過則清除按下旗標 關閉閃光燈 並存檔
    if(bPress){
      bPress = false;
      bcm2835_gpio_write(PIN_GPIO_LED_R, 0); // 關閉閃光燈

      char file_name[30];
      time_t now = time(0);
      tm *ltm = localtime(&now);
  
      sprintf(file_name,"%02d%02d%02d.jpg",1+ltm->tm_hour,1+ltm->tm_min,1+ltm->tm_sec); // 以時間當作檔名
      imwrite(file_name,frame); // 儲存影像
      delay(1000); // 畫面暫停一秒
    }

    // 若SW2按下則綠燈閃一下結束程式
    if(bcm2835_gpio_lev(PIN_GPIO_SW2) != 0){
       bcm2835_gpio_write(PIN_GPIO_LED_G, 1);
       delay(50);
       bcm2835_gpio_write(PIN_GPIO_LED_G, 0);
       break;
    }
  }
 
  cap.release();
  BCM2835_End();
  cout << "Done!" <<endl;
  return 0;
}