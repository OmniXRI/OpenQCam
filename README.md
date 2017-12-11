這麼多年來總是從網路上找到許多幫助我完成工作的參考資料或是程式片段，但自己卻從未貢獻過，包括連在討論區回覆都沒有，實在有點不好意思。這是我的第一個開源專案，但與其說是開源，不如說是我把網路上找到的一堆破碎可用片段重新整理好並實做出來。此次選擇用中文撰寫，主要是因為網路上許多有用的內容都是用英文寫的不易閱讀，為方便大家，才選用中文撰寫，未來若有時間或許會再加寫英文版本提供更多網友利用，若有網友願意協助翻譯成各國文字，亦非常歡迎連結推廣。

本專案主要利用樹莓派Pi Zero W, 500萬素相機模組, 2.2吋SPI介面QVGA (320*240像素) 解析度TFT LCD, OpenCV (C++版本), 完成一台迷你可愛版數位相機【OpenQCam】，有兩個按鍵及兩個LED，可按鈕拍照、儲存並同步顯示在TFT LCD上，可做為後續開發更進階的小型嵌入式智慧型攝影機、電腦(機器)視覺、人工智慧影像分析應用的軟硬體前置環境建立，大家可依此架構再擴展成自己想要的功能。完成的作品如圖1-1所示。

![](https://github.com/OmniXRI/OpenQCam/blob/master/images/Fig%201-1%20OpenQCam_s.jpg)

圖1-1 開源迷你相機OpenQCam成品圖

更完整說明請參考下列網址：
https://github.com/OmniXRI/OpenQCam/wiki/OpenQCam%E5%B0%88%E6%A1%88%E8%AA%AA%E6%98%8E

本專案是做為後續利用OpenCV進行影像辨識的很好入門平台，程式的相關註解詳見各程式原始碼。受限於個人能力有限，撰文上難免產生誤解或疏漏，如有任何問題歡迎留言或來信指教！

參考文獻及網址

樹莓派基金會  https://www.raspberrypi.org

記憶卡備份Win32 Disk Imager  https://sourceforge.net/projects/win32diskimager/

遠端操控 RealVNC  https://www.realvnc.com

電腦視覺OpenCV  https://opencv.org/

介面控制 BCM2835  http://www.airspayce.com/

LCD驅動IC資料手冊  https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf 

作者：歐尼克斯實境互動工作室 https://omnixri.blogspot.tw/  (Dec. 2017)
