#[Chronometer](https://plus.google.com/+YingYin/posts/DJm4mcUZyS3?pid=6090191433786061538&oid=108412339293912106370)
##Components
###[Real Time Clock](https://learn.adafruit.com/ds1307-real-time-clock-breakout-board-kit/understanding-the-code)
* The RTC chip has a battery. Even if the Arduino is powered off, it is still running. To reset the RTC chip, remove the battery from the holder while the Arduino is not powered or plugged into USB. Wait 3 seconds and then replace the battery.
* Whenever the RTC chip loses all power, it won't count seconds (its stopped). The clock starts ticking when you set the time. You can use `RTC.adjust` method to set the time. The following line takes the Date and Time according the computer you're using (right when you compile the code) and uses that to set the time of RTC. Then you must press the Upload button to compile and then immediately upload. If you compile and then upload   later, the clock will be off by that amount of time.
```c
  // following line sets the RTC to the date & time this sketch was compiled
  RTC.adjust(DateTime(__DATE__, __TIME__));
```

###[4D Systems Intelligent Display Module (uLCD-43PT)](http://www.4dsystems.com.au/group/4D_Intelligent_Display_Modules/)
4.3" Intelligent LCD module with resistive touch. Use [Workshop4 IDE](http://www.4dsystems.com.au/product/4D_Workshop_4_IDE/) for making the LCD display GUI.
