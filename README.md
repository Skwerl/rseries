Skwerl's R-Series Control
=======

Arduino Controller &amp; Receiver for R2-D2

Controller Hardware Details: Coming Soon! Custom shield for Arduino Fio v3, in prototype phase.  
Receiver Hardware Details: https://code.google.com/p/rseries-open-control/

Controller requires patch to XBee library. As documented [here](http://forum.arduino.cc/index.php?topic=111354.0 "Serial compile issue"), in XBee.cpp, the following code:

`_serial = &Serial;`

Needs to be replaced with:

```
#if defined(USBCON)
  _serial = &Serial1;
#else
  _serial = &Serial;
#endif
```
