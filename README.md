FioSaber R-Series Control
=======

Arduino Controller &amp; Receiver for R2-D2

Note that there are multiple branches for this repository. I'm currently working on an experimental new controller based on a custom shield I designed for the Arduino Fio. See the **traditional** branch for code for the "standard" R-Series controller/receiver hardware by Michael Erwin & Co. My current work (master branch) uses a custom controller with the Erwin receiver.

***

Controller Hardware Details: Coming Soon! Custom shield for Arduino Fio v3, in prototype phase.  
Receiver Hardware Details: https://code.google.com/p/rseries-open-control/

***

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
