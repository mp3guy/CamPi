CamPi
=====

Small "motion detection" program for the Raspberry Pi with camera module. 

Lots of help from [http://thinkrpi.wordpress.com/2013/05/22/opencv-and-camera-board-csi/](http://thinkrpi.wordpress.com/2013/05/22/opencv-and-camera-board-csi/).

Instructions for setup/building are provided in CamPi.pptx. 

cam.sh is a bash version of the system using built in tools. 

Otherwise, use run.sh to run the program. 

RaspiStillYUV.c contains all of the main code, but overall this is just hacked stock Raspi camera module code. The logic is executed in the video\_buffer\_callback method. 

Magic numbers for configuration are contained around line 47 of RaspiStillYUV.c, in the // Configuration section. 

Requires OpenCV on your Pi. 


<p align="center">
  <img src="http://mp3guy.github.io/img/CamPi.png" alt="CamPi"/>
</p>
