# Send_NMEA_COM
Send NMEA sentences to a serial COM port for testing. Primarily for OpenCPN and BR24Radar_pi.
It's a small console program to send NMEA messages to a serial COM port.
Only tested in Windows and built in VS2013.  
Build it by yourself or download the Windows exe file from the Release tab. https://github.com/Hakansv/Send_NMEA_COM/releases

The program searches for free COM-ports and you can choose the one you like.

The following messages are sent:  
$GPRMC,123519,A,5803.246,N,1121.694,E,5.500,282.100,230394,,W*77  
$HCHDM,279.100,M*24  
$IIMTW,14.7,C*11  
$IIDBT,37.9,f,11.5,M,6.3,F*1C  
$VDMWV,44.4,T,9.1,N,A*15  
$VDVHW,,T,279.100,M,5.800,N,,K*47  
$VDMWV,55.3,R,8.5,N,A*11  

You have an option to change any of these variables on init:  
Present Navdata:  
 Initial Latitude: 58.0533  
 Initial  Longitude: 11.3683  
 Course: 282.1  
 Variation: 3  
 Speed knots: 5.5  
Any change you make to the initial navdata will be stored in a local file for next time.  
Using above values the initial position will be updated to follow given course at given speed. 
Thus you'll get a moving boat in for example OpenCPN and also test some Dashboard instruments.  
The course can be changed while the program is running.

![console_clip](https://cloud.githubusercontent.com/assets/7202854/20133092/5b613726-a666-11e6-94f3-dbe0f7412c2a.PNG)

Thanks to:  
// Send_NMEA_COM.cpp : Defines the entry point for the console application.  
// A simple serial port writing example  
// Written by Ted Burke - last updated 13-2-2013  
// published http://batchloaf.wordpress.com/2013/02/13/writing-bytes-to-a-serial-port-in-c/  
// Adapted for NMEA simulation by Douwe Fokkema 11-11-2014  
// Adapted with moving position and NMEA chechsum by Håkan Svensson 2014-11-20  
// Added routine to find a COM port and more NMEA strings by Håkan 2016-06-09  
// Functions for user input of Nav-Data. Håkan 2016-11-08  
