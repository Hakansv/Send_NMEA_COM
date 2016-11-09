// Send_NMEA_COM.cpp : Defines the entry point for the console application.
// A simple serial port writing example
// Written by Ted Burke - last updated 13-2-2013
// published http://batchloaf.wordpress.com/2013/02/13/writing-bytes-to-a-serial-port-in-c/
// Adapted for NMEA simulation by Douwe Fokkema 11-11-2014
// Adapted with moving position and NMEA chechsum by Håkan Svensson 2014-11-20
// Added routine to find a COM port and more NMEA strings by Håkan 2016-06-09
// Functions for user input of Nav-Data. Håkan 2016-11-08 

This Send_NMEA_COM is set to transfer NMEA sentences to a serial(COM) port.
The present setup is for test of OpenCPN projects. It's built on the origin project described below.
This program is free software under the terms of the GNU General Public License.


========================================================================
    CONSOLE APPLICATION : Send_NMEA_COM Project Overview
========================================================================

AppWizard has created this Send_NMEA_COM application for you.

This file contains a summary of what you will find in each of the files that
make up your Send_NMEA_COM application.


Send_NMEA_COM.vcxproj
    This is the main project file for VC++ projects generated using an Application Wizard.
    It contains information about the version of Visual C++ that generated the file, and
    information about the platforms, configurations, and project features selected with the
    Application Wizard.

Send_NMEA_COM.vcxproj.filters
    This is the filters file for VC++ projects generated using an Application Wizard. 
    It contains information about the association between the files in your project 
    and the filters. This association is used in the IDE to show grouping of files with
    similar extensions under a specific node (for e.g. ".cpp" files are associated with the
    "Source Files" filter).

Send_NMEA_COM.cpp
    This is the main application source file.

/////////////////////////////////////////////////////////////////////////////
Other standard files:

StdAfx.h, StdAfx.cpp
    These files are used to build a precompiled header (PCH) file
    named Send_NMEA_COM.pch and a precompiled types file named StdAfx.obj.

/////////////////////////////////////////////////////////////////////////////
Other notes:

AppWizard uses "TODO:" comments to indicate parts of the source code you
should add to or customize.

/////////////////////////////////////////////////////////////////////////////
