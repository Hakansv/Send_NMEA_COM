// Send_NMEA_COM.cpp : Defines the entry point for the console application.
// A simple serial port writing example
// Written by Ted Burke - last updated 13-2-2013
// published http://batchloaf.wordpress.com/2013/02/13/writing-bytes-to-a-serial-port-in-c/
// Adapted for NMEA simulation by Douwe Fokkema 11-11-2014
// Adapted with moving position and NMEA chechsum by Håkan Svensson 2014-11-20
// Added routine to find a COM port and more NMEA strings by Håkan 2016-06-09
// Functions for user input of Nav-Data. Håkan 2016-11-08
// More UI functions. Håkan 2016-11-10
// Read course from serial input RAHDT or ECAPB 2016-11-28
 
#include "stdafx.h"
#include <iostream>
#include <fstream>
using namespace std;

//Initial Nav-data variables when no config file is present
string s_Cog = "272.1";  //Degres value for Cog
string s_Mag = "272.1"; //Heading for NMEA
double d_Course = 282.1;  //Course to steer
double d_SOG = 6.5;    //Speed to run
double Def_Lat = 5803.200, Def_Long = 01122.100; //NMEA-Format!! Initial position for the cruise, N/E :)
string N_S = "N", E_W = "E";
double wmm = 3.0; //Variation to calc HDM from Course

//Others
double SecToNextPos = 2.5; //Time, e.g. distance to wait before next posistion change.
double d_AWS_kn = 5; //App wind speed
double d_AWA = 270; // App wind angle
double d_TWS_kn = 8.5; //True Wind speed
double d_TWA = 270; //True wind angle
double d_TWA_init = 310; //True wind angle
double d_DBT = 5.5; //Depth
double angleRadHeading = 1; //Heading in radians
double d_CourseTemp = 0;  //Temp. course to save
double d_BaroPress = 1.013; //Barometer pressure
double d_Heel = 2.0, d_Pitch = 1; //Heel & Pitch angle
double M_PI = 3.141592654;
int InfoCount = 0;
int testsum(string);
int WPL_count = 0;
char NMEA [80], HDM_NMEA [50], MWV_NMEA [50], NMEA_DBT [50], NMEA_MDA [70], NMEA_XDR[80];
char NMEA_WPL[50];
double d_long = Def_Long, d_Lat = Def_Lat;
bool esc = false;
double STW = 6.20;      //STW for VHW
double STW_Upd = 0.003; //STW incr. each cycle
bool Quit = false;
bool TorR = false;
bool hideNMEA = false;
bool RadarHeading = false, RAHeadIsValid = false;
bool SendWPL = false;
clock_t PosTimer = 1; // clock();
clock_t PauseTimer1 = 1; // clock();
clock_t PauseTimer2 = 1; // clock();
clock_t PauseTimer3 = 1; // clock();
clock_t PauseTimer4 = 1; // clock();
clock_t PauseTimer5 = 1; // clock();
clock_t LastWindMes = 1; // clock();
clock_t LastHDTMes = 1; // clock();
clock_t wpltimer = clock();
//TODO check if found:
string userdata = getenv("USERPROFILE");
HANDLE hSerial; //COM port handler
DWORD bytes_written, total_bytes_written = 0;

//Functions
void CalculateNewPos( const double &Lat_in, const double &Long_in );
void CalcWind ( void );
void GetNavData(void);
double deg2rad( const double &Degr, const int &LL );
double rad2deg( const double &Rad_in, const int &LL );
double NMEA_degToDecDegr( const double &NMEA_deg, const int &LL );
double DegrPosToNMEAPos(double, int);
void MakeNMEA(void);
void MakeNMEA_VHW();
void NMEA_HDM(void);
void MakeNMEA_MWV(bool);
void MakeNMEA_DBT( void );
void MakeNMEA_MDA(void);
void MakeNMEA_XDR(void);
void MakeWPL(void);
double GetUserInput( const double &NavData, const int &min, const int &max );
void ReadNavData(void);
void WriteNavdata(void);
void FormatCourseData(void);
void PrintUserInfo(void);
void OnKeyPress(void);
int getNbrOfBytes( void );
void ReadSerial( void );
void StopNMEACourse( void );
enum Lat_long { LAT = 1, LON = 2};

string msg = "\n\n****************** Send NMEA data to a COM port. *****************\n";
string msg1 = "And read NMEA RAHDT from the same port and if available using it as course\n";

int main()
{
    cout << msg << msg1;
    // Define some static NMEA messages
    char NMEA_MTW[] = "$IIMTW,14.7,C*11\r\n";
    //char NMEA_DBT[] = "$IIDBT,37.9,f,11.5,M,6.3,F*1C\r\n";
   
    // Declare variables and structures
    bool Last = false;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
    char s[20];
    wchar_t pcCommPort[20];
    char Port[10];

    ReadNavData(); //Read Navdata from file
    fprintf_s(stderr, "\nNow searching for a useable port.....\n");

    // Use FileOpen()
    // Try all 255 possible COM ports, check to see if it can be opened, or if
    // not, that an expected error is returned.

    for (int j = 1; j<256; j++)
    {
        sprintf_s(s, "\\\\.\\COM%d", j);
        mbstowcs(pcCommPort, s, strlen(s) + 1); //Plus null
        sprintf_s(Port, "COM%d",j);
        // Open the port tentatively
        HANDLE hComm = ::CreateFile(pcCommPort, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

        //  Check for the error returns that indicate a port is there, but not currently useable
        if (hComm == INVALID_HANDLE_VALUE)
        {
            DWORD dwError = GetLastError();

            if (dwError == ERROR_ACCESS_DENIED ||
                dwError == ERROR_GEN_FAILURE ||
                dwError == ERROR_SHARING_VIOLATION ||
                dwError == ERROR_SEM_TIMEOUT)
                fprintf_s(stderr, "\nFound serial port COM%d. But it's not ready to use\n", j);
        }
        else
        {
            CloseHandle(hComm);
            fprintf_s(stderr, "\nFound serial port COM%d and it's ready to use", j);
            _cputs("\nPress key \"y\" to use that port. Any other key continuing search\n");
            if (toupper(_getch()) == 'Y') break;
            //else:
            continue;
        }
        
        Quit = j == 255 ?  true : false; //When for is ended and no port found
    }
    
    if (Quit) {
        _cputs("\nNo useable port found or selected - Press any key to exit program");
        int Dummy = toupper(_getch());
        return 0; //No port found - exit pgm normally
    }
    // Some old stuff for info........
    //char Port[] = "COM1"; //Work around to get text
    //TCHAR *pcCommPort = TEXT("\\\\.\\COM1");
    // The \\\\.\\ is only needed for COM10 and above
         
    // Open serial port number
    fprintf_s(stderr, "\nOpening serial port: %s .........", Port);
    hSerial = CreateFile(
                pcCommPort, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );

    if (hSerial == INVALID_HANDLE_VALUE)
    {
            fprintf_s(stderr, "Error. Press a key to exit\n");
            int Dummy = toupper(_getch());
            return 1;
    }
    else fprintf_s(stderr, "OK\n");
	
     
    // Set device parameters (4800 baud, 1 start bit,
    // 1 stop bit, no parity)
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (GetCommState(hSerial, &dcbSerialParams) == 0)
    {
        fprintf_s(stderr, "Error getting device state. Press a key to exit\n");
        CloseHandle(hSerial);
        int Dummy = toupper(_getch());
        return 1;
    }
     
    dcbSerialParams.BaudRate = CBR_4800;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if(SetCommState(hSerial, &dcbSerialParams) == 0)
    {
        fprintf_s(stderr, "Error setting device parameters. Press a key to exit\n");
        CloseHandle(hSerial);
        int Dummy = toupper(_getch());
        return 1;
    }
 
    // Set COM port timeout settings
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    if(SetCommTimeouts(hSerial, &timeouts) == 0)
    {
        fprintf_s(stderr, "Error setting timeouts. Press a key to exit\n");
        CloseHandle(hSerial);
        int Dummy = toupper(_getch());
        return 1;
    }
 
    GetNavData();        //Ask if Navdata from file shall be changed.
    FormatCourseData();
        
    fprintf_s(stderr, "\nSending bytes........................\n\n");
    NMEA_HDM(); //Make the HDM sentance.
    //Initiating all timers 
    PosTimer = clock();
    PauseTimer1 = clock();
    PauseTimer2 = clock() + 100;
    PauseTimer3 = clock() + 200;
    PauseTimer4 = clock() + 300;
    PauseTimer5 = clock() + 600;
    LastWindMes = clock() + 400;
    LastHDTMes = clock() + 500;

    
    while (!esc) { //Quit on Esc or space *************************************THE BIG WHILE :-)***************:)

        if (( ( clock() - LastWindMes ) ) > 2000) {// Wait 2 sec before next MWV mes.
            CalcWind();
            MakeNMEA_MWV(TorR); //Make the MWV and alter between R and T.
            TorR = !TorR;
            LastWindMes = clock();
            //Send MWV > Wind speeed and realtive angle
            if (!WriteFile(hSerial, MWV_NMEA, strlen(MWV_NMEA), &bytes_written, NULL)) {
                fprintf_s(stderr, "Error. Press a key to exit\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
            }
            if (!hideNMEA) fprintf_s(stderr, MWV_NMEA); //\n finns i strängen
        }
        if (( ( clock() - PosTimer ) / CLOCKS_PER_SEC ) > SecToNextPos)
            CalculateNewPos(d_Lat, d_long); // Wait for enough distance to calc a new pos.

        if (( ( clock() - PauseTimer1 ) ) > 700) {
            if (InfoCount > 20) {
                PrintUserInfo();
                InfoCount = 0;
            }
            static bool firstRunOK = false;
            if (!firstRunOK && InfoCount >= 1) {
                cout << "\nOK it seems to work. Disabling NMEA printing to screen. \nPress P to view them all.\n\n";
                firstRunOK = true;
                hideNMEA = true;
                PrintUserInfo();
            }
            if (!hideNMEA) InfoCount++;

            if (Last) {
                MakeNMEA(); //Make the RMC sentance.
            } else MakeNMEA_VHW();  // Make the VHW sentance. Update each turn

            Last = !Last; //Alter between the two every turn
            if (!WriteFile(hSerial, NMEA, strlen(NMEA), &bytes_written, NULL)) {
                cout << bytes_written << " Bytes written \n";
                fprintf_s(stderr, "Error print NMEA\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
            }

            //fprintf(stderr, "%d bytes NMEA: %s", bytes_written, NMEA); //\n finns i strängen NMEA
            if (!hideNMEA) fprintf_s(stderr, NMEA); //\n finns i strängen NMEA
            PauseTimer1 = clock();
        }

        if (( ( clock() - PauseTimer2 ) ) > 1000) {
            if (!WriteFile(hSerial, HDM_NMEA, strlen(HDM_NMEA), &bytes_written, NULL)) {
                fprintf_s(stderr, "Error. Press a key to exit\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
            }
            if (!hideNMEA) fprintf_s(stderr, HDM_NMEA); //\n finns i strängen Head
            PauseTimer2 = clock();
        }

        if (( ( clock() - PauseTimer3 ) ) > 10000) {
            //Send MTW > Water temperature
            if (!WriteFile(hSerial, NMEA_MTW, strlen(NMEA_MTW), &bytes_written, NULL)) {
                fprintf_s(stderr, "Error. Press a key to exit\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
            }
            if (!hideNMEA) fprintf_s(stderr, NMEA_MTW); //\n finns i strängen
            PauseTimer3 = clock();
        }

        if (( ( clock() - PauseTimer4 ) ) > 1500) {
            //Send NMEA_DBT > Depth
            MakeNMEA_DBT();
            if (!WriteFile(hSerial, NMEA_DBT, strlen(NMEA_DBT), &bytes_written, NULL)) {
                fprintf_s(stderr, "Error. Press a key to exit\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
            }
            if (!hideNMEA) fprintf_s(stderr, NMEA_DBT); //\n finns i strängen
            PauseTimer4 = clock();
        }

        if (( ( clock() - PauseTimer5 ) ) > 5000) {
            //Send NMEA_MDA / XDR > Baro-press
            bool MDA = false;
            if (MDA) {
                MakeNMEA_MDA(); //0
                if (!WriteFile(hSerial, NMEA_MDA, strlen(NMEA_MDA), &bytes_written, NULL)) {
                    fprintf_s(stderr, "Error. Press a key to exit\n");
                    CloseHandle(hSerial);
                    int Dummy = toupper(_getch());
                    return 1;
                }
                if (!hideNMEA) fprintf_s(stderr, NMEA_MDA); //\n finns i strängen
            } else {
                MakeNMEA_XDR(); //1
                if (!WriteFile(hSerial, NMEA_XDR, strlen(NMEA_XDR), &bytes_written, NULL)) {
                    fprintf_s(stderr, "Error. Press a key to exit\n");
                    CloseHandle(hSerial);
                    int Dummy = toupper(_getch());
                    return 1;
                }
                if (!hideNMEA) fprintf_s(stderr, NMEA_XDR); //\n finns i strängen
            }
            PauseTimer5 = clock();
        }

        if (SendWPL && (clock() - wpltimer) > 100000) {
            wpltimer = clock();
            MakeWPL();
            if (!WriteFile(hSerial, NMEA_WPL, strlen(NMEA_WPL), &bytes_written, NULL)) {
                fprintf_s(stderr, "Error. Press a key to exit\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
            }
            if (!hideNMEA) fprintf_s(stderr, NMEA_WPL); //\n finns i strängen
        }

        ReadSerial(); //Read serial port for NMEA messages

        if (_kbhit()) { //Check the buffer for a key press to exit the program or enter a new course
            OnKeyPress();
        }
        
} //End of while()

    // Close serial port
    fprintf_s(stderr, "Closing serial port...");
    if (CloseHandle(hSerial) == 0)
    {
        fprintf_s(stderr, "Error. Press a key to exit\n");
        int Dummy = toupper(_getch());
        return 1;
    }
    fprintf_s(stderr, "OK\n");
 
    // Quit normally
    return 0;
} //End of main()


void MakeNMEA() { 
        
  string s_Lat = static_cast<ostringstream*>(&(ostringstream() << setprecision(4) << fixed << d_Lat))->str();
  string s_Long = static_cast<ostringstream*>(&(ostringstream() << setprecision(4) << fixed << d_long))->str();
  string s_d_SOG = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << d_SOG))->str();
  string nmea = "$GPRMC,";
  nmea += ",";    //Time  123519
  nmea += "A,";         //Valid
  nmea += s_Lat;        //Lat
  nmea += ",";
  nmea += N_S;        //N/S
  nmea += ",";
  nmea += s_Long;       //Long
  nmea += ",";
  nmea += E_W;        //E/W
  nmea += ",";
  nmea += s_d_SOG + ",";  //d_SOG 
  nmea +=  s_Cog;         //s_Cog 
  nmea += ",230394,";    //Date
  nmea += ",";          //WMM
  nmea += "W";          //W/E
  nmea += '*';
  nmea += static_cast<ostringstream*>( &(ostringstream() << hex << testsum(nmea)) )->str();
  nmea += "\r";
  nmea += "\n";
  
  int Lens = nmea.size();
  memset(NMEA, NULL, sizeof(NMEA)); //Clear the array
  for (int a = 0; a < Lens; a++) {
      NMEA[a] = nmea[a];
    }
}

//Create NMEA string: $HCHDM,238.5,M*hh/CR/LF
void NMEA_HDM() {
  string nmea = "$HCHDM,";
  nmea += s_Mag;
  nmea += ',';
  nmea += 'M';
  nmea += '*';
  nmea += static_cast<ostringstream*>( &(ostringstream() << hex << testsum(nmea)) )->str();
  nmea += '\r';
  nmea += '\n';
  int Lens = nmea.size();
  memset(HDM_NMEA, NULL, sizeof(HDM_NMEA)); //Clear the array
  for (int a = 0; a < Lens; a++) {
      HDM_NMEA[a] = nmea[a];
  }
}

/*Create NMEA string VDVHW - Water speed and heading ===
------------------------------------------------------------------------------
1 2 3 4 5 6 7 8 9
| | | | | | | | |
$--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
------------------------------------------------------------------------------
Field Number:
1. Degress True
2. T = True
3. Degrees Magnetic
4. M = Magnetic
5. Knots (speed of vessel relative to the water)
6. N = Knots
7. Kilometers (speed of vessel relative to the water)
8. K = Kilometers
9. Checksum
*/
void MakeNMEA_VHW() {
    STW = d_SOG + 0.3;
    double STW_k = STW;  // /0.53995;
    string s_STW = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << STW))->str();
    string s_STW_k = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << STW_k))->str();
    string nmea = "$VDVHW,";
    //nmea += s_Mag; //HDT
    nmea += ",";
    nmea += "T";
    nmea += ",";
    nmea += s_Mag; //HDM
    nmea += ",";
    nmea += "M";
    nmea += ",";
    nmea += s_STW;
    nmea += ",";
    nmea += "N";
    nmea += ",";
    //nmea += s_STW_k;
    nmea += ",";
    nmea += "K";
    nmea += '*';
    nmea += static_cast<ostringstream*>(&(ostringstream() << hex << testsum(nmea)))->str();
    nmea += "\r";
    nmea += "\n";
    int Lens = nmea.size();
    memset(NMEA, NULL, sizeof(NMEA)); //Clear the array
    for (int a = 0; a < Lens; a++) {
        NMEA[a] = nmea[a];
    }
}

    /*$IIMWV, 217.4, R, 5.7, N, A * 3F
    == = MWV - Wind Speed and Angle == =
        ------------------------------------------------------------------------------
        1 2 3 4 5
        | | | | |
        $--MWV, x.x, a, x.x, a*hh<CR><LF>
        ------------------------------------------------------------------------------
        Field Number :
    1. Wind Angle, 0 to 360 degrees
        2. Reference, R = Relative, T = True
        3. Wind Speed
        4. Wind Speed Units, K / M / N
        5. Status, A = Data Valid
        6. Checksum*/

    void MakeNMEA_MWV(bool b_True) {
        string s_WD = "";
        string s_WS = "";
        string s_R_T; // = b_True ? "T" : "R";
        if ( b_True ) {
            s_WD = static_cast<ostringstream*>( &( ostringstream () << setprecision ( 3 ) << fixed << d_TWA ) )->str ();
            s_WS = static_cast<ostringstream*>( &( ostringstream () << setprecision ( 3 ) << fixed << d_TWS_kn ) )->str ();
            s_R_T = "T";
        } else {
            s_WD = static_cast<ostringstream*>( &( ostringstream () << setprecision ( 3 ) << fixed << d_AWA ) )->str ();
            s_WS = static_cast<ostringstream*>( &( ostringstream () << setprecision ( 3 ) << fixed << d_AWS_kn ) )->str ();
            s_R_T = "R";
        }
        //string WD = "55.3", WS = "8.5";
        //if ( b_True ) { WD = "44.4"; WS = "9.1"; }
        string nmea = "$VDMWV,";
        nmea += s_WD; // 1
        nmea += ',';
        nmea += s_R_T; // 2
        nmea += ',';
        nmea += s_WS; // 3
        nmea += ',';
        nmea += "N"; // 4
        nmea += ',';
        nmea += "A"; // 5
        nmea += '*';
        nmea += static_cast<ostringstream*>(&(ostringstream() << hex << testsum(nmea)))->str();
        nmea += '\r';
        nmea += '\n';
        int Lens = nmea.size();
        memset( MWV_NMEA, NULL, sizeof( MWV_NMEA ) ); //Clear the array
        for (int a = 0; a < Lens; a++) {
            MWV_NMEA[a] = nmea[a];
        }
}

    /*
    ** XDR - Transducer Measurement
    **
    **        1 2   3 4            n
    **        | |   | |            |
    ** $--XDR,a,x.x,a,c--c, ..... *hh<CR><LF>
    **
    ** Field Number:
    **  1) Transducer Type
    **  2) Measurement Data
    **  3) Unit of Measurement, Celcius
    **  4) Name of transducer
    **  ...
    **  n) Checksum
    ** There may be any number of quadruplets like this, each describing a sensor. The last field will be a checksum as usual.
    */

    void MakeNMEA_XDR(void) {
        d_BaroPress > 1.024 ? d_BaroPress = 0.985 : d_BaroPress += 0.0005;
        string s_BaroPress = static_cast<ostringstream*>( &( ostringstream() << setprecision(3) << fixed << d_BaroPress ) )->str();
        d_Heel > 23.1 ? d_Heel = -21.2 : d_Heel += 0.5;
        string s_Heel = static_cast<ostringstream*>( &( ostringstream() << setprecision(3) << fixed << d_Heel ) )->str();
        d_Pitch > 13.1 ? d_Pitch = -11.2 : d_Pitch += 1.2;
        string s_Pitch = static_cast<ostringstream*>( &( ostringstream() << setprecision(3) << fixed << d_Pitch ) )->str();
        string nmea = "$IIXDR,";
        nmea += "P"; // 1. Type P = Pressure
        nmea += ',';
        nmea += s_BaroPress; // 2.Barometric pressure, bars
        nmea += ',';
        nmea += "B"; // 3. Barometric Unit, B = bars
        nmea += ',';
        nmea += "BAROMETER"; // 4.  
        nmea += ',';
        nmea += "A"; // 1. Type Heel or Pitch
        nmea += ',';
        nmea += s_Heel; // 2.Heel angle degr + = starboard
        nmea += ',';
        nmea += ""; // 3. 
        nmea += ',';
        nmea += "ROLL"; // 4.  
        nmea += ',';
        nmea += "A"; // 1. Type Heel or Pitch
        nmea += ',';
        nmea += s_Pitch; // 2.Pitch angle degr + = starboard
        nmea += ',';
        nmea += ""; // 3. 
        nmea += ',';
        nmea += "PTCH"; // 4.  Pitch
        nmea += ',';
        nmea += '*';
        nmea += static_cast<ostringstream*>( &( ostringstream() << hex << testsum(nmea) ) )->str();
        nmea += '\r';
        nmea += '\n';
        int Lens = nmea.size();
        memset(NMEA_XDR, NULL, sizeof(NMEA_XDR)); //Clear the array
        for (int a = 0; a < Lens; a++) {
            NMEA_XDR [a] = nmea [a];
        }
    }

    /*
            1 2  3  4  5  6  7  8  9  10  11  2 13  4 15  6 17  8 19 20
    **MDA,x.x,I,x.x,B,x.x,C,x.x,C,x.x,x.x,x.x,C,x.x,T,x.x,M,x.x,N,x.x,M*hh<CR><LF>
    **    |   |  |  |          Dew point, degrees C
    **    |   |  |  |          Absolute humidity, percent
    **    |   |  |  |          Relative humidity, percent
    **    |   |  |  |        Water temperature, degrees C
    **    |   |  |  |          Air temperature, degrees C
    **    |   |  |----Barometric pressure, bars
    **    |----- Barometric pressure, inches of mercur
    */
     void MakeNMEA_MDA(void) {
        string nmea = "$IIMDA,";
        nmea += ""; // 1. 
        nmea += ',';
        nmea += ""; // 2.
        nmea += ',';
        nmea += "1.015"; // 3. Barometric pressure, bars
        nmea += ',';
        nmea += "B"; // 4.  Barometric Unit, B = bars
        nmea += ',';
        nmea += ""; // 5. 
        nmea += ',';
        nmea += ""; // 6. 
        nmea += ',';
        nmea += ""; // 7. 
        nmea += ',';
        nmea += ""; // 8. 
        nmea += ',';
        nmea += ""; // 9. 
        nmea += ',';
        nmea += ""; // 10. 
        nmea += ',';
        nmea += ""; // 11. 
        nmea += ',';
        nmea += ""; // 12. 
        nmea += ',';
        nmea += ""; // 13. 
        nmea += ',';
        nmea += ""; // 14. 
        nmea += ',';
        nmea += ""; // 15. 
        nmea += ',';
        nmea += ""; // 16. 
        nmea += ',';
        nmea += ""; // 17. 
        nmea += ',';
        nmea += ""; // 18. 
        nmea += ',';
        nmea += ""; // 19. 
        nmea += ',';
        nmea += ""; // 20. 
        nmea += '*';
        nmea += static_cast<ostringstream*>(&(ostringstream() << hex << testsum(nmea)))->str();
        nmea += '\r';
        nmea += '\n';
        int Lens = nmea.size();
        memset( NMEA_MDA, NULL, sizeof( NMEA_MDA ) ); //Clear the array
        for (int a = 0; a < Lens; a++) {
            NMEA_MDA[a] = nmea [a];
        }
} 
    

    //char NMEA_DBT[] = "$IIDBT,37.9,f,11.5,M,6.3,F*1C\r\n";
     void MakeNMEA_DBT(void) {
         d_DBT = d_DBT > 15.6 ? d_DBT = 5.2 : d_DBT = d_DBT += 0.03;
         string s_depth = static_cast<ostringstream*>( &( ostringstream() << setprecision( 3 ) << fixed << d_DBT ) )->str();
        
        string nmea = "$IIDBT,";
        nmea += ""; // 1. Depth, feet
        nmea += ',';
        nmea += "f"; // 2. f = feet
        nmea += ',';
        nmea += s_depth; // 3. Depth, meters
        nmea += ',';
        nmea += "M"; // 4. M = meters
        nmea += ',';
        nmea += ""; // 5. Depth, Fathoms
        nmea += ',';
        nmea += "F"; // 6. F = Fathoms
        nmea += '*';
        nmea += static_cast<ostringstream*>(&(ostringstream() << hex << testsum(nmea)))->str();
        nmea += '\r';
        nmea += '\n';
        int Lens = nmea.size();
        memset( NMEA_DBT, NULL, sizeof( NMEA_DBT ) ); //Clear the array
        for (int a = 0; a < Lens; a++) {
            NMEA_DBT[a] = nmea [a];
        }
}

     void MakeWPL() {
         // $--WPL,llll.ll,a,yyyyy.yy,a,c--c*hh<CR><LF>
         string WP_Name = "WP";
         WPL_count++;
         WP_Name += to_string(WPL_count);
         string s_Lat = static_cast<ostringstream*>(&(ostringstream() << setprecision(4) << fixed << d_Lat))->str();
         string s_Long = static_cast<ostringstream*>(&(ostringstream() << setprecision(4) << fixed << d_long))->str();
         string s_d_SOG = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << d_SOG))->str();
         string nmea = "$IIWPL,";
         nmea += s_Lat;        //Lat
         nmea += ",";
         nmea += N_S;        //N/S
         nmea += ",";
         nmea += s_Long;       //Long
         nmea += ",";
         nmea += E_W;        //E/W
         nmea += ",";
         nmea += WP_Name;        //WP Name
         nmea += '*';
         nmea += static_cast<ostringstream*>(&(ostringstream() << hex << testsum(nmea)))->str();
         nmea += "\r";
         nmea += "\n";

         int Lens = nmea.size();
         memset(NMEA_WPL, NULL, sizeof(NMEA_WPL)); //Clear the array
         for (int a = 0; a < Lens; a++) {
             NMEA_WPL[a] = nmea[a];
         }
     }
   
// Calculates the Checksum for the NMEA string
int testsum(string strN) {
int i;
int XOR = 0;
int c;
// Calculate testsum ignoring any $'s in the string
for ( i = 0; i < 80; i++) {  //strlen(strN)
  c = (unsigned char)strN[i];
  if (c == '*') break;
  if (c != '$') XOR ^= c;
  }
return XOR;
}

void FormatCourseData(void) {
    double d_Mag = d_Course;
    d_Mag = d_Mag - wmm;
    if (d_Mag > 360.0){ d_Mag = d_Mag - 360.0; }
    else { if (d_Mag < 0) d_Mag = 360.0 + d_Mag; }
    s_Mag = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << d_Mag))->str();
    s_Cog = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << d_Course))->str();;
    angleRadHeading = d_Course * M_PI / 180.0; //for POS calculation
}

void CalculateNewPos(const double &Lat_in, const double &Long_in) {
    double secondsPassed = (clock() - PosTimer) / CLOCKS_PER_SEC;
    const double mRadiusEarth = 6378100.0f;
    double mSpeed = d_SOG * 0.514444f; //From knots to m/s
    double mDistance = mSpeed * secondsPassed; //Asume ten seconds so far  (timer1.Interval / 1000) / 3600;
    double distRatio = mDistance / mRadiusEarth;
    double distRatioSine = sin(distRatio);
    double distRatioCosine = cos(distRatio);
    
    double startLatRad = deg2rad(Lat_in, LAT); //Format from NMEA pos to dec.degr pos and to Radians.
    double startLonRad = deg2rad(Long_in, LON);

    double startLatCos = cos(startLatRad); //From rad to dec.degr and format to NMEA pos.
    double startLatSin = sin(startLatRad);

    double endLatRads = asin((startLatSin * distRatioCosine) + (startLatCos * distRatioSine * cos(angleRadHeading)));

    double endLonRads = startLonRad
        + atan2(sin(angleRadHeading) * distRatioSine * startLatCos,
        distRatioCosine - startLatSin * sin(endLatRads));

    double newLatDeg = rad2deg(endLatRads, LAT);
    double newLongDeg = rad2deg(endLonRads, LON);
    d_Lat = newLatDeg, 
    d_long = newLongDeg;
    PosTimer = clock();
}

double deg2rad(const double &Degr, const int &LL) {
    double r = NMEA_degToDecDegr(Degr, LL) * M_PI / 180.0;
    return r;
}
double rad2deg(const double &Rad_in, const int &LL){
    double Rad = Rad_in * 180.0 / M_PI;//To degr
    double d = DegrPosToNMEAPos(Rad, LL);
    return d;
}

double NMEA_degToDecDegr(const double &NMEA_deg, const int &LL) {
    //Lat/Long in NMEA format. Make it to decimal degrees.
    double degs = (int)(NMEA_deg / 100.0);
    double mins = NMEA_deg - degs * 100.0;
    double Dec_degs = degs + mins / 60.0;
    if (LL == LAT) { if (N_S == "S") Dec_degs = -Dec_degs; }
    if (LL == LON) { if (E_W == "W") Dec_degs = -Dec_degs; }
    return Dec_degs;
}

  double DegrPosToNMEAPos(double Pos_in, int LL ) {
      
      if (Pos_in < 0) {
          Pos_in = -Pos_in;
          if (LL == LAT) N_S = "S";
          if (LL == LON) E_W = "W";
      }
      else {
          if (LL == LAT) N_S = "N";
          if (LL == LON) E_W = "E";
      }
      double f_Degr = (int)(Pos_in);
      double f_Min = (Pos_in - f_Degr) * 60.0;
      double NMEA_POS = (f_Degr * 100.0) + f_Min;  // Type: 5802.3
      return NMEA_POS;
  }

  void GetNavData(void) {
      bool b_PrintNavD = true;
      cout << "\n Present Navdata:\n"
          << " Initial Latitude: " << (d_Lat = NMEA_degToDecDegr(d_Lat, LAT)) << "\n"
          << " Initial Longitude: " << (d_long = NMEA_degToDecDegr(d_long, LON)) << "\n"
          << " Course: " << d_Course << "\n"
          << " Variation: " << wmm << "\n"
          << " Speed knots: " << d_SOG;

      _cputs("\n\nPress key \"y\" to accept above values. Any other key will let you change them, one by one.\n\n");
      if (toupper(_getch()) == 'Y') {
          b_PrintNavD = false;
          cout << "Present Navdata accepted\n";
      } else {
          cout << "Enter a latitude instead of: " << d_Lat << ". Enter any char a-z to skip to next\n";
          double test = GetUserInput(d_Lat, -90, 90);
          if (test) { d_Lat = test; } // cout << "New value: " << d_Lat << "\n";

          cout << "Enter a longitude instead of: " << d_long << ". Enter any char a-z to skip to next\n";
          test = GetUserInput(d_long, -180, 180);
          if (test) { d_long = test; } // cout << "New value: " << d_long << "\n";

          cout << "Enter a Course instead of: " << d_Course << ". Enter any char a-z to skip to next\n";
          test = GetUserInput(d_Course, 0, 360);
          if (test) { d_Course = test; } // cout << "New value: " << d_Course << "\n";
          
          cout << "Enter a variation, + > E : - > W, instead of: " << wmm << ". Enter any char a-z to skip to next\n";
          test = GetUserInput(wmm, 0, 360);
          if (test) { wmm = test; } // cout << "New value: " << d_Course << "\n";

          cout << "Enter a Speed instead of: " << d_SOG << ". Enter any char a-z to skip.\n";
          test = GetUserInput(d_SOG, 0, 50);
          if (test) { d_SOG = test; } //cout << "New value: " << d_SOG << "\n"; }
      }
      d_Lat = DegrPosToNMEAPos(d_Lat, LAT); //Back to NMEA format
      d_long = DegrPosToNMEAPos(d_long, LON); //Back to NMEA format
      if (b_PrintNavD) WriteNavdata(); // Print the new Navdata to config file
  }

  double GetUserInput(const double &NavData, const int &min, const int &max) {
          double x;
          cin >> x;
          if ( x < 1 ) x += 0.1; //Avoid 0 (int 0 = false)
          cin.clear();
          cin.ignore(10000, '\n');
          if ( x >= min && x <= max ) {
              return x;
          }
          else {
              cout << "No change to: " << NavData << "\n\n";
              return NULL;
          }
  }
  
  void ReadNavData(void) {
      bool file_eof = false;
      string filePath = userdata;
      filePath += "\\SendNMEACOM\\navdata.cnv";
      ifstream myfile(filePath);
      if (myfile.is_open())
      {
          string s_Nav[7];
          for (int i = 0; i < 7; ++i)
          {
              if (myfile.eof()) {
                  file_eof = true;
                  continue;
              }
                  myfile >> s_Nav[i];
          }
          if (!file_eof) {
              d_Lat = std::stod(s_Nav[0]);
              d_long = std::stod(s_Nav[1]);
              d_Course = std::stod(s_Nav[2]);
              wmm = std::stod(s_Nav[3]);
              d_SOG = std::stod(s_Nav[4]);
              d_TWA_init = std::stod( s_Nav [5] );
              d_TWS_kn = std::stod( s_Nav [6] );
          }
          myfile.close();
      }
      else {
          cout << "Unable to find the Navdata file. I'll make a new one\n";
          WriteNavdata();
      }
      if (file_eof) WriteNavdata(); //Navdata file corrupt. False user input??
  }

  void WriteNavdata(void) {
      string filePath = userdata;
      filePath += "\\SendNMEACOM";
      if (CreateDirectoryA(filePath.c_str(), NULL) ||
          ERROR_ALREADY_EXISTS == GetLastError())
      {
          filePath += "\\navdata.cnv";
          ofstream myfile;
          myfile.open(filePath, ios::trunc);
          if (myfile.is_open())
          {
              myfile << d_Lat << "\n";
              myfile << d_long << "\n";
              myfile << d_Course << "\n";
              myfile << wmm << "\n";
              myfile << d_SOG << "\n";
              myfile << d_TWA_init << "\n";
              myfile << d_TWS_kn << "\n";
              myfile.close();
              cout << "\n" << "New Navdata was saved to:\n" << filePath << "\n";
          }
          else cout << "Unable to open file to write\n";
      }
      else cout << "Sorry, Unable to create file directory: " << filePath << "\n";
  }

  void PrintUserInfo(void) {
      cout << "\n     Press Esc or Space to exit the program.\n"
          << "     Press P to show or hide NMEA messaging to screen\n"
          << "     Press R to restart navigation from initial/saved position\n"
          << "     Press S to save all actual navdata to the config file\n"
          << "     Press ? or _ to instantly change wind direction 10 degr up or down\n"
          << "     Unless course is obtained from serial input you can:\n"
          << "     Press + or - to instantly change course 10 degr up or down\n"
          << "     Press any other key to change the initial course to a new value.\n\n";
  }

  void OnKeyPress(void) {
      char key = _getch();
      //cout << key << "\n";
      bool pIsTouched = false;
      switch (key) {
      case 27: //Esc
      case 32: //Space
          esc = true;
          if ( d_CourseTemp > 0 ) {
              ReadNavData();
              d_Course = d_CourseTemp;
              WriteNavdata();
          }
          break;
      case '+':
          if ( RadarHeading ) break;// No course change while heading from radar
          d_Course += 10;
          if (d_Course > 360) d_Course = d_Course - 360;
          break;
      case '-':
          if ( RadarHeading ) break;// No course change while heading from radar
          d_Course -= 10;
          if (d_Course < 0) d_Course = 360 + d_Course;
          break;
      case 'p':
      case 'P':
          pIsTouched = true;
          if (!hideNMEA) {
              cout << "\n  NMEA printing to screen is disabled.\n";
              PrintUserInfo();
          } else {
              cout << "  Printing NMEA to screen\n";
          }
          hideNMEA = !hideNMEA;
          break;
      case 'r':
      case 'R':
          pIsTouched = true;
          ReadNavData();
          cout << " Navigation restarted\n";
          break;
      case 's':
      case 'S':
          pIsTouched = true;
          WriteNavdata();
          cout << " All present navdata are saved to config file\n";
          break;
      case 'w':
      case 'W':
          pIsTouched = true;
          SendWPL = !SendWPL;
          if (SendWPL) {
              cout << " Now we send WPL\n";
          } else cout << " Stop sending WPL\n";
          break;
      case '?':
          d_TWA_init += 10;
          if ( d_TWA_init > 360 ) d_TWA_init = d_TWA_init - 360;
          pIsTouched = true; //Like
          cout << "True Wind angle increased\n";
          break;
      case '_':
          d_TWA_init -= 10;
          if ( d_TWA_init < 0 ) d_TWA_init = 360 + d_TWA_init;
          pIsTouched = true; // Like
          cout << "True Wind angle decreased\n";
          break;
      default:
         if ( RadarHeading ) break; // No course change while heading from radar
          string keys;
          cout << "Enter a new course instead of: " << d_Course << " Or any letter to quit\n";
          double NewCourse = d_Course;
          NewCourse = GetUserInput(NewCourse, 0, 360);
          if (NewCourse) {
              d_Course = NewCourse;
              d_CourseTemp = d_Course;  //Save the new course in temp.
          }
      }
      if ( !esc && !pIsTouched && !RadarHeading ) {
          FormatCourseData();
          cout << "New course: " << d_Course << " degrees\n";
          NMEA_HDM(); //Update NMEA message after course change
      }
  }
  
  void CalcWind( void ) {
      double TWA, x, y;
      int q = 1, m = 1;
      bool zerofix = false;
      //True wind
      if ( d_TWA_init < 1 || d_TWA_init > 359 ) d_TWA_init = 5; // Avoid zero
      d_TWA = d_Course >= d_TWA_init ? 360 - d_Course + d_TWA_init : d_TWA_init - d_Course;
      d_TWS_kn = 11.7;
      //double d_TWS = d_TWS_kn / 1.94384; // To m/s
      //Apparent wind
      double d_TWA_2Q = ( ( d_TWA > 180 ? 180 - ( d_TWA - 180 ) : d_TWA ) );
      //Upwind
      if ( d_TWA >= 0 && d_TWA <= 90 ) { d_TWA_2Q = d_TWA; q = 0; m = 1; }
      if ( d_TWA > 270 && d_TWA <= 360 ) { d_TWA_2Q = 90 - ( d_TWA - 270 ); q = 360; m = 1; }
      //Downwind
      if ( d_TWA > 90 && d_TWA <= 180 ) { d_TWA_2Q = 90 - ( d_TWA - 90 ); q = 180; m = -1; }
      if ( d_TWA > 180 && d_TWA <= 270 ) { d_TWA_2Q = 270 - ( d_TWA - 270 ); q = 180; m = -1; }
      
      if ( d_TWA_2Q == 0 ) { //Avoid the nasty zero
          d_TWA_2Q += 1; 
          zerofix = true; 
      }

      TWA = ( m * d_TWA_2Q * M_PI / 180 );
      x = sin( TWA ) * d_TWS_kn;
      y = cos( TWA ) * d_TWS_kn;
      
      //Apparent wind angle
      double d_AWA_Q = 0.1+( 180 / M_PI ) * atan( x / ( y + ( d_SOG ) ) );
      if ( m > 0 ) { //Upwind
          d_AWA = q ? q - d_AWA_Q : d_AWA_Q;
      } else {       //Downwind
          d_AWA = d_TWA - ( ( 360 + ( (d_AWA_Q)-q ) ) - d_TWA );
      }
      if ( zerofix )d_AWA > 359 ? d_AWA = 0 : d_AWA -= 1;

      //Apparent wind speed
      double d_AWS_part = ( x / ( sin( (d_AWA_Q)*M_PI / 180 ) ) );
      d_AWS_kn = d_TWS_kn + m * ( d_AWS_part - d_TWS_kn ); //Downwind it's negative
}

  void ReadSerial( void ) {
      DWORD dwReading = 0;
      size_t pos = 0;
      string s;
      string delimiter = ",";
      string token [16];
      string sentense, courseunit, XTE_Dir;
      double d_newcourse = NULL, d_XTE = 0.0;
      char recbuf [50];
      int bytestoread = 0, y = 0;
      

      if ( ( bytestoread = getNbrOfBytes() ) > 1 ) {
          if ( ReadFile( hSerial, recbuf, sizeof( recbuf ), &dwReading, NULL ) ) {
                DWORD i;
                for ( i = 0; i < dwReading; i++ )
                    s += recbuf [i];
                //--HDT,x.x,T*hh<CR><LF>
          
              while ( ( pos = s.find( delimiter ) ) != std::string::npos ) {
                  token [y] = s.substr( 0, pos );
                  s.erase( 0, pos + delimiter.length() );
                  if (token [0] == "$RAHDT" || token [0] == "RAHDT") {
                      switch (y) {
                      case 0:
                          sentense = token [0];
                          delimiter = ",";
                          break;
                      case 1:
                          d_newcourse = stod(token [1]);
                          delimiter = "*"; // Last token has no ','  //$RAHDT  $HCHDM
                          break;
                      case 2:
                          courseunit = token [2];
                          delimiter = "$";  //Find next sentance
                          continue;
                          break;
                      }
                    
                  } else if (token [0] == "$ECAPB" || token [0] == "ECAPB") {
                      static string WP_ID;
                      switch (y) {
                      case 0:
                          sentense = token [0];
                          delimiter = ",";
                          break;
                      case 3:  //Cross Track Error Magnitude
                          d_XTE = stod(token [y]);
                          break;
                      case 4:  //Direction to steer, L or R
                          XTE_Dir = token [y];
                          break;
                      case 6: //Status A = Arrival Circle Entered
                          if ("A" == token[y]) cout << "Waypoint " << WP_ID << " arrived\n";
                          break;
                      //case 7:  // Status: A = Perpendicular passed at waypoint. Not used by OCPN
                      //    if ("A" == token[y]) cout << " Waypoint passed\n";
                      //    break;
                      case 10:  // Dest WP ID. Truncated to 6 by OCPN
                          if (WP_ID != token[y]) WP_ID = token[y];
                          break;
                      case 11: //Bearing, present position to Destination
                          d_newcourse = stod(token [y]);
                          break;
                      case 12:  //M = Magnetic, T = True
                          courseunit = token [y];
                          delimiter = "$";  //Find next sentance
                          continue;
                          break;
                      default:
                          break;
                      } 
                  
                  } else {
                      if (delimiter == "$") {
                          delimiter = ",";
                      } else {
                          delimiter = "$";  //Find next sentance
                      }
                      y = 0;
                  }
                  y++;
              }

              if (d_newcourse && "T" == courseunit) { //True heading
                  // Increased XTE action for the simulation, XTE is normally like 0.015
                  double dXTE_factor = d_XTE > 0.1 ? ( d_XTE > 0.5 ? 10 : 50 ) : 100; // Like Excel hmmmm!?
                  d_newcourse = ("L" == XTE_Dir ? d_newcourse - dXTE_factor * d_XTE : d_newcourse + dXTE_factor * d_XTE);
                      if (d_newcourse < 0.) d_newcourse += 360.;
                      else if (d_newcourse > 360.) d_newcourse -= 360.;
                RAHeadIsValid = true;
                d_Course = d_newcourse; // stod( token [1] );
                FormatCourseData();
                NMEA_HDM(); //Update NMEA message after course change
                LastHDTMes = clock(); //Serial watchdog
                if ( !RadarHeading ) {
                    cout << sentense << " received! Now the course is obtained from serial input.\n";
                    RadarHeading = true;
                }
              } else StopNMEACourse(); //Stop reading heading from serial read

          } else cout << "No succes reading COM port\n" << dwReading << "\n";


      } else StopNMEACourse(); //If reading heading from serial and time out, stop it 
  }

  void StopNMEACourse( void ) {
      if ( RadarHeading && ( ( clock() - LastHDTMes ) / CLOCKS_PER_SEC ) > 15 ) {
          RadarHeading = false; //Switch it off again and
          RAHeadIsValid = false;
          cout << "Heading from serial connection is broken.\n";
          FormatCourseData();
          NMEA_HDM(); //Update NMEA message after course change
      }
  }
  
  int getNbrOfBytes( void ) {
      struct _COMSTAT status;
      int             n;
      unsigned long   etat;
      n = 0;

      if ( hSerial != INVALID_HANDLE_VALUE ) {
          ClearCommError( hSerial, &etat, &status );
          n = status.cbInQue;
      }
      return( n );
  }
