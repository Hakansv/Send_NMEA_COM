// Send_NMEA_COM.cpp : Defines the entry point for the console application.
// A simple serial port writing example
// Written by Ted Burke - last updated 13-2-2013
// published http://batchloaf.wordpress.com/2013/02/13/writing-bytes-to-a-serial-port-in-c/
// Adapted for NMEA simulation by Douwe Fokkema 11-11-2014
// Adapted with moving position and NMEA chechsum by Håkan Svensson 2014-11-20
// Added routine to find a COM port and more NMEA strings by Håkan 2016-06-09
// Functions for user input of Nav-Data. Håkan 2016-11-08  
 
#include "stdafx.h"
using namespace std;

//Nav-data variables
string s_Cog = "272.1";  //Degres value for Cog
string s_Mag = "272.1"; //Heading for NMEA
double d_Course = 282.1;  //Course to steer
double d_SOG = 5.5;    //Speed to run
double Def_Lat = 5803.200, Def_Long = 01122.100; //NMEA-Format!! Initial position for the cruise, N/E :)
string N_S = "N", E_W = "E";
double wmm = 3.0; //Variation to calc HDM from Course
double SecToNextPos = 3.0; //Time, e.g. distance to wait befor next posistion change.

//Others 
double angleRadHeading = 1; //Heading in radians
double Incr_Pos_Lat = 0.0001;  //Increase of Lat for each moving update.
double Incr_Pos_Lon = 0.004;  //Increase of Long for each moving update.
double M_PI = 3.14159;
int PauseTime = 175;  //Pause between each transmitt
int testsum(string);
char NMEA[80], HDM_NMEA[50], MWV_NMEA[50];
double d_long = Def_Long, d_Lat = Def_Lat;
bool b_Move = false;
double STW = 6.20;      //STW for VHW
double STW_Upd = 0.003; //STW incr. each cycle
bool Quit = false;
bool T = false;
clock_t PosTimer = clock();

//Functions
void CalculateNewPos(double, double);
void GetNavData(void);
double deg2rad(double, int);
double rad2deg(double, int);
double NMEA_degToDecDegr(double, int);
double DegrPosToNMEAPos(double, int);
void MakeNMEA(void);
void MakeNMEA_VHW();
void NMEA_HDM(void);
void MakeNMEA_MWV(bool);
double GetUserInput(double, int, int);

enum Lat_long { LAT = 1, LON = 2};
string msg = "\n\n****************** Send NMEA data to a COM port. *****************\n" ;

int main()
{
    cout << msg;
    // Define some static NMEA messages
    //char Head[] = "$HCHDM,030.4,M\r\n";
	//char NMEA[] = "$GPRMC,123519,A,5326.038,N,00611.000,E,005.4,084.4,230394,,W\r\n" ;
    char NMEA_MTW[] = "$IIMTW,14.7,C*11\r\n";
    char NMEA_DBT[] = "$IIDBT,37.9,f,11.5,M,6.3,F*1C\r\n";
   
    // Declare variables and structures
    bool Last = false;
    HANDLE hSerial;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
    char s[20];
    wchar_t pcCommPort[20];
    char Port[10];

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
            _cputs("\nHit key \"y\" to use that port. Any other key continuing search\n");
            if (toupper(_getch()) == 'Y') break;
            //else:
            continue;
        }
        
        Quit = j == 255 ?  true : false; //When for is ended and no port found
    }
    
    if (Quit) {
        _cputs("\nNo useable port found or selected - Hit any key to exit program");
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
            fprintf_s(stderr, "Error. Hit a key to exit\n");
            int Dummy = toupper(_getch());
            return 1;
    }
    else fprintf_s(stderr, "OK\n");
	
     
    // Set device parameters (4800 baud, 1 start bit,
    // 1 stop bit, no parity)
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (GetCommState(hSerial, &dcbSerialParams) == 0)
    {
        fprintf_s(stderr, "Error getting device state. Hit a key to exit\n");
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
        fprintf_s(stderr, "Error setting device parameters. Hit a key to exit\n");
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
        fprintf_s(stderr, "Error setting timeouts. Hit a key to exit\n");
        CloseHandle(hSerial);
        int Dummy = toupper(_getch());
        return 1;
    }
 
    // Send specified text (remaining command line arguments)
    _cputs( "\nHit key \"y\" to get a calculated position based on given speed and Course.\nAny other key gives a dummy moving position\n" );
    if (toupper(_getch()) == 'Y') {
        GetNavData();
    } else { 
        b_Move = true; 
    }

    s_Mag = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << d_Course - wmm))->str();
    s_Cog = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << d_Course))->str();;
    angleRadHeading = d_Course * M_PI / 180.0;
    
    DWORD bytes_written, total_bytes_written = 0;
    fprintf_s(stderr, "\nSending bytes...Any key stroke will exit the program.\n\n");

    NMEA_HDM(); //Make the HDM sentance once.
    
    while(!_kbhit()) { //Quit)
	    Sleep (PauseTime);
        if (Last) {
            if (((clock() - PosTimer) / CLOCKS_PER_SEC) < SecToNextPos) continue; // Wait for enough distance to calc pos.
            MakeNMEA(); //Make the RMC sentance, if "move" update each turn.
        }
        else MakeNMEA_VHW();  // Make the VHW sentance. Update each turn
        Last = !Last; //Alter between the two every second turn
        if(!WriteFile(hSerial, NMEA, strlen(NMEA), &bytes_written, NULL))
        {
            cout << bytes_written << " Bytes written \n";
            fprintf_s(stderr, "Error print NMEA\n");
            CloseHandle(hSerial);
            int Dummy = toupper(_getch());
            return 1;
        }  

         //fprintf(stderr, "%d bytes NMEA: %s", bytes_written, NMEA); //\n finns i strängen NMEA
         fprintf_s(stderr, NMEA); //\n finns i strängen NMEA
        
         if (Last) continue; //Send the rest NMEA mess every second turn only.
    
        Sleep (PauseTime);
	     if(!WriteFile(hSerial, HDM_NMEA, strlen(HDM_NMEA), &bytes_written, NULL))
        {
            fprintf_s(stderr, "Error. Hit a key to exit\n");
            CloseHandle(hSerial);
            int Dummy = toupper(_getch());
            return 1;
        }  

        //fprintf(stderr, "%d bytes Head: %s", bytes_written, HDM_NMEA); //\n finns i strängen Head
         fprintf_s(stderr, HDM_NMEA); //\n finns i strängen Head

         //Send MTW > Water temperature
         Sleep(PauseTime);
         if (!WriteFile(hSerial, NMEA_MTW, strlen(NMEA_MTW), &bytes_written, NULL))
         {
             fprintf_s(stderr, "Error. Hit a key to exit\n");
             CloseHandle(hSerial);
             int Dummy = toupper(_getch());
             return 1;
         }
         fprintf_s(stderr, NMEA_MTW); //\n finns i strängen
         
         //Send NMEA_DBT > Depth
         Sleep(PauseTime);
         if (!WriteFile(hSerial, NMEA_DBT, strlen(NMEA_DBT), &bytes_written, NULL))
         {
             fprintf_s(stderr, "Error. Hit a key to exit\n");
             CloseHandle(hSerial);
             int Dummy = toupper(_getch());
             return 1;
         }
         fprintf_s(stderr, NMEA_DBT); //\n finns i strängen

         //Send MWV > Wind speeed and realtive angle
         Sleep(PauseTime);
         if (!WriteFile(hSerial, MWV_NMEA, strlen(MWV_NMEA), &bytes_written, NULL))
         {
             fprintf_s(stderr, "Error. Hit a key to exit\n");
             CloseHandle(hSerial);
             int Dummy = toupper(_getch());
             return 1;
         }
         fprintf_s(stderr, MWV_NMEA); //\n finns i strängen
         
         MakeNMEA_MWV(T); //Make the MWV in every turn and alter between R and T.
         T = !T;
}

    // Close serial port
    fprintf_s(stderr, "Closing serial port...");
    if (CloseHandle(hSerial) == 0)
    {
        fprintf_s(stderr, "Error. Hit a key to exit\n");
        int Dummy = toupper(_getch());
        return 1;
    }
    fprintf_s(stderr, "OK\n");
 
    // Quit normally
    return 0;
}


void MakeNMEA() { 
    if (b_Move) {
        d_long = d_long - Incr_Pos_Lon;
        if (d_long > (Def_Long + 70)) d_long = Def_Long;
        if (b_Move) d_Lat = d_Lat + Incr_Pos_Lat;
        if (d_Lat > (Def_Lat + 70)) d_Lat = Def_Lat;
    }
    else {
        CalculateNewPos(d_Lat, d_long);
    }
    
  string s_Lat = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << d_Lat))->str();
  string s_Long = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << d_long))->str();
  string s_d_SOG = static_cast<ostringstream*>(&(ostringstream() << setprecision(3) << fixed << d_SOG))->str();
  string nmea = "$GPRMC,";
  nmea += "123519,";    //Time
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
    /*if (STW < 8) STW += STW_Upd;
    else STW = 6.8;*/
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

    void MakeNMEA_MWV(bool True) {
        string R_T = True ? "T" : "R";
        string WD = "55.3", WS = "8.5";
        if (True) { WD = "44.4"; WS = "9.1"; }
        string nmea = "$VDMWV,";
        nmea += WD; // 1
        nmea += ',';
        nmea += R_T; // 2
        nmea += ',';
        nmea += WS; // 3
        nmea += ',';
        nmea += "N"; // 4
        nmea += ',';
        nmea += "A"; // 5
        nmea += '*';
        nmea += static_cast<ostringstream*>(&(ostringstream() << hex << testsum(nmea)))->str();
        nmea += '\r';
        nmea += '\n';
        int Lens = nmea.size();
        //memset(VWR_NMEA, NULL, sizeof(NMEA)); //Clear the array
        for (int a = 0; a < Lens; a++) {
            MWV_NMEA[a] = nmea[a];
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

void CalculateNewPos(double Lat_in, double Long_in) {
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

double deg2rad(double Degr, int LL) {
    double r = NMEA_degToDecDegr(Degr, LL) * M_PI / 180.0;
    return r;
}
double rad2deg(double Rad_in, int LL){
    Rad_in = Rad_in * 180.0 / M_PI;//To degr
    double d = DegrPosToNMEAPos(Rad_in, LL);
    return d;
}

double NMEA_degToDecDegr(double NMEA_deg, int LL) {
    //Lat/Long in NMEA format. Make it to decimal degrees. We assume nothern hemisphere and East longitude for now
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
      cout << "\n Present Data:\n"
          << " Initial Latitude: " << (d_Lat = NMEA_degToDecDegr(d_Lat, LAT)) << "\n"
          << " Initial Longitude: " << (d_long = NMEA_degToDecDegr(d_long, LON)) << "\n"
          << " Course: " << d_Course << "\n"
          << " Variation: " << wmm << "\n"
          << " Speed knots: " << d_SOG;

      _cputs("\n\nHit key \"y\" to update any value. Any other key will keep existing Nav-Data\n\n");
      if (toupper(_getch()) == 'Y') {
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
  }

  double GetUserInput(double NavData, int min, int max) {
          double x;
          cin >> x;
          cin.clear();
          cin.ignore(10000, '\n');
          if (x >= min && x <= max) {
              return x;
          }
          else {
              cout << "OK - No change to: " << NavData << "\n\n";
              return NULL;
          }
  }
  
 