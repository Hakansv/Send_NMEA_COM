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
//#include <winsock2.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library
#define SERVER "127.0.0.255" //"127.0.0.1"  //ip address of udp server192.168.10.255
#define BUFLEN 512  //Max length of buffer
#define PORT 10110   //The port on which to send data

using namespace std;

//Socket

struct sockaddr_in server, si_other;
struct in_addr addr; 
int sock, slen = sizeof(si_other), recv_len;
SOCKET sock_r;
char buf[BUFLEN];
char message[BUFLEN];
char recmessage[BUFLEN];
WSADATA wsa;

//Initial Nav-data variables when no config file is present
string s_Cog = "272.1";  //Degres value for Cog
string s_Mag = "272.1"; //Heading for NMEA
double d_Course = 282.1;  //Course to steer
double d_SOG = 6.5;    //Speed to run
double Def_Lat = 5803.200, Def_Long = 01122.100; //NMEA-Format!! Initial position for the cruise, N/E :)
string s_NS = "N", s_EW = "E";
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
bool SendWPL = false; //Contflict??
bool RecordAISdata = false;
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
string s_navdatafile;
HANDLE hSerial; //COM port handler
DWORD bytes_written, total_bytes_written = 0;
unsigned int PgmMode = 1; // User selected program mode. 1:Serial, 2:UDP-IP, 3:Both
bool ReceiveSerial(false);
bool Last = false;

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
void PrintWPLtoFile(void);
int getNbrOfBytes( void );
void ReadSerial( void );
void StopNMEACourse( void );
void WriteAISdata(void);
bool InitSerialPort(void);
bool InitWinsock(void);
void SendNMEAtoIP(char s_nmea[80]);
void ReceiveUDP(void);

enum Lat_long { LAT = 1, LON = 2};

string msg = "\n\n****************** Send NMEA data to a COM port. *****************\n";
string msg1 = "Read NMEA RAHDT or ECAPB from the same port and if available using it as course\n\n";
string msg2 = "Nav data is read from file : ";

int main(int argc, char *argv [])
{
  //Check for a command argument
  s_navdatafile = argc > 1 ? argv[1] : "navdata.cnv";
  cout << msg << msg1 << msg2 << s_navdatafile << "\n";
    // Define some static NMEA messages
    char NMEA_MTW[] = "$IIMTW,14.7,C*11\r\n";
    //char NMEA_DBT[] = "$IIDBT,37.9,f,11.5,M,6.3,F*1C\r\n";
   
    ReadNavData(); //Read Navdata from file

    //char answer;
    fprintf(stderr, "Do you want to send data based on %s to:\n", s_navdatafile);
    _cputs("Press \"1\" to use IP-UDP port 10110\nPress \"2\" to use Serial-COM port\nPress \"3\" to also use Serial-COM port for incoming mes\n");
    char answer = toupper(_getch());
    //else cout << "Not an expected input\n";

    if (answer == '1') PgmMode = 1;
    if (answer == '2') { PgmMode = 2; ReceiveSerial = true; }
    if (answer == '3') { PgmMode = 1; ReceiveSerial = true; }

    switch (PgmMode) {
      //int Dummy;
    case 1:
        cout << "\nNow connect to IP-net.....\n";
        //Dummy = toupper(_getch()); //Empty getch
        InitWinsock();
        if (ReceiveSerial) {
          cout << "\nNow also searching for a useable port.....\n";
          InitSerialPort();
        }
        break;
   
    case 2:
        cout << "\nNow searching for a useable port.....\n";
        //Dummy = toupper(_getch()); //Empty getch
        InitSerialPort();
        //TODO felhandling
        break;
      
    default:
          return 0;        
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

	if (PgmMode == 1) {
		//InitWinsock();

		
	}

	//test IP adr
	//if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
	//	printf("socket() failed with error code : %d", WSAGetLastError());
	//	exit(EXIT_FAILURE);
	//}

	////setup address structure
	//memset((char *)&si_other, 0, sizeof(si_other));
	//si_other.sin_family = AF_INET;
	//si_other.sin_port = htons(PORT);
	//si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);  //INADDR_ANY; //inet_addr(SERVER);
	//char testmes[] = "$GPTES,\"Test\"";
	//if (sendto(sock, testmes, strlen(testmes), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR) {
	//	int err = WSAGetLastError();

	//	printf("sendto() failed with error code : %d", err); // WSAGetLastError());
	//	exit(EXIT_FAILURE);
	//}
	////if (!hideNMEA) printf("To IP port:%d  %s\n", PORT, s_nmea);

	//closesocket(sock);



    while (!esc) { //Quit on Esc or space *************************************THE BIG WHILE :-)***************:)

        if (( ( clock() - LastWindMes ) ) > 2000) {// Wait 2 sec before next MWV mes.
            CalcWind();
            MakeNMEA_MWV(TorR); //Make the MWV and alter between R and T.
            TorR = !TorR;
            LastWindMes = clock();
            //Send MWV > Wind speeed and realtive angle
            if (PgmMode == 1) {
              SendNMEAtoIP(MWV_NMEA);
            }
            if (PgmMode == 2) {
              if (!WriteFile(hSerial, MWV_NMEA, strlen(MWV_NMEA), &bytes_written, NULL)) {
                fprintf_s(stderr, "Error. Press a key to exit\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
              }
            }
           
            if (!hideNMEA) fprintf_s(stderr, MWV_NMEA); //\n finns i strängen
        }
        if (((clock() - PosTimer) / CLOCKS_PER_SEC) > SecToNextPos) {
            CalculateNewPos(d_Lat, d_long); // Wait for enough distance to calc a new pos.
            if (RecordAISdata ) WriteAISdata();
        }

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
            if (PgmMode == 1) {
              SendNMEAtoIP(NMEA);
            }
            if (PgmMode == 2) {
              if (!WriteFile(hSerial, NMEA, strlen(NMEA), &bytes_written, NULL)) {
                cout << bytes_written << " Bytes written \n";
                fprintf_s(stderr, "Error print NMEA\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
              }
            }

            //fprintf(stderr, "%d bytes NMEA: %s", bytes_written, NMEA); //\n finns i strängen NMEA
            if (!hideNMEA) fprintf_s(stderr, NMEA); //\n finns i strängen NMEA
            PauseTimer1 = clock();
        }

        if (( ( clock() - PauseTimer2 ) ) > 1000) {
          if (PgmMode == 1) {
            SendNMEAtoIP(HDM_NMEA);
          }
          if (PgmMode == 2) {
            if (!WriteFile(hSerial, HDM_NMEA, strlen(HDM_NMEA), &bytes_written, NULL)) {
              fprintf_s(stderr, "Error. Press a key to exit\n");
              CloseHandle(hSerial);
              int Dummy = toupper(_getch());
              return 1;
            }
          }
            if (!hideNMEA) fprintf_s(stderr, HDM_NMEA); //\n finns i strängen Head
            PauseTimer2 = clock();
        }

        if (( ( clock() - PauseTimer3 ) ) > 10000) {
            //Send MTW > Water temperature
          if (PgmMode == 1) {
            SendNMEAtoIP(NMEA_MTW);
          }
          if (PgmMode == 2) {
            if (!WriteFile(hSerial, NMEA_MTW, strlen(NMEA_MTW), &bytes_written, NULL)) {
              fprintf_s(stderr, "Error. Press a key to exit\n");
              CloseHandle(hSerial);
              int Dummy = toupper(_getch());
              return 1;
            }
          }
            if (!hideNMEA) fprintf_s(stderr, NMEA_MTW); //\n finns i strängen
            PauseTimer3 = clock();
        }

        if (( ( clock() - PauseTimer4 ) ) > 1500) {
            //Send NMEA_DBT > Depth
            MakeNMEA_DBT();
            if (PgmMode == 1) {
              SendNMEAtoIP(NMEA_DBT);
            }
            if (PgmMode == 2) {
              if (!WriteFile(hSerial, NMEA_DBT, strlen(NMEA_DBT), &bytes_written, NULL)) {
                fprintf_s(stderr, "Error. Press a key to exit\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
              }
            }
            if (!hideNMEA) fprintf_s(stderr, NMEA_DBT); //\n finns i strängen
            PauseTimer4 = clock();
        }

        if (( ( clock() - PauseTimer5 ) ) > 5000) {
            //Send NMEA_MDA / XDR > Baro-press
            bool MDA = false;
            if (MDA) {
                MakeNMEA_MDA(); //0
                if (PgmMode == 1) {
                  SendNMEAtoIP(NMEA_MDA);
                }
                if (PgmMode == 2) {
                  if (!WriteFile(hSerial, NMEA_MDA, strlen(NMEA_MDA), &bytes_written, NULL)) {
                    fprintf_s(stderr, "Error. Press a key to exit\n");
                    CloseHandle(hSerial);
                    int Dummy = toupper(_getch());
                    return 1;
                  }
                }
                if (!hideNMEA) fprintf_s(stderr, NMEA_MDA); //\n finns i strängen
            } else {
                MakeNMEA_XDR(); //1
                if (PgmMode == 1) {
                  SendNMEAtoIP(NMEA_XDR);
                }
                if (PgmMode == 2) {
                  if (!WriteFile(hSerial, NMEA_XDR, strlen(NMEA_XDR), &bytes_written, NULL)) {
                    fprintf_s(stderr, "Error. Press a key to exit\n");
                    CloseHandle(hSerial);
                    int Dummy = toupper(_getch());
                    return 1;
                  }
                }
                if (!hideNMEA) fprintf_s(stderr, NMEA_XDR); //\n finns i strängen
            }
            PauseTimer5 = clock();
        }

        if (SendWPL && (clock() - wpltimer) > 50000) {
            wpltimer = clock();
            MakeWPL();
            if (PgmMode == 1) {
              SendNMEAtoIP(NMEA_WPL);
            }
            if (PgmMode == 2) {
              if (!WriteFile(hSerial, NMEA_WPL, strlen(NMEA_WPL), &bytes_written, NULL)) {
                fprintf_s(stderr, "Error. Press a key to exit\n");
                CloseHandle(hSerial);
                int Dummy = toupper(_getch());
                return 1;
              }
            }
            if (!hideNMEA) fprintf_s(stderr, NMEA_WPL); //\n finns i strängen
            PrintWPLtoFile();
        }

        if ( ReceiveSerial ) ReadSerial(); //Read serial port for NMEA messages
        //ReceiveUDP();

        if (_kbhit()) { //Check the buffer for a key press to exit the program or enter a new course
            OnKeyPress();
        }
        
} //End of while()

if (PgmMode == 1) {
  //Close socket
  closesocket(sock);
  WSACleanup();
}
if (PgmMode == 2 || ReceiveSerial) {
  // Close serial port
  fprintf_s(stderr, "Closing serial port...");
  if (CloseHandle(hSerial) == 0) {
    fprintf_s(stderr, "Error. Press a key to exit\n");
    int Dummy = toupper(_getch());
    return 1;
  }
  fprintf_s(stderr, "OK\n");
}
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
  nmea += s_NS;        //N/S
  nmea += ",";
  nmea += s_Long;       //Long
  nmea += ",";
  nmea += s_EW;        //E/W
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
         nmea += s_NS;        //N/S
         nmea += ",";
         nmea += s_Long;       //Long
         nmea += ",";
         nmea += s_EW;        //E/W
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
    if (LL == LAT) { if (s_NS == "S") Dec_degs = -Dec_degs; }
    if (LL == LON) { if (s_EW == "W") Dec_degs = -Dec_degs; }
    return Dec_degs;
}

  double DegrPosToNMEAPos(double Pos_in, int LL ) {
      
      if (Pos_in < 0) {
          Pos_in = -Pos_in;
          if (LL == LAT) s_NS = "S";
          if (LL == LON) s_EW = "W";
      }
      else {
          if (LL == LAT) s_NS = "N";
          if (LL == LON) s_EW = "E";
      }
      double f_Degr = (int)(Pos_in);
      double f_Min = (Pos_in - f_Degr) * 60.0;
      double NMEA_POS = (f_Degr * 100.0) + f_Min;  // Type: 5802.3
      return NMEA_POS;
  }

  void GetNavData(void) {
      bool b_PrintNavD = true;
      cout << "\n Present Navdata:\n"
          << " Initial Latitude: " << (d_Lat = NMEA_degToDecDegr(d_Lat, LAT)) << " " << s_NS << "\n"
          << " Initial Longitude: " << (d_long = NMEA_degToDecDegr(d_long, LON)) << " " << s_EW << "\n"
          << " Course: " << d_Course << "\n"
          << " Variation: " << wmm << "\n"
          << " Speed knots: " << d_SOG;

      _cputs("\n\nPress key \"y\" to accept above values. Any other key will let you change them, one by one.\n\n");
      if (toupper(_getch()) == 'Y') {
          b_PrintNavD = false;
          cout << "Present Navdata accepted\n";
      } else {
          cout << "Enter a latitude instead of: " << d_Lat << " Negative for S. Enter zero, 0, to skip to next\n";
          double test = GetUserInput(d_Lat, -90, 90);
          if (test != 0 ) { 
            d_Lat = test; 
            s_NS = d_Lat >= 0.0 ? "N" : "S";
          } // cout << "New value: " << d_Lat << "\n";

          cout << "Enter a longitude instead of: " << d_long << " Negative for W. Enter zero, 0, to skip to next\n";
          test = GetUserInput(d_long, -180, 180);
          if (test != 0) { 
            d_long = test; 
            s_EW = d_long >= 0.0 ? "E" : "S";
          } // cout << "New value: " << d_long << "\n";

          cout << "Enter a Course instead of: " << d_Course << ". Enter zero, 0, to skip to next\n";
          test = GetUserInput(d_Course, 0, 360);
          if (test != 0) { d_Course = test; } // cout << "New value: " << d_Course << "\n";
          
          cout << "Enter a variation, + > E : - > W, instead of: " << wmm << ". Enter zero, 0, to skip to next\n";
          test = GetUserInput(wmm, -161, 161);
          if (test != 0) { wmm = test; } // cout << "New value: " << d_Course << "\n";

          cout << "Enter a Speed instead of: " << d_SOG << ". Enter zero, 0, to skip.\n";
          test = GetUserInput(d_SOG, 0, 50);
          if (test != 0) { d_SOG = test; } //cout << "New value: " << d_SOG << "\n"; }
      }
      d_Lat = DegrPosToNMEAPos(d_Lat, LAT); //Back to NMEA format
      d_long = DegrPosToNMEAPos(d_long, LON); //Back to NMEA format
      if (b_PrintNavD) WriteNavdata(); // Print the new Navdata to config file
  }

  double GetUserInput(const double &NavData, const int &min, const int &max) {
          double x;
          cin >> x;
          //if ( x != 0 ) x += 0.1; //Avoid 0 (int 0 = false)
          cin.clear();
          cin.ignore(10000, '\n');
          if ( x >= min && x <= max && x != 0) {
              return x;
          }
          else {
              cout << "No change to: " << NavData << "\n\n";
              return 0;
          }
  }
  
  void ReadNavData(void) {
      bool file_eof = false;
      string filePath;
      //Check for full file path.
      if (s_navdatafile.find(":\\") == std::string::npos) {
        //Disk path not found. Add the user path
        s_navdatafile = userdata + "\\SendNMEACOM\\" + s_navdatafile;
      }
      filePath = s_navdatafile;

      ifstream myfile(filePath);
      if (myfile.is_open())
      {
          string s_Nav[9];
          for (int i = 0; i < 9; ++i)
          {
              if (myfile.eof()) {
                  file_eof = true;
                  continue;
              }
                  myfile >> s_Nav[i];
          }
          if (!file_eof) {
              d_Lat = std::stod(s_Nav[0]);
              s_NS = s_Nav[1];
              d_long = std::stod(s_Nav[2]);
              s_EW = s_Nav[3];
              d_Course = std::stod(s_Nav[4]);
              wmm = std::stod(s_Nav[5]);
              d_SOG = std::stod(s_Nav[6]);
              d_TWA_init = std::stod( s_Nav [7] );
              d_TWS_kn = std::stod( s_Nav [8] );
          }
          myfile.close();
      }
      else {
          cout << "\n\nUnable to find the Navdata file. I'll make a new one\n";
          WriteNavdata();
      }
      if (file_eof) WriteNavdata(); //Navdata file corrupt. False user input??
  }

  void WriteNavdata(void) {
      string filePath = userdata;
      filePath += "\\SendNMEACOM\\";
      if (CreateDirectoryA(filePath.c_str(), NULL) ||
          ERROR_ALREADY_EXISTS == GetLastError())
      {
          filePath = s_navdatafile; //Correct path fixed in ReadNavData()
          ofstream myfile;
          myfile.open(filePath, ios::trunc);
          if (myfile.is_open())
          {
              myfile << d_Lat << "\n";
              myfile << s_NS << "\n";
              myfile << d_long << "\n";
              myfile << s_EW << "\n";
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
      cout << "\n     Hit Esc or Space to exit the program.\n"
          << "     Hit P to show or hide NMEA messaging to screen\n"
          << "     Hit R to restart navigation from initial/saved position\n"
          << "     Hit S to save all actual navdata to the config file\n"
          << "     Hit A to record cruising data to AIS_replay txt-file\n"
          << "     Hit ? or _ to instantly change wind direction 10 degr up or down\n"
          << "     Unless course is obtained from serial input you can:\n"
          << "     Press + or - to instantly change course 10 degr up or down\n"
          << "     Press W to make WPL message and also print to file\n"
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
      case 'a':
      case 'A':
          pIsTouched = true;
          RecordAISdata = !RecordAISdata;
          cout << (RecordAISdata ? "Now prints AIS data to file\n" : "Stop printing AIS data to file\n");
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
          cout << "Enter a new course instead of: " << d_Course << " Or zero \"0\" to quit\n";
          double NewCourse = d_Course;
          NewCourse = GetUserInput(NewCourse, 0.01, 360);
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

  void PrintWPLtoFile() {
      string filePath = userdata;
      filePath += "\\SendNMEACOM";
      if (CreateDirectoryA(filePath.c_str(), NULL) ||
          ERROR_ALREADY_EXISTS == GetLastError()) {
          filePath += "\\WPLmess.txt";
          ofstream myfile;
          myfile.open(filePath, ios::app);
          if (myfile.is_open()) {
              string WPLString = NMEA_WPL;
              WPLString.pop_back(); //Delete the last char(\n) to avoid empty rows in txt-file
              myfile << WPLString;
              myfile.close();
          }
      }
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
                          d_newcourse = stod(token [y]);
                          delimiter = "*"; // Last token has no ','  //$RAHDT  $HCHDM
                          break;
                      case 2:
                          courseunit = "T"; // token[y];
                          //cout << courseunit;
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

              if (d_newcourse && "T" == (courseunit)) { //True heading
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
  void WriteAISdata(void) {
      string MMSI = "266123456",
          STATUS = "5",
          SPEED = static_cast<ostringstream*>(&(ostringstream() << setprecision(0) << fixed << d_SOG * 10))->str(),
          a_LON = static_cast<ostringstream*>(&(ostringstream() << setprecision(7) << fixed << NMEA_degToDecDegr(d_long, LON)))->str(),
          a_LAT = static_cast<ostringstream*>(&(ostringstream() << setprecision(7) << fixed << NMEA_degToDecDegr(d_Lat, LAT)))->str(),
          COURSE = static_cast<ostringstream*>(&(ostringstream() << setprecision(0) << fixed << d_Course))->str(),
          HEADING = COURSE, //s_Mag,
          TIMESTAMP = "2016-12-28T20:19:47",
          SHIP_ID = "225704";
      string filePath = userdata;
      filePath += "\\SendNMEACOM";
      if (CreateDirectoryA(filePath.c_str(), NULL) ||
          ERROR_ALREADY_EXISTS == GetLastError()) {
          filePath += "\\ais_simul.txt";
          ofstream myfile;
          myfile.open(filePath, ios::app);
          if (myfile.is_open()) {
              myfile << "MMSI=\"" << MMSI << "\" "
                  << "STATUS=\"" << STATUS << "\" "
                  << "SPEED=\"" << SPEED << "\" "
                  << "LON=\"" << a_LON << "\" "
                  << "LAT=\"" << a_LAT << "\" "
                  << "COURSE=\"" << COURSE << "\" "
                  << "HEADING=\"" << HEADING << "\" "
                  << "TIMESTAMP=\"" << TIMESTAMP << "\" "
                  << "SHIP_ID=\"" << SHIP_ID << "\"\n";
              myfile.close();
              //cout << "\n" << "New AISdata was saved to:\n" << filePath << "\n";
          } else cout << "Unable to open file ais_simul.txt to write\n";
      } else cout << "Sorry, Unable to create file directory: " << filePath << "\n";
  }

  bool InitSerialPort(void) {
    // Declare variables and structures
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };
    char s[20];
    wchar_t pcCommPort[20];
    char Port[10];
    // Use FileOpen()
    // Try all 255 possible COM ports, check to see if it can be opened, or if
    // not, that an expected error is returned.

    for (int j = 1; j < 256; j++) {
      sprintf_s(s, "\\\\.\\COM%d", j);
      mbstowcs(pcCommPort, s, strlen(s) + 1); //Plus null
      sprintf_s(Port, "COM%d", j);
      // Open the port tentatively
      HANDLE hComm = ::CreateFile(pcCommPort, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

      //  Check for the error returns that indicate a port is there, but not currently useable
      if (hComm == INVALID_HANDLE_VALUE) {
        DWORD dwError = GetLastError();

        if (dwError == ERROR_ACCESS_DENIED ||
          dwError == ERROR_GEN_FAILURE ||
          dwError == ERROR_SHARING_VIOLATION ||
          dwError == ERROR_SEM_TIMEOUT)
          fprintf_s(stderr, "\nFound serial port COM%d. But it's not ready to use\n", j);
      } else {
        CloseHandle(hComm);
        fprintf_s(stderr, "\nFound serial port COM%d and it's ready to use", j);
        _cputs("\nPress key \"y\" to use that port. Any other key continuing search\n");
        if (toupper(_getch()) == 'Y') break;
        //else:
        continue;
      }

      Quit = j == 255 ? true : false; //When for is ended and no port found
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
      OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
      fprintf_s(stderr, "Error. Press a key to exit\n");
      int Dummy = toupper(_getch());
      return 1;
    } else fprintf_s(stderr, "OK\n");


    // Set device parameters (4800 baud, 1 start bit,
    // 1 stop bit, no parity)
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (GetCommState(hSerial, &dcbSerialParams) == 0) {
      fprintf_s(stderr, "Error getting device state. Press a key to exit\n");
      CloseHandle(hSerial);
      int Dummy = toupper(_getch());
      return 1;
    }

    dcbSerialParams.BaudRate = CBR_4800;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (SetCommState(hSerial, &dcbSerialParams) == 0) {
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
    if (SetCommTimeouts(hSerial, &timeouts) == 0) {
      fprintf_s(stderr, "Error setting timeouts. Press a key to exit\n");
      CloseHandle(hSerial);
      int Dummy = toupper(_getch());
      return 1;
    }
    return true;
  }

  bool InitWinsock(void) {
  //Initialise winsock
  printf("\nInitialising Winsock IP connection...");
  if (WSAStartup(MAKEWORD(1, 1), &wsa) != 0) {
    printf("Failed. Error Code : %d", WSAGetLastError());
    exit(EXIT_FAILURE);
  }
  printf("Initialised on port: %d\n", PORT);

  //Test

  char ac[80];
  if (gethostname(ac, sizeof(ac)) == SOCKET_ERROR) { //gethostname
	  cerr << "Error " << WSAGetLastError() <<
		  " when getting local host name." << endl;
	  return 1;
  }
  cout << "\nHost name is " << ac << "." << endl;

  struct hostent *phe = gethostbyname(ac);
  if (phe == 0) {
	  cerr << "Yow! Bad host lookup." << endl;
	  return 1;
  }

  for (int i = 0; phe->h_addr_list[i] != 0; ++i) {
	  //struct in_addr addr;
	  memcpy(&addr, phe->h_addr_list[i], sizeof(struct in_addr));
	  cout << "Address " << i << ": " << inet_ntoa(addr) << endl;
	  char c_adr[20];
	  //c_adr = inet_ntoa(addr);
	  //getservbyname
	  string adr;
	  adr = inet_ntoa(addr);
	  cout << "\n" << adr;
  }

  }

  void SendNMEAtoIP(char s_nmea[80]) {
    //create socket
    if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR) {
      printf("socket() failed with error code : %d", WSAGetLastError());
      exit(EXIT_FAILURE);
    }

    //setup address structure
    memset((char *)&si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    si_other.sin_addr.S_un.S_addr = inet_addr(inet_ntoa(addr));  //INADDR_ANY; //inet_addr(SERVER); //inet_ntoa(addr)

    if (sendto(sock, s_nmea, strlen(s_nmea), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR) {
		int err = WSAGetLastError();
		printf("sendto() failed with error code : %d", err); // WSAGetLastError());
      exit(EXIT_FAILURE);
    }
    //if (!hideNMEA) printf("To IP port:%d  %s\n", PORT, s_nmea);

    closesocket(sock);
    
  }

  void ReceiveUDP(void){
    cout << "Recieve UDP....\n";
    //create socket
    if ( (sock_r = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR ) { //SOCK_STREAM   SOCK_DGRAM
      printf("Recv_socket() failed with error code : %d", WSAGetLastError());
      exit(EXIT_FAILURE);
    }

  //setup address structure
  memset((char *)&si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(PORT);
  si_other.sin_addr.S_un.S_addr = inet_addr(SERVER); // (SERVER);  //INADDR_ANY; //inet_addr(SERVER);

  //Bind
  if (bind(sock_r, (struct sockaddr *)&si_other, sizeof(si_other)) == SOCKET_ERROR) {
    printf("Bind failed with error code : %d", WSAGetLastError());
    exit(EXIT_FAILURE);
  }
  cout << "Bind done";
  int test;
  char data[100];
  memset(recmessage, '\0', BUFLEN);
  //if (recvfrom(sock_r, (char *)&recmessage, (int)sizeof(recmessage), 0, (struct sockaddr *)&si_other, &slen) < 0) {
  //if (recvfrom(sock_r, (char *)&recmessage, (int)sizeof(recmessage), 0, (struct sockaddr *)&si_other, &slen) == SOCKET_ERROR) {
  //  fprintf(stderr, "UDP receive failed with error %d\n", WSAGetLastError());  //"Error receiving data.\n");
  //  exit(EXIT_FAILURE);
  //} else cout << recmessage;

  int ret = recvfrom(sock_r, recmessage, (int)sizeof(recmessage), 0, (struct sockaddr *) &si_other, &slen);
   
  if (ret == SOCKET_ERROR)
  {
	  printf("Error\nCall to recvfrom(s, szMessage, iMessageLen, 0, (struct sockaddr *) &remote_addr, &iRemoteAddrLen); failed with:\n%d\n", WSAGetLastError());
	  exit(1);
  }

  printf("Packet received\n");
  int iMessageLen = ret;        // Length of the data received
  recmessage[iMessageLen] = '\0';     // Convert to cstring
  cout << recmessage;
  
  closesocket(sock_r);
  //WSACleanup(); //??
  }
