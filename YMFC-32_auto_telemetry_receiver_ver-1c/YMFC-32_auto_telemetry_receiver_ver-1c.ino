///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////

#include <global.h>
#include <base64.h>
#include <WebSocketClient.h>
#include <WebSocketServer.h>
#define _WIFININA_LOGLEVEL_       1
#include <WiFiNINA_Generic.h>
#include "network_parameters.h"

// global var
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
const byte IPLastByte  = 99;
const short webPort     = 80;
const short socketPort  = 8080;

// WiFi
WiFiServer      webServer(webPort);
WiFiServer      socketServer(socketPort);
WebSocketServer webSocketServer;
WiFiClient      socketClient;

// Console Attached
#ifndef TerminalAttached
    // true = terminal attached (send serial messages), false = no terminal attached no messages 
    #define TerminalAttached  true
#endif

// Buffer Constant Both sides must Match
#ifndef Receive_Buffer_Size 55
    // Receiver Buffer Size 
    #define Receive_Buffer_Size   55
#endif

// Buffer Constant Both sides must Match
#ifndef Send_Buffer_Size
    // Send Buffer Size 
    #define Send_Buffer_Size   6
#endif

// telemetry
uint8_t telemetry_lost; 
uint8_t check_byte, temp_byte;

//from YMC-32
uint8_t rec_telemetry_last_byte = Receive_Buffer_Size;                                                               //this is the last byte of the receivers telemetry buffer they have to match the YMC-32's TX
uint8_t receive_buffer[Receive_Buffer_Size + 1], receive_buffer_counter, receive_byte_previous, receive_byte, receive_state;              // rec_telemetry_last_byte + 1

//to YMC-32                                              
uint8_t send_telemetry_last_byte = Send_Buffer_Size;                                                          //this is the last byte of the transmitters telemetry buffer they have to match the YMC-32's RX  
uint8_t telemetry_send_byte, telemetry_loop_counter, ready_to_send;
uint32_t telemetry_buffer_byte;

// YMC-32 commands
uint8_t ymc32_command;
float ymc32_fval;

uint8_t error, alarm_sound, flight_mode;
uint8_t alarm_silence;
uint8_t start_byte, flight_timer_start;
uint8_t hours,minutes,seconds;
uint8_t heading_lock;
uint8_t number_used_sats;
uint8_t fix_type;

uint8_t debug_byte;

int8_t page, previous_page;

uint32_t current_receive, current_receive_min, current_receive_max, last_receive, next_sound, flight_timer, flight_timer_previous, flight_timer_from_start, flight_time_from_eeprom;
uint32_t hours_flight_time, minutes_flight_time, seconds_flight_time;
int32_t loop_timer,loop_timer_min,loop_timer_max, l_lat_gps, l_lon_gps;

int16_t temperature,temperature_min,temperature_max, button_push, button_store,roll_angle,roll_angle_min,roll_angle_max, pitch_angle, pitch_angle_min, pitch_angle_max;
int16_t altitude_meters, altitude_meters_min, altitude_meters_max;
uint16_t key_press_timer;
int16_t manual_takeoff, takeoff_throttle;
uint16_t actual_compass_heading;

float battery_voltage,battery_voltage_min,battery_voltage_max;

float pid_p_gain_roll;               //Gain setting for the pitch and roll P-controller
float pid_i_gain_roll;              //Gain setting for the pitch and roll I-controller
float pid_d_gain_roll;              //Gain setting for the pitch and roll D-controller
float pid_p_gain_yaw;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw;                //Gain setting for the pitch D-controller (default = 0.0).
float pid_p_gain_altitude;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude;          //Gain setting for the altitude D-controller (default = 0.75).

byte led;

// Background
boolean Backgroundinit = true;

// Return to Home
uint8_t return_to_home_command;

// YMC32 webpage, gzipped and base64 encoding to save space
char webpage_base64[] = "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";

void printWifiStatus()
{
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("Signal strength (RSSI): "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
    Serial.print("Gateway: "); Serial.println(WiFi.gatewayIP());
    Serial.print("Netmask: "); Serial.println(WiFi.subnetMask());
    Serial.print("Webpage is at http://"); Serial.print(WiFi.localIP()); Serial.println("/");
    Serial.print("Websocket is at http://"); Serial.print(WiFi.localIP()); Serial.println(":" + (String)socketPort + "/");
}

void WiFiConnect() 
{
  while (WiFi.status() != WL_CONNECTED) 
  {
   if(TerminalAttached) {
       Serial.println("Connecting to " + (String)ssid + " ...");
   }
    WiFi.begin(ssid, pass);
    delay(5000);
  }
  
  IPAddress IP = WiFi.localIP();
  IP[3] = IPLastByte;
  
  WiFi.config(IP, WiFi.gatewayIP(), WiFi.gatewayIP(), WiFi.subnetMask());
 if(TerminalAttached) {
   Serial.println("Connected to " + (String)ssid);
 }
  
  webServer.begin();
  socketServer.begin();
 if(TerminalAttached) {
   printWifiStatus();
 }
  //WiFi.lowPowerMode();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(0, OUTPUT);                                   // Green LED
                                                        //Set the serial output to 9600bps.
  Serial1.begin(9600);  
  delay(250);  

  led = 0;
  ready_to_send = 0;
  receive_state = 1;
  check_byte = 0;

  // check and make sure we are using the lastest Firmware
  String fv = WiFi.firmwareVersion();

  // Serial port initialization
  if(TerminalAttached) {
     Serial.begin(57600);
     delay(5000);
     Serial.println("\nYMFC-32 - MRK1010 Receiver/Web Server Started");
     Serial.println("Version " + String(WIFININA_GENERIC_VERSION));

     // check and make sure we are using the lastest Firmware
     if (fv < WIFI_FIRMWARE_LATEST_VERSION)
     {
         Serial.print("Your current firmware NINA FW v");
         Serial.println(fv);
         Serial.print("Please upgrade the firmware to NINA FW v");
         Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
     }
 }
}

void loop() {
  if(WiFi.status() != WL_CONNECTED)
  {
     if(TerminalAttached) {
         Serial.println("Lost WiFi connection");
     }
      WiFi.end();
      WiFiConnect();
  }

  WiFiClient webClient = webServer.available();

  if(webClient.connected())
  {
     if(TerminalAttached) {
         Serial.print("New client: "); Serial.print(webClient.remoteIP()); Serial.print(":"); Serial.println(webClient.remotePort());
     }
      String header = "";

      while(webClient.available())
      {
          char ch = webClient.read();

          if (ch != '\r') 
          {
          header += ch;
          }
      
          if (header.substring(header.length() - 2) == "\n\n") 
          {
         if(TerminalAttached) {
             Serial.print(header.substring(0, header.length() - 1));
         }
        
          if (header.indexOf("GET / HTTP") > -1) 
          {
              const int webpage_base64_length = sizeof(webpage_base64);
              const int webpage_gz_length = base64_dec_len(webpage_base64, webpage_base64_length);
              char webpage_gz[webpage_gz_length];
              base64_decode(webpage_gz, webpage_base64, webpage_base64_length);
              int packetsize = 1024;
              int done = 0;
              
              webClient.println("HTTP/1.1 200 OK\nContent-Type: text/html\nContent-Encoding: gzip\n");
          
              while (webpage_gz_length > done) 
              {
                  if (webpage_gz_length - done < packetsize) {
              
                  packetsize = webpage_gz_length - done;
                  }
            
                  webClient.write(webpage_gz + done, packetsize * sizeof(char));
                  done = done + packetsize;
              }
             if(TerminalAttached) {
                 Serial.println("--Interface webpage sent");
             }
          } 
          else 
          {
              webClient.println("HTTP/1.1 404 Not Found\nContent-Type: text/plain; charset=utf-8\n\n404 Not Found\n");
             if(TerminalAttached) {
                 Serial.println("--Page not found");
             }
          }
        
          webClient.stop();
         if(TerminalAttached) {
             Serial.println("--Client disconnected");
         }
      }
    }
  }

  if(!socketClient.connected()) 
  {
    socketClient = socketServer.available();
    
    if (socketClient.connected() && webSocketServer.handshake(socketClient)) 
    {
       if(TerminalAttached) {
           Serial.print("\n--Websocket connected to: ");
           Serial.print(socketClient.remoteIP());
           Serial.print(":");
           Serial.println(socketClient.remotePort());
       }
    } 
    else 
    {
        socketClient.stop();
        delay(100);
    }
  }

  if(socketClient.connected()) 
  {
    // Background Init - setup the background tasks, runs only once
    if(Backgroundinit == true) {
        Backgroundinit = false;

        String data = webSocketServer.getData();
       if(TerminalAttached) {
           Serial.println("Websocket Flushed");
       }

        // flush the serial1 port - Arduion to Arduino comms
        while(Serial1.available()) {
            char ch = Serial1.read();
        }
       if(TerminalAttached) {
           Serial.println("Serial Port 1 Flushed");
           Serial.println("Background Init Complete");
       }
    }
    
    // Background Process 1
    // see if we have a command/request from the user 
    String data = webSocketServer.getData();
    if (data.length() > 0) 
    {
      //String cmd = data.substring(0, data.indexOf(":"));
      //String setting = data.substring(data.indexOf(":") + 1);
      // get tools to parse incoming request
      char buf[data.length()+1];
      data.toCharArray(buf, sizeof(buf));
      char *token;
      char *pter = buf;
      byte i = 0;
      String cmd;
      String subcommand;
      String usrVal; 
      while ((token = strtok_r(pter, ":", &pter)))
      {
        switch(i) {
          case 0:
          cmd = token;
        break;
          case 1:
          subcommand = token;
        break;
          case 2:
          usrVal = token;
        break;
        }
        i++;
      }
     if(TerminalAttached) {
       Serial.println("CMD: " + String(cmd));
       Serial.println("Subcommand: " + String(subcommand));
       Serial.println("UsrVal: " + String(usrVal));
     }

      // process command
      switch (cmd.toInt()) {
        //do something (1-9 are the PID buttons - they don't get sent, they are added to 20 and 21)
        // minus
        case 20:
            subcommand = "Minus";
            break;
        // plus
        case 21:
            subcommand = "Plus";
            break;
        // user command
        case 30:
            if(subcommand == "Ledon") {
                digitalWrite(0, HIGH);
                webSocketServer.sendData("R:" + cmd + " - " + subcommand);
            } else if (subcommand == "Ledoff") {
                digitalWrite(0, LOW);
                webSocketServer.sendData("R:" + cmd + " - " + subcommand);
            }else if (subcommand == "Pidr") {
                webSocketServer.sendData("R:" + String(pid_p_gain_roll) + ":" + String(pid_i_gain_roll) + ":" + String(pid_d_gain_roll));
            }else if (subcommand == "Pidy") {
                webSocketServer.sendData("R:" + String(pid_p_gain_yaw) + ":" + String(pid_i_gain_yaw) + ":" + String(pid_d_gain_yaw));
            } else if (subcommand == "Pida") {
                webSocketServer.sendData("R:" + String(pid_p_gain_altitude) + ":" + String(pid_i_gain_altitude) + ":" + String(pid_d_gain_altitude));
            } else if (subcommand == "Rst") {
              reset_data();
              webSocketServer.sendData("R:Reset Sent");
            } else {
                webSocketServer.sendData("E:" + cmd + " - " + subcommand);
            }
            break;
        // user command
        case 40:
            if (subcommand == "CRP") {
              ymc32_command = 1;
              init_telemetry_data(usrVal);
              webSocketServer.sendData("P:" + String(pid_p_gain_roll) + ">>" + String(usrVal));             
            } else if (subcommand == "CRI") {
              ymc32_command = 2;
              init_telemetry_data(usrVal);
              webSocketServer.sendData("P:" + String(pid_i_gain_roll) + ">>" + String(usrVal));
            } else if (subcommand == "CRD") {
              ymc32_command = 3;
              init_telemetry_data(usrVal);
              webSocketServer.sendData("P:" + String(pid_d_gain_roll) + ">>" + String(usrVal));
            } else if (subcommand == "CYP") {
              ymc32_command = 4;
              init_telemetry_data(usrVal);
              webSocketServer.sendData("P:" + String(pid_p_gain_yaw) + ">>" + String(usrVal));
            } else if (subcommand == "CYI") {
              ymc32_command = 5;
              init_telemetry_data(usrVal);
               webSocketServer.sendData("P:" + String(pid_i_gain_yaw) + ">>" + String(usrVal));
           } else if (subcommand == "CYD") {
              ymc32_command = 6;
              init_telemetry_data(usrVal);
              webSocketServer.sendData("P:" + String(pid_d_gain_yaw) + ">>" + String(usrVal));
            } else if (subcommand == "CAP") {
              ymc32_command = 7;
              init_telemetry_data(usrVal);
              webSocketServer.sendData("P:" + String(pid_p_gain_altitude) + ">>" + String(usrVal));
            } else if (subcommand == "CAI") {
              ymc32_command = 8;
              init_telemetry_data(usrVal);
              webSocketServer.sendData("P:" + String(pid_i_gain_altitude) + ">>" + String(usrVal));
            } else if (subcommand == "CAD") {
              ymc32_command = 9;
              init_telemetry_data(usrVal);
              webSocketServer.sendData("P:" + String(pid_d_gain_altitude) + ">>" + String(usrVal));
            } else {
                webSocketServer.sendData("E:" + cmd + " - " + subcommand);
            }
            break;
            case 50:
                if (subcommand == "TTC") {
                  ymc32_command = 10;
                  init_telemetry_data("0");
                  webSocketServer.sendData("R:" + cmd + " - MTT Sent");
                }
                if (subcommand == "SPD") {
                  ymc32_command = 11;
                  init_telemetry_data("0");
                  webSocketServer.sendData("R:" + cmd + " - Save PID Sent");
                }
                if (subcommand == "RTH") {
                  ymc32_command = 12;
                  init_telemetry_data("0");
                  webSocketServer.sendData("R:" + cmd + " - RTH Sent");
                }
                if (subcommand == "RDC") {
                  ymc32_command = 99;
                  init_telemetry_data("0");
                  webSocketServer.sendData("R:" + cmd + " - Drop Requested");
                }
            break;
        default:
            break;
      }

      // process response
      switch (cmd.toInt()) {
          // minus response
          case 20:
              webSocketServer.sendData("R:" + cmd + " - " + subcommand);
              break;
          // plus response
          case 21:
              webSocketServer.sendData("R:" + cmd + " - " + subcommand);
              break;
          case 30:
              break;
          case 40:
              break;
          case 50:
              break;
          default:
              webSocketServer.sendData("E:" + cmd + " - " + subcommand);
              break;
        }
    }
  }

    if(Serial1.available()) {                                                //If there are bytes available.
      receive_byte = Serial1.read();                                       //Load them in the received_buffer array.
        switch (receive_state) {
            // waiting for start
            case 1:
                if(receive_byte == 'B') {
                    if(receive_byte_previous == 'J') {
                        receive_buffer_counter = 2;
                        receive_buffer[0] = 'J';
                        receive_buffer[1] = 'B';
                        receive_state = 2;
                    }
                }
                receive_byte_previous = receive_byte;
            break;
            // building buffer
            case 2:
                receive_buffer[receive_buffer_counter++] = receive_byte;
                // see if we have a valid buffer
                if(receive_buffer_counter == rec_telemetry_last_byte) {
                    check_byte = 0;                                                                         //Reset the check_byte variable.
                    for(temp_byte=0;temp_byte < (rec_telemetry_last_byte - 1); temp_byte ++) {
                        check_byte ^= receive_buffer[temp_byte];                                            //Calculate the check_byte.
                    }
                    // valid buffer
                    if(check_byte == receive_buffer[(rec_telemetry_last_byte - 1)]) {
                      get_data();                                                           //If there are two start signatures detected there could be a complete data set available.
                      process_data();
                     if(TerminalAttached) {
                       print_telemetry_data();
                     }
                      
                      // send status up date to webserver
                      if(socketClient.connected()) {
                        webpage_data();
                      }
      
                    } else {
                        receive_byte_previous = receive_byte;
                        receive_state = 1;
                    }
                } else if (receive_buffer_counter > rec_telemetry_last_byte) {
                    receive_byte_previous = receive_byte;
                    receive_state = 1;
                }
            break;
        }
    }

  // see if we have something to send, if so do it
  if(ready_to_send == 1) {
    send_telemetry_data();
  }

  // background delay and heart beat
  if(led == 0) {
    led = 1;
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    led = 0;
    digitalWrite(LED_BUILTIN, LOW);
  }
}

//When there are two start signatures received the received data between them can be tested to se if it is a valid data stream.
void get_data(void){
    current_receive = millis() - last_receive;
    last_receive = millis();                                                              //Remember when this reception has arrived.
    //In the following lines the different variables are restored from the valid data stream.
    //The name of the variables are the same as in the YMFC-32 flight controller program.
    error = receive_buffer[2];
    flight_mode = receive_buffer[3];
    debug_byte = receive_buffer[4];
    temperature = receive_buffer[5] | receive_buffer[6] << 8;
    roll_angle = receive_buffer[7] - 100;
    pitch_angle = receive_buffer[8] - 100;
    start_byte = receive_buffer[9];
    altitude_meters = (receive_buffer[10] | receive_buffer[11] << 8) - 1000;
    takeoff_throttle = receive_buffer[12] | receive_buffer[13] << 8;
    actual_compass_heading = receive_buffer[14] | receive_buffer[15] << 8;
    heading_lock = receive_buffer[16];
    number_used_sats = receive_buffer[17];
    fix_type = receive_buffer[18];
    l_lat_gps = (int32_t)receive_buffer[19] | (int32_t)receive_buffer[20] << 8 | (int32_t)receive_buffer[21] << 16 | (int32_t)receive_buffer[22] << 24;
    l_lon_gps = (int32_t)receive_buffer[23] | (int32_t)receive_buffer[24] << 8 | (int32_t)receive_buffer[25] << 16 | (int32_t)receive_buffer[26] << 24;
    loop_timer = (int32_t)receive_buffer[27] | (int32_t)receive_buffer[28] << 8 | (int32_t)receive_buffer[29] << 16 | (int32_t)receive_buffer[30] << 24;
    pid_p_gain_roll = (float)(receive_buffer[31] | receive_buffer[32] << 8)/1000;
    pid_i_gain_roll = (float)(receive_buffer[33] | receive_buffer[34] << 8)/1000;
    pid_d_gain_roll = (float)(receive_buffer[35] | receive_buffer[36] << 8)/1000;
    pid_p_gain_yaw = (float)(receive_buffer[37] | receive_buffer[38] << 8)/1000;
    pid_i_gain_yaw = (float)(receive_buffer[39] | receive_buffer[40] << 8)/1000;
    pid_d_gain_yaw = (float)(receive_buffer[41] | receive_buffer[42] << 8)/1000;
    pid_p_gain_altitude = (float)(receive_buffer[43] | receive_buffer[44] << 8)/1000;
    pid_i_gain_altitude = (float)(receive_buffer[45] | receive_buffer[46] << 8)/1000;
    pid_d_gain_altitude = (float)(receive_buffer[47] | receive_buffer[48] << 8)/1000;
    battery_voltage = (float)(receive_buffer[49] | receive_buffer[50] << 8)/100;
    manual_takeoff = receive_buffer[51] | receive_buffer[52] << 8;
    return_to_home_command = receive_buffer[53];
}

//Process the data for the Web page
void process_data(void){
    //System Status
    //create battery min and max values
    if((battery_voltage <= battery_voltage_min) | (battery_voltage_min == 0)) {
    battery_voltage_min = battery_voltage;
    }
    if(battery_voltage >= battery_voltage_max) {
    battery_voltage_max = battery_voltage;
    }

    //telemetry processing time
    if((current_receive <=  current_receive_min) | ( current_receive_min == 0)) {
       current_receive_min =  current_receive;
    }
    if(current_receive >=  current_receive_max) {
       current_receive_max =  current_receive;
    }

    //loop_timer processing time
    if((loop_timer <=  loop_timer_min) | ( loop_timer_min == 0)) {
       loop_timer_min =  loop_timer;
    }
    if(loop_timer >=  loop_timer_max) {
       loop_timer_max =  loop_timer;
    }

    // Flight Status
    //Roll angle
    if((roll_angle <=  roll_angle_min) | ( roll_angle_min == 0)) {
       roll_angle_min =  roll_angle;
    }
    if(roll_angle >=  roll_angle_max) {
       roll_angle_max =  roll_angle;
    }

    //Pitch angle
    if((pitch_angle <=  pitch_angle_min) | ( pitch_angle_min == 0)) {
       pitch_angle_min =  pitch_angle;
    }
    if(pitch_angle >=  pitch_angle_max) {
       pitch_angle_max =  pitch_angle;
    }

    //Altitude Meters
    if((altitude_meters <=  altitude_meters_min) | ( altitude_meters_min == 0)) {
       altitude_meters_min =  altitude_meters;
    }
    if(altitude_meters >=  altitude_meters_max) {
       altitude_meters_max =  altitude_meters;
    }

    //Temperature
    if((temperature <=  temperature_min) | ( temperature_min == 0)) {
       temperature_min =  temperature;
    }
    if(temperature >=  temperature_max) {
       temperature_max =  temperature;
    }
}

//Process the data for the Web page
void reset_data(void){
    ymc32_command = 0;
    ymc32_fval = 0;
    check_byte = 0;
    ready_to_send = 1; 
    battery_voltage_min = battery_voltage_max = 0;
    current_receive_min = current_receive_max = 0;
    loop_timer_min = loop_timer_max = 0;
    roll_angle_min = roll_angle_max = 0;
    pitch_angle_min = pitch_angle_max = 0;
    altitude_meters_min = altitude_meters_max = 0;
    temperature_min = temperature_max = 0;
}
