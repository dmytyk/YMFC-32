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
#ifndef Receive_Buffer_Size 54
    // Receiver Buffer Size 
    #define Receive_Buffer_Size   54
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
                  webSocketServer.sendData("R:" + cmd + " - Toggle Sent");
                }
                if (subcommand == "SPD") {
                  ymc32_command = 11;
                  init_telemetry_data("0");
                  webSocketServer.sendData("R:" + cmd + " - Save PID Sent");
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
