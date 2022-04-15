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
