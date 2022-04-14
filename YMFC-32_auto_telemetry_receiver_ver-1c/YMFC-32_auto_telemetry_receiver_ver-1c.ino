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
