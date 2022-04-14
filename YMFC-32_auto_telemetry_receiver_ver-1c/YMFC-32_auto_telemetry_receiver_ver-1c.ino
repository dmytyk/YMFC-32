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
