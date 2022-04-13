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
    #define TerminalAttached  false
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
  WiFi.lowPowerMode();
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
  
  // Serial port initialization
 if(TerminalAttached) {
     Serial.begin(57600);
     delay(5000);
     Serial.println("\nYMFC-32 - MRK1010 Receiver/Web Server Started");
     Serial.println("Version " + String(WIFININA_GENERIC_VERSION));

     // check and make sure we are using the lastest Firmware
     String fv = WiFi.firmwareVersion();
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
