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
                }
                webSocketServer.sendData("R:" + cmd + " - Toggle Sent");
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
