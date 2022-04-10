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
#ifndef Receive_Buffer_Size 52
    // Receiver Buffer Size 
    #define Receive_Buffer_Size   52
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
int16_t takeoff_throttle;
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
char webpage_base64[] = "H4sICD0gU2IEAHdlYnBhZ2UuaHRtbADlXG1v0zAQ/s6vMEGCVsBa1g3G2lWCbcDEBoiNNwGq3MRtLdw4it2VgvbfOSdO0iRtlrgtCKgEpM69PPf4Yjtnl87No9eHF5/eHKORHLPujY76BzHsDg8s4lqqgWCnewPBpyOpZKT76ay1ff+CMDIm0p+hE1cSf4Bt0mkE97Us3MTIHmFfEHlgTeTg/p6lbwk5C8T0p8+dGfoZfI2bsP1t6POJ69y3OeP+Prp1vHu8++xZOxa7iq+8jPKAu/L+AI8pm+2jJz7F7B4S2BX3BfHpYJEFh15uDRgdjqSQWE5ExqDHBZWUu/sI9wVnE0naqfuSe/vowa73Pd3MyEBCezNqz/sceoK6A27ibvuxgT+POp7PGTNxuCy+1k6xQ2rqcLdl5tAxdfi4aebQm+Gpib9ljO7s7BQzauhvt2XmzzH097hp5s/DTJr4W8bn7s5eMZ+G/nZbZv4cQ3+Pm2b+bBgPfW70SDxo7Rk8ExMYaFdwur1n8uRP6YCuf/R++LAgV8VMSDJewWmzqlfVnf2JlNzlg0F2zuS+Q2CmBHfUaS+4FU2lfQaza/ua+fYpm2Qg6xvTEc0G42HHoe5wH0E46k/67pQ6cqSCVZ2Xn6gF/UHg7nb2pj3xhfLncaoWGBn6yHd5H8OMDQTbRN2/hi3XlKwg3OvIeu4T4v71bNl8PMZuwFglvnJ0vPZh+ZiJOpd7eT4gKPizmI+9Qjqaa6aj09BL1E4jXP121BoV/oGnHtkMC3Fgza8XrWh9jPuMoED3wFqwhh0En3bEJgQbEqJp1WZCU374JWlwFIfCw+6BtWshhR+uQvxW5FMbfpAxrFlsqfEGvHT63WcBenQewO80+l1YvDuJR/jmF4IZKYVRrjUDq3s48X24LCV7Rt1ycvh7Kbl3LpVCSVYIy+ke+z730SF3nWD8DmiB9izf1Dmwjt/2Yjmr26zIoNMF8v2wC0iRm0CsF4iZeLnA34iaLC5GPpeSFbrSsj0ta+RPZ9YZdwpdhWI9EDPy8oJgNWigU25/K3Kj5XpKzsBPZFJNy1b3rXqxeOIONYl54bx/pdILVMC7gVYPngpTTfz9Ok0f+gDiqh2RIcxg9dXYeUOlPapIT6BTnZ9AzYygQPUPMfSEQYFk4pSnRytU4kbrVCZG61Vi5YyAuliRlQsy9ogP85BfnphEpxI3iVplehLVSgwV5I2+wv1woQFLi+6N1ApDV4f+vsXF8zfn6ASQ+2Os5kaD5YXTfTUZ94mP+ACdw8THGCyjRdFAH8r3Xg96ibzBgN99Rr+ji5lXOHeBTA9kjCauJ7acYAYLjLEHnYz0/FTkLdToaY2e1jDxfYrDR7zIWyhjNimfcnd4vQMQijxUeyyUdvhyIqzwGUm+h3qxWFTojOSShjl3+kVHQk9Ghizk4jF8exBiPXz7JrYw/+ZtIe7ajNrfDiyobuv3phq5hDDvBRbugXodpsSTo/tvkJqTO41QOXQfhJWHTLOQaQXI2xHkk5Uhn5SH7GQhOxUgtyLIRytDPioPWZVrtb/4e0nAOxrwp3WkxSc8LYWXZvDS8nh3I7zryImyeJ0MXqc83ocR3nUkRFm8qtys3cXfS+J9pPE+WUc+wAKsFF6awUvL492L8K4jH8ridTJ4nfJ4H0d415EPi/EuXnolVfPfvfraNl19NfXqS0WLDkPwRiuvw5Gq3im1/L0Odb2J1H2l6mhW1MuAk/5QM2fT6t52+8JrFw6kzbhDU1XHVI+6DtiNelXp1a0YXCheObiE51aW5+5bIjxYShCxktFuR9GCfYJ1qBDDmAiBh0RospIGTUKmzedToRIhsAtGH0ITLPq4y2YQcGS+W23xpB2prZm/NrPfAfjVUjtMpkq5rRiLk3unTHK3yiW3MpzJ7nNo/ptyGyK4JP58emfbNBNRczbJd+Ikf7TGJFdbgX+mML96jn+gz6jKcZfY6t3ZvEIfZvvEpTYODJ1S9xvaj97MwsryWS8lY1X3ceJeYggEHWE47TPw+Rh9OjtsbWccvT0+7AWFdav7irvEqFPDzdY/063GJZG4W88D9Oa9+a/ut6RLgU+xBFMzTVPp2hzksda0KlT0Eq2qtcBEs0op8D1ncvVaaXTq7oKOq5RLtVoP1CpWTOc0DYqmc9qV6qZnlLFVyTrl3KvGk9KoTJFSMmJHKRoQY/tcVCgoC9unngzvXmIfhRMxOkDWVOw3Gha6C6OV6/DpFuPhNLA14kKqCR5uWft7zb1mw2on+rCTRiTouxPGkmYbM0YcaB5gJkjSHi5m3mMGt5rZ5hMnbA2aBxM3nPLscPY7DxzV6nMnERxuT8bA4NaQyGNG1OXT2YlTy89k9S0KRvwXF2en4OJOR50WCM8gAJXEgZEydBLUXtXN7h0FQ3+SGMkUfSB9DSWkrp4T3IK3UY+4IB/FEKJOPgvpWW9Uap/BjeMiThJWcrYhj1svyOahj+exx70OoNQcD4LjLQcutoQHNfaatW/V09HQAapF0p+bX9EBZNqxFdksFXd6rZiK+m7kX+XmF9dqr2BV2KpWeQGDxEFlrRdEPY9p71cIEejchQy8/e8ZeFONAXgjXnf4YLJq7KCycuBHCwMXU7URnQg/+JqVSoYOMGw9tfbju6UjThYqmaEjdrv9tb2KWTXlLTPdWtU0/r7M9E6B6T68Nn5rFzB5bMLk/FkfEy6vRXVugmruaNBGQL00AZU5RLQZZM9MkCVnjjYD6oUJqPkTShtB9c4E1aLt9I2ge2+CTm/Ab6YXP5ggWrxJvxF8H03whdv6G8r7T0aI4BzA5iBdmEBKva4a4Sppeu3TZtr8xqbOUxNW1Wvu+glVVjfDpbK8WRrfmtAYnew0m1BLmF07kbHpzTH5xoTJ4AjoBqgM7G6Iy8D2hsl8YkLmE2Y+sVxrdO00asOb4/DIhMPkbOl6aUzsrp/JxPYmyHTIAE+YNOAy3mpahuhBdURX6Xf9woIaUb6XVgITSZtxQWr14upcIJQxtoHSIQ9+r2V1j6iwF5UPkw8cs1EzI5/IWqpKe0/9YK2ZiiaJKVPQ1fXTqOoMv/46Vid2TinskAHCmtUncHSYTFzGsWPdSwVfjqKr9o0itq/q2Yrz/Omh5OTQvD86CPbooaCTVLSzndFoqPqPHFGhZRBcYfBwSdA9BE8KNEgEr8YpLbIFL/HDACO8ObwKnKA788ec0n2Qq6gnn3RVPemLoDaVBzslCPbYdSUK3la0usjKDagvZIBfgddSECeWKkA73HJkMzjdkNHUOm7CSV5E4EuiaMECpIgquGcdRDpJT8Rh3oQ4s72gE05IiHp5aS+2Uc897qBXui+uKnWle2dpd1FnaScH+Tj/ZCXsqTg5I7BzM6zFGlGWZ1I8f5wqnd+JgYW0wrvFAqIZ7JKAVVC6pooK488lTn5SnB8PFb5avOlkwd8RO1GDdlSvVvHVjoOeuJNXnWPQxPsVTPvqV0+1FJmLJwWQyShnntCyY/uS0rja0nvFkTr+d05YMIjftHSVPD8wZ7Ijfx4pjijp/WzPK41rul6JJH1fqt+11XrJTTIlnunk63sn1zN5gjoNvXMKV/rXv43gv8j5BZ6ZC2oyRwAA";

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
            }else if (subcommand == "Ppisr") {
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
