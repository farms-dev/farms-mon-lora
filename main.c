/*******************************************************************************
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ddbased in https://github.com/tftelkamp/single_chan_pkt_fwd
 *
 ******************************************************************************
 *
 * this code is a modification of the
 * https://github.com/tftelkamp/single_chan_pkt_fwd
 *
 *******************************************************************************/

#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>

#include <sys/ioctl.h>
#include <net/if.h>

using namespace std;

#include <wiringPi.h>
#include <wiringPiSPI.h>

//###########################
#include <curl/curl.h>
#include "include/nlohmann/json.hpp"

using json = nlohmann::json;


typedef bool boolean;
typedef unsigned char byte;

static const int CHANNEL = 0;

byte currentMode = 0x81;

char message[256];
char b64[256];

bool sx1272 = true;

byte receivedbytes;

struct sockaddr_in si_other;
int s, slen=sizeof(si_other);
struct ifreq ifr;

uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;

enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };

/*******************************************************************************
 *
 * Configure these values!
 *
 *******************************************************************************/

// SX1272 - Raspberry connections
int ssPin = 6;
int dio0  = 7;
int RST   = 0;

// Set spreading factor (SF7 - SF12)
sf_t sf = SF7;

// Set center frequency
uint32_t  freq = 868100000; // in Mhz! (868.1)

// Set location
float lat=0.0;
float lon=0.0;
int   alt=0;

// #############################################
// #############################################

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB	    0x1F
#define REG_PKT_SNR_VALUE	    0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH      0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD	            0x39
#define REG_VERSION	            0x42

#define SX72_MODE_RX_CONTINUOS      0x85
#define SX72_MODE_TX                0x83
#define SX72_MODE_SLEEP             0x80
//#define SX72_MODE_STANDBY         0x81
#define SX72_MODE_STANDBY           0x01

#define OPMODE_RX        0x05

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66

#define BUFLEN 2048  //Max length of buffer

#define PROTOCOL_VERSION  1
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK  1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK  4

#define TX_BUFF_SIZE  2048
#define STATUS_SIZE	  1024

void selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

void unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}

byte readRegister(byte addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}

void writeRegister(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}

boolean receivePkt(char *payload)
{
    // clear rxDone
    writeRegister(REG_IRQ_FLAGS, 0x40);

    int irqflags = readRegister(REG_IRQ_FLAGS);

    cp_nb_rx_rcv++;

    //  payload crc: 0x20
    if((irqflags & 0x20) == 0x20)
    {
        printf("CRC error\n");
        writeRegister(REG_IRQ_FLAGS, 0x20);
        return false;
    } else {

        cp_nb_rx_ok++;

        byte currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readRegister(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;

        writeRegister(REG_FIFO_ADDR_PTR, currentAddr);

        for(int i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readRegister(REG_FIFO);
        }
    }
    return true;
}

void SetupLoRa()
{
    digitalWrite(RST, HIGH);
    delay(100);
    digitalWrite(RST, LOW);
    delay(100);

    byte version = readRegister(REG_VERSION);

    if (version == 0x22) {
        // sx1272
        printf("SX1272 detected, starting.\n");
        sx1272 = true;
    } else {
        // sx1276?
        digitalWrite(RST, LOW);
        delay(100);
        digitalWrite(RST, HIGH);
        delay(100);
        version = readRegister(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            printf("SX1276 detected, starting.\n");
            sx1272 = false;
        } else {
            printf("Unrecognized transceiver.\n");
            //printf("Version: 0x%x\n",version);
            exit(1);
        }
    }

    //writeRegister(REG_OPMODE, SX72_MODE_SLEEP);
    writeRegister(REG_OPMODE, 0x00);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeRegister(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeRegister(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    writeRegister(REG_SYNC_WORD, 0x34); // LoRaWAN public sync word

    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG,0x0B);
        } else {
            writeRegister(REG_MODEM_CONFIG,0x0A);
        }
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    } else {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG3,0x0C);
        } else {
            writeRegister(REG_MODEM_CONFIG3,0x04);
        }
        writeRegister(REG_MODEM_CONFIG,0x72);
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    writeRegister(REG_MAX_PAYLOAD_LENGTH,0x80);
    writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
    writeRegister(REG_HOP_PERIOD,0xFF);
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));

    // Set Continous Receive Mode
    writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
    //writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);

}

void  clientAPI(char* sensor_id, char* sensor_data) {
  CURL *curl;
  CURLcode res;

  res = curl_global_init(CURL_GLOBAL_DEFAULT);
  if(res != CURLE_OK) {
    fprintf(stderr, "curl_global_init() failed: %s\n",
      curl_easy_strerror(res));
  }

  curl = curl_easy_init();
  if(curl) {
  
    char json[200] = "{\"query\": \"mutation { addSensorDataServerTime(sensorId: ";
    strcat (json, sensor_id);
    strcat (json, " data: ");
    strcat (json, sensor_data);
    strcat (json, " ) {id} } \" } ");
    printf("\nquery: %s", json);
  
    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers, "charset: utf-8");
    
    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:4000/graphql");
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json);
    
    res = curl_easy_perform(curl);
    if(res != CURLE_OK) {
      fprintf(stderr, "curl_easy_perform() failed: %s\n",
      curl_easy_strerror(res));
      curl_easy_cleanup(curl);
    }
  }
  curl_global_cleanup();
}

void sendToAPI (char* message) {

  printf("\n========== test begin ==========\n\n");
  time_t rawtime;
  struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );

  printf("local time: %s", asctime (timeinfo));
  printf("from the radio: %s", message);

  std::string message_str = message;
  int begin = message_str.find("|¡|");

  printf("position of '|¡|': %d", begin);

  if (begin >= 0) {

    int end = message_str.find("|!|");

    printf("\nposition of '|!|': %d", end);

    if (end > 0) {
      std::string data = message_str.substr(begin + 4, end - 4);
    
      std::cout << "\n\ndata: " << data << "\n\n";
  
      try
      {
        auto sensor_json = json::parse(data);
    
        std::string counter = sensor_json["count"];
        std::cout << "count: " <<  counter << '\n';
    
        float counter_num = strtof((counter).c_str(), 0);
        if (counter_num > -1 ) {
          printf("counter num: %f \n", counter_num);
    
          json list_json = sensor_json["list"];
    
          if (!list_json.empty()) {
    
            for (auto& sensor : list_json) {

    	      std::cout << sensor << '\n';
              std::string sensor_id;

              sensor_id = sensor["sensorId"];

              std::cout << "sensorId: " << sensor_id << '\n';

              if (sensor_id.length() > 0) {
      
            	 float sensor_id_num; 	
                 sensor_id_num = strtof((sensor_id).c_str(), 0);
        	 if (sensor_id_num > 0 ) {
        
        	    std::string sensor_data;
      	    	    sensor_data = sensor["value"];
                  
		    std::cout << "value: " << sensor_data << '\n';
      
      	    	    if (sensor_data.length() > 0) {
            	      float sensor_data_num; 	
      	      	      sensor_data_num = strtof((sensor_data).c_str(), 0);
      		      
		      // TODO fix it, because it can be 0 or less
      	      	      if (sensor_data_num > 0 ) {
      	    	
			cout << "sensor_data: " + sensor_data;
      
        		char *sensor_data_char = new char[sensor_data.length() + 1];
        		strcpy(sensor_data_char, sensor_data.c_str());	
        		char *sensor_id_char = new char[sensor_id.length() + 1];
        		strcpy(sensor_id_char, sensor_id.c_str());	
      
      			clientAPI(sensor_id_char, sensor_data_char);
                      } else {
            	        printf("\nSensor data is not a num");
      	      	    }
                  } else {
      	      	    printf("\nSendor data not found");
                  }
                } else {
            	  printf("\nSensor id is not a num");
      	  	}
              } else {
                printf("\nSensor id not found");
              }
    	    }
          } else {
            printf("\nJson is empty");
          }
        } else {
          printf("\ninvalid counter");
        }
      }
      catch (json::exception& e)
      {
        std::cout << "message: " << e.what() << '\n' << "exception id: " << e.id << std::endl;
      }
    } else {
      printf("End marker not found");
    }
  } else {
    printf("Begin marker not found");
  }
  printf("\n=========== test end ===========\n");
}

void receivepacket() {
    long int SNR;
    int rssicorr;

    if(digitalRead(dio0) == 1)
    {
        if(receivePkt(message)) {
            byte value = readRegister(REG_PKT_SNR_VALUE);
            if( value & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                SNR = -value;
            }
            else
            {
                // Divide by 4
                SNR = ( value & 0xFF ) >> 2;
            }
            
            if (sx1272) {
                rssicorr = 139;
            } else {
                rssicorr = 157;
            }

            printf("Packet RSSI: %d, ",readRegister(0x1A)-rssicorr);
            printf("RSSI: %d, ",readRegister(0x1B)-rssicorr);
            printf("SNR: %li, ",SNR);
            printf("Length: %i",(int)receivedbytes);
            printf("\n");

            fflush(stdout);

	    sendToAPI(message);

        } // received a message

    } // dio0=1
}

int main () {
    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);

    wiringPiSPISetup(CHANNEL, 500000);

    SetupLoRa();

    printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
    printf("------------------\n");

    uint8_t u = SX72_MODE_SLEEP; //maybe another name
    if (sx1272 == false)
        u |= 0x8;   // TBD: sx1276 high freq
    writeRegister(REG_OPMODE, u);
   
    writeRegister(REG_OPMODE, SX72_MODE_STANDBY);
    writeRegister(REG_OPMODE, OPMODE_RX);

    while(1) {
        receivepacket();
        delay(1);
    }

    return (0);
}
