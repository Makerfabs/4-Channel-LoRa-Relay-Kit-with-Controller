/*
Author: Yuki
Date:2025.7.29
Code version: v1.0.0
Note: 

Library version:
Arduino IDE v2.3.6
Arduino AVR Board v1.8.6
RadioLib v4.6.0

Tools:
Board: Arduino Pro or Pro Mini
Processor: ATmega328P(3.3V, 8 MHz)
*/
#include <RadioLib.h>

#define LED1 A4
#define KEY2 7
#define KEY3 A0
#define KEY1 4
#define KEY4 6
#define Power_ON_OFF 8

#define LORA_LED_PIN A3
unsigned long LEDStartTime = 0;
unsigned long LEDruntime = 500;
bool LEDRun = false;

#define DIO0 2
#define DIO1 A1
#define LORA_CS 10
#define LORA_RST 9

SX1276 radio = new Module(LORA_CS, DIO0, LORA_RST, DIO1);

#define FREQUENCY 915        // Operating frequency in MHz (typical: 433, 868, 915 depending on region)
#define BANDWIDTH 125.0      // Bandwidth in kHz (7.8–500). Larger BW = higher data rate, lower sensitivity and range
#define SPREADING_FACTOR 12  // Spreading Factor (6–12). Higher SF = longer range, lower data rate
#define CODING_RATE 7        // Coding Rate denominator (5–8, meaning CR = 4/5 to 4/8). Higher = better error correction, lower speed
#define OUTPUT_POWER 7       // Output power in dBm (2–20). Higher = longer range, higher power consumption
#define PREAMBLE_LEN 8       // Preamble length in symbols (6–65535)
#define GAIN 0               // Gain setting (0 = auto gain, 1–6 = manual gain if supported)


char remote_id[7] = "070002"; //ID number

bool key1_pressed = false;
bool key2_pressed = false;
bool key3_pressed = false;
bool key4_pressed = false;

uint8_t key_control = 0;

uint8_t lora_send_data[20];
uint8_t lora_send_len = 0;
bool send_flag = false;
uint8_t receive_data[20];
bool receive_flag = false;

unsigned long start_receive_Millis=0;
unsigned long receive_timeout = 5000;

unsigned long start_work_time=0;
unsigned long work_timeout = 15000;

void setup()
{
  pin_conf();
  Serial.begin(115200);
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE, SX127X_SYNC_WORD, OUTPUT_POWER, PREAMBLE_LEN, GAIN);

  if (state == ERR_NONE)
  {
      Serial.println(F("success!"));
  }
  else
  {
      Serial.print(F("failed, code "));
      Serial.println(state);
      digitalWrite(Power_ON_OFF, LOW);//sleep
      //while (true);
  }

  start_work_time = millis();
}

void loop()
{
  key_scan();
}

void key_scan()
{
  if(send_flag==false && receive_flag == false)
  {
    bool key1_state = digitalRead(KEY1);
    bool key2_state = digitalRead(KEY2);
    bool key3_state = digitalRead(KEY3);
    bool key4_state = digitalRead(KEY4);

    if (key1_state == LOW)
    {
      key1_pressed = true;
    }
    if (key1_state == HIGH && key1_pressed)
    {
      Serial.println("KEY1 press");
      key1_pressed = false;
      key_control = 1;
      send_flag = true;
    }

    if (key2_state == LOW)
    {
      key2_pressed = true;
    }
    if (key2_state == HIGH && key2_pressed)
    {
      Serial.println("KEY2 press");
      key2_pressed = false;
      key_control = 2;
      send_flag = true;
    }

    if (key3_state == LOW)
    {
      key3_pressed = true;
    }
    if (key3_state == HIGH && key3_pressed)
    {
      Serial.println("KEY3 press");
      key3_pressed = false;
      key_control = 3;
      send_flag = true;
    }

    if (key4_state == LOW)
    {
      key4_pressed = true;
    }
    if (key4_state == HIGH && key4_pressed)
    {
      Serial.println("KEY4 press");
      key4_pressed = false;
      key_control = 4;
      send_flag = true;
    }

  }
  else if(send_flag)
  {
    key_send();
  }
  else if(receive_flag)
  {
    receive_lora();
  }

  if(LEDRun)
  {
    if (millis() - LEDStartTime >= LEDruntime)
    {
      digitalWrite(LORA_LED_PIN, LOW);
      LEDRun = false;
      Serial.println("LED stop-->sleep");
      digitalWrite(Power_ON_OFF, LOW);//sleep
    }
  }

  if (millis() - start_work_time >= work_timeout)
  {
    Serial.println("work timeout-->sleep");
    digitalWrite(Power_ON_OFF, LOW);//sleep
  }

}

void key_send()
{
    digitalWrite(LED1, LOW);
    lora_send_len = 0;

    memcpy(lora_send_data, remote_id, 6);
    lora_send_len += 6;

    lora_send_data[lora_send_len++] = key_control;

    uint8_t len = lora_send_len;
    lora_send_data[lora_send_len++] = len;

    uint16_t crc_send = CRC16_CCITT(lora_send_data, len);
    lora_send_data[lora_send_len++] = crc_send >> 8;
    lora_send_data[lora_send_len++] = crc_send & 0xff;

    Serial.print("Send data:  ");
    for(int i = 0; i < lora_send_len; i++)
    {
      Serial.print(lora_send_data[i], HEX);
      Serial.print("-");
    }
    Serial.println();
    
    int tx_state = radio.transmit(lora_send_data, lora_send_len);

    if(tx_state == ERR_NONE)
    {
      Serial.println("send success");
    }
    else
    {
      Serial.print("send fail, error code: ");
      Serial.println(tx_state);
    }

    Serial.print("change to receive!!!\n");
    
    receive_flag=true;
    send_flag = false;
    start_receive_Millis = millis();
}

void receive_lora()
{
    memset(receive_data, 0, sizeof(receive_data));
    int state = radio.receive(receive_data);
    int sent_cnt = 0;
    if (state == ERR_NONE)
    {
        int len = radio.getPacketLength();
        // Serial.println(F("received success!"));
        // Serial.print(F("[SX1276] RSSI:\t\t\t"));
        // Serial.print(radio.getRSSI());
        // Serial.println(F(" dBm"));

        // Serial.print(F("[SX1276] SNR:\t\t\t"));
        // Serial.print(radio.getSNR());
        // Serial.println(F(" dB"));

        // // print frequency error
        // // of the last received packet
        // Serial.print(F("[SX1276] Frequency error:\t"));
        // Serial.print(radio.getFrequencyError());
        // Serial.println(F(" Hz"));

        // Serial.print(F("[SX1276] Data:\t\t\t"));

        // for(int i = 0; i < len; i++)
        // {
        //   Serial.print(receive_data[i], HEX);
        // }
        Serial.println("Receive OK");
        Serial.println();
        Serial.println();

        digitalWrite(LORA_LED_PIN, HIGH);
        LEDRun=true;
        LEDStartTime = millis();
        receive_flag=false;
        return;
    }
    else if (state == ERR_RX_TIMEOUT)
    {
        // timeout occurred while waiting for a packet
        Serial.println(F("lora timeout!"));
        if(sent_cnt<3)
        {
          int tx_state = radio.transmit(lora_send_data, lora_send_len);
          if(tx_state == ERR_NONE)
          {
            Serial.println("resend success " + String(sent_cnt));
          }
          else
          {
            Serial.print("resend fail " + String(sent_cnt) + "error code: ");
            Serial.println(tx_state);
          }
        }
        sent_cnt++;
    }
    else if (state == ERR_CRC_MISMATCH)
    {
        // packet was received, but is malformed
        Serial.println(F("CRC error!"));
        int tx_state = radio.transmit(lora_send_data, lora_send_len);
    }
    else
    {
        // some other error occurred
        Serial.print(F("failed, code "));
        Serial.println(state);
        int tx_state = radio.transmit(lora_send_data, lora_send_len);
    }

    if (millis() - start_receive_Millis > receive_timeout) 
    {
        Serial.println("receive timeout-->sleep");
        Serial.println();
        Serial.println();
        receive_flag = false;
        digitalWrite(Power_ON_OFF, LOW);//sleep
        return;
    }
  
}

void pin_conf()
{
  pinMode(Power_ON_OFF, OUTPUT);
  digitalWrite(Power_ON_OFF, HIGH); //open power

  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

  pinMode(KEY1, INPUT_PULLUP);
  pinMode(KEY2, INPUT_PULLUP);
  pinMode(KEY3, INPUT_PULLUP);
  pinMode(KEY4, INPUT_PULLUP);

  pinMode(LORA_LED_PIN, OUTPUT);
  digitalWrite(LORA_LED_PIN, LOW);
}

/****************************Info********************************************** 
 * Name:    CRC-16/CCITT        x16+x12+x5+1 
 * Width:	16
 * Poly:    0x1021 
 * Init:    0x0000 
 * Refin:   True 
 * Refout:  True 
 * Xorout:  0x0000 
 * Alias:   CRC-CCITT,CRC-16/CCITT-TRUE,CRC-16/KERMIT 
 *****************************************************************************/ 
unsigned short CRC16_CCITT(unsigned char *data, unsigned int datalen)
{
	unsigned short wCRCin = 0x0000;
	unsigned short wCPoly = 0x1021;
	unsigned char wChar = 0;
	
	while (datalen--) 	
	{
		wChar = *(data++);
		InvertUint8(&wChar,&wChar);
		wCRCin ^= (wChar << 8);
		for(int i = 0;i < 8;i++)
		{
			if(wCRCin & 0x8000)
				wCRCin = (wCRCin << 1) ^ wCPoly;
			else
				wCRCin = wCRCin << 1;
		}
	}
	InvertUint16(&wCRCin,&wCRCin);
	return (wCRCin);

}

/****************************Info********************************************** 
 * Name:    InvertUint8 
 * Note: 	把字节颠倒过来，如0x12变成0x48
			0x12: 0001 0010
			0x48: 0100 1000
 *****************************************************************************/
void InvertUint8(unsigned char *dBuf,unsigned char *srcBuf)
{
	int i;
	unsigned char tmp[4]={0};
 
	for(i=0;i< 8;i++)
	{
		if(srcBuf[0]& (1 << i))
		tmp[0]|=1<<(7-i);
	}
	dBuf[0] = tmp[0];
	
}

void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf)
{
	int i;
	unsigned short tmp[4]={0};
 
	for(i=0;i< 16;i++)
	{
		if(srcBuf[0]& (1 << i))
		tmp[0]|=1<<(15 - i);
	}
	dBuf[0] = tmp[0];

}
