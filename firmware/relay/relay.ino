/*
Author: Yuki
Date:2025.7.14
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
#include <EEPROM.h>

#define set_id_key 0

#define DIO0 2
#define DIO1 6
#define DIO2 7
#define DIO5 8

#define LORA_RST 9
#define LORA_CS 10

#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK 13

#define RELAY1 4
#define RELAY2 3
#define RELAY3 A3
#define RELAY4 A2

bool relay1_status = false;
bool relay2_status = false;
bool relay3_status = false;
bool relay4_status = false;

uint8_t remote_id[7];
bool set_id_flag = false;

uint8_t lora_send_data[20];
uint8_t lora_send_len = 0;

uint8_t receive_data[20];

uint8_t relay_control_value = 0; //1-->relay1, 2-->relay2, 3-->relay3, 4-->relay4,

#define FREQUENCY 915        // Operating frequency in MHz (typical: 433, 868, 915 depending on region)
#define BANDWIDTH 125.0      // Bandwidth in kHz (7.8–500). Larger BW = higher data rate, lower sensitivity and range
#define SPREADING_FACTOR 12  // Spreading Factor (6–12). Higher SF = longer range, lower data rate
#define CODING_RATE 7        // Coding Rate denominator (5–8, meaning CR = 4/5 to 4/8). Higher = better error correction, lower speed
#define OUTPUT_POWER 13       // Output power in dBm (2–20). Higher = longer range, higher power consumption
#define PREAMBLE_LEN 8       // Preamble length in symbols (6–65535)
#define GAIN 0               // Gain setting (0 = auto gain, 1–6 = manual gain if supported)


SX1276 radio = new Module(LORA_CS, DIO0, LORA_RST, DIO1);

void setup()
{
    Serial.begin(115200);
    pin_conf();
    read_id();  //read remote ID

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
        while (true)
            ;
    }

    key_scan();
}

void loop()
{
    int state = radio.receive(receive_data,20);

    if (state == ERR_NONE)
    {
        Serial.println(F("receive_success!"));

        Serial.print(F("[SX1278] RSSI:\t\t\t"));
        Serial.print(radio.getRSSI());
        Serial.println(F(" dBm"));
        Serial.print(F("[SX1278] SNR:\t\t\t"));
        Serial.print(radio.getSNR());
        Serial.println(F(" dB"));
        Serial.print(F("[SX1278] Frequency error:\t"));
        Serial.print(radio.getFrequencyError());
        Serial.println(F(" Hz"));

        Serial.print(F("[SX1276] Data:\t\t\t"));
        for(int i = 0; i < 13; i++)
        {
          Serial.print(receive_data[i], HEX);
        }
        Serial.println();

        uint16_t analyse_crc = CRC16_CCITT(receive_data, receive_data[7]);
        Serial.print("analyse_crc:");
        Serial.println(analyse_crc, HEX);
        uint16_t crc = (receive_data[8] << 8) | receive_data[9];
        Serial.print("crc:");
        Serial.println(crc, HEX);
    
        if(crc == analyse_crc)
        {   
            Serial.println("crc is right");

            if(set_id_flag)
            {
                memcpy(remote_id, receive_data, 6);
                remote_id[6] = '\0';

                for (int i = 0; i < 7; i++)
                {
                    EEPROM.write(i, remote_id[i]);
                }

                Serial.println("Remote ID saved to EEPROM.");
                int tx_state = radio.transmit("ID saved OK");
                            
                if (tx_state == ERR_NONE)
                {
                    Serial.println("Send: ID saved OK");
                }
                else
                {
                    Serial.print("Send fail, error code: ");
                    Serial.println(tx_state);
                }

                set_id_flag = false;
            }
            else
            {
                analyse_lora_data(receive_data);
            }
        }
        else
        {
            Serial.println("crc is not right");
        }

    }
    else if (state == ERR_RX_TIMEOUT)
    {
        // timeout occurred while waiting for a packet
        //Serial.println(F("timeout!"));
    }
    else if (state == ERR_CRC_MISMATCH)
    {
        // packet was received, but is malformed
        Serial.println(F("CRC error!"));
    }
    else
    {
        // some other error occurred
        Serial.print(F("failed, code "));
        Serial.println(state);
    }
}

void analyse_lora_data(uint8_t *data)
{
    if(memcmp(remote_id, data, 6) == 0)
    {
        relay_control_value = data[6];
        relay_control(relay_control_value);

        lora_send_len = 0;
        memset(lora_send_data, 0, sizeof(lora_send_data));
        memcpy(lora_send_data, data, 6);
        lora_send_len += 6;
        lora_send_data[lora_send_len++] = relay_control_value;
        uint8_t len = lora_send_len;
        lora_send_data[lora_send_len++] = len;
        uint16_t crc_send = CRC16_CCITT(lora_send_data, len);
        lora_send_data[lora_send_len++] = crc_send >> 8;
        lora_send_data[lora_send_len++] = crc_send & 0xff;

        Serial.print("lora_send_data:");
        for(int i = 0; i < lora_send_len; i++)
        {
            Serial.print(lora_send_data[i], HEX);
        }
        Serial.println();
        Serial.println();
        radio.transmit(lora_send_data, lora_send_len);

        memset(receive_data, 0, sizeof(receive_data));                
    }
    else
    {
        Serial.println("ID is wrong");
    }
}

void relay_control(uint8_t control_value)
{
    Serial.print("control_value");
    Serial.println(control_value, HEX);

    if(control_value == 1 )
    {
        relay1_status = !relay1_status;
        digitalWrite(RELAY1, relay1_status ? HIGH : LOW);
    }
    else if(control_value == 2)
    {
        relay2_status = !relay2_status;
        digitalWrite(RELAY2, relay2_status ? HIGH : LOW);
    }
    else if(control_value == 3)
    {
        relay3_status = !relay3_status;
        digitalWrite(RELAY3, relay3_status ? HIGH : LOW);
    }
    else if(control_value == 4)
    {
        relay4_status = !relay4_status;
        digitalWrite(RELAY4, relay4_status ? HIGH : LOW);
    }
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

//继电器及按键初始化
void pin_conf()
{
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    pinMode(RELAY3, OUTPUT);
    pinMode(RELAY4, OUTPUT);
    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);
    digitalWrite(RELAY3, LOW);
    digitalWrite(RELAY4, LOW);

    pinMode(set_id_key, INPUT_PULLUP);
}

void key_scan()
{
    bool key_state = digitalRead(set_id_key);
    if (key_state==LOW)
    {
        delay(50);
        if (key_state==LOW)
        {
            set_id_flag = true;
        }
    }
}

//read remote id
void read_id()
{
    for (uint8_t i = 0; i < 7; i++)
    {
        remote_id[i] = EEPROM.read(i);
        if (remote_id[i] == '\0') break;
    }

    Serial.print("READ ID from EEPROM: ");
    Serial.println((char*)remote_id);
}