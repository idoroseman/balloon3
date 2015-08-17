#include <TinyGPS.h>
#include <util/crc16.h>
#include <SoftwareSerial.h>

#define DORJI_RX 2
#define DORJI_TX 10
#define DORJI_PD 4
#define DORJI_PTT 5
#define DORJI_PWM 3

#define RELAY 7
#define BUZZER 9
#define LED 13 
#define LED2 6
#define GPS_RX 11
#define GPS_TX 12

/*
Ganey Tikva
Sats HDOP Latitude Longitude Fix  Date       Time       Date Alt     Course Speed Card  Distance Course Card  Chars Sentences Checksum
          (deg)    (deg)     Age                        Age  (m)     --- from GPS ----  ---- to London  ----  RX    RX        Fail
--------------------------------------------------------------------------------------------------------------------------------------
4    254  32.06388 34.86903  487  06/27/2015 12:17:33   545  104.40  0.00   0.07  N     3564     317.72 NW    17160 58        112      


NO FIX

|$GPGGA,130003.02,,,,,0,00,99.99,,,,,,*65
|$GPGGA,130004.00,,,,,0,00,99.99,,,,,,*60
|$GPGGA,130005.00,,,,,0,00,99.99,,,,,,*61
|$GPGGA,130006.00,,,,,0,00,99.99,,,,,,*62
Fix!
|$GPGGA,130029.00,3203.83323,N,03452.14076,E,1,04,11.89,104.6,M,17.4,M,,*6E
|$GPGGA,130030.00,3203.83277,N,03452.14068,E,1,04,11.90,104.8,M,17.4,M,,*6F
|$GPGGA,130031.00,3203.83253,N,03452.14062,E,1,04,11.91,103.8,M,17.4,M,,*64
|$GPGGA,130032.00,3203.83230,N,03452.14067,E,1,04,11.91,104.3,M,17.4,M,,*6B
|$GPGGA,130033.00,3203.83216,N,03452.14070,E,1,04,11.92,104.9,M,17.4,M,,*61
|$GPGGA,130034.00,3203.83106,N,03452.14077,E,1,05,6.12,106.4,M,17.4,M,,*53

*/
static const uint8_t PROGMEM _sine_table[] = {
#include "sine_table.h"
};

#define APRS_TX_ENABLE 5
#define APRS_PWM 3

#define RTTY_BAUD 50     // RTTY Baud rate (Recommended = 50)
#define BAUD_RATE      (1200)
#define TABLE_SIZE     (512)
#define PREAMBLE_BYTES (150)
#define REST_BYTES     (5)

int aprs_tx_status = 0, aprs_attempts = 0;
unsigned long startTime;
char comment[3]={
  ' ', ' ', '\0'};
  
#define APRS_TX_INTERVAL 0.25  // APRS TX Interval in minutes
#define APRS_CALLSIGN "4X6UB"
#define APRS_SSID     (11)
  
uint8_t sats = 0; 
int32_t lat = 0, lon = 0, alt = 0;
int32_t maxalt = 0, lat_dec = 0, lon_dec =0 ,tslf=0;
int aprstxstatus=0; 
int temperature1 = 0;
int hasJettisoned = 0;
int count, errorstatus=0;

volatile static uint8_t *_txbuf = 0;
volatile static uint8_t  _txlen = 0;

#define PLAYBACK_RATE    (F_CPU / 256)
#define SAMPLES_PER_BAUD (PLAYBACK_RATE / BAUD_RATE)
#define PHASE_DELTA_1200 (((TABLE_SIZE * 1200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_2200 (((TABLE_SIZE * 2200L) << 7) / PLAYBACK_RATE)
#define PHASE_DELTA_XOR  (PHASE_DELTA_1200 ^ PHASE_DELTA_2200)

TinyGPS gps;
SoftwareSerial nss(GPS_RX, GPS_TX);
SoftwareSerial mySerial(DORJI_RX, DORJI_TX); // RX, TX

long origin_lat = 0.0, origin_lon = 0.0;
long prev_alt = 0, origin_alt = 0;
bool had_fix = false;
bool im_lost = true;
long prev_time = TinyGPS::GPS_INVALID_TIME;
double vslp = 0;
long sim_time_delta = 0;

//#define APRS_TEST

/*****************************************************************************************/
/*                                      M A I N                                          */
/*****************************************************************************************/

void setup()  
{
  Serial.begin(9600);
  Serial.print("Welcome to gsbc.il\r\n");
  pinMode(BUZZER,OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(RELAY,OUTPUT);
  digitalWrite(RELAY, LOW);
  pinMode(LED2,OUTPUT);
  pinMode(LED,OUTPUT);
  ax25_init();
  initialise_interrupt(); 
  nss.begin(4800);
  init_gps();
  delay(4000);
  mySerial.begin(9600);
  init_dorji();
  send_APRS_msg("i'm up and running");
}


void loop()   
{
  bool newdata = false;
  unsigned long start = millis();
  float flat, flon;
  long llat, llon, lalt;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  int sats = 0;
  
  Serial.print("|");
  
  // Every second we print an update
  while (millis() - start < 1000)
  {
    if (feedgps())
    {
      Serial.write('v');
      newdata = true;
    }
  }
  digitalWrite(LED2, !digitalRead(LED2));
  
  // age is calculated with millis() which is currupted by APRS ISR
  gps.get_position(&llat, &llon, &age);
  gps.get_datetime(&date, &time, &age);
  lalt = (long)gps.altitude();
  sats = gps.satellites();
    
#ifdef APRS_TEST
      age = 500;
      llat = 32063880; //32.06388 ; // milionth of deg
      llon = 34869030; //34.86903 ; // milionth of deg
      lalt = 10000 + 260 * sim_time_delta; // in cm
      sats = 3;
      time = 10000000 + 100 * (sim_time_delta++); // hhmmsscc
#endif 

  bool is_fix_good = ((sats != TinyGPS::GPS_INVALID_SATELLITES) 
                   && (llat != TinyGPS::GPS_INVALID_ANGLE) 
                   && (llon != TinyGPS::GPS_INVALID_ANGLE));
  bool is_fix_new = (time != prev_time);
  
  if (is_fix_good && is_fix_new)
  {
    im_lost = false;
    if (!had_fix)
    {
      //first good fix
      Serial.print("\r\nFix!");
      had_fix = true;
      origin_lat = flat;
      origin_lon = flon;
      origin_alt = alt;
      print_fix_data(flat, flon, alt, sats);
      dorji_play_tone();
      prev_time = time;
    }
 
    lat = (long)(llat * 10);
    lon = (long)(llon * 10);
    alt = lalt / 100;
    
    double vertical_speed = (alt - prev_alt) / (timestamp(time) - timestamp(prev_time));
    prev_alt = alt;
    prev_time = time;
    vslp = (vslp + vertical_speed) / 2; 

    // release mechanism    
    if (!hasJettisoned)
    {
      if (alt > 300)
      Jettison(1);  
    if (lon > 349400000) // 34.94
      Jettison(2);
//    if ( uptime > 7 * 60) // seconds
//     Jettison(3);  
//    if (temperature1 < -35) 
//      Jettison(4);
//    if (battvaverage < 2.0)
//      Jettison(5);
    }

#ifndef APRS_TEST
    if (alt < 500) 
      digitalWrite(BUZZER, !digitalRead(BUZZER));
#endif
      
    if (aprs_tx_status==0)
    {
      startTime=millis();
      aprs_tx_status=1;
    }
    digitalWrite(LED, aprstxstatus);
    if(millis() - startTime > (APRS_TX_INTERVAL*60000)) 
    {
      Serial.print("\r\nTX!");
      aprs_tx_status=0;
      send_APRS_possition();
      aprs_attempts++;
    }
  }
  else // no fix
  {
    if (had_fix && !im_lost)
       send_APRS_msg("i'm lost");
    im_lost = true;    
  }  
}

void Jettison(int reason)
{
  char msg[50];
  char* myStrings[] = {"", "altitude", "longitude", "timeout", "temp", "battery", "lost gps"};
  strcpy(msg, "Jetison: ");
  strcat(msg, myStrings[reason]);
  digitalWrite(RELAY, HIGH);
  send_APRS_msg(msg);
  digitalWrite(RELAY, LOW);
      hasJettisoned = reason;
}

/*****************************************************************************************/
/*                                   Dorji                                               */
/*****************************************************************************************/

bool dorji_cmnd(char* cmnd)
{
  bool rc = false;
  Serial.print(">>");
  Serial.print(cmnd);
  mySerial.println(cmnd);
  delay(100);
  Serial.print("<<");
  while (mySerial.available())
  {
    rc = true;
    Serial.write(mySerial.read());
  }
  Serial.print("\r\n");  
  delay(100);
  return rc;
}

void init_dorji()
{
  bool success = false;
  pinMode(DORJI_TX,OUTPUT);
  pinMode(DORJI_PD,OUTPUT);
  pinMode(DORJI_PTT,OUTPUT);
  pinMode(DORJI_PWM,OUTPUT);
  digitalWrite(DORJI_PD, HIGH);   
  digitalWrite(DORJI_PTT, HIGH);   

  mySerial.listen();
  while (!success)
  {
    success =  dorji_cmnd("AT+DMOCONNECT\r\n");
    success &= dorji_cmnd("AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000\r\n");                                                                                                                                                                                                                                                                                                                                          
    if (!success)
    {
      Serial.print("retry in 3 sec\r\n");
      delay(3000);
    }
  }
  nss.listen();
}

void dorji_play_tone()
{
      digitalWrite(DORJI_PTT, LOW);
      delay(500);
      tone(DORJI_PWM,100); delay(150);
      tone(DORJI_PWM,300); delay(150);
      tone(DORJI_PWM,700); delay(150);
      digitalWrite(DORJI_PTT, HIGH);
}

/*****************************************************************************************/
/*                                    APRS RealTime                                      */
/*****************************************************************************************/

void initialise_interrupt() 
{
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1;  // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);   // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

ISR(TIMER1_COMPA_vect)
{

}

ISR(TIMER2_OVF_vect)
{

  static uint16_t phase  = 0;
  static uint16_t step   = PHASE_DELTA_1200;
  static uint16_t sample = 0;
  static uint8_t rest    = PREAMBLE_BYTES + REST_BYTES;
  static uint8_t byte;
  static uint8_t bit     = 7;
  static int8_t bc       = 0;
  /* Update the PWM output */
  OCR2B = pgm_read_byte(&_sine_table[(phase >> 7) & 0x1FF]);
  phase += step;

  if(++sample < SAMPLES_PER_BAUD) return;
  sample = 0;

  /* Zero-bit insertion */
  if(bc == 5)
  {
    step ^= PHASE_DELTA_XOR;
    bc = 0;
    return;
  }

  /* Load the next byte */
  if(++bit == 8)
  {
    bit = 0;

    if(rest > REST_BYTES || !_txlen)
    {
      if(!--rest)
      {
        /* Disable radio and interrupt */
        PORTD |= _BV(APRS_TX_ENABLE); // Same as digitalWrite(APRS_TX_ENABLE, HIGH); but more efficient
        aprstxstatus=0;
        TIMSK2 &= ~_BV(TOIE2);

        /* Prepare state for next run */
        phase = sample = 0;
        step  = PHASE_DELTA_1200;
        rest  = PREAMBLE_BYTES + REST_BYTES;
        bit   = 7;
        bc    = 0;
        return;
      }

      /* Rest period, transmit ax.25 header */
      byte = 0x7E;
      bc = -1;
    }
    else
    {
      /* Read the next byte from memory */
      byte = *(_txbuf++);
      if(!--_txlen) rest = REST_BYTES + 2;
      if(bc < 0) bc = 0;
    }
  }

  /* Find the next bit */
  if(byte & 1)
  {
    /* 1: Output frequency stays the same */
    if(bc >= 0) bc++;
  }
  else
  {
    /* 0: Toggle the output frequency */
    step ^= PHASE_DELTA_XOR;
    if(bc >= 0) bc = 0;
  }

  byte >>= 1;
}

void ax25_init(void)
{
  /* Fast PWM mode, non-inverting output on OC2A */
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  pinMode(APRS_PWM, OUTPUT);
}

bool isTransmiting()
{
  return digitalRead(DORJI_PTT) == LOW;
}

long timestamp(long time)
{
  long hour = time / 1000000;
  long minute = (time / 10000) % 100;
  long second = (time / 100) % 100;
  long hundredths = time % 100;
  return hour * 3600 + minute * 60 + second;
}

/*****************************************************************************************/
/*                                        AX.25                                          */
/*****************************************************************************************/

void ax25_frame(char *scallsign, char sssid, char *dcallsign, char dssid,
char *path1, char ttl1, char *path2, char ttl2, char *data, ...)
{
  static uint8_t frame[100];
  uint8_t *s;
  uint16_t x;
  va_list va;

  va_start(va, data);

  /* Pause while there is still data transmitting */
  while(_txlen);

  /* Write in the callsigns and paths */
  s = _ax25_callsign(frame, dcallsign, dssid);
  s = _ax25_callsign(s, scallsign, sssid);
  if(path1) s = _ax25_callsign(s, path1, ttl1);
  if(path2) s = _ax25_callsign(s, path2, ttl2);

  /* Mark the end of the callsigns */
  s[-1] |= 1;

  *(s++) = 0x03; /* Control, 0x03 = APRS-UI frame */
  *(s++) = 0xF0; /* Protocol ID: 0xF0 = no layer 3 data */

  vsnprintf((char *) s, 100 - (s - frame) - 2, data, va);
  va_end(va);

  /* Calculate and append the checksum */
  for(x = 0xFFFF, s = frame; *s; s++)
    x = _crc_ccitt_update(x, *s);

  *(s++) = ~(x & 0xFF);
  *(s++) = ~((x >> 8) & 0xFF);

  /* Point the interrupt at the data to be transmit */
  _txbuf = frame;
  _txlen = s - frame;

  /* Enable the timer and key the radio */
  TIMSK2 |= _BV(TOIE2);
  //PORTA |= TXENABLE;
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}

static uint8_t *_ax25_callsign(uint8_t *s, char *callsign, char ssid)
{
  char i;
  for(i = 0; i < 6; i++)
  {
    if(*callsign) *(s++) = *(callsign++) << 1;
    else *(s++) = ' ' << 1;
  }
  *(s++) = ('0' + ssid) << 1;
  return(s);
}

uint16_t crccat(char *msg)
{
  uint16_t x;


  while(*msg == '$') msg++;


  for(x = 0xFFFF; *msg; msg++)
    x = _crc_xmodem_update(x, *msg);


  snprintf_P(msg, 8, PSTR("*%04X\n"), x);

  return(x);
}

/*****************************************************************************************/
/*                                    APRS Background                                    */
/*****************************************************************************************/
uint16_t seq = 0;
  
void send_APRS_possition() 
{
  ax25_init();
  gps_off();
  tx_aprs_pos();
  while (isTransmiting())
   ;
  gps_on();
}

void send_APRS_msg(char* msg) 
{
  static int  msgid = 1;
  int retrys = 3;
  
  ax25_init();
  gps_off();
  Serial.print("APRS MSG(");
  Serial.print(msgid);
  Serial.print("): ");
  Serial.print(msg);
  Serial.print(" retry ");
  for (int retry=0; retry<retrys; retry++)
      {
        aprs_tx_status=0;
        tx_aprs_msg(msg, msgid);
        aprs_attempts++; 
        Serial.print(retry);
#ifndef APRS_TEST
        delay(10000);       
#endif
      }
  msgid++;
  Serial.println();
  gps_on();
}

void tx_aprs_pos()
{
  aprstxstatus=1;
  PORTD &= ~_BV(APRS_TX_ENABLE); // Same as digitalWrite(APRS_TX_ENABLE, LOW); but more efficient
  char slat[5];
  char slng[5];
  char stlm[9];

  double aprs_lat, aprs_lon;

  /* Convert the UBLOX-style coordinates to
   	 * the APRS compressed format */
  aprs_lat = 900000000 - lat ;
  aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
  aprs_lon = 900000000 + lon/2;
  aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
  int32_t aprs_alt = alt * 32808 / 10000;


  /* Construct the compressed telemetry format */
  ax25_base91enc(stlm + 0, 2, seq);
  ax25_frame(
    APRS_CALLSIGN, APRS_SSID,
    "APRS", 0,
    //0, 0, 0, 0,
    "WIDE1", 1, "WIDE2",1,
    //"WIDE2", 1,
    "!/%s%sO   /A=%06ld|%s|%s/%s,%d,%i,vs=%i,%02X, //bit.ly/1Et51q9",
    ax25_base91enc(slat, 4, aprs_lat),
    ax25_base91enc(slng, 4, aprs_lon),
    aprs_alt, stlm, comment,APRS_CALLSIGN, count, errorstatus, (int)vslp, hasJettisoned
      );
  seq++;
}

void tx_aprs_msg(char* msg, int msgid)
{
  aprstxstatus=1;
  PORTD &= ~_BV(APRS_TX_ENABLE); // Same as digitalWrite(APRS_TX_ENABLE, LOW); but more efficient

  /* Construct the compressed telemetry format */
  ax25_frame(
    APRS_CALLSIGN, APRS_SSID,
    "APRS", 0,
    //0, 0, 0, 0,
    "WIDE1", 1, "WIDE2",1,
    //"WIDE2", 1,
//    ":4X6UB    :%s{%05d",msg, msgid
      ":BLN1BALON:%s{%05d",msg, msgid
    );  
  seq++;
}

/*****************************************************************************************/
/*                                         GPS                                          */
/*****************************************************************************************/

void init_gps()
{
  // Turning off all GPS NMEA strings apart from GPGGA on the uBlox modules
  nss.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  nss.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  nss.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  nss.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  nss.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  nss.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
}

void gps_off()
{
  // during aprs transmition, the softserial interrupt disturbs pwm timing
  // so as a hack I switch to the dorji softserial which is idle
  mySerial.listen();
}

void gps_on()
{
  nss.listen();
}

static bool feedgps()
{
  char c;
  while (nss.available())
  {
    c = nss.read();
//    Serial.write(c);
    if (gps.encode(c))
      return true;
  }
  return false;
}

void print_fix_data(double flat, double flon, double alt, int sats)
{
      Serial.print("sats=");
      Serial.print(sats);
     Serial.print(" lat=");
      Serial.print(flat);
      Serial.print(" lon=");
      Serial.print(flon);
      Serial.print(" alt=");
      Serial.print(alt);
      Serial.println();
}
