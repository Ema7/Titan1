/*
 TITAN HAB flight computer software
 Modifications copyright 2015 by Emanuel Bombasaro

 This code is suitable for the Arduino Mega 2560 rev3.
 Function file contianing all relvant functions regarding MTX2 and APRS

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
created 25-XII-2014 EMBO
edited  14- II-2015 EMBO
*/

// MTX2 ###################################################################
// function setMTX2Frequency ++++++++++++++++++++++++++++++++++++++++++++++
void setMTX2Frequency()
{
  float _mtx2comp;
  int _mtx2int;
  long _mtx2fractional;
  char _mtx2command[17];
  MTX2_EN.begin(9600);
  _mtx2comp=(MTX2_FREQ+0.0015)/6.5;
  _mtx2int=_mtx2comp;
  _mtx2fractional=float(((_mtx2comp-_mtx2int)+1)*524288);
  snprintf(_mtx2command,17,"@PRG_%02X%06lX\r",_mtx2int-1, _mtx2fractional);
  delay(100);
  MTX2_EN.print(_mtx2command);
  delay(50);
  MTX2_EN.end();
}
// end setMTX2Frequency function

// APRS ###################################################################
// start send_APRS function +++++++++++++++++++++++++++++++++++++++++++++++
void send_APRS() {
  ax25_init();
  tx_aprs();
}
//end send_APRS function

// start tx_aprs function +++++++++++++++++++++++++++++++++++++++++++++++++
void tx_aprs()
{
  aprstxstatus=1;
  PORTD |= _BV(HX1_ENABLE); // Same as digitalWrite(HX1_ENABLE, HIGH); but more efficient
  char slat[5];
  char slng[5];
  char stlm[9];
  static uint16_t seq = 0;
  double aprs_lat, aprs_lon;

  /* Convert the UBLOX-style coordinates to
   	 * the APRS compressed format */
  aprs_lat = 900000000 - lat;
  aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
  aprs_lon = 900000000 + lon / 2;
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
    "!/%s%sO   /A=%06ld|%s|%s/%s,%d,%i,%i'C",
    ax25_base91enc(slat, 4, aprs_lat),
    ax25_base91enc(slng, 4, aprs_lon),
    aprs_alt, stlm, comment,APRS_CALLSIGN, count, errorstatus,temperature1);
  seq++;
}
// end tx_aprs function

// start ax25_frame function ++++++++++++++++++++++++++++++++++++++++++++++
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
  //TIMSK2 |= _BV(TOIE2);
  TIMSK3 |= _BV(TOIE3);
  //PORTA |= TXENABLE;
}
// end ax25_frame function

// start _ax25_callsign function ++++++++++++++++++++++++++++++++++++++++++
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
// end _ax25_callsign function

// start ax25_base91enc function ++++++++++++++++++++++++++++++++++++++++++
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
// end ax25_base91enc function

// EOF ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
