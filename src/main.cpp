/*
  NASA Clipper Duet Echo Sounder/Log to NMEA2000 converter (ClipperDuet2N2k)
  2023-01-30 by Soenke J. Peters

  This code reads the display data from the ht1621 lcd driver of the NASA Clipper Duet
  and converts it to useful values to be send over the NMEA2000 network.
*/

// SPDX-License-Identifier: MIT

// TODO: Refactoring

/*
 Timeout in s for Trip and Total distance
 if more than this time has elapsed between the display of the two values,
 the Trip record is considered invalid and therefore no distance log values are sent
 */
#define DISTANCE_TIMEOUT 60
// Consider this a safe sensor to keel distance (in m) in case the offset has not been read from the device
// This is a positive value
#define SAFE_OFFSET 3

#define PIN_HTDATA GPIO_NUM_13    // HT1621 DATA is SPI MOSI on our ESP32 SPI Slave implementation
#define PIN_HTDATAOUT GPIO_NUM_12 // unused, but must a usable pin for SPI MISO
#define PIN_HTCLK GPIO_NUM_14     // HT1621 WR is SPI Clk on our ESP32 SPI Slave implementation
#define PIN_HTCS GPIO_NUM_27      // HT1621 CS is SPI CS on our ESP32 SPI Slave implementation

#define ESP32_CAN_TX_PIN GPIO_NUM_5
#define ESP32_CAN_RX_PIN GPIO_NUM_4

// Uncomment to have some printf() status messages on the serial console.
// ATTN: Output of NMEA2000 to serial well be changed to text format (default: Actisense)
// #define DEBUG

/* *********************************************************************************************
  No user-configurable stuff below
*/

// This is set from git describe by version.py run as a pre:extra_script in PlatformIO
#ifndef N2K_SOFTWARE_VERSION
#warning "no N2K_SOFTWARE_VERSION from git"
#define N2K_SOFTWARE_VERSION "0.0.0.0 (#__DATE__)"
#endif
#ifndef GIT_DESCRIBE
#warning "no GIT_DESCRIBE from git"
#define GIT_DESCRIBE "unknown"
#endif

// define for debugging communications with the HT1621 LCD controller (print raw received data, not useful for production)
// #define DEBUG_COMMS

#ifdef DEBUG_COMMS
#ifndef DEBUG
#define DEBUG
#endif
#endif
#ifdef DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

#include <Arduino.h>
#include <Preferences.h>

// See https://github.com/ttlappalainen/NMEA2000/
#include <NMEA2000_CAN.h> // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
// See https://github.com/hideakitai/ESP32SPISlave
#include <ESP32SPISlave.h>

// 7-segment to 7bit character mapping (dot/8th segment is treated separately to save on array space)
// ATTN: also change character comparisons here in this code if mappings are changed!
#include <sevenseg2char.h>

Preferences preferences;
ESP32SPISlave slave;

// TODO: adjust buffer size
static constexpr uint32_t BUFFER_SIZE{36};
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

// receive buffer contains 3 command bits followed by 6 address bits, then lcd memory follows
#define SHIFT_BY (3 + 6)
#define segdata(seg, com, buf) ((buf[(seg * 4 + com + SHIFT_BY) / 8] >> (7 - ((seg * 4 + com + SHIFT_BY) % 8))) & 1)

// Bit order 7..0: (dot) g f e d c b a
#define mkdigit0(buf) (segdata(13, 1, buf) << 6) | (segdata(14, 0, buf) << 5) | (segdata(14, 1, buf) << 4) | \
                          (segdata(15, 1, buf) << 3) | (segdata(12, 1, buf) << 2) | (segdata(12, 0, buf) << 1) | (segdata(13, 0, buf))

#define mkdigit1(buf) (segdata(11, 1, buf) << 6) | (segdata(15, 0, buf) << 5) | (segdata(7, 1, buf) << 4) | \
                          (segdata(7, 0, buf) << 3) | (segdata(10, 1, buf) << 2) | (segdata(10, 0, buf) << 1) | (segdata(11, 0, buf))

#define mkdigit2(buf) (segdata(8, 1, buf) << 6) | (segdata(9, 0, buf) << 5) | (segdata(9, 1, buf) << 4) |                              \
                          (segdata(6, 1, buf) << 3) | (segdata(16, 1, buf) << 2) | (segdata(16, 0, buf) << 1) | (segdata(8, 0, buf)) | \
                          (segdata(6, 0, buf) << 7)

#define mkdigit3(buf) (segdata(26, 1, buf) << 6) | (segdata(25, 0, buf) << 5) | (segdata(25, 1, buf) << 4) | \
                          (segdata(28, 1, buf) << 3) | (segdata(27, 1, buf) << 2) | (segdata(27, 0, buf) << 1) | (segdata(26, 0, buf))

#define mkdigit4(buf) (segdata(3, 1, buf) << 6) | (segdata(4, 1, buf) << 5) | (segdata(4, 0, buf) << 4) | \
                          (segdata(3, 0, buf) << 3) | (segdata(2, 0, buf) << 2) | (segdata(2, 1, buf) << 1) | (segdata(5, 1, buf))

#define mkdigit5(buf) (segdata(0, 1, buf) << 6) | (segdata(1, 1, buf) << 5) | (segdata(1, 0, buf) << 4) |                               \
                          (segdata(0, 0, buf) << 3) | (segdata(17, 0, buf) << 2) | (segdata(17, 1, buf) << 1) | (segdata(23, 1, buf)) | \
                          (segdata(23, 0, buf) << 7)

#define mkdigit6(buf) (segdata(18, 1, buf) << 6) | (segdata(22, 1, buf) << 5) | (segdata(22, 0, buf) << 4) | \
                          (segdata(18, 0, buf) << 3) | (segdata(19, 0, buf) << 2) | (segdata(19, 1, buf) << 1) | (segdata(21, 1, buf))

#define i1_trip(buf) (segdata(28, 0, buf) << 7)
#define i1_total(buf) (segdata(29, 0, buf) << 6)
// The following one is special due to SPI implementation
// It is not: #define i1_kts(buf) (segdata(31, 1, buf) << 5)
#define i1_kts(buf) (buf[16] & 1)
#define i1_mph(buf) (segdata(30, 0, buf) << 4)
#define i1_km(buf) (segdata(30, 1, buf) << 3)
// The following one is special due to SPI implementation
// It is not: #define i1_ph(buf) (segdata(31, 0, buf) << 2)
#define i1_ph(buf) ((buf[16] >> 1) & 1)
#define i1_n(buf) (segdata(29, 1, buf) << 1)
#define i1_miles(buf) (segdata(24, 1, buf))
#define mkinfo1(buf) (i1_trip(buf) | i1_total(buf) | i1_kts(buf) | i1_mph(buf) | i1_km(buf) | i1_ph(buf) | i1_n(buf) | i1_miles(buf))

#define i2_rowadot(buf) (segdata(6, 0, buf) << 5)
#define i2_rowbdot(buf) (segdata(23, 0, buf) << 4)
#define i2_bell(buf) (segdata(5, 0, buf) << 3)
#define i2_line(buf) (segdata(24, 0, buf) << 2)
#define i2_depthft(buf) (segdata(20, 1, buf) << 1)
#define i2_depthm(buf) (segdata(20, 0, buf))
#define mkinfo2(buf) (i2_rowadot(buf) | i2_rowbdot(buf) | i2_bell(buf) | i2_line(buf) | i2_depthft(buf) | i2_depthm(buf))

uint8_t digit0, digit1, digit2, digit3, digit4, digit5, digit6, info1, info2;
double rowa = 0.0;

/*
 Character representation of the LCD memory
 row a: digits 1,2,3 (top left to bottom right)
 row b: digits 4,5,6
 digit 0 (small digit top left)
 info1 TRIP, TOTAL, MPH, km, /h, KTS, N., miles
 info2 rowadot, rowbdot, bell, line, depth_ft, depth_m
 */
struct ClipperLCD
{
  uint8_t digit0, digit1, digit2, digit3, digit4, digit5, digit6, info1, info2;
} clipperlcd;

/*
 Holds the data which has been parsed from the display
 Values are stored in the N2k library's  base units.
 NMEA2000 Library Base units are m for depth, m/s for speed, m for distance
 this is not optimal, as it involves some back-and-forth conversion from int to float values
 */
struct ClipperData
{
  double speed;                                                // speed in m/s
  double depth;                                                // depth in m
  double shallow_alarm;                                        // in m
  double speed_alarm;                                          // in m/s
  double total;                                                // in m (accumulated by unit)
  double trip;                                                 // in m (since power-on)
  unsigned long last_depth, last_speed, last_total, last_trip; // millis() on reception, computed once a reception cycle
  double offset;                                               // keel offset in m
  double threshold;                                            // depth threshold in m to adjust transducer gain (see NASA Clipper Duet manual)
  double cal;                                                  // Calibration value in % for the speed
};
ClipperData clipperdata;

/* Reads LCD memory and sorts seg/com into digits and info data */
void buf2clipperlcd()
{
  clipperlcd.digit0 = SevenSeg2Char[mkdigit0(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit1 = SevenSeg2Char[mkdigit1(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit2 = SevenSeg2Char[mkdigit2(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit3 = SevenSeg2Char[mkdigit3(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit4 = SevenSeg2Char[mkdigit4(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit5 = SevenSeg2Char[mkdigit5(spi_slave_rx_buf) & 0x7f];
  clipperlcd.digit6 = SevenSeg2Char[mkdigit6(spi_slave_rx_buf) & 0x7f];
  clipperlcd.info1 = mkinfo1(spi_slave_rx_buf);
  clipperlcd.info2 = mkinfo2(spi_slave_rx_buf);
}

/*
 Convert characters and a dot to an integer value
 to be used to convert the characters of row a/b to a numeric value

 returns int on success with the int value 10 times of the value being shown to account for the decimal separator,
 returns N2kUInt32NA (0xffffffff) on conversion error
 */
uint32_t digits2int(const uint8_t d1, const uint8_t d2, const uint8_t d3, bool dot)
{
  int x = N2kUInt32NA;

  if (d1 == ' ')
  {
    x = 0.0;
  }
  else if (d1 >= '0' && d1 <= '9')
  {
    x = d1 - '0';
    x *= 10;
  }
  else
  {
    return N2kUInt32NA;
  }

  if (d2 >= '0' && d2 <= '9')
  {
    x += d2 - '0';
    x *= 10;
  }
  else
  {
    return N2kUInt32NA;
  }

  if (d3 >= '0' && d3 <= '9')
  {
    x += d3 - '0';
  }
  else
  {
    return N2kUInt32NA;
  }

  if (dot)
  {
    return x;
  }
  else
  {
    return x * 10;
  }
}

/*
 Converts value on upper LCD row to a numeric value
 returns double float value on success, N2kDoubleNA (-1e9) on error
*/
double rowa2double()
{
  uint32_t val = digits2int(clipperlcd.digit1, clipperlcd.digit2, clipperlcd.digit3, ((clipperlcd.info2 & (1 << 5)) == (1 << 5)));
  if (val != N2kUInt32NA)
  {
    return ((double)val) / 10.0;
  }
  else
  {
    return N2kDoubleNA;
  }
}
/*
 Converts value on lower LCD row to a numeric value
 returns double float value on success, N2kDoubleNA (-1e9) on error
*/
double rowb2double()
{
  uint32_t val = digits2int(clipperlcd.digit4, clipperlcd.digit5, clipperlcd.digit6, ((clipperlcd.info2 & (1 << 4)) == (1 << 4)));
  if (val != N2kUInt32NA)
  {
    return ((double)val) / 10.0;
  }
  else
  {
    return N2kDoubleNA;
  }
}

/* print a buffer as binary 01 representation of bytes; debug stuff only */
void printBits(void const *const ptr, size_t const size)
{
  unsigned char *b = (unsigned char *)ptr;
  unsigned char byte;
  int i, j;

  for (i = 0; i < size; i++)
  {
    for (j = 7; j >= 0; j--)
    {
      byte = (b[i] >> j) & 1;
      printf("%u", byte);
    }
    printf(" ");
  }
}

// From https://github.com/ttlappalainen/NMEA2000/blob/master/Examples/ESP32/NMEA2000ToWiFiAsSeaSmart/BoardSerialNumber.cpp
uint32_t GetSerialNumber()
{
#if defined(ARDUINO_ARCH_ESP32)
  uint8_t chipid[6];
  esp_efuse_mac_get_default(chipid);
  return chipid[0] + (chipid[1] << 8) + (chipid[2] << 16) + (chipid[3] << 24);
#else
  return 999999;
#endif
}

unsigned long now;
unsigned long received_systemtime; // millis() timestamp of last SecondsSinceMidnight/PGN126992 reception
uint8_t SID;
uint16_t DaysSince1970, received_DaysSince1970;
double SecondsSinceMidnight, received_SecondsSinceMidnight;

// Set the information for other bus devices, which messages we support
const unsigned long ReceiveMessages[] PROGMEM = {129029L, // GNSS (used for time)
                                                 126992L, // System time
                                                 65361L,  // Seatalk: Silence Alarm
                                                 0};
const unsigned long TransmitMessages[] PROGMEM = {128275L, // Distance Log
                                                  128259L, // Boat speed
                                                  128267L, // Depth
                                                  65288L,  // Seatalk: Alarm
                                                  0};

/// @brief Setting up PGN 65288 Message "Seatalk: Alarm"
/// @param N2kMsg
/// @param SID
/// @param alarm_status   0=Alarm condition not met; 1=Alarm condition met and not silenced; 2=Alarm condition met and silenced
/// @param alarm_id       1=Shallow Depth; 2=Boat Speed High
/// @param alarm_group    0=Instrument; 3=Chart Plotter
/// @param alarm_priority
void SetN2kPGN65288(tN2kMsg &N2kMsg, unsigned char SID, uint8_t alarm_status, uint8_t alarm_id, uint8_t alarm_group, uint16_t alarm_priority)
{
  N2kMsg.SetPGN(65288L);
  N2kMsg.Priority = 1;
  N2kMsg.AddByte(alarm_status & 0x03); // 0=Alarm condition not met; 1=Alarm condition met and not silenced; 2=Alarm condition met and silenced
  N2kMsg.AddByte(alarm_id);            // 1=Shallow Depth; 2=Boat Speed High
  N2kMsg.AddByte(alarm_group);
  N2kMsg.Add2ByteUInt(alarm_priority);
}

/// @brief Alias of SetN2kPGN65288
/// @param N2kMsg
/// @param SID
/// @param alarm_status   0=Alarm condition not met; 1=Alarm condition met and not silenced; 2=Alarm condition met and silenced
/// @param alarm_id       1=Shallow Depth; 2=Boat Speed High
/// @param alarm_group    0=Instrument; 3=Chart Plotter
/// @param alarm_priority
inline void SetN2kSeatalkAlarm(tN2kMsg &N2kMsg, unsigned char SID, uint8_t alarm_status, uint8_t alarm_id, uint8_t alarm_group, uint16_t alarm_priority)
{
  SetN2kPGN65288(N2kMsg, SID, alarm_status, alarm_id, alarm_group, alarm_priority);
}

/// @brief Parsing the content of a "Seatalk: Silence Alarm" message - PGN 65361
/// @param N2kMsg
/// @param alarm_id        1=Shallow Depth; 2=Boat Speed High
/// @param alarm_group     0=Instrument; 3=Chart Plotter
/// @param reserved_field
/// @return
bool ParseN2kPGN65361(const tN2kMsg &N2kMsg, uint8_t &alarm_id, uint8_t &alarm_group, uint32_t &reserved_field)
{
  if (N2kMsg.PGN != 65361L)
  {
    return false;
  }

  int Index = 0;
  alarm_id = N2kMsg.GetByte(Index);
  alarm_group = N2kMsg.GetByte(Index);
  alarm_group = N2kMsg.Get2ByteUInt(Index);

  return true;
}

/// @brief Alias of parseN2kPGN65361
/// @param N2kMsg
/// @param alarm_id
/// @param alarm_group
/// @param reserved_field
/// @return
inline bool ParseN2kSeatalkSilenceAlarm(const tN2kMsg &N2kMsg, uint8_t &alarm_id, uint8_t &alarm_group, uint32_t &reserved_field)
{
  return ParseN2kPGN65361(N2kMsg, alarm_id, alarm_group, reserved_field);
}

// PGN129029 handler
void handle_GNSS(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double Latitude;
  double Longitude;
  double Altitude;
  tN2kGNSStype GNSStype;
  tN2kGNSSmethod GNSSmethod;
  unsigned char nSatellites;
  double HDOP;
  double PDOP;
  double GeoidalSeparation;
  unsigned char nReferenceStations;
  tN2kGNSStype ReferenceStationType;
  uint16_t ReferenceSationID;
  double AgeOfCorrection;

  if (ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight,
                   Latitude, Longitude, Altitude,
                   GNSStype, GNSSmethod,
                   nSatellites, HDOP, PDOP, GeoidalSeparation,
                   nReferenceStations, ReferenceStationType, ReferenceSationID,
                   AgeOfCorrection))
  {
    received_systemtime = millis();
    received_DaysSince1970 = DaysSince1970;
    received_SecondsSinceMidnight = SecondsSinceMidnight;
  }
}

// TODO: decide wether there should be a precedence in the use of time sources
// PGN126992 handler
void handle_SystemTime(const tN2kMsg &N2kMsg)
{
  unsigned char tSID;
  tN2kTimeSource TimeSource;

  if (ParseN2kSystemTime(N2kMsg, tSID, DaysSince1970, SecondsSinceMidnight, TimeSource))
  {
    received_systemtime = millis();
    received_DaysSince1970 = DaysSince1970;
    received_SecondsSinceMidnight = SecondsSinceMidnight;
  }
}

void handle_SeatalkSilenceAlarm(const tN2kMsg &N2kMsg)
{
  uint8_t alarm_id;
  uint8_t alarm_group;
  uint32_t reserved_field;

  if (ParseN2kSeatalkSilenceAlarm(N2kMsg, alarm_id, alarm_group, reserved_field))
  {
    if (alarm_group == 0) // 0=Instrument
    {
      if (alarm_id == 1) // Shallow Depth
      {
      }
      else if (alarm_id == 2) // Boat Speed High
      {
      }
    }
  }
}

// Update DaysSince1970 and SecondsSinceMidnight from delta t
void TimeUpdate()
{
  unsigned long deltat_s = (millis() - received_systemtime) / 1000;
  DaysSince1970 = received_DaysSince1970 + (deltat_s / (24 * 3600));
  SecondsSinceMidnight = received_SecondsSinceMidnight + (deltat_s % (24 * 3600));
}

typedef struct
{
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

tNMEA2000Handler NMEA2000Handlers[] = {
    {126992L, &handle_SystemTime},
    {129029L, &handle_GNSS},
    {65361L, &handle_SeatalkSilenceAlarm},
    {0, 0}};

// NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{
  int iHandler;
  // Find handler
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++)
    ;
  if (NMEA2000Handlers[iHandler].PGN != 0)
  {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

#define PP_STRINGIFY_IMPL(X) #X
#define PP_STRINGIFY(X) PP_STRINGIFY_IMPL(X)

#define PINDESCRIPTION "MOSI(HT_DATA)=" PP_STRINGIFY(PIN_HTDATA) ",MISO=" PP_STRINGIFY(PIN_HTDATAOUT) ",CLK(HT_WR)= " PP_STRINGIFY(PIN_HTCLK) ",CS=" PP_STRINGIFY(PIN_HTCS) ",CAN_TX=" PP_STRINGIFY(ESP32_CAN_TX_PIN) ",CAN_RX=" PP_STRINGIFY(ESP32_CAN_RX_PIN)

void InitNMEA2000()
{
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(200);

  char SnoStr[33];
  uint32_t SerialNumber = GetSerialNumber();
  snprintf(SnoStr, 32, "%lu", (long unsigned int)SerialNumber);

  NMEA2000.SetProductInformation(SnoStr,            // Manufacturer's Model serial code
                                 1337,              // Manufacturer's product code
                                 "ClipperDuet2N2k", // Manufacturer's Model ID
                                 N2K_SOFTWARE_VERSION,
                                 "1.0.0.0 (1998-01-01)" // original PCB states "COPYRIGHT 1998 A.J.MULLEY"
  );

  NMEA2000.SetDeviceInformation(SerialNumber, // Unique number. Use e.g. Serial number.
                                60,           // Device function: Navigation
                                135,          // Device class: Bottom Depth/Speed
                                275           // Manufacturer code: Navico; TODO: decide wether to use a different number
  );
  NMEA2000.SetInstallationDescription1("ClipperDuet2N2k " GIT_DESCRIBE " by Soenke J. Peters");
  NMEA2000.SetInstallationDescription2(PINDESCRIPTION);

#ifdef DEBUG
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text instead of Actisense format.
#endif
  NMEA2000.SetForwardStream(&Serial);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 32);
  NMEA2000.EnableForward(true);

  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  NMEA2000.Open();
}

void setup()
{
  clipperdata.depth = N2kDoubleNA;
  preferences.begin("ClipperDuet2N2k", false);
  // TODO: think of different Clipper Duet hardware revisions
  // preferences.getUInt("hardwarerev", 1);
  clipperdata.offset = preferences.getDouble("offset", -SAFE_OFFSET);
  clipperdata.cal = preferences.getDouble("cal", 100);
  clipperdata.threshold = preferences.getDouble("threshold", 0.0);
  clipperdata.shallow_alarm = preferences.getDouble("shallow_alarm", 0.0);
  clipperdata.speed_alarm = preferences.getDouble("speed_alarm", 0.0);

  Serial.begin(115200);
  InitNMEA2000();
  
  printf("\n\nClipperDuet2N2k %s\n\n", GIT_DESCRIBE);

  slave.setDataMode(SPI_MODE3);
  slave.begin(HSPI, PIN_HTCLK, PIN_HTDATAOUT, PIN_HTDATA, PIN_HTCS);

  // clear buffer
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}

// bit field to queue the saves to NVM
// speed_alarm | shallow_alarm | cal | threshold | offset
const uint32_t BF_OFFSET = 1;
const uint32_t BF_THRESHOLD = 1 << 1;
const uint32_t BF_CAL = 1 << 2;
const uint32_t BF_SHALLOW_ALARM = 1 << 3;
const uint32_t BF_SPEED_ALARM = 1 << 4;

uint32_t queue_save = 0;
tN2kMsg N2kMsg;
void loop()
{
  now = millis();
  TimeUpdate();

  NMEA2000.ParseMessages();

  // if there is no transaction in queue, add transaction
  if (slave.remained() == 0)
  {
    slave.queue(spi_slave_rx_buf, BUFFER_SIZE);
  }

  while (slave.available())
  {
    uint32_t num = slave.size();

    // omit "10001100 00000000" packages sent as keep-alive
    if (num > 2 && num < BUFFER_SIZE)
    {
      if ((spi_slave_rx_buf[0] & 0b11100000) == 0b10100000)
      {
        // LCD Write to display memory
        // NASA Clipper Duet sends entire memory from address 0 to end
        // htmem_address = ((spi_slave_rx_buf[0] & 0xf) << 1) | (spi_slave_rx_buf[1] >> 7);
        // a check for num == 17 and htmem_address == 0 is omitted for optimization reasons
#ifdef DEBUG_COMMS
        printf("%d Mem: ", num);
#else
        buf2clipperlcd();

        if ((clipperlcd.info2 & (1 << 2)) == (1 << 2))
        {
          // Line LCD segment is displayed, so unit is not in settings mode

          // Increment NMEA2000 SID
          SID = (++SID) & 0xff;

          // Check if there are any values to be saved to NVM
          if (queue_save > 0)
          {
            // saving of shallow_alarm and speed_alarm values to NVM is delayed until return to normal screen
            if ((queue_save & BF_SHALLOW_ALARM) == BF_SHALLOW_ALARM)
            {
              // Save shallow_alarm to NVM
              preferences.putDouble("shallow_alarm", clipperdata.shallow_alarm);
              DEBUG_PRINT("shallow_alarm saved: %fm\n", clipperdata.shallow_alarm);
            }
            if ((queue_save & BF_SPEED_ALARM) == BF_SPEED_ALARM)
            {
              // Save speed_alarm to NVM
              preferences.putDouble("speed_alarm", clipperdata.speed_alarm);
              DEBUG_PRINT("speed_alarm saved: %fm\n", clipperdata.speed_alarm);
            }
            if ((queue_save & BF_OFFSET) == BF_OFFSET)
            {
              // Save offset to NVM
              preferences.putDouble("offset", clipperdata.offset);
              DEBUG_PRINT("Setting offset saved: %fm\n", clipperdata.offset);
            }
            if ((queue_save & BF_THRESHOLD) == BF_THRESHOLD)
            {
              // Save threshold to NVM
              preferences.putDouble("threshold", clipperdata.threshold);
              DEBUG_PRINT("Setting threshold saved: %fm\n", clipperdata.threshold);
            }
            if ((queue_save & BF_CAL) == BF_CAL)
            {
              // Save paddle wheel calibration to NVM
              preferences.putDouble("cal", clipperdata.cal);
              DEBUG_PRINT("Setting cal saved: %fm\n", clipperdata.cal);
            }
            queue_save = 0;
          }

          double depth = rowb2double();
          if (depth != N2kDoubleNA)
          {
            // The dot is not transferred when depth measurement has error,
            // so look for depth_m or depth_ft segments
            if ((clipperlcd.info2 & 1) == 1)
            {
              // Depth in m
              clipperdata.depth = depth;
              clipperdata.last_depth = now;
            }
            else if ((clipperlcd.info2 & (1 << 1)) == (1 << 1))
            {
              // Depth in ft
              clipperdata.depth = depth / 3.281; // store as meters
              clipperdata.last_depth = now;
            }
            else
            {
              clipperdata.depth = N2kDoubleNA;
            }
          }

          SetN2kWaterDepth(N2kMsg, SID, clipperdata.depth, clipperdata.offset, N2kDoubleNA); // TODO: send proper Range instead of N2kDoubleNA
          NMEA2000.SendMsg(N2kMsg);

          double speed = N2kDoubleNA;
          double distance = -1;

          if (clipperlcd.info1 < (1 << 6))
          {
            // Speed is displayed
            speed = rowa2double();
            if (speed != N2kDoubleNA)
            {
              if ((clipperlcd.info1 & 0b00100000) == 0b00100000)
              {
                // KTS
                clipperdata.speed = speed * 0.514444444444; // store as m/s
              }
              else if ((clipperlcd.info1 & 0b00001100) == 0b00001100)
              {
                // km/h
                clipperdata.speed = speed / 3.6; // store as m/s
              }
              else if ((clipperlcd.info1 & 0b00010000) == 0b00010000)
              {
                // miles/h
                clipperdata.speed = speed / 2.237; // store as m/s
              }
              else
              {
                // Set speed to 0.0 on conversion error, seems to be sane as it is the default when no sensor connected
                clipperdata.speed = 0.0; // TODO: decide wether N2kDoubleNA is better
              }

              clipperdata.last_speed = now;
            }
            else
            {
              clipperdata.speed = N2kDoubleNA;
            }

            SetN2kBoatSpeed(N2kMsg, SID, clipperdata.speed, N2kDoubleNA, N2kSWRT_Paddle_wheel);
            NMEA2000.SendMsg(N2kMsg);

            DEBUG_PRINT("Speed: %fm/s, Depth: %fm\n", clipperdata.speed, clipperdata.depth);
          }
          else
          {
            // A distance is displayed, we will determine later if total or trip
            distance = rowa2double();
            if (distance != N2kDoubleNA)
            {
              if ((clipperlcd.info1 & 0b00000011) == 0b00000011)
              {
                // N.miles
                distance *= 1852; // store as m
              }
              else if ((clipperlcd.info1 & 0b00001000) == 0b00001000)
              {
                // km
                distance *= 1000; // store as m
              }
              else if ((clipperlcd.info1 & 0b00000001) == 0b00000001)
              {
                // miles
                distance *= 1609; // store as m
              }
              else
              {
                // Must be an error
                distance = N2kDoubleNA;
              }
            }

            if (distance != N2kDoubleNA)
            {
              if ((clipperlcd.info1 & 0b10000000) == 0b10000000)
              {
                // Trip display
                clipperdata.trip = distance;
                clipperdata.last_trip = now;

                // Send distance data (no DISTANCE_TIMEOUT check)
                SetN2kDistanceLog(N2kMsg, DaysSince1970, SecondsSinceMidnight, clipperdata.total, clipperdata.trip);
                NMEA2000.SendMsg(N2kMsg);

                DEBUG_PRINT("Trip: %fm, Total: %fm, DaysSince1970: %u, SecondsSinceMidnight: %f\n", clipperdata.trip, clipperdata.total, DaysSince1970, SecondsSinceMidnight);
              }
              else if ((clipperlcd.info1 & 0b01000000) == 0b01000000)
              {
                // Total display
                clipperdata.total = distance;
                clipperdata.last_total = now;

                if ((now - clipperdata.last_trip) < (DISTANCE_TIMEOUT * 1000))
                {
                  // Trip and total distanca data are within the DISTANCE_TIMOUT, so good to send both
                  SetN2kDistanceLog(N2kMsg, DaysSince1970, SecondsSinceMidnight, clipperdata.total, clipperdata.trip);
                  NMEA2000.SendMsg(N2kMsg);
                  DEBUG_PRINT("Total: %fm, Trip: %fm, DaysSince1970: %u, SecondsSinceMidnight: %f\n", clipperdata.total, clipperdata.trip, DaysSince1970, SecondsSinceMidnight);
                }
                else
                {
                  // Do not send Trip log if data is too old
                  DEBUG_PRINT("Total: %fm\n", clipperdata.total);
                }
              }
            }
          }
        }
        else
        {
          // Settings dialogues

          if (clipperlcd.digit0 == 'W' && clipperlcd.digit1 == '(') // "u_underline Con"
          {
            // keel offset setting confirmed
            double offset = rowb2double();
            if (offset != N2kDoubleNA)
            {
              if ((clipperlcd.info2 & 1) == 1)
              {
                // Offset in m
                offset = -offset; // negative as it denotes offset to keel
              }
              else if ((clipperlcd.info2 & (1 << 1)) == (1 << 1))
              {
                // Offset in ft
                offset = -(offset / 3.281); // store as meters, negative as it denotes offset to keel
              }
              else
              {
                offset = -SAFE_OFFSET; // Set an offset which is considered to be safe
              }

              if (offset != clipperdata.offset)
              {
                clipperdata.offset = offset;
                // Mark value to be saved to NVM on return to normal screen
                queue_save |= BF_OFFSET;
              }
            }
          }
          else if (clipperlcd.digit0 == 'T' && clipperlcd.digit1 == '(') // "t Con"
          {
            // transducer gain threshold setting confirmed
            double threshold = rowb2double();
            if (threshold != N2kDoubleNA)
            {
              if ((clipperlcd.info2 & 1) == 1)
              {
                // Threshold in m
                // threshold = threshold;
              }
              else if ((clipperlcd.info2 & (1 << 1)) == (1 << 1))
              {
                // Threshold in ft
                threshold = (threshold / 3.281); // store as meters
              }
              else
              {
                threshold = 0.0; // Store 0.0 as a threshold which is considered a safe default
              }

              if (threshold != clipperdata.threshold)
              {
                clipperdata.threshold = threshold;
                // Mark value to be saved to NVM on return to normal screen
                queue_save |= BF_THRESHOLD;
              }
            }
          }
          else if (clipperlcd.digit4 == '(' && ((clipperlcd.info2 & 0b100011) == 0)) // "Con" on bottom row, no depth unit, no dot on upper row
          {
            // paddle wheel calibration setting
            double cal = rowa2double();
            if (cal != N2kDoubleNA)
            {
              if (cal != clipperdata.cal)
              {
                clipperdata.cal = cal;
                // Mark value to be saved to NVM on return to normal screen
                queue_save |= BF_CAL;
              }
            }
          }
          else if (clipperlcd.digit1 == '5' && clipperlcd.digit2 == 'X') // "SHA" on top row
          {
            // shallow depth alarm setting
            double shallow_alarm = rowb2double();
            if (shallow_alarm != N2kDoubleNA)
            {
              if ((clipperlcd.info2 & 1) == 1)
              {
                // Offset in m
                clipperdata.shallow_alarm = shallow_alarm; // negative as it denotes offset to keel
              }
              else if ((clipperlcd.info2 & (1 << 1)) == (1 << 1))
              {
                // Offset in ft
                clipperdata.shallow_alarm = (shallow_alarm / 3.281); // store as meters, negative as it denotes offset to keel
              }
              else
              {
                clipperdata.shallow_alarm = SAFE_OFFSET + 1; // Set an offset which is considered to be reasonably safe
              }

              // Mark value to be saved to NVM on return to normal screen
              queue_save |= BF_SHALLOW_ALARM;
            }
          }
          else if (clipperlcd.digit4 == '5' && clipperlcd.digit5 == 'P') // "SPd" on bottom row
          {
            // speed alarm setting

            /*
            TODO: as there are not too many uses for a speed alarm setting,
                  maybe use this as a side-channel to set different things like enabling debug msg, reset NVM preferences, etc.
            */
            double speed_alarm = rowa2double();
            if (speed_alarm != N2kDoubleNA)
            {
              if ((clipperlcd.info1 & 0b00100000) == 0b00100000)
              {
                // KTS
                clipperdata.speed_alarm = speed_alarm * 0.514444444444; // store as m/s
              }
              else if ((clipperlcd.info1 & 0b00001100) == 0b00001100)
              {
                // km/h
                clipperdata.speed_alarm = speed_alarm / 3.6; // store as m/s
              }
              else if ((clipperlcd.info1 & 0b00010000) == 0b00010000)
              {
                // miles/h
                clipperdata.speed_alarm = speed_alarm / 2.237; // store as m/s
              }
            }
            else
            {
              // There might be an "OFF" shown which we also treat as 0.0
              speed_alarm = 0.0;
            }
            // Mark value to be saved to NVM on return to normal screen
            queue_save |= BF_SPEED_ALARM;
          }
          else
          {
            // Unrecognized setting, print for debug
            DEBUG_PRINT("?: %c %c%c%c %c%c%c\n", clipperlcd.digit0, clipperlcd.digit1, clipperlcd.digit2, clipperlcd.digit3, clipperlcd.digit4, clipperlcd.digit5, clipperlcd.digit6);
          }
        }
#endif
      }
#ifdef DEBUG_COMMS
      else if ((spi_slave_rx_buf[0] & 0b11100000) == 0b10000000)
      {
        // LCD Command
        printf("%d, Command: ", num);
      }

      printBits(spi_slave_rx_buf, num);
      printf("\n");
#endif
    }

    slave.pop();
  }

  // Dummy to empty serial input buffer
  if (Serial.available())
  {
    Serial.read();
  }
}
