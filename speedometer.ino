#include <Arduino.h>
#include <ctype.h>
#include <ESP8266WiFi.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <WiFiUdp.h>


/** CONFIGURATION **/   
// Set WiFi SSID and password
#define WIFI_SSID "******SSID******"
#define WIFI_PASS "****PASSWORD****"

// IP address of the router (10.1.8.1 in my case)
IPAddress snmp_host(10, 1, 8, 1);

// Which I/O line will be used for which measurement
#define UPLOAD_PIN      5
#define DOWNLOAD_PIN    4
#define LOADAVG_PIN     0

// UDP
#define UDP_PORT 161
WiFiUDP UDP;

// Value to write to PWM to move the voltmeters' needles to the maximum value (~3V effective)
#define RANGE_MAX 955


// Counters to keep track of time and traffic deltas
unsigned long c_fastloop = 0;
unsigned long c_slowloop = 0;
unsigned long long counters[2] = {0ULL, 0ULL};

// Event loop definitions
#define BYTES_PER_MSEC 125000  // Max bytes per LOOP_SLOW_LIMIT interval 
#define LOOP_FAST_LIMIT 50
#define LOOP_SLOW_LIMIT 500  // milliseconds per loop
#define MAX_SPEED_PER_INTERVAL (BYTES_PER_MSEC * LOOP_SLOW_LIMIT)
#define MIN_SPEED_PER_INTERVAL (MAX_SPEED_PER_INTERVAL / 1000)
double log_max_speed = 3.0f;


// Timer counters
long unsigned int loop_fast = 0;
long unsigned int loop_slow = 0;
#define LOOP_FAST_LIMIT 50
#define LOOP_SLOW_LIMIT 500

// Current values of meters
long unsigned int in_speed = 0;
long unsigned int out_speed = 0;
uint8_t loadavg = 0;

// Target value of meters
long unsigned int in_speed_target = 0;
long unsigned int out_speed_target = 0;
uint8_t loadavg_target = 0;



// ---- SNMP STUFF BEGINS HERE ----

#define SNMP_REQUEST_LENGTH 78
uint8_t snmp_request [SNMP_REQUEST_LENGTH] = {
  // Taken from:
  // snmpget -d -c public -v 2c 10.1.8.1 1.3.6.1.2.1.2.2.1.10.13 1.3.6.1.2.1.2.2.1.16.13 1.3.6.1.4.1.2021.10.1.5.1
  // Counter is bytes 17 - 20
  // TODO: Turn the hard-coded counter into an actual sequence number.
  0x30, 0x4C, 0x02, 0x01, 0x01, 0x04, 0x06, 0x70, 0x75, 0x62, 0x6C, 0x69, 0x63, 0xA0, 0x3F, 0x02,
  0x04, 0x27, 0x68, 0x64, 0x38, 0x02, 0x01, 0x00, 0x02, 0x01, 0x00, 0x30, 0x31, 0x30, 0x0E, 0x06,
  0x0A, 0x2B, 0x06, 0x01, 0x02, 0x01, 0x02, 0x02, 0x01, 0x0A, 0x0D, 0x05, 0x00, 0x30, 0x0E, 0x06,
  0x0A, 0x2B, 0x06, 0x01, 0x02, 0x01, 0x02, 0x02, 0x01, 0x10, 0x0D, 0x05, 0x00, 0x30, 0x0F, 0x06,
  0x0B, 0x2B, 0x06, 0x01, 0x04, 0x01, 0x8F, 0x65, 0x0A, 0x01, 0x05, 0x01, 0x05, 0x00
};

// Reserve enough memory for a full-sized Ethernet frame, just in case.
#define SNMP_RESPONSE_MAX_LENGTH 1500
int snmp_response_length;
uint8_t snmp_response[SNMP_RESPONSE_MAX_LENGTH];   // Response should be 88 bytes

// The OIDs to pull from the SNMP response
uint8_t OID_DOWNLOAD[10] =  {0x2B, 0x06, 0x01, 0x02, 0x01, 0x02, 0x02, 0x01, 0x0A, 0x0D};
uint8_t OID_UPLOAD[10]   =  {0x2B, 0x06, 0x01, 0x02, 0x01, 0x02, 0x02, 0x01, 0x10, 0x0D};
uint8_t OID_LOADAVG[11]  =  {0x2B, 0x06, 0x01, 0x04, 0x01, 0x8F, 0x65, 0x0A, 0x01, 0x05, 0x01};

// Table of contents of the SNMP response. 10 should be enough space.
#define TOC_MAX_SIZE 10
uint8_t toc[TOC_MAX_SIZE];


/* 
 *  Dump a SNMP message through the serial port. 
 *  Used when a response can't be parsed.
 */
void dump_response() {
  for (int i = 0; i < snmp_response_length; i++) {
    if (snmp_response[i] < 0x10) printf("0");
    printf("%2x", snmp_response[i]);
  }
  printf("\n");
}


/*
 * Read a variable-length integer from the SNMP response. 
 */
unsigned long long read_int(int base, int len) {  
  unsigned long long value = 0;
  while (len > 0) {
    value <<= 8;
    value |= snmp_response[base];
    base++;
    len--;
  }

  if (value == 0) {
    printf("Read from :%d len: %d value: %lld\n", base, len, value);    
  }
  return value;
}


/*
 * Validate an ASN.1 response and build an index pointing to
 * each MIB value within the response, so that individual values
 * can be looked up quickly in-place without re-parsing the entire thing,
 * or using extra memory to store (OID, Value) pairs.
 * 
 * Because the PDU returned by the query above should have just 3 OIDs
 * and 3 values of known types, parsing the response is straightforward.
 */
bool parse_asn1_response() {
  uint8_t idx = 0;
  int entry = 0;
  int len;

  // Clear ToC
  for (int x = 0; x < TOC_MAX_SIZE; x++) {
    toc[x] = 0; 
  }

  // Check magic numbers
  if(snmp_response[0x00] != 0x30 || snmp_response[0x0D] != 0xA2) {
    printf("Invalid response signature\n");
    dump_response();
    return false;
  }
  if(snmp_response[0x17] != 0 || snmp_response[0x1A] != 0) {
    printf("Errors in response\n");
    dump_response();
    return false;
  }

  // Start of response PDU
  idx = 0x1B;
  if (snmp_response[idx] != 0x30) {
    printf("Error in response(1)\n");
    dump_response();
    return false;
  }

  // Jump to first element
  idx += 2;
  while (idx < snmp_response_length && entry < TOC_MAX_SIZE) {
    // idx points to the start of the element entry in the results list
    if (snmp_response[idx] != 0x30) {
      printf("Error in response(2)\n");
      printf("idx: %d\n", idx);
      dump_response();
      return false;
    }
  
    toc[entry] = idx;
    entry++;

    // Jump to the next entry
    idx += snmp_response[idx + 1] + 2;
  }
  return true;
}


/*
 * Checks if a particular position within the response buffer matches an OID
 * Used to traverse the list of OIDs in the response.
 */
bool is_oid(int buf_idx, uint8_t oid[], int len) {
  for (int i = 0; i < len; i++) {
    if (snmp_response[buf_idx + i] != oid[i]) {
      // Disabled - Used for debugging.
      //printf("Oid no match: [%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x] != [%xd,%xd,%xd,%xd,%xd,%xd,%xd,%xd,%xd,%xd]\n",
      //snmp_response[buf_idx + 0],snmp_response[buf_idx + 1],snmp_response[buf_idx + 2],snmp_response[buf_idx + 3],snmp_response[buf_idx + 4],snmp_response[buf_idx + 5],snmp_response[buf_idx + 6],snmp_response[buf_idx + 7],snmp_response[buf_idx + 8],snmp_response[buf_idx + 9],
      //oid[0],oid[1],oid[2],oid[3],oid[4],oid[5],oid[6],oid[7],oid[8],oid[9]); 
      return false;
    }
  }
  return true;
}


/*
 * Returns the integer value of a given OID within the response/
 */
unsigned long long value_of(uint8_t *oid) {
  int val_idx;
  for (int x = 0; x < TOC_MAX_SIZE; x++) {
    // printf("Searching ["); printf(toc[x] + 4); printf("]");
    if (is_oid(toc[x] + 4, oid, 10)) {
      // We found the entry. Advance by 4 + length
      val_idx = toc[x] + 4 + snmp_response[toc[x] + 3];
      if (snmp_response[val_idx] != 0x41 && snmp_response[val_idx] != 0x02){
        printf("Error parsing integer value\nval_idx: %d\n", val_idx);
        return 0;
      }
      unsigned long long result = read_int(val_idx + 2, snmp_response[val_idx + 1]);
      if (result == 0) {
        printf("read_int returned 0\n");
      }
      return result;
    }
  }

  /* 
   * Dump a bunch of information through the serial port if the OID isn't found in the response.
   * This shouldn't happen often because the SNMP request we send is always the same, and well known, 
   * so a missing OID usually means that something went wrong on the server side.
   */
  printf("OID not found in response: [%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x,%0x]\n", oid[0],oid[1],oid[2],oid[3],oid[4],oid[5],oid[6],oid[7],oid[8],oid[9]);
  printf("toc: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n", toc[0],toc[1],toc[2],toc[3],toc[4],toc[5],toc[6],toc[7],toc[8],toc[9]);
  dump_response();
  return 0;
}

// ---- SNMP STUFF ENDS HERE ----


/*
 * Calculates a delta between the current value of a counter (in this case a timer unit)
 * and the previous value. Updates the previous value, and returns the delta.
 */
long unsigned int timedelta(long unsigned int *prev, long unsigned int current) {
  long unsigned int diff;
  long unsigned int prevval = *prev;
  
  if(current < prevval) {
    // The value rolled over
    diff = (ULONG_MAX - prevval) + current;
  } else {
    diff = current - prevval;    
  }
  
  *prev = current;
  
  return diff;
}


/*
 * Calculates a delta between the current value of a counter (in this case a 64-bit byte counter)
 * and the previous value. Updates the previous value, and returns the delta.
 * For some reason the compiler doesn't like "unsigned long long *" parameters,
 * so I had to keep the previous values in an array and reference them by index.
 */
unsigned long long counterdelta(int counter, unsigned long long current) {
  unsigned long long diff;
  unsigned long long prevval = counters[counter];
  
  if(current < prevval) {
    // The value rolled over
    diff = (ULONG_MAX - prevval) + current;
    printf ("Roll from %lld to %lld. Diff:%lld\n", prevval, current, diff);
  } else {
    diff = current - prevval;    
  }
  
  counters[counter] = current;

  if (diff == 0) {
    printf("delta: diff is 0: %lld -> %lld\n", prevval, current);
  }
  
  return diff;
}


/*
 * Calculates the value to output to PWM based on last polled number of bytes sent/received,
 * and maps it to a logarithmic scale, with the assumption that:
 *   Max speed: 1Gbps
 *   SNMP polling frequency: 2Hz
 *   Max PWM value: RANGE_MAX
 */
unsigned long int calculate_effective_speed(int counter_index, unsigned long long instant_value) {
  
  unsigned long delta = counterdelta(counter_index, instant_value);         // -> range: 0 - 62.000.000 Bytes/500ms
  double delta_scaled = (double)delta / MIN_SPEED_PER_INTERVAL;             // -> range: 0 - 1000
  double logdelta = min(log10(max(1.0, delta_scaled)), 3.0);                // -> range: 0 - 3

  //printf ("In range: Delta: %d | Delta Scaled: %f | Logdelta: %f\n", delta, delta_scaled, logdelta);
  return max((int)floor(logdelta * RANGE_MAX / log_max_speed), 1);
}


/*
 * Calculates the value to output to PWM for a given value, and a given set of upper and lower bounds.
 */
unsigned long scale(unsigned long long min_value, unsigned long long max_value, unsigned long long value) {
  unsigned long long result;
  
  // Clamp values to the acceptable range
  if(value < min_value) value = min_value;
  if(value > max_value) value = max_value;

  // Offset with minimum value
  value = value - min_value;

  result = (RANGE_MAX * value) / (max_value - min_value);
}


/*
 * Setup function. 
 * Set up Serial port
 * Set up PWM and I/O mode of appropriate pins
 * Connect to WiFi
 */
void setup() {
  // Setup serial port
  Serial.begin(230400);
  printf("In setup()\n");

  // Set up analog outputs
  pinMode(UPLOAD_PIN, OUTPUT);
  pinMode(DOWNLOAD_PIN, OUTPUT);
  pinMode(LOADAVG_PIN, OUTPUT);
  
  analogWrite(UPLOAD_PIN, 0);
  analogWrite(DOWNLOAD_PIN, 0);
  analogWrite(LOADAVG_PIN, 0);

  // Begin WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
   
  // Connecting to WiFi...
  printf("Connecting to ");
  printf(WIFI_SSID);
 
  // Loop continuously while WiFi is not connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    printf(".");
  }
   
  // Connected to WiFi
  Serial.println();
  Serial.println("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  UDP.begin(UDP_PORT);
  Serial.println("Listening on UDP port ");
  Serial.println(UDP_PORT);
}


/*
 * Main loop
 */
void loop() { 
  int read_len;
  long unsigned int current_time = millis();


  /* 
   * Fast event loop (Approximately 20 times per second)
   * Instead of just outputting the measured value, approximate it asymptotically over time
   * so that the needle doesn't get thrown around and hit the ends of the scale if values
   * vary too much too quickly.
   */
  long unsigned int delta = timedelta(&c_fastloop, current_time);
  loop_fast += delta;
  if (loop_fast >= LOOP_FAST_LIMIT) {

    long unsigned int step;
    // In
    step = (max(in_speed_target, in_speed) - min(in_speed_target, in_speed)) / 8;
    in_speed = (in_speed_target > in_speed)?(in_speed + step):(in_speed - step);

    // out
    step = (max(out_speed_target, out_speed) - min(out_speed_target, out_speed)) / 8;
    out_speed = (out_speed_target > out_speed)?(out_speed + step):(out_speed - step);


    // Loadavg
    step = (max(loadavg_target, loadavg) - min(loadavg_target, loadavg))  / 8;
    loadavg = (loadavg_target > loadavg)?(loadavg + step):(loadavg - step);

    analogWrite(UPLOAD_PIN, min ((int)out_speed, RANGE_MAX));
    analogWrite(DOWNLOAD_PIN, min ((int)in_speed, RANGE_MAX));
    analogWrite(LOADAVG_PIN, min(RANGE_MAX, (loadavg*RANGE_MAX/400)));
 
    loop_fast -= LOOP_FAST_LIMIT;
  }
  
  
  /* 
   * Slow event loop (Approximately twice per second)
   * Send an SNMP request to the router.
   * The response will come back some time later and will be processed then.
   */
  delta = timedelta(&c_slowloop, current_time);
  loop_slow += delta;
  if (loop_slow >= LOOP_SLOW_LIMIT) {

    // Send SNMP request
    UDP.beginPacket(snmp_host, UDP_PORT);
    UDP.write(snmp_request, SNMP_REQUEST_LENGTH); 
    UDP.endPacket();

    loop_slow -= LOOP_SLOW_LIMIT;
  }
  

  /* 
   * Process any incoming packets 
   */
  snmp_response_length = UDP.parsePacket();
  if (snmp_response_length > 0) {
  
    read_len = UDP.read(snmp_response, SNMP_RESPONSE_MAX_LENGTH);
    
    if (UDP.remotePort() != 161 || UDP.remoteIP() != snmp_host) {
      UDP.flush();
      return;
    }

    if (!parse_asn1_response()) {
      return;
    }

    // Obtain new speed and loadavg values
    unsigned long int read_in_speed = calculate_effective_speed(0, value_of(OID_DOWNLOAD));
    unsigned long int read_out_speed = calculate_effective_speed(1, value_of(OID_UPLOAD));
    unsigned long int read_loadavg = value_of(OID_LOADAVG);

    /* 
     * Copy new measured values to the target values for the fast loop.
     * Ignore invalid (i.e. 0) values. 
     */
    if (read_in_speed != 0) in_speed_target = read_in_speed;
    if (read_out_speed != 0) out_speed_target = read_out_speed;
    if (read_loadavg != 0) loadavg_target = read_loadavg;

    // Some debug code, disabled by default
    if (0 && (read_in_speed == 0 || read_out_speed == 0 || read_loadavg == 0)) {
      printf("parsed value is 0. Dumping frame\n");
      dump_response();
      printf("\n");
      printf("In: %u\n", in_speed_target);
      printf("Out: %u\n", out_speed_target);
      printf("Load: %u\n", loadavg_target);
      return;
    }    
  }
}
