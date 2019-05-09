/**
 * PWM7003:
 * PINs 1,2 - 5V
 * PINs 3,4 - GND
 * PIN 9 - D2 (D3 must not be used)
 * 
 * MH Z-19B:
 * GND - GND
 * VIN - 5V
 * PWM - D9
 * 
 * DHT22:
 * + - 5V
 * - - GND
 * out - D4
 */
#include <DHT.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

SoftwareSerial pmsSerial(2, 3);

#define DHTPIN 4 
#define DHTTYPE DHT22   
DHT dht(DHTPIN, DHTTYPE);

void setup()
{
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  dht.begin(); 
  pinMode(9, INPUT);
}

struct pms7003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms7003data data;
    
void loop()
{
  int h = int(dht.readHumidity());
  int t = int(dht.readTemperature());
  
  int ppm_pwm = readCO2PWM();

  pmsSerial.begin(9600);
  delay(800);
  readPMSdata(&pmsSerial);  
  pmsSerial.end();
  
  lcd.clear();
  lcd.setCursor(0,0);
   lcd.print("H: "); lcd.print(h); lcd.print("%");     
   lcd.print("        T: "); lcd.print(t); lcd.print("C");
  lcd.setCursor(0,1);
    lcd.print("CO2: "); lcd.print(ppm_pwm); lcd.print("ppm"); 
  lcd.setCursor(0,2);
    lcd.print("pm2.5/10: "); lcd.print(data.pm25_env); 
    lcd.print(" / ");lcd.print(data.pm100_env);     
  lcd.setCursor(0,3);
   lcd.print("PC: "); lcd.print(data.particles_03um);   
   lcd.print(" "); lcd.print(data.particles_05um);   
   lcd.print(" "); lcd.print(data.particles_10um); 
   lcd.print(" "); lcd.print(data.particles_25um);
  //delay(700);
}


boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}
int readCO2PWM() {
  unsigned long th, tl, ppm_pwm = 0;
  do {
    th = pulseIn(9, HIGH, 1004000) / 1000;
    tl = 1004 - th;
    ppm_pwm = 5000 * (th-2)/(th+tl-4);
  } while (th == 0);
  return ppm_pwm;  
}
