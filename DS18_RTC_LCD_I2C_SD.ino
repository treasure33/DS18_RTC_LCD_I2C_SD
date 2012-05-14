//#define PRINT_MESSAGES 1
//#define INTERRUPT 1

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include "RTClib.h"
#include <OneWire.h>

#include <SdFat.h>
#include <SdFatUtil.h>  // define FreeRam()

#define CHIP_SELECT SS_PIN

// file system object
SdFat sd;

// text file for logging
ofstream logfile;

// Serial print stream
ArduinoOutStream cout(Serial);

// buffer to format data - makes it eaiser to echo to Serial
char buf[80];
// store error strings in flash to save RAM
#define error(s) sd.errorHalt_P(PSTR(s))


RTC_DS1307 RTC;           // Часы реального времени
OneWire  ds(7);           // Определим пин для взаимодейсвие с датчиками темературы по 1-wire интерфейсу

DateTime prev;            // Предыдущее значение времени 

volatile float lastTemperature = 0.0;   // Последнее значение температуры
volatile unsigned long count = 3L;      // Счетчик срабатывания прерывания с частотой 1Гц
volatile unsigned int raw = 0;          // Значение датчика температуры
volatile int cels = 0;                  // Температура в градусах Цельсия
volatile boolean setupInProgress = true;


char  *Day[] = {"Вс","Пн","Вт","Ср","Чт","Пт","Сб"}; // Дни неделеи

byte gradus[8] = {         // Перепределяем вывод значка градус
B00110,
B01001,
B00110,
B00000,
B00000,
B00000,
B00000,
};

#define DS1307_ADDRESS 0x68 // Адрес часов реального времени на i2c интерфейсе

int ledPin = 6;
volatile int state = LOW;

// initialize the library with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x20, 16, 2);    // Инициализируем LCD 

//------------------------------------------------------------------------------
// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {
    DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}
//------------------------------------------------------------------------------
// format date/time
ostream& operator << (ostream& os, DateTime& dt) {
  os << dt.year() << '/' << int(dt.month()) << '/' << int(dt.day()) << ',';
  os << int(dt.hour()) << ':' << setfill('0') << setw(2) << int(dt.minute());
  os << ':' << setw(2) << int(dt.second()) << setfill(' ');
  return os;
}


void sqw() // set to 1Hz
{
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0x07); // move pointer to SQW address
  Wire.write(0x10); //  sends 0x10 (hex) 00010000 (binary)
  Wire.endTransmission();
}

void converttall()
{
  ds.reset();
  ds.skip();
  ds.write(0x44,1);         // start conversion, with parasite power on at the end  
}

float getTemperature(bool convertt=1)
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius = 36.6;
  
  if ( !ds.search(addr)) {

#ifdef PRINT_MESSAGES
   Serial.println("No more addresses.");
   Serial.println();
#endif
   ds.reset_search();
   delay(250);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
#ifdef PRINT_MESSAGES
  Serial.println("CRC is not valid!");
#endif
  return celsius;
}
  
//  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
#ifdef PRINT_MESSAGES
      Serial.println("  Chip = DS18S20");  // or old DS1820
#endif
      type_s = 1;
      break;
    case 0x28:
#ifdef PRINT_MESSAGES
      Serial.println("  Chip = DS18B20");
#endif
      type_s = 0;
      break;
    case 0x22:
#ifdef PRINT_MESSAGES
      Serial.println("  Chip = DS1822");
#endif
      type_s = 0;
      break;
    default:
#ifdef PRINT_MESSAGES
      Serial.println("Device is not a DS18x20 family device.");
#endif
      return celsius;
  }

  if (convertt) 
  {
    converttall();
    delay(750);     
  }
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

#ifdef PRINT_MESSAGES
    Serial.print("  Data = ");
    Serial.print(present,HEX);
    Serial.print(" ");
#endif    
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
#ifdef PRINT_MESSAGES
      Serial.print(data[i], HEX);
      Serial.print(" ");
#endif
  }

#ifdef PRINT_MESSAGES
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();
#endif
  
  // convert the data to actual temperature

  raw = (data[1] << 8) | data[0];

#ifdef PRINT_MESSAGES
  Serial.print("raw=");
  Serial.print(raw);
#endif

  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  } 

#ifdef PRINT_MESSAGES
  Serial.print(" raw=");
  Serial.print(raw);
#endif

  int sign = 1;
  if (data[1]&128)
  {
    raw = ~raw+1;
    sign=-1;
  }
  cels = raw * sign;

#ifdef PRINT_MESSAGES
  Serial.print(" raw=");
  Serial.print(raw);    
  Serial.println();
#endif
    
  celsius = (float)cels / 16.0;
  
  return celsius;
}

void setup() {
  
  setupInProgress = true;

  Serial.begin(9600);

#ifdef PRINT_MESSAGES
  Serial.begin(9600);
#endif
  // pstr stores strings in flash to save RAM
  cout << endl << pstr("FreeRam: ") << FreeRam() << endl;

#ifdef INTERRUPT
  pinMode(ledPin, OUTPUT);              // порт как выход
  attachInterrupt(0, blink, RISING); // привязываем 0-е прерывание к функции blink().
#endif
  // set up the LCD's number of columns and rows: 
  lcd.init();
  lcd.createChar(1, gradus);
    
  Wire.begin();
  RTC.begin(); 
  
  if (! RTC.isrunning()) {
#ifdef PRINT_MESSAGES
    Serial.println("RTC is NOT running!");
#endif
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  // set date time callback function
  SdFile::dateTimeCallback(dateTime);

  converttall();
  delay(750);
  lastTemperature = getTemperature(0);

  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  // if SD chip select is not SS, the second argument to init is CS pin number
  if (!sd.init(SPI_HALF_SPEED, CHIP_SELECT)) sd.initErrorHalt();

  // create a new file in root, the current working directory
  char name[] = "LOGGER00.CSV";

  for (uint8_t i = 0; i < 100; i++) {
    name[6] = i/10 + '0';
    name[7] = i%10 + '0';
    if (sd.exists(name)) continue;
    logfile.open(name);
    break;
  }
  if (!logfile.is_open()) error("file.open");

  cout << pstr("Logging to: ") << name << endl;

  // format header in buffer
  obufstream bout(buf, sizeof(buf));

  bout << pstr("deviceId, date, time, temp") << endl;
  
  logfile << buf << flush;

#ifdef INTERRUPT
  sqw(); 
#endif

}

void writeEEPROM()
{
    // use buffer stream to format line
    obufstream bout(buf, sizeof(buf));
    bout << "1, "<< prev;

    bout << ',' << lastTemperature;
    bout << endl;

    cout << buf;

    // log data and flush to SD
    logfile << buf << flush;
    // check for error
    if (!logfile) error("write data failed");
}

void loop() {
  
  digitalWrite(ledPin, state);          // выводим state  

  DateTime now = RTC.now();
  char stringOne[17] = "";
  char stringTwo[17] = "";
  
  if (now.year()!=prev.year() || now.month()!=prev.month() || now.day()!=prev.day())
  { 
  // set the cursor to column 0, line 1
    lcd.setCursor(0, 0);
    // print the day, month, year and dayofweek
    sprintf(stringOne,"%02d/%02d/%04d %s", now.day(), now.month(), now.year(), Day[now.dayOfWeek()]);
    lcd.print(stringOne);

  }

  // get ds18b20 temperature data on one minute period
    
  if (now.second()%10==9 && now.second()!=prev.second()) // every 10 (in 9-th) second send convertt command to 1-wire
  {
    converttall();
  }

  if (now.second()%10==0 && now.second()!=prev.second())   // every 10 (in 10-th) second read convertt result from 1-wire
  {
    lastTemperature = getTemperature(0);
#ifdef PRINT_MESSAGES
    Serial.println(lastTemperature);
#endif
    if (now.minute()!=prev.minute())
      writeEEPROM();
  }

  lcd.setCursor(0, 1);
  int t = (lastTemperature - (int)lastTemperature) * 100 ;
  sprintf(stringTwo,"%02d:%02d:%02d %c%02d.%02d%c", now.hour(), now.minute(), now.second(), lastTemperature<0?'-':(lastTemperature==0?' ':'+'), abs((int)lastTemperature), abs(t), 1);
  lcd.print(stringTwo);    
  prev = now; 
  setupInProgress = false;
  
}


void blink()
{
  if (!setupInProgress)
  {
    if (count%10 == 0)
    {
    //writeEEPROM();
    }
    state = !state;                    // меняем значение на противоположное
    count +=1;    
  }
}


