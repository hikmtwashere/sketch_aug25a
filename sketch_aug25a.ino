/*
--------------YAPILACAKLAR--------------
+ sensör kütüphaneleri ve sensörleri başlatma
+ kalman filtresi chatgpt hallet.
+ test algoritmasının benzerini uygula
+ 5 kez negatif oldu mu şartı ekle
+ kurtarmalar ekle
+ haberleşme kısmını taslak olarak hazırla
+ YAZILAN KODU MODÜLER HALE GETİR DAHA ANLAŞILIR VE PARÇALARA AYRILMIŞ OLSUN FONSKİYON STRUCTER GİBİ
-------------------------------------------
*/

/*SON GÜNCELLEME
    05.07.25
MPL3115A2 için başlatma ve veri çekme eklendi düzeltmeler daha sonra yapılacaktır.
*/

//--------Kütüphane TANIMLAMALARI----------

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_MPL3115A2.h>
// YAVUZUN BİLGİSAYARININ ŞİFRESİ 8262


/*struct SensorVerileri{//sturcter simdilik gereksiz gibi belki daha sonra eklerim
float basinc;
float sicaklik;
float irtifa;
float yaw, pitch, roll;
};
SensorVerileri veriler;*/


//--------DEĞİŞKEN TANIMLAMALARI----------
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool apogee_tespiti_irtifa_kilidi, apogee_tespiti,apogee_tespiti_gyro_kilidi, ana_parasut;
float anlik_irtifa, onceki_irtifa, max_irtifa; 

//--------LPS22HB TANIMLAMALARI----------
#define LPS22HB_ADDRESS        0x5C
#define LPS22HB_WHO_AM_I       0x0F
#define LPS22HB_CTRL_REG1      0x10
#define LPS22HB_STATUS_REG     0x27
#define LPS22HB_PRESS_OUT_XL   0x28
#define LPS22HB_PRESS_OUT_L    0x29
#define LPS22HB_PRESS_OUT_H    0x2A
#define LPS22HB_TEMP_OUT_L     0x2B
#define LPS22HB_TEMP_OUT_H     0x2C
#define SEA_LEVEL_PRESSURE     1013.25

float referenceAltitude = 0.0;
bool calibrated = false;

byte readRegister(byte reg) {
  Wire.beginTransmission(LPS22HB_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LPS22HB_ADDRESS, 1);
  return Wire.read();
}

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(LPS22HB_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
//--------------------------------------------

//---------------MPL3115A2 tanımlamaları-------
Adafruit_MPL3115A2 baro;
//---------------------------------------------

//suruklenme parasütü aktifleştirme fonksiyonu
  void suruklenme_parasutu_ac(){
    //10.dijital pin aktifleştirerek piro kartuşuna sinyal gönderirir (biz led kullanıcaz aktiflerştimeye)
    digitalWrite(10,HIGH);
       }

    void ana_parasutu_ac(){
      //12.dijital pin aktifleştirerek ana piro kartuşuna sinyal gönderirir (biz led kullanıcaz aktiflerştimeye)
      digitalWrite(12,HIGH);
       }

void setup(void){


  //-------------LPS22HB BAŞLATMA-------------
  Wire.begin();               // Mega: SDA = 20, SCL = 21
  Serial.begin(115200);
  delay(1000);

  byte whoami = readRegister(LPS22HB_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);

  if (whoami != 0xB1) {
    Serial.println(" LPS22HB sensörü bulunamadı!");
    while (1);
  }

  writeRegister(LPS22HB_CTRL_REG1, 0b01000000); // 1 Hz, PD=1
  Serial.println("Sensör başlatıldı. Kalibrasyon bekleniyor...");
  //--------------------------------------------


//----------------BNO055 BAŞLATMA----------------------
  if(!bno.begin())
  {
    Serial.print("BNO055 yokk!");
    while(1);
  }
  delay(1000);
    bno.setExtCrystalUse(true);
//----------------------------------------------------


//------------------MPL3115A2 başlatma----------------
 if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }
baro.setSeaPressure(1015);// Aksaray 5 temmuz deniz seviyesine göre basıncı olarak ekledim
//------------------------------------------------------------


//Pin konfigürasyonları
  pinMode(10,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(9,OUTPUT);
}
void loop()
{

//-------------LPS22HB VERİ ALMA-------------
  byte status = readRegister(LPS22HB_STATUS_REG);

  if ((status & 0x03) == 0x03) {
    uint8_t pressureXL = readRegister(LPS22HB_PRESS_OUT_XL);
    uint8_t pressureL = readRegister(LPS22HB_PRESS_OUT_L);
    uint8_t pressureH = readRegister(LPS22HB_PRESS_OUT_H);
    int32_t pressureRaw = ((int32_t)pressureH << 16) | ((int32_t)pressureL << 8) | pressureXL;
    float pressure = pressureRaw / 4096.0;

    uint8_t tempL = readRegister(LPS22HB_TEMP_OUT_L);
    uint8_t tempH = readRegister(LPS22HB_TEMP_OUT_H);
    int16_t tempRaw = ((int16_t)tempH << 8) | tempL;
    float temperature = tempRaw / 100.0;

    float currentAltitude = 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));

    if (!calibrated && pressure > 300 && pressure < 1100) {
      referenceAltitude = currentAltitude;
      calibrated = true;
      Serial.print(" Başlangıç yüksekliği referans alındı: ");
      Serial.print(referenceAltitude, 2);
      Serial.println(" m");
    }

   anlik_irtifa = currentAltitude - referenceAltitude;

    Serial.print("Basınç: ");
    Serial.print(pressure, 2);
    Serial.print(" hPa | Sıcaklık: ");
    Serial.print(temperature, 2);
    Serial.print(" °C | Yükseklik (∆): ");
    Serial.print(anlik_irtifa, 2);
    Serial.println(" m");
  }
//-----------------------------------------------------

//---------------MPL3115A2 VERİ ALMA-------------------
  float pressure = baro.getPressure();
  float altitude = baro.getAltitude();
  float temperature = baro.getTemperature();

  Serial.println("-----------------");
  Serial.print("Basinc = ");
  Serial.print(pressure); // variable lar değiştirilecek unutma!!!!
  Serial.println(" hPa");
  Serial.print("altitude = "); 
  Serial.print(altitude); 
  Serial.println(" m");
  Serial.print("temperature = "); 
  Serial.print(temperature); 
  Serial.println(" C");
//------------------------------------------------

//----------------BNO055 VERİ ALMA----------------------
  sensors_event_t event;
  bno.getEvent(&event);

  Serial.print(F("EULER ACILARI: "));
   Serial.print(" | Yaw: ");
  Serial.print((float)event.orientation.x);

  Serial.print(F(" "));
    Serial.print(" | Pitch: ");
  Serial.print((float)event.orientation.y);

  Serial.print(F(" "));
    Serial.print(" | Roll: ");
  Serial.print((float)event.orientation.z);

  Serial.println(F(""));
//-----------------------------------------------------
// irtifa kilidi sürüklenme için
  
//-------------KURTARMA SİSTEMİ ALGORİTMASI-------------
if(!apogee_tespiti){
if((anlik_irtifa-onceki_irtifa)<-5){
  apogee_tespiti_irtifa_kilidi=true;
  digitalWrite(9,HIGH);
  Serial.println("Suruklenme parasutu irtifa onayi.");
}  
else{
  onceki_irtifa=anlik_irtifa;
}

 // gyro kilidi sürüklenme için
  if ((abs((float)event.orientation.y) > 45.00) || (abs((float)event.orientation.z) > 45.00)) {
    apogee_tespiti_gyro_kilidi=true;
    digitalWrite(12,HIGH); //sari
    Serial.println("Suruklenme parasutu gyro onayi.");
  }

  //sürüklenme aktifleştirme 
  if((apogee_tespiti_irtifa_kilidi) && (apogee_tespiti_gyro_kilidi)){
suruklenme_parasutu_ac();
apogee_tespiti=true;
Serial.println("Suruklenme parasutu aktiflestirildi.");

}
}

  if(apogee_tespiti==true && anlik_irtifa<100){
ana_parasutu_ac();
ana_parasut=true;
digitalWrite(11,HIGH);//BUZZER ÖTÜŞÜ
Serial.println("Ana parasut aktiflestirildi.");
  }
  delay(1000);
}
//-------------------------------------------------


//sürüklenme gyro sarı 12. pin
//sürüklenme baro yeşil 13.pin
//sürüklenme paraşüt açma  kırmızı 10. pin
//ana paraşüt buzzer 11.pin
