#include <LiquidCrystal.h>
LiquidCrystal lcd(10, 8, 5, 4, 3, 2);
int led_kontrol;
int analog0degeri;
void setup() {
  pinMode(9, OUTPUT);//9. pwm pinini çıkış olarak kullanacağımızı belirttik
lcd.begin(16, 2);  /*lcd nin kurulumunu kaç satır kaç sütun olduğunu belirttik  */

}

void loop() {
  analog0degeri=analogRead(A0);//potansiyometre ile ayarladığımız voltaj değerini sayısal karşılığını değişkene atadık
led_kontrol=map(analog0degeri,0,1023,0,255);//map fonksiyonu ile orantılama yolu ile değişkene işlemler uyguladık
analogWrite(9,led_kontrol);//analog yazma işlemi ile 9. pwm pinine led_kontrol değişkenine göre şekillenmesini sağladık
lcd.clear();

lcd.setCursor(0,0);
lcd.print("PWM degeri: ");
lcd.print(led_kontrol);

lcd.setCursor(0,1);
lcd.print("a0 degeri: ");
lcd.print(analog0degeri);
delay(10);


}
