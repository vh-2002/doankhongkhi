#define BLYNK_TEMPLATE_ID "TMPL6hVY1G-pi"
#define BLYNK_TEMPLATE_NAME "kiểm tra không khí"
#define BLYNK_AUTH_TOKEN "e0-Dlx_-EP2e8VElYm6tWbrFC_4mektP"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#include <DHT.h>
#include<Servo.h>


#define DHTPIN 4 
#define DHTTYPE DHT11 
#define quat_PIN 13
#define motor_PIN 12
#define SENSOR_PIN 33
char ssid[] = "Phương Hằng";
char pass[] = "hungtran041002";

// Blynk auth token
char auth[] = BLYNK_AUTH_TOKEN;

 //Initialize the BlynkTimer
BlynkTimer timer;

const int GAS_SENSOR_PIN= 35;  // Chân ADC để đọc giá trị cảm biến MQ-5
const int buzzer_pin = 26; // Chân số 25 được sử dụng cho còi (buzzer)

DHT dht(DHTPIN, DHTTYPE);
const int gas_threshold = 500; 
int measurePin = 34;
int ledPower = 32;
unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;
const int sampleWindow = 50;                              // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
String gas = "";
Servo cua; 
void setup(){
  Serial.begin(9600); 
  pinMode(ledPower,OUTPUT);
  pinMode(motor_PIN, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(GAS_SENSOR_PIN,INPUT);
  pinMode(quat_PIN, OUTPUT);
  pinMode(DHTPIN, INPUT);
  cua.attach(14);
  pinMode (SENSOR_PIN, INPUT); // Set the signal pin as input  
  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  analogReadResolution(12); 
  // Kết nối Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, pass);
  
  // Chờ kết nối Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Kết nối Blynk
  Serial.println("Connecting to Blynk...");
  Blynk.begin(auth, ssid, pass);
  
  // Chờ kết nối Blynk
  while (!Blynk.connected()) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Blynk");


}

void loop(){
  Blynk.run();
  digitalWrite(ledPower,LOW);
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH);
  delayMicroseconds(sleepTime);
  // Đọc giá trị từ cảm biến MQ-5
  int gas_level = analogRead(GAS_SENSOR_PIN); 
  gas += gas_level;

  Serial.println(gas);
  gas = "";
  // Kiểm tra xem giá trị khí gas vượt quá ngưỡng cho phép hay không
  if (gas_level > gas_threshold) {
    // Kích hoạt còi để phát ra âm thanh cảnh báo
    digitalWrite(buzzer_pin, HIGH);

     lcd.setCursor(0, 2);
     lcd.print("BAO DONG_WARNING");
    
  } else { 
    // Tắt còi
    digitalWrite(buzzer_pin, LOW);
     lcd.setCursor(0, 2);
     lcd.print("Binh thuong");
  }

  calcVoltage = voMeasured*(3.3/1024);
  dustDensity = 0.17*calcVoltage-0.1;

  if ( dustDensity < 0)
  {
    dustDensity = 0.00;
  }
  if (dustDensity > 1.1) {
    digitalWrite(motor_PIN, HIGH); // Bật quạt
    Serial.println("Fan turned on!");
  } else {
    digitalWrite(motor_PIN, LOW); // Tắt quạt
    Serial.println("Fan turned off!");
  }
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  if (temperature > 32) {
    digitalWrite(quat_PIN, HIGH); // Bật quạt
    Serial.println("Fan turned on!");
  } else {
    digitalWrite(quat_PIN, LOW); // Tắt quạt
    Serial.println("Fan turned off!");
  }
  unsigned long startMillis= millis();                   // Start of sample window
  float peakToPeak = 0;                                  // peak-to-peak level
  unsigned int signalMax = 0;                            //minimum value
  unsigned int signalMin = 4095;   
  while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(SENSOR_PIN);                    //get reading from microphone
      if (sample < 4095)                                  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;                           // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;                           // save just the min levels
         }
      }
   }
  peakToPeak = signalMax - signalMin;                    // max - min = peak-peak amplitude
  int db = map(peakToPeak,20,900,49.5,90);   

 
  if(db<63){
    cua.write(180);
  }
  else{
    cua.write(0);
  }
  int sensorValue = analogRead(GAS_SENSOR_PIN); // Đọc giá trị từ cảm biến

  // Chuyển đổi giá trị analog sang nồng độ khí gas
  float gasConcentration = map(sensorValue, 0, 4095, 0, 1000);
 
  Blynk.virtualWrite(V0,temperature);
  Blynk.virtualWrite(V1,humidity);
  Blynk.virtualWrite(V2, db);
  Blynk.virtualWrite(V3,dustDensity );
  Blynk.virtualWrite(V4,gasConcentration );


  // In giá trị nồng độ lên Serial Monitor
  Serial.print("Gas concentration: ");
  Serial.print(gasConcentration);
  Serial.println(" ppm");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.println("Raw Signal Value (0-1023):");
  Serial.println(voMeasured);
  Serial.println("Voltage:");
  Serial.println(calcVoltage);
  Serial.println("Dust Density:");
  Serial.println(dustDensity);
  Serial.print("Loudness: ");
  Serial.print(db);
  Serial.println("dB");
   // Display the results on the LCD
  lcd.setCursor(0, 0);
  lcd.print("bui: ");
  lcd.print(dustDensity);
  lcd.print(" ug/m3");
  lcd.setCursor(0, 1); // Di chuyển đến hàng đầu tiên
  lcd.print("t:");
  lcd.print(temperature);
  lcd.print(" C");
  lcd.print("H: ");
  lcd.print(humidity);
  lcd.print(" %");
  lcd.setCursor(0, 3); 
  lcd.print("Loudness: ");
  lcd.print(db);
  lcd.print(" dB");

  delay(1000);
}