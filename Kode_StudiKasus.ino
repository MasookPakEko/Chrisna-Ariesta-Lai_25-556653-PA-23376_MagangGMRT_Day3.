//Chrisna Ariesta Lai
//NIM 25/556653/PA/23376

//Library
#include <Wire.h>             //sensor I2C
#include <Adafruit_MPU6050.h> //library sensor MPU6050
#include <Adafruit_Sensor.h>  //library sensor
#include <ESP32Servo.h>       //Library servo khusus ESP32

//Bikin variabel sensor & servo
Adafruit_MPU6050 gyro;         //nama dari MPU6050
Servo servo1, servo2, servo3, servo4, servo5; //Bikin 5 variabel servo

//Pin dr servo yg dipasang ke esp32
const int servo1_pin = 19;
const int servo2_pin = 18;
const int servo3_pin = 5;
const int servo4_pin = 17;
const int servo5_pin = 16;
const int ir_pin = 23; // Pinnya IR
//pin I2C (SDA=21 dan SCL=22) g usah ditulis, udah default

//variabel global bantuan
sensors_event_t a, g, temp; //tempat nyimpen data mentah sensor
const int servo_awal = 90; //posisi awal
int statusPIR_sebelumnya = LOW; //penanda apakah servo sedang dikunci (utk IR)

//variabel khusus utk poin 3 (Yaw)
bool yaw_bergerak = false;     //penanda kalau sistem lagi muter yaw
unsigned long yaw_waktu_berhenti = 0; //catatan jam kapan yaw berhenti
bool servo5_awal = true;   //penanda klo servo 5 sedang di posisi awal

void setup() {
  //tampilkan ke monitor buat ngecek
  Serial.begin(115200);
  while (!Serial); // Tunggu Serial siap (penting buat bbrp board)
  Serial.println("Stabilizer Goyang Mulai!");

  //Nyalain I2C
  Wire.begin(); 

  //nyalakan MPU6050
  if (!gyro.begin()) {
    Serial.println("MPU6050 is not detected!");
    while (1) delay(10); //stop klo g ketemu
  }
  Serial.println("MPU6050 ready!");

  //setting sensor
  gyro.setAccelerometerRange(MPU6050_RANGE_8_G); //rentang akselerometer
  gyro.setGyroRange(MPU6050_RANGE_500_DEG);     //Rentang giiroskop
  gyro.setFilterBandwidth(MPU6050_BAND_21_HZ); //filter biar g terlalu goyang

  //Setting pin Infrared
  //atur pin IR sebagai input
  pinMode(ir_pin, INPUT); 

  //timer utk servo ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50); // Frekuensi standar servo 50Hz
  servo2.setPeriodHertz(50); 
  servo3.setPeriodHertz(50); 
  servo4.setPeriodHertz(50); 
  servo5.setPeriodHertz(50); 

  //attach servo ke pin
  //Angka 500 & 2400 itu batas pulsa servo (biar 0-180 derajatnya pas)
  servo1.attach(servo1_pin, 500, 2400); 
  servo2.attach(servo2_pin, 500, 2400);
  servo3.attach(servo3_pin, 500, 2400);
  servo4.attach(servo4_pin, 500, 2400);
  servo5.attach(servo5_pin, 500, 2400); 

  //semua servo ke posisi awal 90 derajat pas baru nyala
  servo1.write(servo_awal);
  servo2.write(servo_awal);
  servo3.write(servo_awal);
  servo4.write(servo_awal);
  servo5.write(servo_awal);
  delay(500); // Kasih waktu servo buat gerak
}

void loop() {
  //poin4: cek sensor PIR 
  int pirState = digitalRead(ir_pin);

  //Cek kalau gerakan baru terdeteksi
  if (pirState == HIGH && statusPIR_sebelumnya == LOW) {
    
    //Pindah ke posisi bebas
    servo1.write(45);
    servo2.write(45);
    servo3.write(45);
    servo4.write(45);
    servo5.write(45);
    delay(500); // Tahan 0.5 detik

    //kembali ke posisi awal
    servo1.write(servo_awal);
    servo2.write(servo_awal);
    servo3.write(servo_awal);
    servo4.write(servo_awal);
    servo5.write(servo_awal);
    delay(500); // Tahan 0.5 detik untuk stabilisasi
    
    statusPIR_sebelumnya = HIGH; // Tandai bahwa kita sudah tahu PIR sedang HIGH
  } 
  // Kalau PIR sudah mati (LOW) lagi
  else if (pirState == LOW) {
    statusPIR_sebelumnya = LOW; // Reset status
    
    //sensor membaca data
    gyro.getEvent(&a, &g, &temp);

    //Poin 1: roll (servo 1 & 2)
    //bagi nilai akselerasi dgn gravitasi (9.81)
    float roll_input = a.acceleration.y / 9.81;
    roll_input = constrain(roll_input, -1.0, 1.0);
    float roll_rad = asin(roll_input);

    // Ubah radian ke derajat
    float roll_sudut = roll_rad * 180.0 / PI;

    /*Ubah skala sudut Roll (-90 sampai90) jadi posisi Servo (0 sampai 180 derajat)
    Tapi dibalik supaya sservo melawan arah miringnya*/
    int servo1_pos = map(roll_sudut, -90, 90, 180, 0); 
    
    //pastikan nilainya tdk lebih dari 180 atau krg dari 0
    servo1_pos = constrain(servo1_pos, 0, 180);
    
    //gerakkan servo 1 & 2
    servo1.write(servo1_pos);
    servo2.write(servo1_pos);


    //poin2: pitch (servo 3 & 4)
    //Bagi nilai akselerasi dgn gravitasi (9.81)
    float pitch_input = -a.acceleration.x / 9.81;
    pitch_input = constrain(pitch_input, -1.0, 1.0);
    float pitch_rad = asin(pitch_input);

    // Ubah radian ke derajat (-90 s/d +90)
    float pitch_sudut = pitch_rad * 180.0 / PI;
    
    //Ubah skala sudut pitch jadi posisi servo
    int servo3_pos = map(pitch_sudut, -90, 90, 0, 180); 
    
    //jaga-jaga agar tidak tembus batas 0 hingga 180
    servo3_pos = constrain(servo3_pos, 0, 180); 
    
    //servo 3 dan 4 bergerak
    servo3.write(servo3_pos);
    servo4.write(servo3_pos);

    //poin 3: yaw (servo 5)
    /* Pake data Giroskop Z (g.gyro.z)
    Library Adafruit satuannya rad/s, ubah dlu ke deg/s */
    float yaw_speed = g.gyro.z * 57.3; // 57.3 itu kira2 180/PI

    //misalnya kalau kecepatannya 20 itu artinya sudah bergerak
    if (abs(yaw_speed) > 20.0) {
      yaw_bergerak = true;       //ubah kondisi menjadi sedang bergerak
      servo5_awal = false;      //tidak di posisi awal
      
      //ubah skala kecepatan (misal -200 sampai 200) jadi posisi servo (0 - 180). Searah
      int servo5_pos = map(yaw_speed, -200, 200, 0, 180);
      
      //Jaga-jaga dan gerakkan servo
      servo5.write(constrain(servo5_pos, 0, 180));
      
      //Catat jam terakhir dia gerak (buat reset timer 1 detik)
      yaw_waktu_berhenti = millis(); 
    } 
    else if (yaw_bergerak) { //Klo tadi gerak, tapi skrg diem
      yaw_bergerak = false;    //diam
      //servo tdk diberi perintah apa2, jd servo diem di posisi terakhir
      yaw_waktu_berhenti = millis(); //mulai hitung timer 1 detik dr sekarang
    }
  }

  //Cek kalau sudah diam 1 detik dan servo belum di posisi awal
  if (!yaw_bergerak && !servo5_awal) { 
    if (millis() - yaw_waktu_berhenti > 1000) { //kalau timer sudah lebih dari 1000ms (1 detik)
      servo5.write(servo_awal); //kembali ke posisi 90 derajat
      servo5_awal = true;   //sudah kembali ke posisi awal
    }
  }
  
  delay(20);
}