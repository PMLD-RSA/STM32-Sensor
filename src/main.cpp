#include <Arduino.h>

// HC-SR04 (VCC ke pin 5V board, board diberi daya lewat micro USB, GND ke G board)
//   Pin 5.0V ST-Link clone tidak cukup untuk sensor (ECHO tidak pernah merespons).
//   TRIG -> PA0
//   ECHO -> PB10, langsung tanpa resistor. PB10 pin FT (tahan 5V) dengan syarat:
//     - STM32 & sensor dapat daya dari sumber yang sama (ST-Link)
//     - pin tetap INPUT biasa, jangan INPUT_PULLUP / analogRead
//   JANGAN pindah ECHO ke PA0-PA7 / PB0-PB1: bukan FT, bisa rusak kena 5V.
const uint8_t PIN_TRIG = PA0;
const uint8_t PIN_ECHO = PB10;

// Hasil ukur terakhir, global supaya bisa dibaca lewat ST-Link tanpa serial.
// 0 = tidak ada pantulan (sensor tidak terbaca / di luar jangkauan).
volatile uint32_t jarak_cm = 0;
volatile uint32_t jumlah_ukur = 0;

// LED PC13 aktif LOW
void led(bool nyala) { digitalWrite(LED_BUILTIN, nyala ? LOW : HIGH); }

// Lama pulsa HIGH di ECHO (us), 0 kalau timeout.
// Tidak pakai pulseIn(): di core STM32duino ini stateMask-nya uint8_t, jadi
// untuk pin nomor 8-15 (mis. PB10) mask terpotong jadi 0 dan selalu timeout.
uint32_t ukurPulsaEcho(uint32_t timeout_us) {
  uint32_t t0 = micros();
  while (!digitalRead(PIN_ECHO)) {
    if (micros() - t0 > timeout_us) return 0;
  }
  uint32_t naik = micros();
  while (digitalRead(PIN_ECHO)) {
    if (micros() - t0 > timeout_us) return 0;
  }
  return micros() - naik;
}

uint32_t ukurJarak() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // ECHO modul ini naik ~2.3 ms setelah TRIG; 40 ms cukup untuk ~4 m
  uint32_t durasi = ukurPulsaEcho(40000);
  if (durasi == 0 || durasi > 30000) return 0;  // tidak ada benda / > ~5 m
  return durasi / 58;  // us -> cm
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  led(false);
  Serial.begin(115200);
}

void loop() {
  static uint32_t terakhirUkur = 0, terakhirKedip = 0;
  static bool ledNyala = false;

  if (millis() - terakhirUkur >= 60) {  // HC-SR04 butuh jeda >= 60 ms
    terakhirUkur = millis();
    jarak_cm = ukurJarak();
    jumlah_ukur++;
    Serial.print("Jarak: ");
    Serial.print(jarak_cm);
    Serial.println(" cm");
  }

  // Indikator LED:
  //   tidak ada pantulan -> kedip singkat tiap 2 detik (tanda program hidup)
  //   ada benda          -> kedip, makin dekat makin cepat (10 ms per cm)
  if (jarak_cm == 0) {
    ledNyala = false;
    led(millis() % 2000 < 50);
    return;
  }
  uint32_t interval = constrain(jarak_cm * 10, 50, 1000);
  if (millis() - terakhirKedip >= interval) {
    terakhirKedip = millis();
    ledNyala = !ledNyala;
    led(ledNyala);
  }
}
