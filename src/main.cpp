#include <Arduino.h>

// =============================================================================
// Satu firmware untuk dua peran, dipilih dari environment platformio.ini:
//   env:node  -> baca sensor, kirim ke STM relay lewat LoRa
//   env:relay -> baca sensor sendiri + teruskan frame dari node ke Raspberry Pi
//
// Topologi LoRa (E220 mode "fixed transmission" di STM, tiap hop beda channel):
//
//   STM node  ---CH 0x42--->  STM relay  ---CH 0x41--->  Raspberry Pi
//   (0x0002, CH 0x43)         (0x0001, CH 0x42)          (0x0000, CH 0x41)
//
// Node memancar di channel relay, jadi Raspi tidak pernah menerima langsung
// dari node (tidak ada data dobel). Raspi tetap mode transparan, config.py
// tidak perlu diubah.
//
// Frame (sama dengan yang di-parse raspi-gateway/sensor/processor.py):
//   JARAK,<DEVICE_CODE>,<cm>,<nomor_urut>\n
// =============================================================================

#ifndef DEVICE_CODE
#error "DEVICE_CODE belum di-set di build_flags platformio.ini"
#endif
#ifndef ROLE_RELAY
#define ROLE_RELAY 0
#endif

// HC-SR04 (VCC ke pin 5V board, board diberi daya lewat micro USB, GND ke G board)
//   Pin 5.0V ST-Link clone tidak cukup untuk sensor (ECHO tidak pernah merespons).
//   TRIG -> PA0
//   ECHO -> PB10, langsung tanpa resistor. PB10 pin FT (tahan 5V) dengan syarat:
//     - STM32 & sensor dapat daya dari sumber yang sama (ST-Link)
//     - pin tetap INPUT biasa, jangan INPUT_PULLUP / analogRead
//   JANGAN pindah ECHO ke PA0-PA7 / PB0-PB1: bukan FT, bisa rusak kena 5V.
const uint8_t PIN_TRIG = PA0;
const uint8_t PIN_ECHO = PB10;

// LoRa E220-900T22D (VCC ke 5V, GND ke G). UART2, Serial (UART1) tetap debug.
//   E220 RXD -> PA2 (USART2 TX)     E220 TXD -> PA3 (USART2 RX)
//   E220 M0  -> PB12                E220 M1  -> PB13
//   E220 AUX -> PB14
const uint8_t PIN_M0  = PB12;
const uint8_t PIN_M1  = PB13;
const uint8_t PIN_AUX = PB14;
// Core STM32duino versi ini: kelas UART konkret bernama Uart (HardwareSerial abstrak)
Uart LoraSerial(PA3, PA2);  // (RX, TX)

// Alamat & channel tiap perangkat. REG0 harus sama di semua modul,
// termasuk LORA_CFG di raspi-gateway/config.py (0x62 = 9600 bps, air 2.4k).

// Daya pancar (REG1 bit 1-0), boleh beda tiap modul karena hanya untuk TX:
//   0x00 = 22 dBm, 0x01 = 17 dBm, 0x02 = 13 dBm, 0x03 = 10 dBm (paling kecil)
// Sedang 10 dBm untuk tes relay jarak dekat; kembalikan ke 0x00 untuk lapangan.
const uint8_t LORA_DAYA = 0x03;
const uint8_t RASPI_ADDH = 0x00, RASPI_ADDL = 0x00, RASPI_CH = 0x41;
const uint8_t RELAY_ADDH = 0x00, RELAY_ADDL = 0x01, RELAY_CH = 0x42;
const uint8_t NODE_ADDH  = 0x00, NODE_ADDL  = 0x02, NODE_CH  = 0x43;

// REG3 0x43: bit6 = fixed transmission, LBT off, WOR 2000 ms (sama dgn Raspi)
#if ROLE_RELAY
const uint8_t LORA_CFG[6] = {RELAY_ADDH, RELAY_ADDL, 0x62, LORA_DAYA, RELAY_CH, 0x43};
const uint8_t TUJUAN[3]   = {RASPI_ADDH, RASPI_ADDL, RASPI_CH};
#else
const uint8_t LORA_CFG[6] = {NODE_ADDH, NODE_ADDL, 0x62, LORA_DAYA, NODE_CH, 0x43};
const uint8_t TUJUAN[3]   = {RELAY_ADDH, RELAY_ADDL, RELAY_CH};
#endif

// Kirim tiap 5 detik + jitter acak 0-1 s, supaya node dan relay tidak terus
// memancar bersamaan (E220 half-duplex: relay tuli saat sedang memancar).
const uint32_t KIRIM_INTERVAL_MS = 5000;
const uint32_t KIRIM_JITTER_MS   = 1000;

// Hasil ukur terakhir, global supaya bisa dibaca lewat ST-Link tanpa serial.
// 0 = tidak ada pantulan (sensor tidak terbaca / di luar jangkauan).
volatile uint32_t jarak_cm = 0;
volatile uint32_t jumlah_ukur = 0;
volatile uint32_t jumlah_kirim = 0;
volatile uint32_t jumlah_terusan = 0;  // khusus relay
volatile bool lora_siap = false;

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

// -----------------------------------------------------------------------------
// E220
// -----------------------------------------------------------------------------

// AUX LOW = modul sibuk (self-check, memancar, atau memproses konfigurasi)
bool tungguAux(uint32_t timeout_ms = 2000) {
  uint32_t t0 = millis();
  while (!digitalRead(PIN_AUX)) {
    if (millis() - t0 > timeout_ms) return false;
  }
  delay(2);  // settling time minimal setelah AUX HIGH
  return true;
}

// M0 M1: 0 0 = normal, 1 1 = konfigurasi (UART wajib 9600 8N1)
void setMode(uint8_t m0, uint8_t m1) {
  tungguAux();
  digitalWrite(PIN_M0, m0);
  digitalWrite(PIN_M1, m1);
  delay(2);
  tungguAux();
}

// Baca register, tulis permanen (C0) hanya kalau berbeda. Sama seperti
// E220.konfigurasi() di raspi-gateway/lora/e220.py.
bool konfigurasiE220() {
  const uint8_t N = sizeof(LORA_CFG);
  uint8_t resp[3 + N];

  setMode(1, 1);
  while (LoraSerial.available()) LoraSerial.read();

  const uint8_t perintahBaca[3] = {0xC1, 0x00, N};
  LoraSerial.write(perintahBaca, 3);
  if (LoraSerial.readBytes(resp, 3 + N) != 3 + N || resp[0] != 0xC1) {
    Serial.println("[E220] Gagal baca register");
    setMode(0, 0);
    return false;
  }

  if (memcmp(resp + 3, LORA_CFG, N) != 0) {
    Serial.println("[E220] Konfigurasi berbeda, menulis ulang...");
    // Jeda singkat sebelum tulis agar modul tidak membalas FF FF FF
    delay(50);
    tungguAux();
    const uint8_t perintahTulis[3] = {0xC0, 0x00, N};
    LoraSerial.write(perintahTulis, 3);
    LoraSerial.write(LORA_CFG, N);
    if (LoraSerial.readBytes(resp, 3 + N) != 3 + N || resp[0] != 0xC1) {
      Serial.println("[E220] Gagal tulis konfigurasi");
      setMode(0, 0);
      return false;
    }
  }

  setMode(0, 0);
  Serial.println("[E220] Siap");
  return true;
}

// Mode fixed: 3 byte pertama (ADDH ADDL CHAN) = tujuan, sisanya payload.
// Ditulis tanpa jeda supaya E220 mengemasnya jadi satu paket radio.
void kirimLora(const char *data, size_t len) {
  tungguAux();
  LoraSerial.write(TUJUAN, 3);
  LoraSerial.write((const uint8_t *)data, len);
  LoraSerial.flush();
}

void kirimJarakSendiri() {
  char frame[48];
  int len = snprintf(frame, sizeof(frame), "JARAK,%s,%lu,%lu\n", DEVICE_CODE,
                     (unsigned long)jarak_cm, (unsigned long)jumlah_kirim);
  kirimLora(frame, len);
  jumlah_kirim++;
  Serial.print("[Kirim] ");
  Serial.write(frame, len);
}

#if ROLE_RELAY
// Kumpulkan byte dari node per baris, teruskan apa adanya ke Raspi kalau
// formatnya frame JARAK. device_code di dalam frame tidak diubah, jadi
// gateway tetap tahu data berasal dari node mana.
void teruskanDariNode() {
  static char baris[64];
  static size_t panjang = 0;
  static bool kepanjangan = false;
  static uint32_t byteTerakhir = 0;

  // Sisa baris yang tidak pernah ditutup '\n' dibuang setelah 500 ms
  if (panjang > 0 && millis() - byteTerakhir > 500) {
    panjang = 0;
    kepanjangan = false;
  }

  while (LoraSerial.available()) {
    char c = LoraSerial.read();
    byteTerakhir = millis();
    if (c == '\r') continue;
    if (c != '\n') {
      if (panjang < sizeof(baris) - 1) baris[panjang++] = c;
      else kepanjangan = true;
      continue;
    }

    if (!kepanjangan && panjang > 6 && strncmp(baris, "JARAK,", 6) == 0) {
      baris[panjang++] = '\n';
      kirimLora(baris, panjang);
      jumlah_terusan++;
      Serial.print("[Relay] ");
      Serial.write(baris, panjang);
    } else {
      Serial.println("[Relay] Frame tidak dikenal, dibuang");
    }
    panjang = 0;
    kepanjangan = false;
  }
}
#endif

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_AUX, INPUT);
  led(false);

  Serial.begin(115200);
  LoraSerial.begin(9600);
  LoraSerial.setTimeout(1000);

  Serial.print(ROLE_RELAY ? "Peran: RELAY, " : "Peran: NODE, ");
  Serial.println(DEVICE_CODE);

  // Seed dari UID chip supaya jitter tiap board berbeda
  randomSeed(HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2());

  tungguAux(3000);  // self-check E220 setelah power-on
  lora_siap = konfigurasiE220();
}

void loop() {
  static uint32_t terakhirUkur = 0, terakhirKedip = 0;
  static uint32_t terakhirKirim = 0, jedaKirim = KIRIM_INTERVAL_MS;
  static uint32_t terakhirCobaLora = 0;
  static bool ledNyala = false;

  // E220 belum merespons: coba konfigurasi ulang tiap 5 detik, LED kedip cepat
  if (!lora_siap) {
    led(millis() % 200 < 100);
    if (millis() - terakhirCobaLora >= 5000) {
      terakhirCobaLora = millis();
      lora_siap = konfigurasiE220();
    }
    return;
  }

#if ROLE_RELAY
  teruskanDariNode();
#endif

  if (millis() - terakhirUkur >= 60) {  // HC-SR04 butuh jeda >= 60 ms
    terakhirUkur = millis();
    jarak_cm = ukurJarak();
    jumlah_ukur++;
  }

  if (millis() - terakhirKirim >= jedaKirim) {
    terakhirKirim = millis();
    jedaKirim = KIRIM_INTERVAL_MS + random(KIRIM_JITTER_MS);
    kirimJarakSendiri();
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
