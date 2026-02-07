#include <Wire.h>

// ===================== User config =====================
static const uint8_t LSM6_ADDR = 0x6B;   // 你已经确认 scanner 是 0x6B
static const uint32_t I2C_HZ   = 100000; // 先用 100kHz 最稳
// =======================================================

// -------- LSM6DSM register map (subset) --------
static const uint8_t REG_FUNC_CFG_ACCESS = 0x01;
static const uint8_t REG_WHO_AM_I        = 0x0F;

static const uint8_t REG_CTRL1_XL        = 0x10;
static const uint8_t REG_CTRL2_G         = 0x11;
static const uint8_t REG_CTRL3_C         = 0x12;

static const uint8_t REG_STATUS_REG      = 0x1E;

static const uint8_t REG_OUT_TEMP_L      = 0x20; // 0x20-0x21
static const uint8_t REG_OUTX_L_G        = 0x22; // 0x22-0x27 (Gx,Gy,Gz)
static const uint8_t REG_OUTX_L_XL       = 0x28; // 0x28-0x2D (Ax,Ay,Az)

// -------- CTRL3_C bits --------
static const uint8_t CTRL3_SW_RESET = (1 << 0);
static const uint8_t CTRL3_BDU      = (1 << 6);
static const uint8_t CTRL3_IF_INC   = (1 << 2);

// -------- Helpers: I2C read/write --------
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  size_t got = Wire.requestFrom(addr, (uint8_t)n, (uint8_t)true);
  if (got != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

uint8_t i2cRead8(uint8_t addr, uint8_t reg) {
  uint8_t v = 0xFF;
  i2cRead(addr, reg, &v, 1);
  return v;
}

int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// -------- Minimal init for LSM6DS* --------
bool lsm6_init_minimal() {
  // 1) Read WHO_AM_I (print only, do not hard-fail)
  uint8_t who = i2cRead8(LSM6_ADDR, REG_WHO_AM_I);
  Serial.print("WHO_AM_I (0x0F) = 0x");
  if (who < 16) Serial.print("0");
  Serial.println(who, HEX);

  // 2) Software reset
  if (!i2cWrite8(LSM6_ADDR, REG_CTRL3_C, CTRL3_SW_RESET)) {
    Serial.println("Write CTRL3_C SW_RESET failed");
    return false;
  }
  delay(50);

  // Wait reset bit clears (best-effort)
  for (int i = 0; i < 20; i++) {
    uint8_t c3 = i2cRead8(LSM6_ADDR, REG_CTRL3_C);
    if ((c3 & CTRL3_SW_RESET) == 0) break;
    delay(10);
  }

  // 3) Enable BDU + auto-increment (IF_INC)
  // BDU prevents reading mixed high/low bytes across updates; IF_INC enables burst reads.
  uint8_t c3 = CTRL3_BDU | CTRL3_IF_INC;
  if (!i2cWrite8(LSM6_ADDR, REG_CTRL3_C, c3)) {
    Serial.println("Write CTRL3_C (BDU|IF_INC) failed");
    return false;
  }
  delay(10);

  // 4) Configure accelerometer: ODR=104Hz, FS=±2g
  // CTRL1_XL: [ODR_XL(7:4)] [FS_XL(3:2)] [BW_XL(1:0)]
  // ODR 104Hz = 0b0100 << 4
  // FS 2g     = 0b00   << 2
  uint8_t ctrl1_xl = (0b0100 << 4) | (0b00 << 2) | (0b00);
  if (!i2cWrite8(LSM6_ADDR, REG_CTRL1_XL, ctrl1_xl)) {
    Serial.println("Write CTRL1_XL failed");
    return false;
  }

  // 5) Configure gyro: ODR=104Hz, FS=2000 dps
  // CTRL2_G: [ODR_G(7:4)] [FS_G(3:2)] [FS_125(1)] [0]
  // ODR 104Hz = 0b0100 << 4
  // FS 2000dps typically = 0b11 << 2
  uint8_t ctrl2_g = (0b0100 << 4) | (0b11 << 2);
  if (!i2cWrite8(LSM6_ADDR, REG_CTRL2_G, ctrl2_g)) {
    Serial.println("Write CTRL2_G failed");
    return false;
  }

  delay(50);

  // Quick sanity: read STATUS_REG
  uint8_t st = i2cRead8(LSM6_ADDR, REG_STATUS_REG);
  Serial.print("STATUS_REG (0x1E) = 0x");
  if (st < 16) Serial.print("0");
  Serial.println(st, HEX);

  return true;
}

// -------- Read raw data burst --------
bool lsm6_read_raw(int16_t &ax, int16_t &ay, int16_t &az,
                   int16_t &gx, int16_t &gy, int16_t &gz,
                   int16_t &t) {
  uint8_t buf[14]; // temp(2) + gyro(6) + accel(6)
  if (!i2cRead(LSM6_ADDR, REG_OUT_TEMP_L, buf, sizeof(buf))) return false;

  t  = le16(&buf[0]);
  gx = le16(&buf[2]);
  gy = le16(&buf[4]);
  gz = le16(&buf[6]);
  ax = le16(&buf[8]);
  ay = le16(&buf[10]);
  az = le16(&buf[12]);
  return true;
}

// -------- Convert to engineering units (typical LSM6DSM sensitivities) --------
// Accel ±2g: 0.061 mg/LSB  (typical for LSM6DSM family)
// Gyro 2000 dps: 70 mdps/LSB
// Temp: datasheet uses: Temp(degC) = 25 + (t/16)  (common in LSM6DS family)
float accel_ms2_from_raw(int16_t a_raw) {
  const float mg_per_lsb = 0.061f;
  const float g = 9.80665f;
  return (a_raw * mg_per_lsb / 1000.0f) * g;
}

float gyro_dps_from_raw(int16_t g_raw) {
  const float mdps_per_lsb = 70.0f;
  return (g_raw * mdps_per_lsb) / 1000.0f;
}

float temp_c_from_raw(int16_t t_raw) {
  return 25.0f + (t_raw / 16.0f);
}

// ===================== Arduino setup/loop =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();                // ESP32: Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_HZ);
  delay(50);

  Serial.println("=== Minimal LSM6DSM I2C bring-up ===");
  Serial.print("I2C addr = 0x"); Serial.println(LSM6_ADDR, HEX);

  if (!lsm6_init_minimal()) {
    Serial.println("LSM6 init FAILED at I2C level (writes/reads).");
    while (1) delay(1000);
  }

  Serial.println("Init OK. Reading data...");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz, t;
  if (!lsm6_read_raw(ax, ay, az, gx, gy, gz, t)) {
    Serial.println("Read burst failed.");
    delay(200);
    return;
  }

  float ax_ms2 = accel_ms2_from_raw(ax);
  float ay_ms2 = accel_ms2_from_raw(ay);
  float az_ms2 = accel_ms2_from_raw(az);

  float gx_dps = gyro_dps_from_raw(gx);
  float gy_dps = gyro_dps_from_raw(gy);
  float gz_dps = gyro_dps_from_raw(gz);

  float tc = temp_c_from_raw(t);

  Serial.print("T[C]="); Serial.print(tc, 2);

  Serial.print(" | A[m/s^2]=");
  Serial.print(ax_ms2, 3); Serial.print(",");
  Serial.print(ay_ms2, 3); Serial.print(",");
  Serial.print(az_ms2, 3);

  Serial.print(" | G[dps]=");
  Serial.print(gx_dps, 2); Serial.print(",");
  Serial.print(gy_dps, 2); Serial.print(",");
  Serial.println(gz_dps, 2);

  delay(50);
}
