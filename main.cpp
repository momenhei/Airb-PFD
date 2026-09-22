#define SDL_MAIN_USE_CALLBACKS 1 //use the callbacks instead of main()

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <iostream>
#include <array>
#include <algorithm>
#include <memory>
#include <math.h>
#include <iomanip>
#include <ctime>
#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define VERSION "0.6"
#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080

// ---------------- IMU (MPU6050) ----------------
#define MPU6050_ADDRESS          0x68 // AD0 auf GND, bei AD0 auf VCC: 0x69
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define ACCEL_SCALE 16384.0f // LSB/g bei +-2g
#define GYRO_SCALE  65.5f    // LSB/(deg/s) bei +-500 deg/s (250 deg/s wird bei schnellen Bewegungen schnell erreicht)
#define RAD_TO_DEG  (180.0f / 3.14159265f)

#define GYRO_WEIGHT     0.98f  // Komplementaerfilter: Anteil des Gyros
#define GRAVITY         9.81f  // m/s^2
#define ACCEL_DEADBAND  0.15f  // m/s^2, darunter wird Rauschen ignoriert
#define VELOCITY_DECAY  0.995f // zieht die Geschwindigkeit langsam gegen 0 (gegen Drift)
#define CALIBRATION_SAMPLES 200
#define IMU_INTERVAL_MS 20     // 50Hz, damit der Horizont fluessig laeuft

#define MOUNT_ROTATION 3 //um 180 Grad gedreht um Achse: (0=keine, 1=X, 2=Y, 3=Z)


#define AUTO_LEVEL 1 // Auf 0 setzen, wenn es beim Start nicht waagerecht steht

#define GYRO_SIGN_X 1.0f // Drehrichtung des Gyros relativ zum Beschleunigungssensor. Pruefen mit Taste I: waehrend der Bewegung muss Gyro X dasselbe Vorzeichen haben. Normalerweise +1: Gyro und Beschleunigungssensor sitzen auf demselben Chip,
#define GYRO_SIGN_Y 1.0f

#define ACCEL_SIGN_X -1.0f // Vorzeichen der Beschleunigungs-X-Achse (vorne/hinten). Falscher Wert = Pitch bewegt sich erst richtig (Gyro) und kriecht dann langsam in die falsche Richtung

// Offsets
#define ROLL_OFFSET  0.0f
#define PITCH_OFFSET 0.0f

// ---------------- IMU -> Kuenstlicher Horizont ----------------
#define HORIZON_ROLL_SIGN     1.0f   // Rechtskurve -> Horizont dreht gegen den Uhrzeigersinn
#define HORIZON_PITCH_SIGN   -1.0f   // Nase hoch -> Boden wandert nach unten
#define HORIZON_DEG_PER_UNIT 60.0f   // Grad Pitch fuer eine Verschiebung um aHSize (passend zur Pitch-Leiter: 10 Grad = aHSize/6)
#define HORIZON_RADIUS_LIMIT 1.5f

// ---------------- Geschwindigkeitsquelle ----------------
#define GPS_TIMEOUT_MS 2000 // ohne gueltigen $GPRMC-Fix laenger als das -> IMU als Fallback
#define ALTITUDE_IN_FEET 0  // 0 = Meter, 1 = Fuss fuer das Hoehenband

// Rechnet Sensorachsen in Flugzeugachsen um (X vorne, Z oben)
void applyMount(float& x, float& y, float& z){
    switch (MOUNT_ROTATION){
        case 1: y = -y; z = -z; break;
        case 2: x = -x; z = -z; break;
        case 3: x = -x; y = -y; break;
        default: break;
    }
}

// Neigung des Einbaus (wird in calibrateGyro() gemessen)
static float levelCosRoll = 1.0f, levelSinRoll = 0.0f;
static float levelCosPitch = 1.0f, levelSinPitch = 0.0f;

// Dreht einen Vektor aus der schraegen Einbaulage in die waagerechte
void applyLevel(float& x, float& y, float& z){
    float y1 =  y * levelCosRoll - z * levelSinRoll; // Einbau-Roll zurueckdrehen (um X)
    float z1 =  y * levelSinRoll + z * levelCosRoll;
    float x2 =  x * levelCosPitch + z1 * levelSinPitch; // Einbau-Pitch zurueckdrehen (um Y)
    float z2 = -x * levelSinPitch + z1 * levelCosPitch;
    x = x2; y = y1; z = z2;
}

// Winkel auf -180..180 Grad bringen
float wrapAngle(float angle){
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

struct ImuData{
    float accelX, accelY, accelZ; // g
    float gyroX, gyroY, gyroZ;    // deg/s
    float temperature;            // deg C
    float roll, pitch;            // deg (Komplementaerfilter)
    float accelRoll, accelPitch;  // deg, nur aus dem Beschleunigungssensor (Debug)
    float velocityForward;        // m/s, horizontal
    float velocityRight;          // m/s, horizontal
    float speed;                  // m/s, Betrag der Horizontalgeschwindigkeit
    bool valid;
};

enum class SpeedSource { None, GPS, IMU };

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static float fWidth  = 0.0f;
static float fHeight = 0.0f;
static float aHSize =  0.0f;
static float aHHeight = 0.0f;

//static constexpr int vertexCount = 36;
//static std::unique_ptr<SDL_Vertex[]> mask;
static std::vector<SDL_Vertex> mask;
static std::unique_ptr<SDL_Vertex[]> horizon;

static float horizonRotation=90.0f;
static float horizonRadius=0.0f;
static float horizonRotationTrim=0.0f; // manuelle Korrektur per A/D
static float horizonRadiusTrim=0.0f;   // manuelle Korrektur per W/S
static float speed=0.0;
static int altitude=0;
static float heading = 0.0f; // 0-360°, 0 = Nord
static SpeedSource speedSource = SpeedSource::None;

static int gpsFileDescriptor=-1;
static std::string gpsText = "Warte auf GPS";
static float gpsSpeed = 0.0f;       // km/h, nur gueltig wenn gpsSpeedTicks aktuell
static Uint64 gpsSpeedTicks = 0;    // SDL_GetTicks() des letzten gueltigen Fixes
static bool gpsSpeedReceived = false;
static float gpsAltitude = 0.0f;    // m ueber Meeresspiegel aus $GPGGA
static Uint64 gpsAltitudeTicks = 0; // SDL_GetTicks() der letzten gueltigen Hoehe
static bool gpsAltitudeReceived = false;

static SDL_Mutex *dataMutex = NULL;     // schuetzt GPS- und IMU-Daten (Timer-Thread <-> Render-Thread)
static SDL_TimerID gpsTimer = 0;
static SDL_TimerID imuTimer = 0;
static int imuFileDescriptor = -1;
static ImuData imuData{};
static bool showImuDebug = false;           // Taste I
static float mountTiltRoll = 0.0f, mountTiltPitch = 0.0f; // deg, fuer Debug-Anzeige
static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f; // deg/s
static float imuVelocityForward = 0.0f; // m/s
static float imuVelocityRight = 0.0f;   // m/s

struct GPSData
{
    int utcHour;
    int utcMinute;
    int utcSecond;

    int day;
    int month;
    int year;

    bool summerTime;

    std::string utcTime;	    // "HH:MM:SS"   (GPS in UTC)
    std::string localTime;	    // "HH:MM:SS"   (CET/CEST)
    std::string date;		    // "DD.MM.YYYY" (GPS)
    std::string localDate;	    // "DD.MM.YYYY" (Germany)
    std::string weekday;	    // Wochentag    
    std::string localWeekday;   // Wochentag    
};

static GPSData gps{};

int openSerialPort(const char* name){
    int fileDescriptor;
    fileDescriptor = open(name, O_RDONLY | O_NOCTTY | O_NDELAY); // rad only & not controling terminal & don't care about state of DCD signal line
    if (fileDescriptor<0){
        SDL_Log("Failed to Open Serial Port");
        return -1;
    }

    struct termios tty{};
    if (tcgetattr(fileDescriptor, &tty) != 0)
    {
        SDL_Log("tcgetattr failed");
        close(fileDescriptor);
        return -1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fileDescriptor, TCSANOW, &tty) != 0)
    {
        SDL_Log("tcsetattr failed");
        close(fileDescriptor);
        return -1;
    }

    return fileDescriptor;
}

int readSerialPort(int fileDescriptor, char* buffer, size_t size){
    return read(fileDescriptor, buffer, size);
}

void closeSerialPort(int fileDescriptor){
    close(fileDescriptor);
}

float knotsToKmH(float velocity){
    return velocity*1.852;
}

// ================= IMU =================
int writeRegister(int fileDescriptor, uint8_t reg, uint8_t value){
    uint8_t buffer[2] = {reg, value};
    if (write(fileDescriptor, buffer, 2) != 2){
        return -1;
    }
    return 0;
}

int readRegisters(int fileDescriptor, uint8_t reg, uint8_t* buffer, size_t size){
    if (write(fileDescriptor, &reg, 1) != 1){ // Registeradresse setzen
        return -1;
    }
    if (read(fileDescriptor, buffer, size) != (ssize_t)size){
        return -1;
    }
    return 0;
}

int initMpu6050(int fileDescriptor){
    writeRegister(fileDescriptor, MPU6050_REG_PWR_MGMT_1, 0x80); // Reset
    SDL_Delay(100);

    if (writeRegister(fileDescriptor, MPU6050_REG_PWR_MGMT_1, 0x01) != 0){ // aufwecken, Takt von Gyro X PLL
        SDL_Log("Failed to wake up MPU6050");
        return -1;
    }
    SDL_Delay(100);

    uint8_t powerManagement = 0;
    readRegisters(fileDescriptor, MPU6050_REG_PWR_MGMT_1, &powerManagement, 1);
    SDL_Log("PWR_MGMT_1: 0x%02X (0x01 = wach, 0x40 = Sleep)", powerManagement);

    writeRegister(fileDescriptor, MPU6050_REG_SMPLRT_DIV, 0x07);   // 1kHz / (1+7) = 125Hz
    writeRegister(fileDescriptor, MPU6050_REG_CONFIG, 0x03);       // Tiefpass ~44Hz
    writeRegister(fileDescriptor, MPU6050_REG_GYRO_CONFIG, 0x08);  // +-500 deg/s
    writeRegister(fileDescriptor, MPU6050_REG_ACCEL_CONFIG, 0x00); // +-2g

    return 0;
}

int openI2C(const char* name, int address){
    int fileDescriptor;
    fileDescriptor = open(name, O_RDWR);
    if (fileDescriptor<0){
        SDL_Log("Failed to Open I2C Bus %s", name);
        return -1;
    }

    if (ioctl(fileDescriptor, I2C_SLAVE, address) < 0){
        SDL_Log("Failed to set I2C Address 0x%02X", address);
        close(fileDescriptor);
        return -1;
    }

    uint8_t whoAmI = 0;
    if (readRegisters(fileDescriptor, MPU6050_REG_WHO_AM_I, &whoAmI, 1) != 0){
        SDL_Log("Failed to read WHO_AM_I");
        close(fileDescriptor);
        return -1;
    }
    if (whoAmI != 0x68){
        SDL_Log("Unexpected WHO_AM_I: 0x%02X (Clone?)", whoAmI); // nur Warnung
    }

    if (initMpu6050(fileDescriptor) != 0){
        close(fileDescriptor);
        return -1;
    }

    return fileDescriptor;
}

void closeI2C(int fileDescriptor){
    close(fileDescriptor);
}

void calibrateGyro(int fileDescriptor){
    uint8_t buffer[14];
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    float sumAX = 0.0f, sumAY = 0.0f, sumAZ = 0.0f;
    int samples = 0;

    SDL_Log("Kalibriere Gyro, Sensor nicht bewegen");
    for (int i = 0; i < CALIBRATION_SAMPLES; i++){
        if (readRegisters(fileDescriptor, MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer)) == 0){
            sumX += (int16_t)((buffer[8]  << 8) | buffer[9])  / GYRO_SCALE;
            sumY += (int16_t)((buffer[10] << 8) | buffer[11]) / GYRO_SCALE;
            sumZ += (int16_t)((buffer[12] << 8) | buffer[13]) / GYRO_SCALE;
            sumAX += (int16_t)((buffer[0] << 8) | buffer[1]) / ACCEL_SCALE;
            sumAY += (int16_t)((buffer[2] << 8) | buffer[3]) / ACCEL_SCALE;
            sumAZ += (int16_t)((buffer[4] << 8) | buffer[5]) / ACCEL_SCALE;
            samples++;
        }
        SDL_Delay(5);
    }

    if (samples > 0){
        gyroBiasX = sumX / samples;
        gyroBiasY = sumY / samples;
        gyroBiasZ = sumZ / samples;
    }
    SDL_Log("Gyro Bias: X:%.2f Y:%.2f Z:%.2f deg/s", gyroBiasX, gyroBiasY, gyroBiasZ);

    if (samples > 0){ // Einbaulage kontrollieren
        float ax = sumAX / samples, ay = sumAY / samples, az = sumAZ / samples;
        SDL_Log("Accel in Ruhe roh:        X:%.2f Y:%.2f Z:%.2f g", ax, ay, az);
        applyMount(ax, ay, az);
        ax *= ACCEL_SIGN_X;
        SDL_Log("Accel in Ruhe umgerechnet: X:%.2f Y:%.2f Z:%.2f g (Z sollte ~ +1 sein)", ax, ay, az);
        if (az < 0.0f){
            SDL_Log("WARNUNG: Z negativ -> MOUNT_ROTATION passt nicht zur Einbaulage");
        } else if (AUTO_LEVEL){
            float n = sqrtf(ay*ay + az*az);
            float m = sqrtf(ax*ax + n*n);
            if (n > 0.5f && m > 0.5f){ // nur bei plausiblen Werten (~1g)
                levelCosRoll  = az / n;  levelSinRoll  = ay / n;
                levelCosPitch = n / m;   levelSinPitch = -ax / m;
                mountTiltRoll  = atan2f(ay, az) * RAD_TO_DEG;
                mountTiltPitch = atan2f(-ax, n) * RAD_TO_DEG;
                SDL_Log("Einbauneigung: Roll %.1f deg, Pitch %.1f deg -> wird herausgerechnet",
                        atan2f(ay, az) * RAD_TO_DEG, atan2f(-ax, n) * RAD_TO_DEG);
            }
        }
    }
}

Uint32 updateImu(void* userdata, SDL_TimerID timerID, Uint32 interval){
    uint8_t buffer[14]; // Accel XYZ, Temp, Gyro XYZ je 2 Byte
    if (imuFileDescriptor < 0){
        return interval;
    }

    static int errorCount = 0;
    bool readOk = readRegisters(imuFileDescriptor, MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer)) == 0;

    bool allZero = true;
    for (size_t i = 0; readOk && i < sizeof(buffer); i++){
        if (buffer[i] != 0){
            allZero = false;
            break;
        }
    }

    if (!readOk || allZero){ // Lesefehler oder Sensor schlaeft (z.B. nach Spannungseinbruch)
        SDL_LockMutex(dataMutex);
        imuData.valid = false;
        SDL_UnlockMutex(dataMutex);

        errorCount++;
        if (errorCount * interval >= 500){ // ca. 0,5s ohne gueltige Daten -> komplett neu initialisieren
            SDL_Log("MPU6050 %s, initialisiere neu", readOk ? "liefert nur Nullen" : "antwortet nicht");
            initMpu6050(imuFileDescriptor);
            errorCount = 0;
        }
        return interval;
    }
    errorCount = 0;

    int16_t raw[7];
    for (int i = 0; i < 7; i++){
        raw[i] = (int16_t)((buffer[2*i] << 8) | buffer[2*i+1]);
    }

    ImuData data{};
    data.accelX = raw[0] / ACCEL_SCALE;
    data.accelY = raw[1] / ACCEL_SCALE;
    data.accelZ = raw[2] / ACCEL_SCALE;
    data.temperature = raw[3] / 340.0f + 36.53f;
    data.gyroX = raw[4] / GYRO_SCALE - gyroBiasX;
    data.gyroY = raw[5] / GYRO_SCALE - gyroBiasY;
    data.gyroZ = raw[6] / GYRO_SCALE - gyroBiasZ;

    applyMount(data.accelX, data.accelY, data.accelZ);
    applyMount(data.gyroX,  data.gyroY,  data.gyroZ);
    data.accelX *= ACCEL_SIGN_X;
    data.gyroX *= GYRO_SIGN_X;
    data.gyroY *= GYRO_SIGN_Y;
    applyLevel(data.accelX, data.accelY, data.accelZ);
    applyLevel(data.gyroX,  data.gyroY,  data.gyroZ);

    float dt = interval / 1000.0f;

    // Lage: Gyro integrieren, langsam durch die Erdbeschleunigung korrigieren
    static float filteredRoll = 0.0f;
    static float filteredPitch = 0.0f;
    static bool filterInitialised = false;
    float accelRoll  = atan2f(data.accelY, data.accelZ) * RAD_TO_DEG - ROLL_OFFSET;
    float accelPitch = atan2f(-data.accelX, sqrtf(data.accelY*data.accelY + data.accelZ*data.accelZ)) * RAD_TO_DEG - PITCH_OFFSET;
    if (!filterInitialised){
        filteredRoll = accelRoll;
        filteredPitch = accelPitch;
        filterInitialised = true;
    }
    // Komplementaerfilter mit Winkel-Wrap, sonst springt der Filter bei +-180 Grad
    filteredRoll  = wrapAngle(filteredRoll  + data.gyroX * dt);
    filteredPitch = wrapAngle(filteredPitch + data.gyroY * dt);
    filteredRoll  = wrapAngle(filteredRoll  + (1.0f - GYRO_WEIGHT) * wrapAngle(accelRoll  - filteredRoll));
    filteredPitch = wrapAngle(filteredPitch + (1.0f - GYRO_WEIGHT) * wrapAngle(accelPitch - filteredPitch));
    data.roll = filteredRoll;
    data.pitch = filteredPitch;
    data.accelRoll = accelRoll;
    data.accelPitch = accelPitch;

    // Beschleunigung in die Horizontalebene drehen (Erdbeschleunigung faellt dabei raus)
    float rollRad = filteredRoll / RAD_TO_DEG;
    float pitchRad = filteredPitch / RAD_TO_DEG;
    float sinRoll = sinf(rollRad), cosRoll = cosf(rollRad);
    float sinPitch = sinf(pitchRad), cosPitch = cosf(pitchRad);

    float accelForward = (data.accelX * cosPitch + data.accelY * sinPitch * sinRoll + data.accelZ * sinPitch * cosRoll) * GRAVITY;
    float accelRight   = (data.accelY * cosRoll - data.accelZ * sinRoll) * GRAVITY;

    if (fabsf(accelForward) < ACCEL_DEADBAND) accelForward = 0.0f; // Rauschen unterdruecken
    if (fabsf(accelRight) < ACCEL_DEADBAND)   accelRight = 0.0f;

    SDL_LockMutex(dataMutex);
    // Geschwindigkeit wird bei gueltigem GPS in filterData() auf den GPS-Wert gesetzt, ab dem GPS-Ausfall integriert die IMU von dort aus weiter
    imuVelocityForward = imuVelocityForward * VELOCITY_DECAY + accelForward * dt;
    imuVelocityRight   = imuVelocityRight   * VELOCITY_DECAY + accelRight * dt;

    data.velocityForward = imuVelocityForward;
    data.velocityRight = imuVelocityRight;
    data.speed = sqrtf(imuVelocityForward*imuVelocityForward + imuVelocityRight*imuVelocityRight);
    data.valid = true;

    imuData = data;
    SDL_UnlockMutex(dataMutex);

    return interval;
}

//Calender-logic
int dayOfWeek(int y, int m, int d);

std::string weekdayName(int wd) {
    static const std::array<std::string, 7> names = {
        "Sonntag",
		"Montag",
		"Dienstag",
		"Mittwoch",
        "Donnerstag",
		"Freitag",
		"Samstag"
    };
    return names[wd];
}

bool isLeapYear(int y) {
	return ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
}

int daysInMonthOf(int y, int m) {
    static const int daysPerMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
	if (m == 2 && isLeapYear(y))
		return 29;
    return daysPerMonth[m - 1];
}

int lastSundayOfMonth(int y, int m) {
    int lastDay = daysInMonthOf(y, m);
    int wd = dayOfWeek(y, m, lastDay); // 0 = Sonntag
    return lastDay - wd;
}

// EU-Regel: Umstellung jeweils um 01:00 UTC.
// Letzter Sonntag im März -> Sommerzeit (CEST, UTC+2)
// Letzter Sonntag im Oktober -> Winterzeit (CET, UTC+1)
bool isSummerTime(int y, int m, int d, int hourUTC) {
    if (m < 3 || m > 10) return false;
    if (m > 3 && m < 10) return true;

    if (m == 3) {
        int marchLastSunday = lastSundayOfMonth(y, 3);
        if (d > marchLastSunday) return true;
        if (d < marchLastSunday) return false;
        return hourUTC >= 1;
    }
    // m == 10
    int octLastSunday = lastSundayOfMonth(y, 10);
    if (d < octLastSunday) return true;
    if (d > octLastSunday) return false;
    return hourUTC < 1;
}

// Sakamoto-Algorithm: 0 = Sonntag, 1= Montag, usw.
int dayOfWeek(int y, int m, int d) {
    static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
	if (m < 3){
		y -= 1;
	}
    return (y + y/4 - y/100 + y/400 + t[m - 1] + d) % 7;
}

void parseTime(const std::string& timeField)
{
	if (timeField.length() < 6)
		return;
	
		gps.utcHour 	= std::stoi(timeField.substr(0,2));
		gps.utcMinute 	= std::stoi(timeField.substr(2,2));
		gps.utcSecond	= std::stoi(timeField.substr(4,2));
		
		gps.utcTime 	= timeField.substr(0,2) + ":" +
					  timeField.substr(2,2) + ":" +
					  timeField.substr(4,2);
}

void parseDate(const std::string& dateField)
{
	if (dateField.length() < 6)
		return;

		gps.day		= std::stoi(dateField.substr(0,2));
		gps.month	= std::stoi(dateField.substr(2,2));
		gps.year	= 2000 + std::stoi(dateField.substr(4,2));
		
		gps.date    = dateField.substr(0,2) + "." +
				  dateField.substr(2,2) + ".20" +
				  dateField.substr(4,2);	
}

void updateWeekdayAndLocalTime() {
     // Prüfen, ob bereits gültige GPS-Daten vorhanden sind
    if (gps.day == 0 || gps.month < 1 || gps.month > 12 || gps.year == 0 ||
        gps.utcHour < 0 || gps.utcMinute < 0 || gps.utcSecond < 0)
    {
        return;
    }
	
    gps.summerTime = isSummerTime(gps.year, gps.month, gps.day, gps.utcHour);
    // CEST = UTC+2, CET = UTC+1
	int offset = gps.summerTime ? 2 : 1;

    int localHour  = gps.utcHour + offset;
    int localDay   = gps.day;
    int localMonth = gps.month;
    int localYear  = gps.year;

	int wd = dayOfWeek(gps.year, gps.month, gps.day);
    gps.weekday = weekdayName(wd);
    
	// Tagesüberlauf behandeln (z.B. 23:xx UTC + 1h/2h -> nächster Tag)
    if (localHour >= 24) {
        localHour -= 24;
        localDay += 1;
        if (localDay > daysInMonthOf(localYear, localMonth)) {
            localDay = 1;
            localMonth += 1;
            if (localMonth > 12) {
                localMonth = 1;
                localYear += 1;
            }
        }
    }

	// Lokales Datum speichern
    gps.localDate =
        (localDay < 10 ? "0" : "") + std::to_string(localDay) + "." +
        (localMonth < 10 ? "0" : "") + std::to_string(localMonth) + "." +
        std::to_string(localYear);

    // Wochentag anhand des lokalen Datums berechnen
    wd = dayOfWeek(localYear, localMonth, localDay);

    gps.localWeekday = weekdayName(wd);
	
    char buffer[9];

	std::snprintf(
		buffer,
		sizeof(buffer),
		"%02d:%02d:%02d",
		localHour,
		gps.utcMinute,
		gps.utcSecond
	);

	gps.localTime = buffer;
}

void filterData(){
    if (gpsText.compare(0,6,"$GPRMC")==0){
        std::string copy=gpsText;
        size_t pos;
        int i=0;
        while ((pos = copy.find(",")) != std::string::npos){
            std::string field = copy.substr(0,pos);
            copy.erase(0, pos+1);
            switch(i){
            case 1:  //UTC of position
                parseTime(field);
                break;
            case 2:  //Position status (A = data valid, V = data invalid)
                if(field != "A")
                {
                    return; // kein Fix -> gpsSpeedTicks veraltet -> IMU-Fallback
                }
                break;
            case 3:  //Latitude (DDmm.mm)
                
                break;
            case 4:  //Latitude direction: (N = North, S = South)
                
                break;
            case 5:  //Longitude (DDDmm.mm)
                
                break;
            case 6:  //Longitude direction: (E = East, W = West)
                
                break;
            case 7:  //Speed over ground, knots
                try{
                    gpsSpeed = knotsToKmH(std::stof(field));
                    gpsSpeedTicks = SDL_GetTicks();
                    gpsSpeedReceived = true;
                    // IMU-Integration auf GPS-Geschwindigkeit stuetzen, damit der Fallback nicht bei 0 anfaengt
                    imuVelocityForward = gpsSpeed / 3.6f;
                    imuVelocityRight = 0.0f;
                }catch (const std::exception& e) {
                    // ungueltiges Feld -> Zeitstempel nicht erneuern
                }
                break;
            case 8:  //Track made good, degrees True
                
                break;
            case 9:  //Date: dd/mm/yy
                parseDate(field);
				break;
            case 10: //Magnetic variation, degrees
                
                break;
            case 11: //Magnetic variation direction E/W
                
                break;
            case 12: //Positioning system mode indicator, NMEA quality indicator
                
                break;
            }
            i++;
        }
        updateWeekdayAndLocalTime();

    }
}

// $GPGGA: 6 = Fix-Qualitaet (0 = kein Fix), 9 = Hoehe ueber Meeresspiegel, 10 = Einheit (M)
void parseGGA(const std::string& line){
    std::string fields[15];
    int count = 0;
    size_t start = 0;
    while (count < 15){
        size_t pos = line.find(',', start);
        fields[count++] = line.substr(start, pos == std::string::npos ? std::string::npos : pos - start);
        if (pos == std::string::npos) break;
        start = pos + 1;
    }
    if (count < 11) return;
    if (fields[6].empty() || fields[6] == "0") return; // kein Fix -> Zeitstempel veraltet
    if (fields[9].empty()) return;
    try{
        gpsAltitude = std::stof(fields[9]);
        gpsAltitudeTicks = SDL_GetTicks();
        gpsAltitudeReceived = true;
    }catch (const std::exception& e) {
        // ungueltiges Feld -> ignorieren
    }
}

Uint32 updateData(void* userdata, SDL_TimerID timerID, Uint32 interval){
    char buffer[256];
    static std::string serialBuffer;
    if (gpsFileDescriptor < 0){
        return interval;
    }
    int n = read(gpsFileDescriptor, buffer, sizeof(buffer));
    if (n > 0){
	    serialBuffer.append(buffer, n);
	    size_t end;
	    while ((end = serialBuffer.find("\n")) != std::string::npos){ // jede vollstaendige Zeile auswerten
            std::string line = serialBuffer.substr(0, end);
	        serialBuffer.erase(0, end+1);

            size_t start = line.find("$GPRMC");
            if (start != std::string::npos){
                SDL_LockMutex(dataMutex);
                gpsText = line.substr(start);
                filterData();
                SDL_UnlockMutex(dataMutex);
                continue;
            }
            start = line.find("$GPGGA");
            if (start != std::string::npos){
                SDL_LockMutex(dataMutex);
                parseGGA(line.substr(start));
                SDL_UnlockMutex(dataMutex);
            }
	    }
        if (serialBuffer.size() > 4096){ // Muell ohne Zeilenende nicht endlos sammeln
            serialBuffer.clear();
        }
    }
    return interval;
}

// Holt IMU- und GPS-Werte (thread-sicher) und setzt Horizont + Geschwindigkeit
void updateFromSensors(){
    SDL_LockMutex(dataMutex);
    ImuData imu = imuData;
    bool gpsOk = gpsSpeedReceived && (SDL_GetTicks() - gpsSpeedTicks) < GPS_TIMEOUT_MS;
    float gpsKmH = gpsSpeed;
    bool gpsAltOk = gpsAltitudeReceived && (SDL_GetTicks() - gpsAltitudeTicks) < GPS_TIMEOUT_MS;
    float gpsAltM = gpsAltitude;
    SDL_UnlockMutex(dataMutex);

    // Kuenstlicher Horizont aus Roll/Pitch
    if (imu.valid){
        horizonRotation = 90.0f - HORIZON_ROLL_SIGN * imu.roll + horizonRotationTrim;
        float radius = HORIZON_PITCH_SIGN * imu.pitch / HORIZON_DEG_PER_UNIT + horizonRadiusTrim;
        horizonRadius = std::clamp(radius, -HORIZON_RADIUS_LIMIT, HORIZON_RADIUS_LIMIT);
    }

    // Geschwindigkeit: GPS bevorzugt, sonst IMU
    if (gpsOk){
        speed = gpsKmH;
        speedSource = SpeedSource::GPS;
    } else if (imu.valid){
        speed = imu.speed * 3.6f;
        speedSource = SpeedSource::IMU;
    } else {
        speedSource = SpeedSource::None; // letzter Wert bleibt stehen (Pfeiltasten zum Testen)
    }

    // Hoehe aus GPS; ohne Fix bleibt der letzte Wert stehen (Bild auf/ab zum Testen)
    if (gpsAltOk){
        altitude = static_cast<int>(std::lround(ALTITUDE_IN_FEET ? gpsAltM * 3.28084f : gpsAltM));
    }
}

float degreeToRad(float dgr){
    return dgr*3.141/180;
}

void calculateHorizonVertex(int index, float offsetPhi,float offsetR){
    offsetPhi=degreeToRad(offsetPhi-90);
    float hrzRot=degreeToRad(horizonRotation);
    float phi= hrzRot+atan2(offsetR*sin(offsetPhi-hrzRot),(aHSize*horizonRadius)+offsetR*cos(offsetPhi-hrzRot));
    float r= sqrt(pow((aHSize*horizonRadius),2)+pow(offsetR,2)+2*(aHSize*horizonRadius)*offsetR*cos(offsetPhi-hrzRot));
    horizon[index].position.x=r*cos(phi)+fWidth/2;
    horizon[index].position.y=r*sin(phi)+fHeight/2;
    horizon[index].color= {0.4f, 0.2f, 0.08f, 1.0f};
}

void updateHorizon(){
    calculateHorizonVertex(0,horizonRotation,fWidth);
    calculateHorizonVertex(1,180+horizonRotation,fWidth);
    calculateHorizonVertex(2,90+horizonRotation,fWidth);
}

SDL_FPoint horizonPoint(float localX, float localY){
    float pitchOffset = aHSize*horizonRadius;  // gleicher Wert wie in calculateHorizonVertex
    float y = localY + pitchOffset;
    float rot = degreeToRad(horizonRotation - 90.0f);
    float rx = localX*cos(rot) - y*sin(rot);
    float ry = localX*sin(rot) + y*cos(rot);
    return { rx + fWidth/2, ry + fHeight/2 };
}

void renderPitchLadder(){
    const float stepPx  = aHSize/6.0f; // Pixelabstand zwischen zwei Strichen
    const int   stepDeg = 10;          // Beschriftungsschritt
    const float gap     = aHSize*0.06f; // Lücke in der Mitte (Platz fürs Flugzeug-Symbol)

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    for (int i = -3; i <= 3; i++){
        if (i == 0) continue; // die Horizontlinie selbst ist bereits der "0°"-Strich

        // Striche werden zu den Rändern hin kürzer
        float halfWidth = aHSize/4.0f - std::abs(i)*aHSize/22.0f;
        if (halfWidth <= gap) continue; // zu kurz für sinnvolle Darstellung, überspringen

        // Linkes Segment: von -halfWidth bis -gap
        SDL_FPoint lp1 = horizonPoint(-halfWidth, i*stepPx);
        SDL_FPoint lp2 = horizonPoint(-gap,        i*stepPx);
        SDL_RenderLine(renderer, lp1.x, lp1.y, lp2.x, lp2.y);

        // Rechtes Segment: von gap bis halfWidth
        SDL_FPoint rp1 = horizonPoint(gap,        i*stepPx);
        SDL_FPoint rp2 = horizonPoint(halfWidth,  i*stepPx);
        SDL_RenderLine(renderer, rp1.x, rp1.y, rp2.x, rp2.y);

        // Zahl jeweils außen an beiden Enden, gleiche Skalierung wie die Baender
        std::string label = std::to_string(std::abs(i*stepDeg));
        float scaleX = fWidth  / 384.0f;
        float scaleY = fHeight / 216.0f;
        float charW  = 8.0f * scaleX;          // reale Pixelbreite eines Debug-Zeichens
        float charH  = 8.0f * scaleY;          // reale Pixelhoehe eines Debug-Zeichens
        float pad    = charW * 0.5f;           // Abstand zwischen Strichende und Zahl
        float textW  = label.length() * charW;

        SDL_SetRenderScale(renderer, scaleX, scaleY);

        // links: Zahl endet kurz vor dem Strich
        SDL_FPoint textPosL = horizonPoint(-halfWidth - pad, i*stepPx);
        SDL_RenderDebugText(renderer, (textPosL.x - textW)/scaleX, (textPosL.y - charH/2.0f)/scaleY, label.c_str());

        // rechts: Zahl beginnt kurz nach dem Strich
        SDL_FPoint textPosR = horizonPoint(halfWidth + pad, i*stepPx);
        SDL_RenderDebugText(renderer, textPosR.x/scaleX, (textPosR.y - charH/2.0f)/scaleY, label.c_str());

        SDL_SetRenderScale(renderer, 1, 1);
    }

     const float minorHalfWidth = aHSize/16.0f;
    for (int i = -3; i <= 2; i++){
        float minorPitch = (i + 0.5f) * stepPx;
        SDL_FPoint mp1 = horizonPoint(-minorHalfWidth, minorPitch);
        SDL_FPoint mp2 = horizonPoint( minorHalfWidth, minorPitch);
        SDL_RenderLine(renderer, mp1.x, mp1.y, mp2.x, mp2.y);
    }
}

void addArcFan(SDL_FPoint corner, SDL_FPoint circleCenter, float radius, float startDeg, float endDeg, int segments, SDL_FColor color){
    float startRad = degreeToRad(startDeg);
    float endRad   = degreeToRad(endDeg);
    for (int i = 0; i < segments; i++){
        float a0 = startRad + (endRad-startRad) * i     / segments;
        float a1 = startRad + (endRad-startRad) * (i+1) / segments;
        SDL_FPoint p0 = { circleCenter.x + radius*cos(a0), circleCenter.y + radius*sin(a0) };
        SDL_FPoint p1 = { circleCenter.x + radius*cos(a1), circleCenter.y + radius*sin(a1) };
        mask.push_back({{corner.x, corner.y}, color});
        mask.push_back({{p0.x, p0.y}, color});
        mask.push_back({{p1.x, p1.y}, color});
    }
}

void updateMask(){
    int width, height;
    SDL_GetWindowSize(window, &width, &height);
    fWidth  = static_cast<float>(width);
    fHeight = static_cast<float>(height);
    aHSize  = 0.5f*std::min(fHeight,fWidth);
    aHHeight = aHSize * 1.25f;
    const float hSpacer = 0.5f*(fWidth-aHSize);
    const float vSpacer = 0.5f*(fHeight-aHHeight);
    const float rTop    = aHSize/2.0f;
    const float rBottom = aHSize/2.0f;

    mask.clear();
    SDL_FColor black = {0.0f, 0.0f, 0.0f, 1.0f};
    auto v   = [&](float x,float y){ mask.push_back({{x,y}, black}); };
    auto tri = [&](float x1,float y1,float x2,float y2,float x3,float y3){ v(x1,y1); v(x2,y2); v(x3,y3); };
    auto rec = [&](float x1,float y1,float x2,float y2){ tri(x1,y1,x1,y2,x2,y2); tri(x1,y1,x2,y2,x2,y1); };

    // left & right boundaries
    rec(0.0f, 0.0f, hSpacer, fHeight);
    rec(fWidth, fHeight, fWidth-hSpacer, 0.0f);
    // top & bottom boundaries
    rec(0.0f, 0.0f, fWidth, vSpacer);
    rec(fWidth, fHeight, 0.0f, fHeight-vSpacer);

    // oben: echter Halbkreis (beide Bögen teilen sich denselben Mittelpunkt)
    addArcFan({hSpacer, vSpacer},               {hSpacer+rTop, vSpacer+rTop},               rTop, 180, 270, 12, black);
    addArcFan({fWidth-hSpacer, vSpacer},        {fWidth-hSpacer-rTop, vSpacer+rTop},        rTop, 270, 360, 12, black);

    // unten: nur leicht abgerundet, Seiten bleiben ansonsten gerade
    addArcFan({hSpacer, fHeight-vSpacer},               {hSpacer+rBottom, fHeight-vSpacer-rBottom},        rBottom,  90, 180, 8, black);
    addArcFan({fWidth-hSpacer, fHeight-vSpacer},        {fWidth-hSpacer-rBottom, fHeight-vSpacer-rBottom}, rBottom,   0,  90, 8, black);
}


void renderDeviders(){
    float width=fWidth/5;
    float height=fHeight/10;
    for (int i=1;i<6;i++){
        SDL_RenderLine(renderer,i*width,0,i*width,height);
    }
}

void renderText(){
    SDL_LockMutex(dataMutex); // Strings werden im Timer-Thread geschrieben
    std::string localTime    = gps.localTime;
    std::string localWeekday = gps.localWeekday;
    std::string localDate    = gps.localDate;
    SDL_UnlockMutex(dataMutex);

    SDL_SetRenderScale(renderer, fWidth/384, fHeight/216);
    SDL_SetRenderDrawColor(renderer, 44, 255, 5, 255);

    std::string speedStr = std::to_string(static_cast<int>(std::round(speed)));
    SDL_RenderDebugText(renderer, 38.4 -speedStr.length()*3.5, 4, speedStr.c_str());
    SDL_RenderDebugText(renderer, 38.4-strlen("km/h")*3.5,    14, "km/h");

    SDL_RenderDebugText(renderer, 115.2-strlen("G/S")*3.5,     4, "G/S");
    // Quelle der Geschwindigkeit anzeigen, IMU-Fallback in Gelb
    const char* sourceStr = "---";
    if (speedSource == SpeedSource::GPS){
        sourceStr = "GPS";
    } else if (speedSource == SpeedSource::IMU){
        sourceStr = "IMU";
        SDL_SetRenderDrawColor(renderer, 255, 200, 0, 255);
    }
    SDL_RenderDebugText(renderer, 115.2-strlen(sourceStr)*3.5, 14, sourceStr);
    SDL_SetRenderDrawColor(renderer, 44, 255, 5, 255);
    
  //SDL_RenderDebugText(renderer, 192-strlen("LOC")*3.5,       4, "LOC");
  //SDL_RenderDebugText(renderer, 192-strlen(t.date.c_str())*3.5,       4, t.date.c_str());
    if (localTime.length() > 5) localTime.erase(5);                    // "HH:MM:SS" -> "HH:MM"
    SDL_RenderDebugText(renderer, 192-localTime.length()*3.5,       4, localTime.c_str());
  
  //SDL_RenderDebugText(renderer, 268.8-strlen("CAT2")*3.5,    4, "CAT2");  
  //std::string timeStr = getTimeString();
  //SDL_RenderDebugText(renderer, 268.8 - timeStr.length()*3.5, 4, timeStr.c_str());   
    SDL_RenderDebugText(renderer, 268.8 - localWeekday.length()*3.5, 4, localWeekday.c_str());

  //SDL_RenderDebugText(renderer, 345.6-strlen("AP1")*3.5,     4, "AP1");
  //SDL_RenderDebugText(renderer, 345.6-strlen("FD1")*3.5,    14, "FD1");
    if (localDate.length() == 10) localDate.erase(6, 2);               // "DD.MM.YYYY" -> "DD.MM.YY"
    SDL_RenderDebugText(renderer, 345.6-localDate.length()*3.5,     4, localDate.c_str());
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_SetRenderScale(renderer, 1, 1);
}

void renderVerticalTape(const SDL_FRect& rect, float value, float tickStep, bool ticksOnRight, float minValue = -1e9f, bool invertDirection = false){
    float spacing = rect.h / 6.0f;
    float center  = rect.y + rect.h / 2.0f;
    float offset  = fmodf(value, tickStep) * spacing / tickStep;
    int dir = invertDirection ? -1 : 1;

    float tickLen    = rect.w * 0.4f;
    float tickStartX = ticksOnRight ? rect.x + rect.w : rect.x;
    float tickEndX   = ticksOnRight ? tickStartX - tickLen : tickStartX + tickLen;

    //Skalierungsfaktor
    float scaleX = fWidth  / 384.0f;
    float scaleY = fHeight / 216.0f;
    float charW  = 8.0f * scaleX; // reale Pixelbreite eines Debug-Zeichens bei dieser Skalierung

    for (int i = -3; i <= 3; i++) {
        float y = center + dir * i * spacing + offset;
        if (y < rect.y || y > rect.y + rect.h) continue;

        float tickValue = value - fmodf(value, tickStep) + i * tickStep;
        if (tickValue < minValue) continue;
        
        SDL_RenderLine(renderer, tickStartX, y, tickEndX, y);

        std::string label = std::to_string(static_cast<int>(std::round(tickValue)));
        float textX = ticksOnRight ? tickEndX - label.length()*charW - 4 : tickEndX + 4;

        SDL_SetRenderScale(renderer, scaleX, scaleY);
        SDL_RenderDebugText(renderer, textX/scaleX, (y-6)/scaleY, label.c_str());
        SDL_SetRenderScale(renderer, 1, 1);
    }

    // Live-Wert
    std::string liveLabel = std::to_string(static_cast<int>(std::round(value)));
    float liveX = ticksOnRight ? tickStartX - liveLabel.length()*charW - 20 : tickStartX + 20;
    //SDL_SetRenderScale(renderer, scaleX, scaleY);
    //SDL_RenderDebugText(renderer, liveX/scaleX, (center-6)/scaleY, liveLabel.c_str());
    //SDL_SetRenderScale(renderer, 1, 1);

    // Marker-Dreieck
    float apexX = ticksOnRight ? rect.x + rect.w : rect.x;
    float baseX = ticksOnRight ? apexX + rect.w*0.3f : apexX - rect.w*0.3f;
    SDL_Vertex tri[] = {
        {{apexX, center}, {255,255,255,255}},
        {{baseX, center + rect.h*0.02f}, {255,255,255,255}},
        {{baseX, center - rect.h*0.02f}, {255,255,255,255}}
    };
    SDL_RenderGeometry(renderer, NULL, tri, 3, NULL, 0);
}

//Himmelsrichtungen
std::string headingLabel(int deg){
    deg = ((deg % 360) + 360) % 360; // auf 0-359 normalisieren
    switch(deg){
        case 0:   return "N";
        case 45:  return "NO";
        case 90:  return "O";
        case 135: return "SO";
        case 180: return "S";
        case 225: return "SW";
        case 270: return "W";
        case 315: return "NW";
        default:  return std::to_string(deg);
    }
}

void renderHorizontalTape(const SDL_FRect& rect, float value, float tickStep){
    float spacing = rect.w / 6.0f;
    float center  = rect.x + rect.w / 2.0f;
    float offset  = fmodf(value, tickStep) * spacing / tickStep;

    float tickLen  = rect.h * 0.4f;
    float tickTopY = rect.y;
    float tickBotY = tickTopY + tickLen;

    float scaleX = fWidth  / 384.0f;
    float scaleY = fHeight / 216.0f;
    float charW  = 8.0f * scaleX;

    for (int i = -3; i <= 3; i++) {
        float x = center + i * spacing - offset;
        if (x < rect.x || x > rect.x + rect.w) continue;

         // Strich
        SDL_RenderLine(renderer, x, tickTopY, x, tickBotY);

        float tickValue = value - fmodf(value, tickStep) + i * tickStep;
        int wrapped = (static_cast<int>(std::round(tickValue)) % 360 + 360) % 360;

        // Zahl bei Vielfachen von 45°
        if (wrapped % 45 == 0) {
            std::string label = std::to_string(wrapped);
            SDL_SetRenderScale(renderer, scaleX, scaleY);
            SDL_RenderDebugText(renderer, (x - label.length()*charW/2.0f)/scaleX, (tickBotY+4)/scaleY, label.c_str());
            SDL_SetRenderScale(renderer, 1, 1);
        }
    }

    // Live-Gradzahl
    int liveHeading = (static_cast<int>(std::round(value)) % 360 + 360) % 360;
    std::string liveLabel = std::to_string(liveHeading);
    float liveY = rect.y - rect.h*0.15f - 20;
    //SDL_SetRenderScale(renderer, scaleX, scaleY);
    //SDL_RenderDebugText(renderer, (center - liveLabel.length()*charW/2.0f)/scaleX, liveY/scaleY, liveLabel.c_str());
    //SDL_SetRenderScale(renderer, 1, 1);

    // Marker-Dreieck
    SDL_Vertex tri[] = {
        {{center, rect.y}, {255,255,255,255}},
        {{center - rect.w*0.01f, rect.y - rect.h*0.15f}, {255,255,255,255}},
        {{center + rect.w*0.01f, rect.y - rect.h*0.15f}, {255,255,255,255}}
    };
    SDL_RenderGeometry(renderer, NULL, tri, 3, NULL, 0);
}

void renderIndicators(){
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    float barHeight= fHeight/1.65;
    SDL_FRect re1 = {fWidth/8,             fHeight/2-barHeight/2,   fWidth/12,  barHeight};
    SDL_FRect re2 = {19*fWidth/24, fHeight/2-barHeight/2,   fWidth/12,  barHeight};
    SDL_FRect re3 = {fWidth/2-aHSize/2, fHeight/12*10,   aHSize,  fHeight/10};
    SDL_RenderFillRect( renderer, &re1);
    SDL_RenderFillRect( renderer, &re2);
    SDL_RenderFillRect( renderer, &re3);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    renderVerticalTape(re1, speed, 10.0f, true, 0.0f, true);
    renderVerticalTape(re2, (float)altitude, 100.0f, false, -1e9f, true);
    renderHorizontalTape(re3, heading, 15.0f);
}

// Debug-Anzeige der IMU (Taste I)
void renderImuDebug(){
    SDL_LockMutex(dataMutex);
    ImuData d = imuData;
    SDL_UnlockMutex(dataMutex);

    const float scale = 2.0f;
    SDL_FRect bg = {0.0f, fHeight/10 + 5, 560.0f, 125.0f};
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderFillRect(renderer, &bg);

    SDL_SetRenderScale(renderer, scale, scale);
    SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);
    float x = 5.0f, y = (fHeight/10 + 10) / scale;
    if (!d.valid){
        SDL_RenderDebugText(renderer, x, y, "IMU: keine Daten");
    } else {
        SDL_RenderDebugTextFormat(renderer, x, y,      "Accel [g]  X:%6.2f Y:%6.2f Z:%6.2f", d.accelX, d.accelY, d.accelZ);
        SDL_RenderDebugTextFormat(renderer, x, y+10,   "Gyro [d/s] X:%6.1f Y:%6.1f Z:%6.1f", d.gyroX, d.gyroY, d.gyroZ);
        SDL_RenderDebugTextFormat(renderer, x, y+20,   "Accel Roll:%6.1f  Pitch:%6.1f", d.accelRoll, d.accelPitch);
        SDL_RenderDebugTextFormat(renderer, x, y+30,   "Filter Roll:%6.1f Pitch:%6.1f", d.roll, d.pitch);
        SDL_RenderDebugTextFormat(renderer, x, y+40,   "Einbau Roll:%6.1f Pitch:%6.1f", mountTiltRoll, mountTiltPitch);
    }
    SDL_SetRenderScale(renderer, 1, 1);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[])
{
    SDL_SetAppMetadata("Airb-PFD", VERSION,"");
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("Failed to initialise SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("PFD", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_FULLSCREEN, &window, &renderer)) {
        SDL_Log("Failed to Create Rendererer or Window: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(renderer, 1);

    updateMask();
    horizon = std::make_unique<SDL_Vertex[]>(3);
    updateHorizon();
    SDL_HideCursor();

    dataMutex = SDL_CreateMutex();
    
    gpsFileDescriptor = openSerialPort("/dev/serial0");
    gpsTimer = SDL_AddTimer(100, updateData, nullptr);

    imuFileDescriptor = openI2C("/dev/i2c-1", MPU6050_ADDRESS);
    if (imuFileDescriptor >= 0){
        calibrateGyro(imuFileDescriptor); // Sensor dabei ruhig liegen lassen
    }
    imuTimer = SDL_AddTimer(IMU_INTERVAL_MS, updateImu, nullptr);

    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate){
    updateFromSensors();

    SDL_SetRenderDrawColor(renderer, 3, 169, 244, 255);
    SDL_RenderClear(renderer);
    updateHorizon();
    SDL_RenderGeometry(renderer, NULL, horizon.get(), 3, NULL, 0);

    renderPitchLadder();

    SDL_RenderGeometry(renderer, NULL, mask.data(), (int)mask.size(), NULL, 0);
    
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    renderDeviders();
    renderText();
    renderIndicators();
    if (showImuDebug){
        renderImuDebug();
    }
    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event){
    switch(event->type){
        case SDL_EVENT_QUIT:
            return SDL_APP_SUCCESS;
            break;
        case SDL_EVENT_WINDOW_RESIZED:
            updateMask();
            break;
        case SDL_EVENT_KEY_DOWN:
            switch(event->key.key){
                case SDLK_W:            // Trimmung, wirkt zusaetzlich zur IMU
                    horizonRadiusTrim -= 0.01;
                    horizonRadius -= 0.01;
                    break;
                case SDLK_S:
                    horizonRadiusTrim += 0.01;
                    horizonRadius += 0.01;
                    break;
                case SDLK_A:
                    horizonRotationTrim -= 1;
                    horizonRotation -= 1;
                    break;
                case SDLK_D:
                    horizonRotationTrim += 1;
                    horizonRotation += 1;
                    break;
                case SDLK_I:            // IMU-Debug ein/aus
                    showImuDebug = !showImuDebug;
                    break;
                case SDLK_UP:           // wirkt nur ohne GPS und IMU
                    speed += 1.0;
                    break;
                case SDLK_DOWN:
                    speed -= 1.0;
                    break;
                case SDLK_PAGEUP:
                    altitude += 10;
                    break;
                case SDLK_PAGEDOWN:
                    altitude -= 10;
                    break;
                case SDLK_LEFT:
                    heading -= 1.0f;
                    if (heading < 0.0f) heading += 360.0f;
                    break;
                case SDLK_RIGHT:
                    heading += 1.0f;
                    if (heading >= 360.0f) heading -= 360.0f;
                    break;
            }
            break;
    }
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result)
{
    // Timer stoppen bevor die Schnittstellen geschlossen werden
    if (gpsTimer)
        SDL_RemoveTimer(gpsTimer);
    if (imuTimer)
        SDL_RemoveTimer(imuTimer);
    //SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    if (gpsFileDescriptor >= 0){
        closeSerialPort(gpsFileDescriptor);
    }
    if (imuFileDescriptor >= 0){
        closeI2C(imuFileDescriptor);
    }
    if (dataMutex)
        SDL_DestroyMutex(dataMutex);
    SDL_Quit();
}