#define SDL_MAIN_USE_CALLBACKS 1 //use the callbacks instead of main()

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080

#define MPU6050_ADDRESS          0x68 // AD0 auf GND, bei AD0 auf VCC: 0x69
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

#define ACCEL_SCALE 16384.0f // LSB/g bei +-2g
#define GYRO_SCALE  131.0f   // LSB/(deg/s) bei +-250 deg/s
#define RAD_TO_DEG  (180.0f / 3.14159265f)

struct ImuData{
    float accelX, accelY, accelZ; // g
    float gyroX, gyroY, gyroZ;    // deg/s
    float temperature;            // deg C
    float roll, pitch;            // deg (nur aus Beschleunigung)
    bool valid;
};

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Mutex *imuMutex = NULL;
static SDL_TimerID imuTimer = 0;
static int imuFileDescriptor = -1;
static ImuData imuData{};

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

    if (writeRegister(fileDescriptor, MPU6050_REG_PWR_MGMT_1, 0x01) != 0){ // aufwecken, Takt von Gyro X PLL
        SDL_Log("Failed to wake up MPU6050");
        close(fileDescriptor);
        return -1;
    }
    SDL_Delay(100);

    writeRegister(fileDescriptor, MPU6050_REG_SMPLRT_DIV, 0x07);   // 1kHz / (1+7) = 125Hz
    writeRegister(fileDescriptor, MPU6050_REG_CONFIG, 0x03);       // Tiefpass ~44Hz
    writeRegister(fileDescriptor, MPU6050_REG_GYRO_CONFIG, 0x00);  // +-250 deg/s
    writeRegister(fileDescriptor, MPU6050_REG_ACCEL_CONFIG, 0x00); // +-2g

    return fileDescriptor;
}

void closeI2C(int fileDescriptor){
    close(fileDescriptor);
}

Uint32 update(void* userdata, SDL_TimerID timerID, Uint32 interval){
    uint8_t buffer[14]; // Accel XYZ, Temp, Gyro XYZ je 2 Byte
    if (imuFileDescriptor < 0){
        return interval;
    }

    if (readRegisters(imuFileDescriptor, MPU6050_REG_ACCEL_XOUT_H, buffer, sizeof(buffer)) != 0){
        SDL_LockMutex(imuMutex);
        imuData.valid = false;
        SDL_UnlockMutex(imuMutex);
        return interval;
    }

    int16_t raw[7];
    for (int i = 0; i < 7; i++){
        raw[i] = (int16_t)((buffer[2*i] << 8) | buffer[2*i+1]);
    }

    ImuData data{};
    data.accelX = raw[0] / ACCEL_SCALE;
    data.accelY = raw[1] / ACCEL_SCALE;
    data.accelZ = raw[2] / ACCEL_SCALE;
    data.temperature = raw[3] / 340.0f + 36.53f;
    data.gyroX = raw[4] / GYRO_SCALE;
    data.gyroY = raw[5] / GYRO_SCALE;
    data.gyroZ = raw[6] / GYRO_SCALE;
    data.roll  = atan2f(data.accelY, data.accelZ) * RAD_TO_DEG;
    data.pitch = atan2f(-data.accelX, sqrtf(data.accelY*data.accelY + data.accelZ*data.accelZ)) * RAD_TO_DEG;
    data.valid = true;

    SDL_LockMutex(imuMutex);
    imuData = data;
    SDL_UnlockMutex(imuMutex);

    return interval;
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]){
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("Failed to initialise SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("IMU", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_FULLSCREEN, &window, &renderer)) {
        SDL_Log("Failed to Create Rendererer or Window: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    imuMutex = SDL_CreateMutex();
    imuFileDescriptor = openI2C("/dev/i2c-1", MPU6050_ADDRESS);

    imuTimer = SDL_AddTimer(50, update, nullptr); // 20Hz
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate){
    ImuData data;
    SDL_LockMutex(imuMutex);
    data = imuData;
    SDL_UnlockMutex(imuMutex);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    if (!data.valid){
        SDL_RenderDebugText(renderer, 10, 10, "Warte auf IMU");
    } else {
        SDL_RenderDebugTextFormat(renderer, 10, 10, "Accel [g]    X:%7.2f Y:%7.2f Z:%7.2f", data.accelX, data.accelY, data.accelZ);
        SDL_RenderDebugTextFormat(renderer, 10, 22, "Gyro [deg/s] X:%7.2f Y:%7.2f Z:%7.2f", data.gyroX, data.gyroY, data.gyroZ);
        SDL_RenderDebugTextFormat(renderer, 10, 34, "Temp [C]     %6.1f", data.temperature);
        SDL_RenderDebugTextFormat(renderer, 10, 58, "Roll:  %7.1f deg", data.roll);
        SDL_RenderDebugTextFormat(renderer, 10, 70, "Pitch: %7.1f deg", data.pitch);
    }

    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event){
    switch(event->type){
        case SDL_EVENT_QUIT:
            return SDL_APP_SUCCESS;
            break;
    }
    return SDL_APP_CONTINUE;
}

void SDL_AppQuit(void *appstate, SDL_AppResult result)
{
    if (imuTimer)
        SDL_RemoveTimer(imuTimer); // Timer stoppen bevor der Bus geschlossen wird
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    if (imuFileDescriptor >= 0)
        closeI2C(imuFileDescriptor);
    if (imuMutex)
        SDL_DestroyMutex(imuMutex);
    SDL_Quit();
}