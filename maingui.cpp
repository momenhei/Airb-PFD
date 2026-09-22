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
#include <cstdio>
#include <vector>

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif


#define VERSION "0.4"
#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080

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
static float speed=0.0;
static int altitude=0;
static float heading = 0.0f; // 0-360°, 0 = Nord

static int gpsFileDescriptor=-1;
static std::string gpsText = "Warte auf GPS";

struct GPSData {
    int utcHour = -1, utcMinute = -1, utcSecond = -1;
    int day = 0, month = 0, year = 0;
    bool summerTime = false;

    std::string localTime;      // "HH:MM" (CET/CEST)
    std::string localDate;      // "DD.MM.YYYY"
    std::string localWeekday;   // Wochentag
};
static GPSData gps;
#ifndef _WIN32
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
#endif

float knotsToKmH(float velocity){
    return velocity*1.852;
}

// Uhrzeit + Datum berechnen
std::string weekdayName(int wd) {
    static const std::array<std::string, 7> names = {
        "Sonntag","Montag","Dienstag","Mittwoch","Donnerstag","Freitag","Samstag"
    };
    return names[wd];
}

bool isLeapYear(int y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

int daysInMonthOf(int y, int m) {
    static const int daysPerMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (m == 2 && isLeapYear(y)) return 29;
    return daysPerMonth[m - 1];
}

// Sakamoto-Algorithmus: 0 = Sonntag, 1 = Montag, ...
int dayOfWeek(int y, int m, int d) {
    static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (m < 3) y -= 1;
    return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

int lastSundayOfMonth(int y, int m) {
    int lastDay = daysInMonthOf(y, m);
    return lastDay - dayOfWeek(y, m, lastDay);
}

// EU-Regel: Umstellung um 01:00 UTC am letzten Sonntag im März/Oktober
bool calcIsSummerTime(int y, int m, int d, int hourUTC) {
    if (m < 3 || m > 10) return false;
    if (m > 3 && m < 10) return true;
    int boundary = lastSundayOfMonth(y, m);
    if (m == 3) return d > boundary || (d == boundary && hourUTC >= 1);
    return d < boundary || (d == boundary && hourUTC < 1);
}

void parseTime(const std::string& timeField) {
    if (timeField.length() < 6) return;
    gps.utcHour   = std::stoi(timeField.substr(0,2));
    gps.utcMinute = std::stoi(timeField.substr(2,2));
    gps.utcSecond = std::stoi(timeField.substr(4,2));
}

void parseDate(const std::string& dateField) {
    if (dateField.length() < 6) return;

    gps.day   = std::stoi(dateField.substr(0,2));
    gps.month = std::stoi(dateField.substr(2,2));
    gps.year  = std::stoi(dateField.substr(4,2));

    gps.summerTime = calcIsSummerTime(gps.year, gps.month, gps.day, gps.utcHour);
    int offset = gps.summerTime ? 2 : 1;

    int localHour = gps.utcHour + offset;
    int localDay = gps.day, localMonth = gps.month, localYear = gps.year;
    if (localHour >= 24) {
        localHour -= 24;
        localDay += 1;
        if (localDay > daysInMonthOf(localYear, localMonth)) {
            localDay = 1;
            localMonth += 1;
            if (localMonth > 12) { localMonth = 1; localYear += 1; }
        }
    }

    char timeBuf[6];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", localHour, gps.utcMinute);
    gps.localTime = timeBuf;

    char dateBuf[11];
    snprintf(dateBuf, sizeof(dateBuf), "%02d.%02d.%02d", localDay, localMonth, localYear);
    gps.localDate = dateBuf;

    gps.localWeekday = weekdayName(dayOfWeek(localYear, localMonth, localDay));
}

void filterData(){
    if (gpsText.compare(0,6,"$GPRMC")==0){
        std::string copy=gpsText;
        size_t pos;
        int i=0;
        while ((pos = copy.find(",")) != std::string::npos){
            std::string tmp = copy.substr(0,pos);
            copy.erase(0, pos+1);
            switch(i){
            case 1:  //UTC of position
                parseTime(tmp);
                break;
            case 2:  //Position status (A = data valid, V = data invalid)
                if(tmp != "A")
                {
                    return;
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
                    speed = knotsToKmH(std::stof(tmp));
                }catch (const std::exception& e) {
                    speed=0.0;
                }
                break;
            case 8:  //Track made good, degrees True
                
                break;
            case 9:  //Date: dd/mm/yy
                parseDate(tmp);
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
    }
}

Uint32 updateData(void* userdata, SDL_TimerID timerID, Uint32 interval){
    #ifdef _WIN32
        static float simSpeedKmh = 0.0f;
        simSpeedKmh += 1.0f;
        if (simSpeedKmh > 250.0f) simSpeedKmh = 0.0f;
        float simSpeedKnots = simSpeedKmh / 1.852f;

        time_t now = time(nullptr);
        tm t;
        gmtime_s(&t, &now); // echte UTC-Zeit, wie ein reales GPS-Modul sie liefert

        char sentence[256];
        snprintf(sentence, sizeof(sentence),
            "$GPRMC,%02d%02d%02d,A,4807.038,N,01131.000,E,%.1f,084.4,%02d%02d%02d,003.1,W*6A",
            t.tm_hour, t.tm_min, t.tm_sec,
            simSpeedKnots,
            t.tm_mday, t.tm_mon + 1, t.tm_year % 100);

        gpsText = sentence;
        filterData();
    #else
        char buffer[256];
        static std::string serialBuffer;
        int n = read(gpsFileDescriptor, buffer, sizeof(buffer));
        if (n > 0){
            serialBuffer.append(buffer, n);
            size_t start;
            while ((start = serialBuffer.find("$GPRMC")) != std::string::npos){
                size_t end = serialBuffer.find("\n", start);
                if(end == std::string::npos){
                    break;
                }
                gpsText = serialBuffer.substr(start, end - start);
                filterData();
                serialBuffer.erase(0,end+1);
            }
        }
    #endif
    return interval;
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

SDL_FPoint horizonPoint(float localX, float localY){
    float pitchOffset = aHSize*horizonRadius;  // gleicher Wert wie in calculateHorizonVertex
    float y = localY + pitchOffset;
    float rot = degreeToRad(horizonRotation - 90.0f);
    float rx = localX*cos(rot) - y*sin(rot);
    float ry = localX*sin(rot) + y*cos(rot);
    return { rx + fWidth/2, ry + fHeight/2 };
}

void updateHorizon(){
    calculateHorizonVertex(0,horizonRotation,fWidth);
    calculateHorizonVertex(1,180+horizonRotation,fWidth);
    calculateHorizonVertex(2,90+horizonRotation,fWidth);
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

        // Zahl jeweils außen an beiden Enden
        std::string label = std::to_string(std::abs(i*stepDeg));
        SDL_SetRenderScale(renderer, 2.0f, 2.0f);

        SDL_FPoint textPosL = horizonPoint(-halfWidth-30, i*stepPx);
        SDL_RenderDebugText(renderer, textPosL.x/2.0f, (textPosL.y-6)/2.0f, label.c_str());

        SDL_FPoint textPosR = horizonPoint(halfWidth+10, i*stepPx);
        SDL_RenderDebugText(renderer, textPosR.x/2.0f, (textPosR.y-6)/2.0f, label.c_str());

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
    SDL_SetRenderScale(renderer, fWidth/384, fHeight/216);
    SDL_SetRenderDrawColor(renderer, 44, 255, 5, 255);

    std::string speedStr = std::to_string(static_cast<int>(std::round(speed)));
    SDL_RenderDebugText(renderer, 38.4 -speedStr.length()*3.5, 4, speedStr.c_str());
    SDL_RenderDebugText(renderer, 38.4-strlen("km/h")*3.5,    14, "km/h");

    SDL_RenderDebugText(renderer, 115.2-strlen("G/S")*3.5,     4, "G/S");
    
    SDL_RenderDebugText(renderer, 192-gps.localTime.length()*3.5, 4, gps.localTime.c_str());
  
    SDL_RenderDebugText(renderer, 268.8 - gps.localWeekday.length()*3.5, 4, gps.localWeekday.c_str());

    SDL_RenderDebugText(renderer, 345.6-gps.localDate.length()*3.5,     4, gps.localDate.c_str());
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
    SDL_SetRenderScale(renderer, scaleX, scaleY);
    SDL_RenderDebugText(renderer, liveX/scaleX, (center-6)/scaleY, liveLabel.c_str());
    SDL_SetRenderScale(renderer, 1, 1);

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

        SDL_RenderLine(renderer, x, tickTopY, x, tickBotY);

        float tickValue = value - fmodf(value, tickStep) + i * tickStep;
        int wrapped = (static_cast<int>(std::round(tickValue)) % 360 + 360) % 360;
        std::string label = headingLabel(wrapped);

        SDL_SetRenderScale(renderer, scaleX, scaleY);
        SDL_RenderDebugText(renderer, (x - label.length()*charW/2.0f)/scaleX, (tickBotY+4)/scaleY, label.c_str());
        SDL_SetRenderScale(renderer, 1, 1);
    }

    // Live-Gradzahl
    int liveHeading = (static_cast<int>(std::round(value)) % 360 + 360) % 360;
    std::string liveLabel = std::to_string(liveHeading);
    float liveY = rect.y - rect.h*0.15f - 20;
    SDL_SetRenderScale(renderer, scaleX, scaleY);
    SDL_RenderDebugText(renderer, (center - liveLabel.length()*charW/2.0f)/scaleX, liveY/scaleY, liveLabel.c_str());
    SDL_SetRenderScale(renderer, 1, 1);

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

    #ifndef _WIN32
    gpsFileDescriptor = openSerialPort("/dev/serial0");
    #endif
    SDL_AddTimer(100, updateData, nullptr);

    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate){
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
                case SDLK_W:
                    horizonRadius -= 0.01;
                    break;
                case SDLK_S:
                    horizonRadius += 0.01;
                    break;
                case SDLK_A:
                    horizonRotation -= 1;
                    break;
                case SDLK_D:
                    horizonRotation += 1;
                    break;
                case SDLK_UP: 
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
    //SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    #ifndef _WIN32
    if (gpsFileDescriptor >= 0){
        closeSerialPort(gpsFileDescriptor);
    }
    #endif
    SDL_Quit();
}