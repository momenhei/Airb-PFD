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
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define VERSION "0.4"
#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static float fWidth  = 0.0f;
static float fHeight = 0.0f;
static float aHSize =  0.0f;

static constexpr int vertexCount = 36;
static std::unique_ptr<SDL_Vertex[]> mask;
static std::unique_ptr<SDL_Vertex[]> horizon;

static float horizonRotation=90.0f;
static float horizonRadius=0.0f;
static float speed=0.0;
static int altitude=0;
static std::string gpsTime;

static int gpsFileDescriptor=-1;
static std::string gpsText = "Warte auf GPS";

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
                gpsTime = tmp.substr(0,2)+":"+tmp.substr(2,2)+":"+tmp.substr(4,2);
                break;
            case 2:  //Position status (A = data valid, V = data invalid)
                
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
    char buffer[256];
    std::string serialBuffer;
    int n = read(gpsFileDescriptor, buffer, sizeof(buffer));
    if (n > 0){
	    serialBuffer.append(buffer, n);
	    size_t pos;
	    while ((pos = serialBuffer.find("\n")) != std::string::npos){
	        gpsText = serialBuffer.substr(0, pos);
	        serialBuffer.erase(0,pos+1);
	    }
    }
    filterData();
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

void updateHorizon(){
    calculateHorizonVertex(0,horizonRotation,fWidth);
    calculateHorizonVertex(1,180+horizonRotation,fWidth);
    calculateHorizonVertex(2,90+horizonRotation,fWidth);
}

void updateMask(){ //creating vertices for mask to create window for artificial horizon
    //calculating sclaing variables
    int width;
    int height;
    SDL_GetWindowSize(window, &width, &height);
    fWidth  = static_cast<float>(width);
    fHeight = static_cast<float>(height);
    aHSize  = 0.5f*std::min(fHeight,fWidth);       //size of Artificial Horizon
    const float hSpacer = 0.5f*(fWidth-aHSize);
    const float vSpacer = 0.5f*(fHeight-aHSize);
    int idx = 0;
    auto v = [&] (float x, float y){mask[idx].position.x=x; mask[idx].position.y=y;mask[idx].color= {0.0f, 0.0f, 0.0f, 1.0f}; idx++;};
    auto tri = [&] (float x1, float y1, float x2, float y2, float x3, float y3) {v(x1,y1); v(x2,y2); v(x3,y3);};
    auto rec = [&] (float x1, float y1, float x2, float y2) {tri(x1, y1, x1, y2, x2, y2); tri(x1, y1, x2, y2, x2, y1);};

    //left & right boundaries
    rec(0.0f, 0.0f, hSpacer, fHeight);
    rec(fWidth, fHeight, fWidth-hSpacer, 0.0f);
    
    //top & bottom boundaries
    rec(0.0f, 0.0f, fWidth, vSpacer);
    rec(fWidth, fHeight, 0.0f, fHeight-vSpacer);

    //upper rounding
    tri(fWidth-hSpacer, vSpacer +aHSize/6, fWidth-(hSpacer+aHSize/4), vSpacer, fWidth-hSpacer, vSpacer);
    tri(hSpacer,        vSpacer + aHSize/6, hSpacer+ aHSize/4,         vSpacer, hSpacer,        vSpacer);

    //lower rounding
    tri(fWidth-hSpacer, fHeight-(vSpacer + aHSize/6), fWidth-(hSpacer+aHSize/4), fHeight-vSpacer, fWidth-hSpacer, fHeight-vSpacer);
    tri(hSpacer,        fHeight-(vSpacer + aHSize/6), hSpacer+ aHSize/4,         fHeight-vSpacer, hSpacer,        fHeight-vSpacer);
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

    std::string speedStr = std::to_string(speed);
    SDL_RenderDebugText(renderer, 38.4 -speedStr.length()*3.5, 4, speedStr.c_str());
    SDL_RenderDebugText(renderer, 38.4-strlen("km/h")*3.5,    14, "km/h");

    SDL_RenderDebugText(renderer, 115.2-strlen("G/S")*3.5,     4, "G/S");
    
  //SDL_RenderDebugText(renderer, 192-strlen("LOC")*3.5,       4, "LOC");
  //SDL_RenderDebugText(renderer, 192-strlen(t.date.c_str())*3.5,       4, t.date.c_str());
    SDL_RenderDebugText(renderer, 192-8*3.5,       4, gpsTime.c_str());
  
  //SDL_RenderDebugText(renderer, 268.8-strlen("CAT2")*3.5,    4, "CAT2");  
  //std::string timeStr = getTimeString();
  //SDL_RenderDebugText(renderer, 268.8 - timeStr.length()*3.5, 4, timeStr.c_str());   
    SDL_RenderDebugText(renderer, 268.8 - 2*3.5, 4, "WD");

  //SDL_RenderDebugText(renderer, 345.6-strlen("AP1")*3.5,     4, "AP1");
  //SDL_RenderDebugText(renderer, 345.6-strlen("FD1")*3.5,    14, "FD1");
    SDL_RenderDebugText(renderer, 345.6-4*3.5,     4, "date");
    SDL_RenderDebugText(renderer, 345.6-4*3.5,    14, "year");
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_SetRenderScale(renderer, 1, 1);
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
    
    // speed indicator
    float spacing = barHeight /6.0f;
    float center = fHeight /2.0f;
    float offset=fmodf(speed,5.0f)*spacing/5;
    for (int i=-3; i<3; i++) {
        float y = center + i * spacing + offset;
        SDL_RenderLine(renderer,
            5*fWidth/24,
            y,
            17*fWidth/96,
            y);
    }
    SDL_Vertex tri[] = {
        {{5*fWidth/24,fHeight/2},{255,255,255,255}},
        {{5*fWidth/24+fWidth/75,fHeight/2+fHeight/100},{255,255,255,255}},
        {{5*fWidth/24+fWidth/75,fHeight/2-fHeight/100},{255,255,255,255}}
    };
    SDL_RenderGeometry(renderer, NULL, tri, 3, NULL, 0);

    //altitude indicator
    // TO-DO
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

    mask = std::make_unique<SDL_Vertex[]>(vertexCount);
    updateMask();
    horizon = std::make_unique<SDL_Vertex[]>(3);
    updateHorizon();
    SDL_HideCursor();
    
    gpsFileDescriptor = openSerialPort("/dev/serial0");
    SDL_AddTimer(100, updateData, nullptr);

    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate){
    SDL_SetRenderDrawColor(renderer, 3, 169, 244, 255);
    SDL_RenderClear(renderer);
    updateHorizon();
    SDL_RenderGeometry(renderer, NULL, horizon.get(), 3, NULL, 0);

    SDL_RenderGeometry(renderer, NULL, mask.get(), vertexCount, NULL, 0);
    
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
    if (gpsFileDescriptor >= 0){
        closeSerialPort(gpsFileDescriptor);
    }
    SDL_Quit();
}