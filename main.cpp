#define SDL_MAIN_USE_CALLBACKS 1 //use the callbacks instead of main()

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <iostream>
#include <array>
#include <algorithm>
#include <memory>
#include <math.h>

#define VERSION "0.2"
#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static float fWidth  = 0.0f;
static float fHeight = 0.0f;

static constexpr int vertexCount = 36;
static std::unique_ptr<SDL_Vertex[]> mask;
static std::unique_ptr<SDL_Vertex[]> horizon;

static float horizonRotation=90.0f;
static float horizonRadius=0.0f;

float degreeToRad(float dgr){
    return dgr*3.141/180;
}

void calculateHorizonVertex(int index, float offsetPhi,float offsetR){
    offsetPhi=degreeToRad(offsetPhi-90);
    float hrzRot=degreeToRad(horizonRotation);
    float phi= hrzRot+atan2(offsetR*sin(offsetPhi-hrzRot),horizonRadius+offsetR*cos(offsetPhi-hrzRot));
    float r= sqrt(pow(horizonRadius,2)+pow(offsetR,2)+2*horizonRadius*offsetR*cos(offsetPhi-hrzRot));
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
    const float aHSize  = 0.5f*std::min(fHeight,fWidth);       //size of Artificial Horizon
    const float hSpacer = 0.5f*(fWidth-aHSize);
    const float vSpacer = 0.5f*(fHeight-aHSize);
    auto v = [&] (int i, float x, float y){mask[i].position.x=x; mask[i].position.y=y;mask[i].color= {0.0f, 0.0f, 0.0f, 1.0f};};

    //left & right boundaries
    v(0, 0.0f,    0.0f   ); v( 1, 0.0f,           fHeight); v( 2,  hSpacer,        fHeight);
    v(3, hSpacer, 0.0f   ); v( 4, 0.0f,           0.0f   ); v( 5,  hSpacer,        fHeight);
    v(6, fWidth,  fHeight); v( 7, fWidth-hSpacer, 0.0f   ); v( 8,  fWidth-hSpacer, fHeight);
    v(9, fWidth,  0.0f   ); v(10, fWidth,         fHeight); v(11,  fWidth-hSpacer, 0.0f   );

    //top & bottom boundaries
    v(12, fWidth-hSpacer, 0.0f); v(13, fWidth-hSpacer, vSpacer); v(14, hSpacer,        vSpacer);
    v(15, hSpacer,        0.0f); v(16, hSpacer,        vSpacer); v(17, fWidth-hSpacer, 0.0f   );
    
    v(18, fWidth-hSpacer, fHeight); v(19, fWidth-hSpacer, fHeight-vSpacer); v(20, hSpacer,        fHeight-vSpacer);
    v(21, hSpacer,        fHeight); v(22, hSpacer,        fHeight-vSpacer); v(23, fWidth-hSpacer, fHeight        );

    //upper rounding
    v(24, fWidth-hSpacer, vSpacer +aHSize/6 ); v(25, fWidth-(hSpacer+aHSize/4), vSpacer);  v(26, fWidth-hSpacer, vSpacer);
    v(27, hSpacer,        vSpacer + aHSize/6); v(28, hSpacer+ aHSize/4,         vSpacer);  v(29, hSpacer,        vSpacer);

    //lower rounding
    v(30, fWidth-hSpacer, fHeight-(vSpacer + aHSize/6)); v(31, fWidth-(hSpacer+aHSize/4), fHeight-vSpacer); v(32, fWidth-hSpacer, fHeight-vSpacer);
    v(33, hSpacer,        fHeight-(vSpacer + aHSize/6)); v(34, hSpacer+ aHSize/4,         fHeight-vSpacer); v(35, hSpacer,        fHeight-vSpacer);
}

void renderDeviders(){
    float width=fWidth/5;
    float height=fHeight/10;
    for (int i=1;i<6;i++){
        SDL_RenderLine(renderer,i*width,0,i*width,height);
    }
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[])
{
    SDL_SetAppMetadata("Airb-PFD", VERSION,"");
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("Failed to initialise SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("PFD", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_RESIZABLE, &window, &renderer)) {
        SDL_Log("Failed to Create Rendererer or Window: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_SetRenderVSync(renderer, 1);

    mask = std::make_unique<SDL_Vertex[]>(vertexCount);
    updateMask();
    horizon = std::make_unique<SDL_Vertex[]>(3);
    updateHorizon();
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
                    horizonRadius -= 4;
                    break;
                case SDLK_S:
                    horizonRadius += 4;
                    break;
                case SDLK_A:
                    horizonRotation -= 1;
                    break;
                case SDLK_D:
                    horizonRotation += 1;
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
    SDL_Quit();
}