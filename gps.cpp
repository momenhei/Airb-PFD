#define SDL_MAIN_USE_CALLBACKS 1 //use the callbacks instead of main()

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <iostream>
#include <fcntl.h>

#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static int i = 0;

int openSerialPort(const char* name){
    int fileDescriptor;
    fileDescriptor = open(name, O_RDONLY | O_NOCTTY | O_NDELAY) // rad only & not controling terminal & don't care about state of DCD signal line
    if (fileDescriptor<0){
        SDL_Log("Failed to Open Serial Port");
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

Uint32 update(void* userdata, SDL_TimerID timerID, Uint32 interval){
    i+=1;
    return interval;
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[]){
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("Failed to initialise SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("PFD", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_RESIZABLE, &window, &renderer)) {
        SDL_Log("Failed to Create Rendererer or Window: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    SDL_AddTimer(1000, update, nullptr);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate){
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    std::string str = std::to_string(i);
    SDL_RenderDebugText(renderer, 10, 10, str.c_str());
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
    //SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}