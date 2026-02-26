#define SDL_MAIN_USE_CALLBACKS 1 //use the callbacks instead of main()

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <iostream>

#define VERSION "0.1"
#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[])
{
    SDL_SetAppMetadata("Airb-PFD", VERSION,"");
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("Failed to initialise SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("examples/renderer/geometry", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_FULLSCREEN, &window, &renderer)) {
        SDL_Log("Failed to Create Renderere or Window: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate){
    SDL_SetRenderDrawColor(renderer, 0, 174, 199, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event){
    if (event->type == SDL_EVENT_QUIT){
        return SDL_APP_SUCCESS;
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