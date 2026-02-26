#define SDL_MAIN_USE_CALLBACKS 1 //use the callbacks instead of main()

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <iostream>
#include <array>

#define VERSION "0.1"
#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;

std::array<SDL_Vertex,12> getGeometry(){
    std::array<SDL_Vertex,12> vertices{};

    int width;
    int height;
    SDL_GetWindowSize(window, &width, &height);
    const float fWidth = static_cast<float>(width);
    const float fHeight = static_cast<float>(height);
    const float spacer = fWidth*0.25;
    auto v = [&] (int i, float x, float y){vertices[i].position.x=x; vertices[i].position.y=y;vertices[i].color= {0.0f, 0.0f, 0.0f, 1.0f};};

    v( 0,   0.0f,   0.0f    );   v( 1,  0.0f,           fHeight );  v(  2,  spacer,         fHeight);
    v( 3,   spacer, 0.0f    );   v( 4,  0.0f,           0.0f    );  v(  5,  spacer,         fHeight);
    v( 6,   fWidth, fHeight );   v( 7,  fWidth-spacer,  0.0f    );  v(  8,  fWidth-spacer,  fHeight);
    v( 9,   fWidth, 0.0f    );   v( 10, fWidth,         fHeight );  v(  11, fWidth-spacer,  0.0f);

    return vertices;
}

SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[])
{
    SDL_SetAppMetadata("Airb-PFD", VERSION,"");
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("Failed to initialise SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("PFD", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_RESIZABLE, &window, &renderer)) {
        SDL_Log("Failed to Create Renderere or Window: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult SDL_AppIterate(void *appstate){
    SDL_SetRenderDrawColor(renderer, 0, 174, 199, 255);
    SDL_RenderClear(renderer);
    
    const auto vertices = getGeometry();
    SDL_RenderGeometry(renderer, NULL, vertices.data(), static_cast<int>(vertices.size()), NULL, 0);

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