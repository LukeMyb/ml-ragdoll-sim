#include "raylib.h"

int main(void)
{
    // ウィンドウを作成する（ここで .a と .dll の接続がテストされます）
    InitWindow(800, 450, "Hello World Test");

    while (!WindowShouldClose())
    {
        BeginDrawing();
            ClearBackground(RAYWHITE);
            
            // 画面の真ん中あたりにテキストを描画
            DrawText("Hello World!", 340, 200, 20, DARKGRAY);
            
        EndDrawing();
    }

    CloseWindow();
    return 0;
}