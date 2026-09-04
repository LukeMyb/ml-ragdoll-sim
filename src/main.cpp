#include "raylib.h"

int main(void)
{
    // 画面の初期化 (800x450のウィンドウを作成)
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "Step 1: 3D Camera & Cube");

    // 3Dカメラの設定
    Camera3D camera = { 0 };
    camera.position = Vector3{ 10.0f, 10.0f, 10.0f };
    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };        // 空間の「上」方向はY軸のプラス方向
    camera.fovy = 45.0f;                                // 視野角
    camera.projection = CAMERA_PERSPECTIVE;             // 透視投影（遠近法）

    SetTargetFPS(60); // 60FPSで動作させる

    // メインループ
    while (!WindowShouldClose())
    {
        // --- 描画処理 ---
        BeginDrawing();
            ClearBackground(RAYWHITE); // 背景を白でクリア

            // ここから3D空間の描画モードを開始
            BeginMode3D(camera);
                
                // 原点の少し上に直方体（人形の胴体を想定）を描画
                // DrawCube(中心座標, 幅X, 高さY, 奥行Z, 色)
                DrawCube(Vector3{ 0.0f, 1.0f, 0.0f }, 2.0f, 2.0f, 2.0f, RED);
                
                // 箱の輪郭を黒線で描いて立体感を出す
                DrawCubeWires(Vector3{ 0.0f, 1.0f, 0.0f }, 2.0f, 2.0f, 2.0f, BLACK);
                
                // 空間の広さを把握しやすくするため、床にグリッド（マス目）を描画
                DrawGrid(10, 1.0f);
                
            EndMode3D();
            // 3D描画モード終了

            // 画面の左上にテキストを描画（ここは2D描画）
            DrawText("Step 1: 3D Space Ready", 10, 10, 20, DARKGRAY);

        EndDrawing();
    }

    // 終了処理
    CloseWindow();

    return 0;
}