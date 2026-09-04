#include "raylib.h"

int main(void)
{
    // 画面の初期化 (800x450のウィンドウを作成)
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "3D Camera & Cube");

    // 3Dカメラの設定
    Camera3D camera = { 0 };
    camera.position = Vector3{ 10.0f, 10.0f, 10.0f };
    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };        // 空間の「上」方向はY軸のプラス方向
    camera.fovy = 45.0f;                                // 視野角
    camera.projection = CAMERA_PERSPECTIVE;             // 透視投影（遠近法）

    SetTargetFPS(60); // 60FPS

    while (!WindowShouldClose())
    {
        // 押した瞬間と離した瞬間でカーソルの表示/非表示を切り替える
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) DisableCursor();
        if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) EnableCursor();

        // カメラの制御（右クリックドラッグ時のみ）
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            // 感度とスピードを調整できる変数
            float cameraSpeed = 0.2f;
            float mouseSensitivity = 0.1f; 

            // 視点移動（マウスの移動量からYaw, Pitchの回転量を作成）
            Vector2 mouseDelta = GetMouseDelta();
            Vector3 rotation = Vector3{ mouseDelta.x * mouseSensitivity, mouseDelta.y * mouseSensitivity, 0.0f };

            // 平行移動（WASDキーで前後左右、Q/Eキーで上下）
            Vector3 movement = Vector3{ 0.0f, 0.0f, 0.0f };
            if (IsKeyDown(KEY_W)) movement.x += cameraSpeed; // 前進
            if (IsKeyDown(KEY_S)) movement.x -= cameraSpeed; // 後退
            if (IsKeyDown(KEY_D)) movement.y += cameraSpeed; // 右移動
            if (IsKeyDown(KEY_A)) movement.y -= cameraSpeed; // 左移動
            if (IsKeyDown(KEY_SPACE)) movement.z += cameraSpeed;      // 上昇 
            if (IsKeyDown(KEY_LEFT_SHIFT)) movement.z -= cameraSpeed; // 下降

            // カメラの状態を更新
            UpdateCameraPro(&camera, movement, rotation, 0.0f);
        }

        // --- 描画処理 ---
        BeginDrawing();
            ClearBackground(RAYWHITE); // 背景を白でクリア

            // 3D空間の描画モードを開始
            BeginMode3D(camera);
                
                // 原点の少し上に直方体（人形の胴体を想定）を描画
                // DrawCube(中心座標, 幅X, 高さY, 奥行Z, 色)
                DrawCube(Vector3{ 0.0f, 1.0f, 0.0f }, 2.0f, 2.0f, 2.0f, RED);
                
                // 箱の輪郭を黒線で描いて立体感を出す
                DrawCubeWires(Vector3{ 0.0f, 1.0f, 0.0f }, 2.0f, 2.0f, 2.0f, BLACK);
                
                // 空間の広さを把握しやすくするため、床にグリッド（マス目）を描画
                DrawGrid(10, 1.0f);
                
            EndMode3D();

            // 画面の左上にテキストを描画（ここは2D描画）
            DrawText("Hold RIGHT CLICK to move (WASD/Space/Shift) and look around", 10, 40, 10, DARKGRAY);

        EndDrawing();
    }

    // 終了処理
    CloseWindow();

    return 0;
}