#include "raylib.h"
#include "raymath.h"

// 物理演算用の構造体
struct RigidBody {
    Vector3 position; // 座標
    Vector3 velocity; // 速度
    float mass;       // 質量
    
    // 力を加えて速度を変化させる
    void ApplyForce(Vector3 force) {
        // 加速度 = 力 / 質量 (a = F / m)
        Vector3 acceleration = Vector3Scale(force, 1.0f / mass);
        // 速度 = 速度 + 加速度
        velocity = Vector3Add(velocity, acceleration);
    }

    // 速度を使って位置を更新する（1フレームごとの処理）
    void Update(float deltaTime) {
        // 移動量 = 速度 * 経過時間
        Vector3 deltaPos = Vector3Scale(velocity, deltaTime);
        // 座標 = 座標 + 移動量
        position = Vector3Add(position, deltaPos);
    }
};

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

    // 落下させる箱（RigidBody）の初期化
    RigidBody box;
    box.position = Vector3{ 0.0f, 10.0f, 0.0f }; // 少し高い位置(Y=10)からスタート
    box.velocity = Vector3{ 0.0f, 0.0f, 0.0f };  // 初速はゼロ
    box.mass = 1.0f;                             // 質量は1.0kg

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        // 物理演算の更新処理
        float deltaTime = GetFrameTime(); // 前のフレームから何秒経過したか（約0.016秒）

        // 重力ベクトルを作成（地球の重力加速度 -9.8 * 質量）
        Vector3 gravity = Vector3{ 0.0f, -9.8f * box.mass, 0.0f };
        box.ApplyForce(gravity); // 箱に重力を加える

        // 速度から座標を更新
        box.Update(deltaTime);

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
                DrawCube(box.position, 2.0f, 2.0f, 2.0f, RED);
                DrawCubeWires(box.position, 2.0f, 2.0f, 2.0f, BLACK);
                DrawGrid(10, 1.0f);
            EndMode3D();

            // 画面の左上にテキストを描画（ここは2D描画）
            DrawText("Gravity and Motion", 10, 10, 20, DARKGRAY);
            DrawText("Hold RIGHT CLICK to move (WASD/Space/Shift) and look around", 10, 40, 10, DARKGRAY);

        EndDrawing();
    }

    // 終了処理
    CloseWindow();

    return 0;
}