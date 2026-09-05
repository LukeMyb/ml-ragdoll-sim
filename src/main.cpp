#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <fstream>

#include "graphics/Renderer.h"
#include "physics/RigidBody.h"
#include "physics/Collision.h"
#include "physics/PhysicsWorld.h"

int main(void)
{
    // 画面の初期化 (800x450のウィンドウを作成)
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "Impulse & Rotation Bounce");

    // 3Dカメラの設定
    Camera3D camera = { 0 };
    camera.position = Vector3{ 10.0f, 10.0f, 10.0f };
    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };        // 空間の「上」方向はY軸のプラス方向
    camera.fovy = 45.0f;                                // 視野角
    camera.projection = CAMERA_PERSPECTIVE;             // 透視投影（遠近法）

    // 箱1を作成
    RigidBody box1;
    box1.position = Vector3{ 0.0f, 10.0f, 0.0f };
    box1.velocity = Vector3{ 0.0f, 0.0f, 0.0f };
    box1.mass = 1.0f;
    box1.size = Vector3{ 2.0f, 2.0f, 2.0f };
    box1.orientation = QuaternionFromAxisAngle(Vector3{ 1.0f, 0.0f, 0.0f }, 30.0f * DEG2RAD);
    box1.angularVelocity = Vector3{ 0.0f, 0.0f, 0.0f };
    box1.isStatic = false;
    box1.localInertiaInverse = Vector3{ 1.0f, 1.0f, 1.0f }; // 簡易的な逆慣性テンソル

    // 箱2を作成（箱1と完全に同じ位置に配置＝完全に重なっている状態）
    RigidBody box2;
    box2.position = Vector3{ 0.0f, 10.0f, 0.0f }; // 箱1と同じ位置
    box2.velocity = Vector3{ 0.0f, 0.0f, 0.0f };
    box2.mass = 1.0f;
    box2.size = Vector3{ 1.5f, 1.5f, 1.5f };      // 色分けの代わりに少しサイズを変える
    box2.orientation = QuaternionFromAxisAngle(Vector3{ 0.0f, 0.0f, 1.0f }, 60.0f * DEG2RAD);
    box2.angularVelocity = Vector3{ 0.0f, 0.0f, 0.0f };
    box2.isStatic = false;
    box2.localInertiaInverse = Vector3{ 1.0f, 1.0f, 1.0f };

    // 床を「巨大な固定された箱」として作成
    RigidBody floor;
    // 上の面がY=0になるように、Y位置を厚みの半分だけ下げる
    floor.position = Vector3{ 0.0f, -0.5f, 0.0f }; 
    floor.velocity = Vector3{ 0.0f, 0.0f, 0.0f };
    floor.mass = 0.0f; // 固定物なので関係なし
    floor.size = Vector3{ 20.0f, 1.0f, 20.0f }; // 広さ20x20、厚さ1の箱
    floor.orientation = QuaternionIdentity();
    floor.angularVelocity = Vector3{ 0.0f, 0.0f, 0.0f };
    floor.isStatic = true; // 重力や衝突で動かないようにする
    floor.localInertiaInverse = Vector3{ 0.0f, 0.0f, 0.0f }; // 固定物なので回転しにくさは無限大(逆数は0)

    // 箱1と箱2はお互いに衝突しないように設定
    box1.IgnoreCollisionWith(&box2);

    // 物理ワールドの作成と剛体の登録
    PhysicsWorld world;
    world.AddBody(&box1); 
    world.AddBody(&box2);
    world.AddBody(&floor);

    SetTargetFPS(60);

    // ログファイルの準備
    std::ofstream logFile("physics_log.csv");
    if (logFile.is_open()) {
        logFile << "Frame,PosY,VelY,AngularSpeed\n"; // ヘッダ行
    }
    int frameCount = 0;

    while (!WindowShouldClose())
    {
        frameCount++; // フレームカウント進行

        // 物理演算の更新処理
        float deltaTime = GetFrameTime(); // 前のフレームから何秒経過したか（約0.016秒）

        // 物理ワールドの更新
        world.Step(deltaTime);

        // ログの書き込み（スリープしていない間だけ記録）
        if (logFile.is_open() && !box1.isSleeping) {
            logFile << frameCount << ","
                    << box1.position.y << ","
                    << box1.velocity.y << ","
                    << Vector3Length(box1.angularVelocity) << "\n";
        }


        // カメラの制御（右クリックドラッグ時のみ）
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) DisableCursor();
        if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) EnableCursor();

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
                // Worldに登録されている全ての剛体を自動で描画
                for (RigidBody* body : world.GetBodies()) {
                    // 固定物はグレー、動くものは赤で描画
                    DrawOBB(*body, body->isStatic ? GRAY : RED);
                }
                
                // 衝突点のデバッグ描画（複数対応）
                for (Vector3 contact : world.debugContactPoints) {
                    DrawSphere(contact, 0.15f, BLUE);
                }
            EndMode3D();

            // 画面の左上にテキストを描画（ここは2D描画）
            DrawText("Impulse & Rotation Bounce", 10, 10, 20, DARKGRAY);
            DrawText("Hold RIGHT CLICK to move (WASD/Space/Shift) and look around", 10, 40, 10, DARKGRAY);

        EndDrawing();
    }

    // ログファイルを閉じる
    if (logFile.is_open()) {
        logFile.close();
    }

    // 終了処理
    CloseWindow();

    return 0;
}