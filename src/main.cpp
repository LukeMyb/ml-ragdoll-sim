#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <fstream>

#include "graphics/Renderer.h"
#include "physics/RigidBody.h"
#include "physics/Collision.h"
#include "physics/PhysicsWorld.h"
#include "physics/Ragdoll.h"

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

    PhysicsWorld world;
    world.AddBody(&floor);

    // ラグドールの作成と登録
    Ragdoll ragdoll;
    ragdoll.Build(Vector3{ 0.0f, 10.0f, 0.0f });
    ragdoll.AddToWorld(&world);

    SetTargetFPS(60);

    // ログファイルの準備
    std::ofstream logFile("physics_log.csv");
    if (logFile.is_open()) {
        logFile << "Frame,TargetAngle,CurrentAngle,MotorTorque\n";
    }
    int frameCount = 0;

    while (!WindowShouldClose())
    {
        frameCount++; // フレームカウント進行

        // 物理演算の更新処理
        float deltaTime = GetFrameTime(); // 前のフレームから何秒経過したか（約0.016秒）

        // AI脳のシミュレート（モーターに歩行命令を送る）
        ragdoll.UpdateMotors(GetTime());

        // 物理ワールドの更新
        world.Step(deltaTime);

        // ログの書き込み（左の股関節のデータを記録）
        if (logFile.is_open()) {
            logFile << frameCount << ","
                    << ragdoll.joints[0]->targetAngle << ","
                    << ragdoll.joints[0]->debugCurrentAngle << ","
                    << ragdoll.joints[0]->debugMotorTorque << "\n";
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
                // 剛体の描画
                for (RigidBody* body : world.GetBodies()) {
                    DrawOBB(*body, body->isStatic ? GRAY : RED);
                }
                
                // ジョイントの描画
                for (Joint* joint : world.joints) {
                    Vector3 pA = Vector3Add(joint->bodyA->position, Vector3RotateByQuaternion(joint->localAnchorA, joint->bodyA->orientation));
                    Vector3 pB = Vector3Add(joint->bodyB->position, Vector3RotateByQuaternion(joint->localAnchorB, joint->bodyB->orientation));
                    DrawLine3D(joint->bodyA->position, pA, GREEN);
                    DrawLine3D(joint->bodyB->position, pB, GREEN);
                    DrawSphere(pA, 0.08f, GREEN);
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