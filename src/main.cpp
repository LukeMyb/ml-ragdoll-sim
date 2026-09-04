#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

// 物理演算用の構造体
struct RigidBody {
    Vector3 position; // 座標
    Vector3 velocity; // 速度
    float mass;       // 質量
    Vector3 size;     // 箱のサイズ (幅X, 高さY, 奥行Z)

    // 回転用のパラメータ
    Quaternion orientation;  // 姿勢（クォータニオン）
    Vector3 angularVelocity; // 角速度
    
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

        // 回転（姿勢）の更新
        float angularSpeed = Vector3Length(angularVelocity);
        if (angularSpeed > 0.0f) {
            // 回転軸を求める（角速度ベクトルを正規化）
            Vector3 axis = Vector3Scale(angularVelocity, 1.0f / angularSpeed);
            // deltaTimeで進む分の回転（差分クォータニオン）を作成
            Quaternion deltaRot = QuaternionFromAxisAngle(axis, angularSpeed * deltaTime);
            // 現在の姿勢に差分の回転を掛けて新しい姿勢にする
            orientation = QuaternionMultiply(deltaRot, orientation);
            // 誤差が蓄積しないように正規化
            orientation = QuaternionNormalize(orientation);
        }

        // 床 (Y=0) との当たり判定と応答（簡易ペナルティ法）
        float bottomY = position.y - (size.y / 2.0f);
        
        if (bottomY < 0.0f) {
            position.y = 0.0f + (size.y / 2.0f); // 押し戻し
            velocity.y = 0.0f;                   // 速度をゼロに
        }
    }

    // SAT用の計算関数群
    // 現在の姿勢から、箱のローカルX, Y, Z軸の向き（単位ベクトル）を取得する
    void GetAxes(Vector3 axes[3]) const {
        // 基準となる軸を現在のクォータニオンで回転させる
        axes[0] = Vector3RotateByQuaternion(Vector3{ 1.0f, 0.0f, 0.0f }, orientation);
        axes[1] = Vector3RotateByQuaternion(Vector3{ 0.0f, 1.0f, 0.0f }, orientation);
        axes[2] = Vector3RotateByQuaternion(Vector3{ 0.0f, 0.0f, 1.0f }, orientation);
    }

    // 箱の8つの頂点のワールド座標を取得する
    void GetVertices(Vector3 vertices[8]) const {
        Vector3 axes[3];
        GetAxes(axes);

        float hX = size.x / 2.0f;
        float hY = size.y / 2.0f;
        float hZ = size.z / 2.0f;

        // 8つの頂点を計算（X, Y, Zの各軸方向にプラス/マイナスを組み合わせる）
        for (int i = 0; i < 8; i++) {
            Vector3 v = position;
            v = Vector3Add(v, Vector3Scale(axes[0], (i & 1) ? hX : -hX));
            v = Vector3Add(v, Vector3Scale(axes[1], (i & 2) ? hY : -hY));
            v = Vector3Add(v, Vector3Scale(axes[2], (i & 4) ? hZ : -hZ));
            vertices[i] = v;
        }
    }
};

int main(void)
{
    // 画面の初期化 (800x450のウィンドウを作成)
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "OBB Vertices Test");

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
    box.size = Vector3{ 2.0f, 2.0f, 2.0f };      // 箱のサイズ

    // 回転の初期化
    box.orientation = QuaternionIdentity();            // 無回転状態でスタート
    box.angularVelocity = Vector3{ 2.0f, 1.0f, 1.5f }; // テスト用に適当な角速度（回転の勢い）を与える

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
                // 回転を考慮した箱の描画
                rlPushMatrix(); // 現在の描画座標系を保存

                    // 位置を移動
                    rlTranslatef(box.position.x, box.position.y, box.position.z);

                    // 姿勢（クォータニオン）から回転軸と回転角を抽出して回転
                    Vector3 axis;
                    float angle;
                    QuaternionToAxisAngle(box.orientation, &axis, &angle);
                    // rlRotatefは「度（Degree）」で指定するため、RAD2DEG(180/PI)を掛ける
                    rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);

                    // 原点に箱を描画（すでに移動と回転の変換がかかっているため、見た目上は正しい位置・姿勢になる）
                    DrawCube(Vector3{ 0.0f, 0.0f, 0.0f }, box.size.x, box.size.y, box.size.z, RED); 
                    DrawCubeWires(Vector3{ 0.0f, 0.0f, 0.0f }, box.size.x, box.size.y, box.size.z, BLACK);

                rlPopMatrix(); // 描画座標系を元に戻す

                // SAT用頂点計算のデバッグ描画
                Vector3 vertices[8];
                box.GetVertices(vertices);
                for (int i = 0; i < 8; i++) {
                    DrawSphere(vertices[i], 0.1f, GREEN); // 8つの角に緑の球を描画
                }

                DrawGrid(10, 1.0f);
            EndMode3D();

            // 画面の左上にテキストを描画（ここは2D描画）
            DrawText("OBB Vertices Test", 10, 10, 20, DARKGRAY);
            DrawText("Hold RIGHT CLICK to move (WASD/Space/Shift) and look around", 10, 40, 10, DARKGRAY);

        EndDrawing();
    }

    // 終了処理
    CloseWindow();

    return 0;
}