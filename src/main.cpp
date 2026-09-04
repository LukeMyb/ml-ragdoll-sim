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

    bool isStatic = false; // 動かない物体（床など）を判別するフラグ
    
    // 力を加えて速度を変化させる
    void ApplyForce(Vector3 force) {
        if (isStatic) return; // 固定物は力を無視する

        // 加速度 = 力 / 質量 (a = F / m)
        Vector3 acceleration = Vector3Scale(force, 1.0f / mass);
        // 速度 = 速度 + 加速度
        velocity = Vector3Add(velocity, acceleration);
    }

    // 速度を使って位置を更新する（1フレームごとの処理）
    void Update(float deltaTime) {
        if (isStatic) return; // 固定物は動かない

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

// 衝突情報とSAT判定関数
struct CollisionInfo {
    bool colliding;
    Vector3 normal; // 押し戻す方向（法線）
    float depth;    // めり込み量
};

// SAT（分離軸定理）によるOBB同士の衝突判定
bool CheckCollisionSAT(const RigidBody& a, const RigidBody& b, CollisionInfo* outInfo) {
    Vector3 axesA[3], axesB[3];
    a.GetAxes(axesA);
    b.GetAxes(axesB);

    // 15本の判定軸を作成（Aの3軸、Bの3軸、AとBの軸の外積9本）
    Vector3 testAxes[15];
    int axisCount = 0;
    
    for (int i = 0; i < 3; i++) testAxes[axisCount++] = axesA[i];
    for (int i = 0; i < 3; i++) testAxes[axisCount++] = axesB[i];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Vector3 cross = Vector3CrossProduct(axesA[i], axesB[j]);
            // 軸が平行で外積が0になる場合は除外
            if (Vector3LengthSqr(cross) > 1e-6f) {
                testAxes[axisCount++] = Vector3Normalize(cross);
            }
        }
    }

    Vector3 verticesA[8], verticesB[8];
    a.GetVertices(verticesA);
    b.GetVertices(verticesB);

    float minOverlap = 999999.0f;
    Vector3 smallestAxis = { 0 };

    // 15本すべての軸に投影して重なりをチェック
    for (int i = 0; i < axisCount; i++) {
        Vector3 axis = testAxes[i];
        float minA = 999999.0f, maxA = -999999.0f;
        float minB = 999999.0f, maxB = -999999.0f;

        // Box A の投影
        for (int v = 0; v < 8; v++) {
            float proj = Vector3DotProduct(verticesA[v], axis);
            if (proj < minA) minA = proj;
            if (proj > maxA) maxA = proj;
        }
        // Box B の投影
        for (int v = 0; v < 8; v++) {
            float proj = Vector3DotProduct(verticesB[v], axis);
            if (proj < minB) minB = proj;
            if (proj > maxB) maxB = proj;
        }

        // 重なりがない軸が1つでもあれば衝突していない
        if (maxA < minB || maxB < minA) return false; 

        // 重なりの深さを計算
        float overlap = (maxA < maxB ? maxA : maxB) - (minA > minB ? minA : minB);
        if (overlap < minOverlap) {
            minOverlap = overlap;
            smallestAxis = axis;
        }
    }

    // 押し戻しベクトルが B から A に向かうように向きを統一
    Vector3 dirBA = Vector3Subtract(a.position, b.position);
    if (Vector3DotProduct(smallestAxis, dirBA) < 0.0f) {
        smallestAxis = Vector3Scale(smallestAxis, -1.0f);
    }

    if (outInfo) {
        outInfo->colliding = true;
        outInfo->normal = smallestAxis;
        outInfo->depth = minOverlap;
    }
    return true; // すべての軸で重なっていれば衝突
}

// 描画処理をスッキリさせるためのヘルパー関数
void DrawOBB(const RigidBody& rb, Color color) {
    rlPushMatrix();
        rlTranslatef(rb.position.x, rb.position.y, rb.position.z);
        Vector3 axis;
        float angle;
        QuaternionToAxisAngle(rb.orientation, &axis, &angle);
        rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
        DrawCube(Vector3{ 0.0f, 0.0f, 0.0f }, rb.size.x, rb.size.y, rb.size.z, color); 
        DrawCubeWires(Vector3{ 0.0f, 0.0f, 0.0f }, rb.size.x, rb.size.y, rb.size.z, BLACK);
    rlPopMatrix();
}

int main(void)
{
    // 画面の初期化 (800x450のウィンドウを作成)
    const int screenWidth = 800;
    const int screenHeight = 450;
    InitWindow(screenWidth, screenHeight, "SAT Collision (Position Resolution)");

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
    box.orientation = QuaternionIdentity();            // 無回転状態でスタート
    box.angularVelocity = Vector3{ 2.0f, 1.0f, 1.5f }; // テスト用に適当な角速度（回転の勢い）を与える
    box.isStatic = false;

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

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        // 物理演算の更新処理
        float deltaTime = GetFrameTime(); // 前のフレームから何秒経過したか（約0.016秒）

        // 重力ベクトルを作成（地球の重力加速度 -9.8 * 質量）
        Vector3 gravity = Vector3{ 0.0f, -9.8f * box.mass, 0.0f };
        box.ApplyForce(gravity); 
        box.Update(deltaTime);

        // SATによる衝突判定と押し戻し（位置の解決）
        CollisionInfo info;
        if (CheckCollisionSAT(box, floor, &info)) {
            // めり込んだ分だけ、法線方向に箱を押し戻す
            Vector3 correction = Vector3Scale(info.normal, info.depth);
            box.position = Vector3Add(box.position, correction);

            // まだ跳ね返り(インパルス)の計算をしていないため、
            // 押し戻しが成功したかを視覚的に確認するために速度をゼロにして空中にピタッと止める
            box.velocity = Vector3{ 0.0f, 0.0f, 0.0f };
            box.angularVelocity = Vector3{ 0.0f, 0.0f, 0.0f };
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
                // ヘルパー関数で箱と床を描画
                DrawOBB(box, RED);
                DrawOBB(floor, GRAY); // 床をグレーの箱として描画
                
                // デバッグ用の緑の球
                Vector3 vertices[8];
                box.GetVertices(vertices);
                for (int i = 0; i < 8; i++) DrawSphere(vertices[i], 0.1f, GREEN);
            EndMode3D();

            // 画面の左上にテキストを描画（ここは2D描画）
            DrawText("SAT Collision (Position Resolution)", 10, 10, 20, DARKGRAY);
            DrawText("Hold RIGHT CLICK to move (WASD/Space/Shift) and look around", 10, 40, 10, DARKGRAY);

        EndDrawing();
    }

    // 終了処理
    CloseWindow();

    return 0;
}