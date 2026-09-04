#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <fstream>

#include "physics/RigidBody.h"
#include "physics/Collision.h"

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
    InitWindow(screenWidth, screenHeight, "Impulse & Rotation Bounce");

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

    // 箱の慣性テンソルの逆数を計算 (1 / Ix, Iy, Iz)
    box.localInertiaInverse = Vector3{
        12.0f / (box.mass * (box.size.y * box.size.y + box.size.z * box.size.z)),
        12.0f / (box.mass * (box.size.x * box.size.x + box.size.z * box.size.z)),
        12.0f / (box.mass * (box.size.x * box.size.x + box.size.y * box.size.y))
    };

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

    SetTargetFPS(60);

    // 描画用に衝突情報を保存する変数
    Vector3 debugContactPoint = { 0 };
    bool isCollidingThisFrame = false;

    // ログファイルの準備
    std::ofstream logFile("physics_log.csv");
    if (logFile.is_open()) {
        logFile << "Frame,PosY,VelY,Depth,AngularSpeed\n"; // ヘッダ行
    }
    int frameCount = 0;

    while (!WindowShouldClose())
    {
        frameCount++; // フレームカウント進行

        // 物理演算の更新処理
        float deltaTime = GetFrameTime(); // 前のフレームから何秒経過したか（約0.016秒）

        // 重力ベクトルを作成（地球の重力加速度 -9.8 * 質量）
        Vector3 gravity = Vector3{ 0.0f, -9.8f * box.mass, 0.0f };
        box.ApplyForce(gravity, deltaTime);
        box.Update(deltaTime);

        // SATによる衝突判定と押し戻し（位置の解決）
        CollisionInfo info;
        isCollidingThisFrame = CheckCollisionSAT(box, floor, &info);
        if (isCollidingThisFrame) {
            // 複数の接触点の「平均位置」を計算する
            Vector3 averageContact = { 0 };
            for (int i = 0; i < info.contactCount; i++) {
                averageContact = Vector3Add(averageContact, info.contactPoints[i]);
            }
            averageContact = Vector3Scale(averageContact, 1.0f / (float)info.contactCount);
            debugContactPoint = averageContact; // 平均位置を中心としてインパルスを計算する

            // Baumgarte安定化による位置の押し戻し
            const float slop = 0.01f;     // 微小なめり込みは無視する
            const float percent = 0.2f;
            
            float penetration = info.depth - slop;
            if (penetration < 0.0f) penetration = 0.0f;

            // 位置の押し戻し（めり込み解消）
            Vector3 correction = Vector3Scale(info.normal, penetration * percent);
            box.position = Vector3Add(box.position, correction);
            debugContactPoint = Vector3Add(debugContactPoint, correction); // 青い球も一緒に移動

            // インパルスを用いた物理的な跳ね返り計算
            // 重心から衝突点へのベクトル(r)
            Vector3 r = Vector3Subtract(debugContactPoint, box.position);
            
            // 衝突点での相対速度 = 並進速度 + (角速度 × r)
            Vector3 velocityAtContact = Vector3Add(box.velocity, Vector3CrossProduct(box.angularVelocity, r));
            
            // 法線方向（押し戻される方向）の速度成分
            float relVelAlongNormal = Vector3DotProduct(velocityAtContact, info.normal);
            
            // 互いに近づいている（めり込もうとしている）場合のみ跳ね返り計算を行う
            if (relVelAlongNormal < 0.0f) {
                float restitution = 0.6f; // 反発係数（0.0で弾まない、1.0で完全に弾む）

                // 衝突による回転エネルギーの消失（音や熱への変換変数）
                float spinLoss = 0.95f; // 衝突のたびに回転エネルギーが5%消失する

                // 物理エンジンの定石（Resting Contact）
                // 衝突速度が極めて遅い場合は反発をゼロにし、床での永遠なプルプル振動を防ぐ
                if (relVelAlongNormal > -1.0f) {
                    restitution = 0.0f;
                    spinLoss = 0.95f;
                }
                
                // 力積の分子: -(1 + e) * v_n
                float j_numerator = -(1.0f + restitution) * relVelAlongNormal;
                
                // 力積の分母: 1/m + (I^-1 * (r x n) x r) ・ n
                Vector3 rxn = Vector3CrossProduct(r, info.normal);
                Vector3 invI_rxn = box.ComputeWorldInverseInertia(rxn);
                Vector3 crossTerm = Vector3CrossProduct(invI_rxn, r);
                float j_denominator = (1.0f / box.mass) + Vector3DotProduct(crossTerm, info.normal);
                
                // インパルスの大きさ(j)とベクトル(impulse)
                float j = j_numerator / j_denominator;
                Vector3 impulse = Vector3Scale(info.normal, j);
                
                // 並進速度の更新 (v += J / m)
                box.velocity = Vector3Add(box.velocity, Vector3Scale(impulse, 1.0f / box.mass));
                
                // 角速度の更新 (w += I^-1 * (r x J))
                Vector3 rxJ = Vector3CrossProduct(r, impulse);
                Vector3 angularImpulse = box.ComputeWorldInverseInertia(rxJ);
                box.angularVelocity = Vector3Add(box.angularVelocity, angularImpulse);

                // クーロン摩擦（接線方向のインパルス）
                Vector3 newVelocityAtContact = Vector3Add(box.velocity, Vector3CrossProduct(box.angularVelocity, r));
                float newRelVelAlongNormal = Vector3DotProduct(newVelocityAtContact, info.normal);
                Vector3 tangentVelocity = Vector3Subtract(newVelocityAtContact, Vector3Scale(info.normal, newRelVelAlongNormal));
                float tangentSpeed = Vector3Length(tangentVelocity);
                
                if (tangentSpeed > 0.001f) {
                    Vector3 t = Vector3Scale(tangentVelocity, 1.0f / tangentSpeed);
                    
                    Vector3 rxt = Vector3CrossProduct(r, t);
                    Vector3 invI_rxt = box.ComputeWorldInverseInertia(rxt);
                    Vector3 crossTermT = Vector3CrossProduct(invI_rxt, r);
                    float jt_denominator = (1.0f / box.mass) + Vector3DotProduct(crossTermT, t);
                    
                    float jt = -tangentSpeed / jt_denominator;
                    float mu = 0.5f; 
                    float maxFriction = j * mu;
                    if (jt < -maxFriction) jt = -maxFriction;
                    if (jt > maxFriction) jt = maxFriction;
                    
                    Vector3 frictionImpulse = Vector3Scale(t, jt);
                    box.velocity = Vector3Add(box.velocity, Vector3Scale(frictionImpulse, 1.0f / box.mass));
                    Vector3 rxFriction = Vector3CrossProduct(r, frictionImpulse);
                    Vector3 angularFrictionImpulse = box.ComputeWorldInverseInertia(rxFriction);
                    box.angularVelocity = Vector3Add(box.angularVelocity, angularFrictionImpulse);
                }

                // 衝突によるエネルギー消失を適用
                box.angularVelocity = Vector3Scale(box.angularVelocity, spinLoss);
            }

            // ログの書き込み（衝突している間だけ記録）
            if (logFile.is_open()) {
                logFile << frameCount << ","
                        << box.position.y << ","
                        << box.velocity.y << ","
                        << info.depth << ","
                        << Vector3Length(box.angularVelocity) << "\n";
            }
        }

        // スリープ判定（0.5秒間、速度が極小なら完全静止させる）
        if (!box.isSleeping) {
            // 速度と角速度がどちらも極めて小さい状態かチェック
            if (Vector3Length(box.velocity) < 0.05f && Vector3Length(box.angularVelocity) < 0.05f) {
                box.sleepTimer += deltaTime;
                if (box.sleepTimer > 0.5f) { // 0.5秒間ほぼ止まっていたらスリープ状態へ移行
                    box.isSleeping = true;
                    box.velocity = Vector3{ 0.0f, 0.0f, 0.0f };
                    box.angularVelocity = Vector3{ 0.0f, 0.0f, 0.0f };
                }
            } else {
                box.sleepTimer = 0.0f; // 少しでも動いたらタイマーをリセット
            }
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
                
                // 衝突が発生している間、衝突した角に青い球を描画
                if (isCollidingThisFrame) {
                    DrawSphere(debugContactPoint, 0.15f, BLUE);
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