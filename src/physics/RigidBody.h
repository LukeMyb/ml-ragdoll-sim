#pragma once
#include "raymath.h"

// 物理演算用の構造体
struct RigidBody {
    Vector3 position; // 座標
    Vector3 velocity; // 速度
    float mass;       // 質量
    Vector3 size;     // 箱のサイズ (幅X, 高さY, 奥行Z)

    // 回転用のパラメータ
    Quaternion orientation;  // 姿勢（クォータニオン）
    Vector3 angularVelocity; // 角速度

    // 慣性テンソルの逆数（ローカル座標系）
    Vector3 localInertiaInverse;

    bool isStatic = false; // 動かない物体（床など）を判別するフラグ

    // 物理エンジンのスリープ（休止）システム
    bool isSleeping = false;
    float sleepTimer = 0.0f;
    
    // メソッドの宣言のみ
    void ApplyForce(Vector3 force, float deltaTime);
    void Update(float deltaTime);
    void GetAxes(Vector3 axes[3]) const;
    void GetVertices(Vector3 vertices[8]) const;
    Vector3 ComputeWorldInverseInertia(Vector3 v) const;
};