#pragma once
#include "raymath.h"
#include "RigidBody.h"

// 衝突情報とSAT判定関数
struct CollisionInfo {
    bool colliding;
    Vector3 normal; // 押し戻す方向（法線）
    float depth;    // めり込み量

    Vector3 contactPoints[4];
    int contactCount;
};

// SAT（分離軸定理）によるOBB同士の衝突判定の宣言
bool CheckCollisionSAT(const RigidBody& a, const RigidBody& b, CollisionInfo* outInfo);