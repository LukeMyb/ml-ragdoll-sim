#pragma once
#include <vector>
#include "RigidBody.h"
#include "Collision.h"
#include "Joint.h"

class PhysicsWorld {
public:
    Vector3 gravity = { 0.0f, -9.8f, 0.0f }; // 全体に影響する重力
    std::vector<RigidBody*> bodies;          // 空間に存在する剛体のリスト
    std::vector<Joint*> joints;              // 空間に存在するジョイントのリスト

    // デバッグ用の接触点保持
    std::vector<Vector3> debugContactPoints;

    // 剛体を空間に登録する
    void AddBody(RigidBody* body);

    // ジョイントを空間に登録する
    void AddJoint(Joint* joint);

    // 1フレーム分の物理演算を進める
    void Step(float deltaTime);

    // 登録された剛体を取得
    const std::vector<RigidBody*>& GetBodies() const { return bodies; }

private:
    // aとbの双方が動くことを考慮した汎用的な衝突解決
    void ResolveCollision(RigidBody* a, RigidBody* b, const CollisionInfo& info);

    // ジョイントの拘束（離れないようにする）を解決する関数
    void ResolveJoints(float deltaTime);
};