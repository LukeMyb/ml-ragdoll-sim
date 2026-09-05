#pragma once
#include <vector>
#include "RigidBody.h"
#include "Joint.h"
#include "PhysicsWorld.h"

class Ragdoll {
public:
    std::vector<RigidBody*> bodies;
    std::vector<Joint*> joints;

    Ragdoll();
    ~Ragdoll();

    // 指定した位置を中心に下半身モデルを構築する
    void Build(Vector3 startPosition);

    // 構築したパーツを全て物理ワールドに登録する
    void AddToWorld(PhysicsWorld* world);

    // AIの代わりにモーターを動かす（歩行テスト用）
    void UpdateMotors(float time);

private:
    // パーツ作成を簡略化するヘルパー関数
    RigidBody* CreateBody(Vector3 pos, Vector3 size, float mass);
    Joint* CreateHingeJoint(RigidBody* a, RigidBody* b, Vector3 anchor, float minAng, float maxAng);
};