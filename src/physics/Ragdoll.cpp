#include "Ragdoll.h"

Ragdoll::Ragdoll() {}

Ragdoll::~Ragdoll() {
    // 動的に確保したメモリを解放
    for (RigidBody* body : bodies) delete body;
    for (Joint* joint : joints) delete joint;
}

RigidBody* Ragdoll::CreateBody(Vector3 pos, Vector3 size, float mass) {
    RigidBody* body = new RigidBody();
    body->position = pos;
    body->velocity = Vector3{ 0.0f, 0.0f, 0.0f };
    body->mass = mass;
    body->size = size;
    body->orientation = QuaternionIdentity();
    body->angularVelocity = Vector3{ 0.0f, 0.0f, 0.0f };
    body->isStatic = false;
    
    // 慣性テンソルの逆数を自動計算
    body->localInertiaInverse = Vector3{
        12.0f / (mass * (size.y * size.y + size.z * size.z)),
        12.0f / (mass * (size.x * size.x + size.z * size.z)),
        12.0f / (mass * (size.x * size.x + size.y * size.y))
    };
    
    bodies.push_back(body);
    return body;
}

Joint* Ragdoll::CreateHingeJoint(RigidBody* a, RigidBody* b, Vector3 anchor, float minAng, float maxAng) {
    Joint* joint = new Joint();
    joint->Init(a, b, anchor);
    joint->isHinge = true;
    joint->minAngle = minAng;
    joint->maxAngle = maxAng;
    joint->motorEnabled = true; // デフォルトでモーターON
    joint->motorP = 150.0f;
    joint->motorD = 15.0f;
    joints.push_back(joint);
    return joint;
}

void Ragdoll::Build(Vector3 startPosition) {
    // --- 剛体の作成 ---
    // 胴体（Torso）: 高めの位置に固定してぶら下げる（後で固定は解除します）
    RigidBody* torso = CreateBody(startPosition, Vector3{ 1.0f, 1.5f, 0.5f }, 2.0f);
    torso->isStatic = true; // テストのため最初は空中に固定

    // 左脚（太もも・すね）
    RigidBody* leftThigh = CreateBody(Vector3Add(startPosition, Vector3{-0.3f, -1.5f, 0.0f}), Vector3{0.4f, 1.5f, 0.4f}, 1.0f);
    RigidBody* leftShin  = CreateBody(Vector3Add(startPosition, Vector3{-0.3f, -3.0f, 0.0f}), Vector3{0.3f, 1.5f, 0.3f}, 1.0f);

    // 右脚（太もも・すね）
    RigidBody* rightThigh = CreateBody(Vector3Add(startPosition, Vector3{ 0.3f, -1.5f, 0.0f}), Vector3{0.4f, 1.5f, 0.4f}, 1.0f);
    RigidBody* rightShin  = CreateBody(Vector3Add(startPosition, Vector3{ 0.3f, -3.0f, 0.0f}), Vector3{0.3f, 1.5f, 0.3f}, 1.0f);

    // --- 関節（ジョイント）の作成 ---
    // 股関節（-45度〜45度）
    CreateHingeJoint(torso, leftThigh,  Vector3Add(startPosition, Vector3{-0.3f, -0.75f, 0.0f}), -45.0f, 45.0f);
    CreateHingeJoint(torso, rightThigh, Vector3Add(startPosition, Vector3{ 0.3f, -0.75f, 0.0f}), -45.0f, 45.0f);

    // 膝関節（0度〜90度：後ろにしか曲がらないようにする）
    CreateHingeJoint(leftThigh,  leftShin,  Vector3Add(startPosition, Vector3{-0.3f, -2.25f, 0.0f}), 0.0f, 90.0f);
    CreateHingeJoint(rightThigh, rightShin, Vector3Add(startPosition, Vector3{ 0.3f, -2.25f, 0.0f}), 0.0f, 90.0f);

    // 左右の脚同士の衝突（セルフコリジョン）を無視する
    leftThigh->IgnoreCollisionWith(rightThigh);
    leftShin->IgnoreCollisionWith(rightShin);
    leftThigh->IgnoreCollisionWith(rightShin);
    rightThigh->IgnoreCollisionWith(leftShin);
}

void Ragdoll::AddToWorld(PhysicsWorld* world) {
    for (RigidBody* body : bodies) world->AddBody(body);
    for (Joint* joint : joints)    world->AddJoint(joint);
}

void Ragdoll::UpdateMotors(float time) {
    // 歩行を模倣するサイン波（左脚と右脚でタイミングをズラす）
    float leftPhase = time * 5.0f;
    float rightPhase = time * 5.0f + PI; // 右脚は半周期ズラす

    // 股関節のモーター指示（0番目＝左股、1番目＝右股）
    joints[0]->targetAngle = sinf(leftPhase) * 45.0f;
    joints[1]->targetAngle = sinf(rightPhase) * 45.0f;

    // 膝関節のモーター指示（2番目＝左膝、3番目＝右膝）
    // 膝は常に少し曲がった状態（45度中心）で前後に動かす
    joints[2]->targetAngle = 45.0f + sinf(leftPhase) * 45.0f;
    joints[3]->targetAngle = 45.0f + sinf(rightPhase) * 45.0f;
}