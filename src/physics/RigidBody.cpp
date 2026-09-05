#include "RigidBody.h"

// 力を加えて速度を変化させる
void RigidBody::ApplyForce(Vector3 force, float deltaTime) {
    if (isStatic || isSleeping) return;

    Vector3 acceleration = Vector3Scale(force, 1.0f / mass);
    Vector3 deltaVel = Vector3Scale(acceleration, deltaTime);
    velocity = Vector3Add(velocity, deltaVel);
}

// 速度を使って位置を更新する
void RigidBody::Update(float deltaTime) {
    if (isStatic || isSleeping) return;

    Vector3 deltaPos = Vector3Scale(velocity, deltaTime);
    position = Vector3Add(position, deltaPos);

    float angularSpeed = Vector3Length(angularVelocity);
    if (angularSpeed > 0.0f) {
        Vector3 axis = Vector3Scale(angularVelocity, 1.0f / angularSpeed);
        Quaternion deltaRot = QuaternionFromAxisAngle(axis, angularSpeed * deltaTime);
        orientation = QuaternionMultiply(deltaRot, orientation);
        orientation = QuaternionNormalize(orientation);
    }
}

// 現在の姿勢から、箱のローカルX, Y, Z軸の向きを取得する
void RigidBody::GetAxes(Vector3 axes[3]) const {
    axes[0] = Vector3RotateByQuaternion(Vector3{ 1.0f, 0.0f, 0.0f }, orientation);
    axes[1] = Vector3RotateByQuaternion(Vector3{ 0.0f, 1.0f, 0.0f }, orientation);
    axes[2] = Vector3RotateByQuaternion(Vector3{ 0.0f, 0.0f, 1.0f }, orientation);
}

// 箱の8つの頂点のワールド座標を取得する
void RigidBody::GetVertices(Vector3 vertices[8]) const {
    Vector3 axes[3];
    GetAxes(axes);

    float hX = size.x / 2.0f;
    float hY = size.y / 2.0f;
    float hZ = size.z / 2.0f;

    for (int i = 0; i < 8; i++) {
        Vector3 v = position;
        v = Vector3Add(v, Vector3Scale(axes[0], (i & 1) ? hX : -hX));
        v = Vector3Add(v, Vector3Scale(axes[1], (i & 2) ? hY : -hY));
        v = Vector3Add(v, Vector3Scale(axes[2], (i & 4) ? hZ : -hZ));
        vertices[i] = v;
    }
}

// 慣性テンソルのワールド座標変換
Vector3 RigidBody::ComputeWorldInverseInertia(Vector3 v) const {
    Quaternion invOri = QuaternionInvert(orientation);
    Vector3 vLocal = Vector3RotateByQuaternion(v, invOri);
    Vector3 invI_vLocal = {
        vLocal.x * localInertiaInverse.x,
        vLocal.y * localInertiaInverse.y,
        vLocal.z * localInertiaInverse.z
    };
    return Vector3RotateByQuaternion(invI_vLocal, orientation);
}

// お互いの無視リストに登録する
void RigidBody::IgnoreCollisionWith(RigidBody* other) {
    // 自身(this)のリストに相手を追加
    this->ignoredBodies.push_back(other);
    // 相手(other)のリストに自身を追加
    other->ignoredBodies.push_back(this);
}