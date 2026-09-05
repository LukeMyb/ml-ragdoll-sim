#pragma once
#include "raymath.h"
#include "RigidBody.h"

// 2つの剛体を繋ぐ関節（ジョイント）のデータ構造
struct Joint {
    RigidBody* bodyA;
    RigidBody* bodyB;

    // それぞれの剛体の「ローカル座標系」における接続点（アンカー）
    Vector3 localAnchorA;
    Vector3 localAnchorB;

    // ヒンジジョイントの設定
    bool isHinge = false;    // trueならZ軸をヒンジ軸として振る舞う
    float minAngle = -45.0f; // 最小角度（度）
    float maxAngle = 45.0f;  // 最大角度（度）

    // モーター（PD制御）の設定
    bool motorEnabled = false;
    float targetAngle = 0.0f; // 目指す角度（度）
    float motorP = 150.0f;    // Pゲイン（目標に向かうバネの強さ）
    float motorD = 15.0f;     // Dゲイン（行き過ぎを防ぐブレーキの強さ）

    // ログ出力・デバッグ用の変数
    float debugCurrentAngle = 0.0f;
    float debugMotorTorque = 0.0f;

    // ジョイントの初期化と接続位置の設定
    void Init(RigidBody* a, RigidBody* b, Vector3 anchorWorld) {
        bodyA = a;
        bodyB = b;

        // 指定されたワールド座標(anchorWorld)が、各箱の中心から見てどこにあるかを計算し、
        // さらに現在の傾き（クォータニオンの逆回転）を加味してローカル座標を求める
        Quaternion invOriA = QuaternionInvert(a->orientation);
        Vector3 diffA = Vector3Subtract(anchorWorld, a->position);
        localAnchorA = Vector3RotateByQuaternion(diffA, invOriA);

        Quaternion invOriB = QuaternionInvert(b->orientation);
        Vector3 diffB = Vector3Subtract(anchorWorld, b->position);
        localAnchorB = Vector3RotateByQuaternion(diffB, invOriB);

        // ジョイントで繋がった剛体同士は、自動的に衝突判定を除外する
        a->IgnoreCollisionWith(b);
    }
};