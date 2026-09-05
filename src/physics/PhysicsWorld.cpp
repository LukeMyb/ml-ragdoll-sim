#include "PhysicsWorld.h"

void PhysicsWorld::AddBody(RigidBody* body) {
    bodies.push_back(body);
}

// ジョイントの登録処理
void PhysicsWorld::AddJoint(Joint* joint) {
    joints.push_back(joint);
}

void PhysicsWorld::Step(float deltaTime) {
    debugContactPoints.clear();

    // 重力の適用と位置・姿勢の更新
    for (RigidBody* body : bodies) {
        if (!body->isStatic && !body->isSleeping) {
            Vector3 force = Vector3Scale(gravity, body->mass);
            body->ApplyForce(force, deltaTime);
        }
        body->Update(deltaTime);
    }

    // 衝突判定と解決（総当たり）
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* a = bodies[i];
            RigidBody* b = bodies[j];

            // 両方とも固定物、またはスリープ中の場合は判定スキップ
            if ((a->isStatic || a->isSleeping) && (b->isStatic || b->isSleeping)) continue;

            // 衝突除外リストに含まれているかチェック
            bool shouldIgnore = false;
            for (RigidBody* ignored : a->ignoredBodies) {
                if (ignored == b) {
                    shouldIgnore = true;
                    break;
                }
            }
            if (shouldIgnore) continue; // 無視リストに入っていればSAT判定自体を行わない

            CollisionInfo info;
            if (CheckCollisionSAT(*a, *b, &info)) {
                ResolveCollision(a, b, info);
                
                // 衝突が発生したらスリープを解除
                a->isSleeping = false;
                b->isSleeping = false;
            }
        }
    }

    // ジョイントの解決（繋ぎ止める処理）
    ResolveJoints(deltaTime);

    // スリープ判定
    for (RigidBody* body : bodies) {
        if (body->isStatic) continue;

        if (!body->isSleeping) {
            if (Vector3Length(body->velocity) < 0.05f && Vector3Length(body->angularVelocity) < 0.05f) {
                body->sleepTimer += deltaTime;
                if (body->sleepTimer > 0.5f) {
                    body->isSleeping = true;
                    body->velocity = Vector3{ 0.0f, 0.0f, 0.0f };
                    body->angularVelocity = Vector3{ 0.0f, 0.0f, 0.0f };
                }
            } else {
                body->sleepTimer = 0.0f;
            }
        }
    }
}

void PhysicsWorld::ResolveCollision(RigidBody* a, RigidBody* b, const CollisionInfo& info) {
    float invMassA = a->isStatic ? 0.0f : (1.0f / a->mass);
    float invMassB = b->isStatic ? 0.0f : (1.0f / b->mass);
    float totalInvMass = invMassA + invMassB;

    if (totalInvMass <= 0.0f) return; // 両方固定物なら何もしない

    // Baumgarte安定化 (めり込みの押し戻し)
    const float slop = 0.01f;
    const float percent = 0.2f;
    float penetration = info.depth - slop;
    if (penetration < 0.0f) penetration = 0.0f;

    // 質量の比率に応じて両方のオブジェクトを押し戻す
    Vector3 correction = Vector3Scale(info.normal, (penetration * percent) / totalInvMass);
    if (!a->isStatic) a->position = Vector3Add(a->position, Vector3Scale(correction, invMassA));
    if (!b->isStatic) b->position = Vector3Subtract(b->position, Vector3Scale(correction, invMassB));

    // 接触点の平均位置
    Vector3 averageContact = { 0 };
    for (int i = 0; i < info.contactCount; i++) {
        averageContact = Vector3Add(averageContact, info.contactPoints[i]);
    }
    averageContact = Vector3Scale(averageContact, 1.0f / (float)info.contactCount);
    debugContactPoints.push_back(averageContact);

    // 反発のインパルス計算
    Vector3 rA = Vector3Subtract(averageContact, a->position);
    Vector3 rB = Vector3Subtract(averageContact, b->position);

    Vector3 velA = Vector3Add(a->velocity, Vector3CrossProduct(a->angularVelocity, rA));
    Vector3 velB = Vector3Add(b->velocity, Vector3CrossProduct(b->angularVelocity, rB));
    Vector3 relativeVel = Vector3Subtract(velA, velB);

    float relVelAlongNormal = Vector3DotProduct(relativeVel, info.normal);
    if (relVelAlongNormal > 0.0f) return; // 離れていく場合は処理しない

    float restitution = 0.6f;
    float spinLoss = 0.95f;
    if (relVelAlongNormal > -1.0f) {
        restitution = 0.0f; // 極小速度なら反発しない
    }

    float j_numerator = -(1.0f + restitution) * relVelAlongNormal;

    Vector3 rxn_A = Vector3CrossProduct(rA, info.normal);
    Vector3 invI_rxn_A = a->isStatic ? Vector3{0,0,0} : a->ComputeWorldInverseInertia(rxn_A);
    Vector3 crossTerm_A = Vector3CrossProduct(invI_rxn_A, rA);

    Vector3 rxn_B = Vector3CrossProduct(rB, info.normal);
    Vector3 invI_rxn_B = b->isStatic ? Vector3{0,0,0} : b->ComputeWorldInverseInertia(rxn_B);
    Vector3 crossTerm_B = Vector3CrossProduct(invI_rxn_B, rB);

    float j_denominator = totalInvMass 
        + Vector3DotProduct(crossTerm_A, info.normal) 
        + Vector3DotProduct(crossTerm_B, info.normal);

    float j = j_numerator / j_denominator;
    Vector3 impulse = Vector3Scale(info.normal, j);

    if (!a->isStatic) {
        a->velocity = Vector3Add(a->velocity, Vector3Scale(impulse, invMassA));
        Vector3 rxJ_A = Vector3CrossProduct(rA, impulse);
        a->angularVelocity = Vector3Add(a->angularVelocity, a->ComputeWorldInverseInertia(rxJ_A));
        a->angularVelocity = Vector3Scale(a->angularVelocity, spinLoss);
    }
    if (!b->isStatic) {
        b->velocity = Vector3Subtract(b->velocity, Vector3Scale(impulse, invMassB));
        // bに対するインパルスは逆向き（-impulse）なので、外積の結果を反転させる
        Vector3 rxJ_B_neg = Vector3Scale(Vector3CrossProduct(rB, impulse), -1.0f);
        b->angularVelocity = Vector3Add(b->angularVelocity, b->ComputeWorldInverseInertia(rxJ_B_neg));
        b->angularVelocity = Vector3Scale(b->angularVelocity, spinLoss);
    }

    // クーロン摩擦（双方向対応）
    velA = Vector3Add(a->velocity, Vector3CrossProduct(a->angularVelocity, rA));
    velB = Vector3Add(b->velocity, Vector3CrossProduct(b->angularVelocity, rB));
    relativeVel = Vector3Subtract(velA, velB);

    float newRelVelAlongNormal = Vector3DotProduct(relativeVel, info.normal);
    Vector3 tangentVelocity = Vector3Subtract(relativeVel, Vector3Scale(info.normal, newRelVelAlongNormal));
    float tangentSpeed = Vector3Length(tangentVelocity);

    if (tangentSpeed > 0.001f) {
        Vector3 t = Vector3Scale(tangentVelocity, 1.0f / tangentSpeed);

        Vector3 rxt_A = Vector3CrossProduct(rA, t);
        Vector3 invI_rxt_A = a->isStatic ? Vector3{0,0,0} : a->ComputeWorldInverseInertia(rxt_A);
        Vector3 crossTermT_A = Vector3CrossProduct(invI_rxt_A, rA);

        Vector3 rxt_B = Vector3CrossProduct(rB, t);
        Vector3 invI_rxt_B = b->isStatic ? Vector3{0,0,0} : b->ComputeWorldInverseInertia(rxt_B);
        Vector3 crossTermT_B = Vector3CrossProduct(invI_rxt_B, rB);

        float jt_denominator = totalInvMass 
            + Vector3DotProduct(crossTermT_A, t) 
            + Vector3DotProduct(crossTermT_B, t);

        float jt = -tangentSpeed / jt_denominator;
        float mu = 0.5f; 
        float maxFriction = j * mu;
        if (jt < -maxFriction) jt = -maxFriction;
        if (jt > maxFriction) jt = maxFriction;

        Vector3 frictionImpulse = Vector3Scale(t, jt);

        if (!a->isStatic) {
            a->velocity = Vector3Add(a->velocity, Vector3Scale(frictionImpulse, invMassA));
            Vector3 rxFriction_A = Vector3CrossProduct(rA, frictionImpulse);
            a->angularVelocity = Vector3Add(a->angularVelocity, a->ComputeWorldInverseInertia(rxFriction_A));
        }
        if (!b->isStatic) {
            b->velocity = Vector3Subtract(b->velocity, Vector3Scale(frictionImpulse, invMassB));
            Vector3 rxFriction_B_neg = Vector3Scale(Vector3CrossProduct(rB, frictionImpulse), -1.0f);
            b->angularVelocity = Vector3Add(b->angularVelocity, b->ComputeWorldInverseInertia(rxFriction_B_neg));
        }
    }
}

// ジョイントの拘束を解決する実装
void PhysicsWorld::ResolveJoints(float deltaTime) {
    for (Joint* joint : joints) {
        RigidBody* a = joint->bodyA;
        RigidBody* b = joint->bodyB;

        // ローカルのアンカー位置を、現在の傾きに合わせてワールド座標のベクトルに変換
        Vector3 rA = Vector3RotateByQuaternion(joint->localAnchorA, a->orientation);
        Vector3 rB = Vector3RotateByQuaternion(joint->localAnchorB, b->orientation);

        // アンカーの実際のワールド座標
        Vector3 pA = Vector3Add(a->position, rA);
        Vector3 pB = Vector3Add(b->position, rB);

        // アンカー間のズレ
        Vector3 diff = Vector3Subtract(pB, pA);

        float invMassA = a->isStatic ? 0.0f : (1.0f / a->mass);
        float invMassB = b->isStatic ? 0.0f : (1.0f / b->mass);
        float totalInvMass = invMassA + invMassB;

        if (totalInvMass <= 0.0f) continue;

        // --- 位置の直接補正 (Baumgarte安定化) ---
        // めり込み解消と同じ理屈で、ズレた分だけ強制的に座標を寄せる
        const float percent = 0.2f; 
        Vector3 correction = Vector3Scale(diff, percent / totalInvMass);
        if (!a->isStatic) a->position = Vector3Add(a->position, Vector3Scale(correction, invMassA));
        if (!b->isStatic) b->position = Vector3Subtract(b->position, Vector3Scale(correction, invMassB));

        // --- 速度の補正 (インパルス拘束) ---
        // 衝突の摩擦と同じ理屈で、X, Y, Zの3軸についてアンカー間の相対速度をゼロにする
        Vector3 axes[3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
        for (int i = 0; i < 3; i++) {
            Vector3 axis = axes[i];

            // アンカーポイントでのそれぞれの速度
            Vector3 velA = Vector3Add(a->velocity, Vector3CrossProduct(a->angularVelocity, rA));
            Vector3 velB = Vector3Add(b->velocity, Vector3CrossProduct(b->angularVelocity, rB));
            Vector3 relVel = Vector3Subtract(velA, velB);

            // 注目している軸方向の速度差
            float relVelAlongAxis = Vector3DotProduct(relVel, axis);

            // 速度差を打ち消すための力積（インパルス）の分母計算
            Vector3 rxn_A = Vector3CrossProduct(rA, axis);
            Vector3 invI_rxn_A = a->isStatic ? Vector3{0,0,0} : a->ComputeWorldInverseInertia(rxn_A);
            Vector3 crossTerm_A = Vector3CrossProduct(invI_rxn_A, rA);

            Vector3 rxn_B = Vector3CrossProduct(rB, axis);
            Vector3 invI_rxn_B = b->isStatic ? Vector3{0,0,0} : b->ComputeWorldInverseInertia(rxn_B);
            Vector3 crossTerm_B = Vector3CrossProduct(invI_rxn_B, rB);

            float j_denominator = totalInvMass 
                + Vector3DotProduct(crossTerm_A, axis) 
                + Vector3DotProduct(crossTerm_B, axis);

            if (j_denominator > 0.0f) {
                // 速度差をちょうど0にする力積
                float j = -relVelAlongAxis / j_denominator;
                Vector3 impulse = Vector3Scale(axis, j);

                // 力積の適用（速度と角速度の変化）
                if (!a->isStatic) {
                    a->velocity = Vector3Add(a->velocity, Vector3Scale(impulse, invMassA));
                    Vector3 rxJ_A = Vector3CrossProduct(rA, impulse);
                    a->angularVelocity = Vector3Add(a->angularVelocity, a->ComputeWorldInverseInertia(rxJ_A));
                }
                if (!b->isStatic) {
                    b->velocity = Vector3Subtract(b->velocity, Vector3Scale(impulse, invMassB));
                    Vector3 rxJ_B_neg = Vector3Scale(Vector3CrossProduct(rB, impulse), -1.0f);
                    b->angularVelocity = Vector3Add(b->angularVelocity, b->ComputeWorldInverseInertia(rxJ_B_neg));
                }
            }
        }

        // --- 回転拘束（ヒンジ制限） ---
        if (joint->isHinge) {
            // AとBのローカル軸（X, Y, Z）がワールド空間でどちらを向いているか取得
            Vector3 aX = Vector3RotateByQuaternion(Vector3{1,0,0}, a->orientation);
            Vector3 aY = Vector3RotateByQuaternion(Vector3{0,1,0}, a->orientation);
            Vector3 aZ = Vector3RotateByQuaternion(Vector3{0,0,1}, a->orientation);

            Vector3 bX = Vector3RotateByQuaternion(Vector3{1,0,0}, b->orientation);
            Vector3 bY = Vector3RotateByQuaternion(Vector3{0,1,0}, b->orientation);
            Vector3 bZ = Vector3RotateByQuaternion(Vector3{0,0,1}, b->orientation);

            // ヒンジ軸(Z軸)のズレを直す
            // AのZ軸とBのZ軸が常に平行になるように（横に捻じれないように）押し戻す
            Vector3 alignAxis = Vector3CrossProduct(aZ, bZ); 
            Vector3 alignCorrection = Vector3Scale(alignAxis, 0.2f / totalInvMass);
            if (!a->isStatic) a->angularVelocity = Vector3Add(a->angularVelocity, Vector3Scale(alignCorrection, invMassA));
            if (!b->isStatic) b->angularVelocity = Vector3Subtract(b->angularVelocity, Vector3Scale(alignCorrection, invMassB));

            // Z軸周りの曲がり角度を制限する
            // AのXY平面から見て、BのY軸がどれくらい傾いているかを計算 (atan2を使用)
            float dotY = Vector3DotProduct(bY, aY);
            float dotX = Vector3DotProduct(bY, aX);
            float currentAngle = atan2f(dotX, dotY) * RAD2DEG; // 現在の曲がり角度(度)

            // 制限角度を超えていたら、押し戻すための角度(補正量)を計算
            float correctionAngle = 0.0f;
            if (currentAngle < joint->minAngle) correctionAngle = joint->minAngle - currentAngle;
            if (currentAngle > joint->maxAngle) correctionAngle = joint->maxAngle - currentAngle;

            // 角度の限界を超えていた場合、Z軸を軸にして回転力を加える
            if (correctionAngle != 0.0f) {
                Vector3 limitCorrection = Vector3Scale(aZ, (correctionAngle * DEG2RAD * 0.2f) / totalInvMass);
                if (!a->isStatic) a->angularVelocity = Vector3Subtract(a->angularVelocity, Vector3Scale(limitCorrection, invMassA));
                if (!b->isStatic) b->angularVelocity = Vector3Add(b->angularVelocity, Vector3Scale(limitCorrection, invMassB));
            }
        }
    }
}