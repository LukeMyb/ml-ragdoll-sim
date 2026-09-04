#include "Collision.h"

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

    // 接触マニフォールド（多点衝突）の構築
    // 押し戻し方向と「逆向き」に深く進んでいるAの頂点を「複数」取得する
    float minProj = Vector3DotProduct(verticesA[0], smallestAxis);
    for (int i = 1; i < 8; i++) {
        float proj = Vector3DotProduct(verticesA[i], smallestAxis);
        if (proj < minProj) minProj = proj;
    }

    int cCount = 0;
    float tolerance = 0.01f; // 許容誤差（この範囲内の頂点は全て接触しているとみなす）
    for (int i = 0; i < 8; i++) {
        float proj = Vector3DotProduct(verticesA[i], smallestAxis);
        if (proj <= minProj + tolerance) {
            if (cCount < 4) {
                if (outInfo) outInfo->contactPoints[cCount] = verticesA[i];
                cCount++;
            }
        }
    }

    if (outInfo) {
        outInfo->colliding = true;
        outInfo->normal = smallestAxis;
        outInfo->depth = minOverlap;
        outInfo->contactCount = cCount;
    }
    return true; // すべての軸で重なっていれば衝突
}