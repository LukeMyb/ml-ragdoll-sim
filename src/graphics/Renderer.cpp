#include "Renderer.h"
#include "raymath.h"
#include "rlgl.h"

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