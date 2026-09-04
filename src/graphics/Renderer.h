#pragma once
#include "raylib.h"
// 相対パスでphysicsディレクトリのRigidBodyを読み込む
#include "../physics/RigidBody.h"

// 描画処理をスッキリさせるためのヘルパー関数
void DrawOBB(const RigidBody& rb, Color color);