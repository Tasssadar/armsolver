#include "RBControl_arm.hpp"
#include <math.h>

#include <QDebug>

namespace rb {

template<typename T> T Arm::roundCoord(Arm::AngleType val) {
    return T(round(val));
}

template<> float Arm::roundCoord(Arm::AngleType val) { return float(val); }
template<> double Arm::roundCoord(Arm::AngleType val) { return double(val); }

ArmBuilder::ArmBuilder() {

}

ArmBuilder::~ArmBuilder() {

}


ArmBuilder& ArmBuilder::body(Arm::CoordType height_mm, Arm::CoordType radius_mm) {
    m_def.body_height = height_mm;
    m_def.body_radius = radius_mm;
    return *this;
}

ArmBuilder& ArmBuilder::armOffset(Arm::CoordType x_mm, Arm::CoordType y_mm) {
    m_def.arm_offset_x = x_mm;
    m_def.arm_offset_y = y_mm;
    return *this;
}

BoneBuilder ArmBuilder::bone(Arm::CoordType length_mm) {
    std::shared_ptr<Arm::BoneDefinition> bone(new Arm::BoneDefinition);
    bone->length = length_mm;
    m_bones.push_back(bone);
    return BoneBuilder(bone);
}

std::unique_ptr<Arm> ArmBuilder::build() {
    m_def.bones.reserve(m_bones.size());
    for(auto bone : m_bones) {
        m_def.bones.push_back(*bone);
    }
    m_bones.clear();
    return std::unique_ptr<Arm>(new Arm(m_def));
}

BoneBuilder::BoneBuilder(std::shared_ptr<Arm::BoneDefinition> bone) : m_def(bone) {

}

BoneBuilder::BoneBuilder(BoneBuilder&& other) {
    m_def = std::move(other.m_def);
}

BoneBuilder::~BoneBuilder() {

}

BoneBuilder& BoneBuilder::relStops(Arm::AngleType min_rad, Arm::AngleType max_rad) {
    m_def->rel_min = min_rad;
    m_def->rel_max = max_rad;
    return *this;
}

BoneBuilder& BoneBuilder::absStops(Arm::AngleType min_rad, Arm::AngleType max_rad) {
    m_def->abs_min = min_rad;
    m_def->abs_max = max_rad;
    return *this;
}

BoneBuilder& BoneBuilder::baseRelStops(Arm::AngleType min_rad, Arm::AngleType max_rad) {
    m_def->base_rel_min = min_rad;
    m_def->base_rel_max = max_rad;
    return *this;
}

BoneBuilder& BoneBuilder::calcServoAng(std::function<Arm::AngleType(Arm::AngleType)> func) {
    m_def->calcServoAng = func;
    return *this;
}

void Bone::updatePos(Bone *prev) {
    if(prev != nullptr) {
        angle = Arm::clampAng(prev->angle + relAngle);
    } else {
        angle = relAngle;
    }

    x = Arm::roundCoord(cos(angle) * def.length);
    y = Arm::roundCoord(sin(angle) * def.length);
    if(prev != nullptr) {
        x += prev->x;
        y += prev->y;
    }
}

Arm::AngleType Arm::clampAng(Arm::AngleType val) {
   val = fmod(val, M_PI*2);
   if(val < -M_PI)
        val += M_PI*2;
   else if(val > M_PI)
        val -= M_PI*2;
   return val;
}

Arm::Arm(const Arm::Definition& def) : m_def(def) {
    m_bones.reserve(m_def.bones.size());
    for(const auto& def : m_def.bones) {
        m_bones.push_back(Bone(def));
    }
}

Arm::~Arm() {

}

bool Arm::solve(Arm::CoordType target_x, Arm::CoordType target_y) {
    bool modified = false;
    for(size_t i = 0; i < 10; ++i) {
        if(solveIteration(target_x, target_y, modified))
            return true;
        if(!modified)
            break;
    }

    return false;
}

bool Arm::solveIteration(Arm::CoordType target_x, Arm::CoordType target_y, bool& modified) {
    Bone *prev = nullptr;
    for(size_t i = 0; i < m_bones.size(); ++i) {
        m_bones[i].updatePos(prev);
        prev = &m_bones[i];
    }

    if(target_x < m_def.body_radius - m_def.arm_offset_x) {
        target_y = std::min(target_y, m_def.arm_offset_y);
    } else {
        target_y = std::min(target_y, CoordType(m_def.arm_offset_y + m_def.body_height));
    }

    auto end_x = m_bones.back().x;
    auto end_y = m_bones.back().y;
    CoordType bx, by;
    modified = false;
    for(int32_t i = int32_t(m_bones.size())-1; i >= 0; --i) {
        if(i == 0) {
            bx = by = 0;
        } else {
            bx = m_bones[i-1].x;
            by = m_bones[i-1].y;
        }

        // Get the vector from the current bone to the end effector position.
        AngleType to_end_x = end_x - bx;
        AngleType to_end_y = end_y - by;
        AngleType to_end_mag = sqrt(to_end_x*to_end_x + to_end_y*to_end_y);

        // Get the vector from the current bone to the target position.
        AngleType to_target_x = target_x - bx;
        AngleType to_target_y = target_y - by;
        AngleType to_target_mag = sqrt(to_target_x*to_target_x + to_target_y*to_target_y);

        // Get rotation to place the end effector on the line from the current
        // joint position to the target postion.
        AngleType cos_rot_ang, sin_rot_ang;
        AngleType end_target_mag = to_end_mag * to_target_mag;

        if(end_target_mag <= 0.0001) {
            cos_rot_ang = 1;
            sin_rot_ang = 0;
        } else {
            cos_rot_ang = (to_end_x*to_target_x + to_end_y*to_target_y) / end_target_mag;
            sin_rot_ang = (to_end_x*to_target_y - to_end_y*to_target_x) / end_target_mag;
        }

        // Clamp the cosine into range when computing the angle (might be out of range
        // due to floating point error).
        AngleType rot_ang = acos(std::max(AngleType(-1), std::min(AngleType(1), cos_rot_ang)));
        if(sin_rot_ang < 0)
            rot_ang = -rot_ang;

        // Rotate the current bone in local space (this value is output to the user)
        rot_ang = rotateArm(i, rot_ang);
        cos_rot_ang = cos(rot_ang);
        sin_rot_ang = sin(rot_ang);

        // Rotate the end effector position.
        end_x = roundCoord(bx + cos_rot_ang*to_end_x - sin_rot_ang*to_end_y);
        end_y = roundCoord(by + sin_rot_ang*to_end_x + cos_rot_ang*to_end_y);

        // Check for termintation
        auto dist_x = target_x - end_x;
        auto dist_y = target_y - end_y;
        if(dist_x*dist_x + dist_y*dist_y <= 100) {
            return true;
        }

        modified = modified || abs(rot_ang)*to_end_mag > 0.000001;
    }
    return false;
}

Arm::AngleType Arm::rotateArm(size_t idx, Arm::AngleType rot_ang) {
    auto& me = m_bones[idx];
    auto& base = m_bones[0];

    AngleType new_rel_ang = clampAng(me.relAngle + rot_ang);
    new_rel_ang = std::max(me.def.rel_min, std::min(me.def.rel_max, new_rel_ang));

    CoordType x = 0;
    CoordType y = 0;
    AngleType prev_ang = 0;
    for(size_t i = 0; i < m_bones.size(); ++i) {
        auto& b = m_bones[i];
        auto angle = b.relAngle;
        if(i == idx) {
            angle = new_rel_ang;
        }
        angle = clampAng(prev_ang + angle);

        if(i == idx) {
            if(angle < b.def.abs_min) {
                angle = b.def.abs_min;
                new_rel_ang = clampAng(angle - prev_ang);
            } else if(angle > b.def.abs_max){
                angle = b.def.abs_max;
                new_rel_ang = clampAng(angle - prev_ang);
            }
        }

        auto nx = roundCoord(x + (cos(angle) * b.def.length));
        auto ny = roundCoord(y + (sin(angle) * b.def.length));

        if(nx < m_def.body_radius - m_def.arm_offset_x) {
            if(ny > m_def.arm_offset_y)
                return 0;
        } else {
            if(ny > m_def.arm_offset_y + m_def.body_height)
                return 0;
        }

        if(i > 0) {
            if(angle - base.angle < b.def.base_rel_min) {
                base.angle = clampAng(angle - b.def.base_rel_min);
            } else if(angle - base.angle > b.def.base_rel_max) {
                base.angle = clampAng(angle - b.def.base_rel_max);
            }
        }

        x = nx;
        y = ny;
        prev_ang = angle;
    }

    auto res = clampAng(new_rel_ang - me.relAngle);
    me.relAngle = new_rel_ang;
    return res;
}

}; // namespace rb
