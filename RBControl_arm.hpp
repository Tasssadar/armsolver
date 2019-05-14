#ifndef RBCONTROL_ARM_HPP
#define RBCONTROL_ARM_HPP

#include <vector>
#include <stdint.h>
#include <functional>
#include <memory>
#include <math.h>

namespace rb {

class ArmBuilder;
class Bone;

class Arm
{
    friend class ArmBuilder;
    friend class BoneBuilder;
    friend class Bone;
public:
    typedef float AngleType;

    struct BoneDefinition {
        BoneDefinition() {
            length = 0;
            rel_min = abs_min = base_rel_min = -M_PI;
            rel_max = abs_max = base_rel_max = M_PI;
            calcServoAng = [](AngleType angle) -> AngleType { return angle; };
        }

        uint16_t length;
        AngleType rel_min, rel_max;
        AngleType abs_min, abs_max;
        AngleType base_rel_min, base_rel_max;

        std::function<AngleType(AngleType)> calcServoAng;
    };

    struct Definition {
        Definition() {
            body_height = 0;
            body_radius = 0;

            arm_offset_x = 0;
            arm_offset_y = 0;
        }

        uint16_t body_height;
        uint16_t body_radius;

        uint16_t arm_offset_x;
        uint16_t arm_offset_y;

        std::vector<BoneDefinition> bones;
    };

    static AngleType clampAng(AngleType ang);
    static AngleType deg(AngleType rad) { return rad * (180.0/M_PI); }

    ~Arm();

    bool solve(int32_t target_x, int32_t target_y);

    const Definition& definition() const { return m_def; }
    const std::vector<Bone>& bones() const { return m_bones; }

private:
    Arm(const Definition &def);

    bool solveIteration(int32_t target_x, int32_t target_y, bool& modified);
    AngleType rotateArm(size_t idx, AngleType rot_ang);

    const Definition m_def;
    std::vector<Bone> m_bones;
};

class Bone {
    friend class Arm;
public:
    const Arm::BoneDefinition& def;

    Arm::AngleType relAngle;

    int32_t x, y;
    Arm::AngleType angle;

    Arm::AngleType servoAng() const { return def.calcServoAng(angle); }

private:
    Bone(const Arm::BoneDefinition& def) : def(def) {
        relAngle = -M_PI/2;
        x = y = 0;
        angle = 0;
    }

    void updatePos(Bone *prev);
};

class ArmBuilder;

class BoneBuilder {
    friend class ArmBuilder;
public:
    ~BoneBuilder();

    BoneBuilder& relStops(Arm::AngleType min_rad, Arm::AngleType max_rad);
    BoneBuilder& absStops(Arm::AngleType min_rad, Arm::AngleType max_rad);
    BoneBuilder& baseRelStops(Arm::AngleType min_rad, Arm::AngleType max_rad);
    BoneBuilder& calcServoAng(std::function<Arm::AngleType(Arm::AngleType)> func);

private:
    BoneBuilder(std::shared_ptr<Arm::BoneDefinition> def);
    BoneBuilder(BoneBuilder&& other);

    std::shared_ptr<Arm::BoneDefinition> m_def;
};

class ArmBuilder {
public:
    ArmBuilder();
    ~ArmBuilder();

    ArmBuilder& body(uint16_t height_mm, uint16_t radius_mm);
    ArmBuilder& armOffset(uint16_t x_mm, uint16_t y_mm);
    BoneBuilder bone(uint16_t length_mm);

    std::unique_ptr<Arm> build();

private:
    Arm::Definition m_def;
    std::vector<std::shared_ptr<Arm::BoneDefinition>> m_bones;
};

}; // namespace rb

#endif // RBCONTROL_ARM_HPP
