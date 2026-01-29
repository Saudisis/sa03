#include <glm/glm.hpp>
#include <cmath>

// --- helpers (avoid extra GLM transform headers) ---
static inline glm::vec3 NormalizeSafe(const glm::vec3& v) {
    float l = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (l < 1e-8f) return glm::vec3(0.0f, 0.0f, 0.0f);
    return v / l;
}

// Rodrigues' rotation formula: rotate vector v around unit axis k by angle a
static inline glm::vec3 RotateAroundAxis(const glm::vec3& v, const glm::vec3& axisUnit, float a) {
    float c = std::cos(a);
    float s = std::sin(a);
    return v * c + glm::cross(axisUnit, v) * s + axisUnit * (glm::dot(axisUnit, v) * (1.0f - c));
}

static inline glm::mat4 RotX(float a) {
    float c = std::cos(a), s = std::sin(a);
    glm::mat4 R(1.0f);
    R[1][1] = c;  R[2][1] = -s;
    R[1][2] = s;  R[2][2] =  c;
    return R;
}

static inline glm::mat4 RotY(float a) {
    float c = std::cos(a), s = std::sin(a);
    glm::mat4 R(1.0f);
    R[0][0] =  c; R[2][0] = s;
    R[0][2] = -s; R[2][2] = c;
    return R;
}

static inline glm::mat4 RotZ(float a) {
    float c = std::cos(a), s = std::sin(a);
    glm::mat4 R(1.0f);
    R[0][0] = c;  R[1][0] = -s;
    R[0][1] = s;  R[1][1] =  c;
    return R;
}

// Right-handed perspective (OpenGL-style) without relying on glm::perspective
static inline glm::mat4 PerspectiveRH(float fovy, float aspect, float zn, float zf) {
    float f = 1.0f / std::tan(fovy * 0.5f);
    glm::mat4 P(0.0f);
    P[0][0] = f / aspect;
    P[1][1] = f;
    P[2][2] = (zf + zn) / (zn - zf);
    P[2][3] = -1.0f;
    P[3][2] = (2.0f * zf * zn) / (zn - zf);
    return P;
}

// Build a standard RH view matrix from camera basis (r,u,f) and position.
// f is the *forward direction in world* (where camera looks).
static inline glm::mat4 ViewFromBasisRH(const glm::vec3& r, const glm::vec3& u, const glm::vec3& f, const glm::vec3& pos) {
    // In RH lookAt, camera forward maps to -Z in view space.
    glm::vec3 fn = NormalizeSafe(f);
    glm::vec3 rn = NormalizeSafe(r);
    glm::vec3 un = NormalizeSafe(u);

    glm::mat4 V(1.0f);
    // Column-major in GLM:
    V[0][0] = rn.x; V[1][0] = rn.y; V[2][0] = rn.z; V[3][0] = -glm::dot(rn, pos);
    V[0][1] = un.x; V[1][1] = un.y; V[2][1] = un.z; V[3][1] = -glm::dot(un, pos);
    V[0][2] = -fn.x; V[1][2] = -fn.y; V[2][2] = -fn.z; V[3][2] =  glm::dot(fn, pos);
    V[0][3] = 0.0f; V[1][3] = 0.0f; V[2][3] = 0.0f; V[3][3] = 1.0f;
    return V;
}

glm::mat4 MakeViewProjectionLookInDirection(glm::vec3 Pos, float Yaw, float Pitch, float Roll,
                                            float FOVy, float Ar, float nearPlane, float farPlane) {
    // Convention: yaw=0,pitch=0 looks towards -Z.
    float cy = std::cos(Yaw),  sy = std::sin(Yaw);
    float cp = std::cos(Pitch), sp = std::sin(Pitch);

    glm::vec3 forward = NormalizeSafe(glm::vec3(cp * sy, sp, -cp * cy)); // -Z when yaw=0
    glm::vec3 worldUp(0.0f, 1.0f, 0.0f);

    glm::vec3 right = NormalizeSafe(glm::cross(forward, worldUp));
    glm::vec3 up    = NormalizeSafe(glm::cross(right, forward));

    // Apply roll about the forward axis
    if (std::fabs(Roll) > 1e-8f) {
        right = RotateAroundAxis(right, forward, Roll);
        up    = RotateAroundAxis(up,    forward, Roll);
    }

    glm::mat4 V = ViewFromBasisRH(right, up, forward, Pos);
    glm::mat4 P = PerspectiveRH(FOVy, Ar, nearPlane, farPlane);

    // FIX: If your pipeline is Vulkan/D3D-style, you need this Y flip
    P[1][1] *= -1.0f;

    return P * V;
}

glm::mat4 MakeViewProjectionLookAt(glm::vec3 Pos, glm::vec3 Target, glm::vec3 Up, float Roll,
                                   float FOVy, float Ar, float nearPlane, float farPlane) {
    glm::vec3 forward = NormalizeSafe(Target - Pos);
    if (glm::dot(forward, forward) < 1e-8f) forward = glm::vec3(0.0f, 0.0f, -1.0f);

    glm::vec3 upRef = NormalizeSafe(Up);
    if (glm::dot(upRef, upRef) < 1e-8f) upRef = glm::vec3(0.0f, 1.0f, 0.0f);

    glm::vec3 right = NormalizeSafe(glm::cross(forward, upRef));
    // If Up was collinear with forward, pick a fallback
    if (glm::dot(right, right) < 1e-8f) {
        glm::vec3 fallback(0.0f, 1.0f, 0.0f);
        if (std::fabs(glm::dot(fallback, forward)) > 0.99f) fallback = glm::vec3(1.0f, 0.0f, 0.0f);
        right = NormalizeSafe(glm::cross(forward, fallback));
    }
    glm::vec3 up = NormalizeSafe(glm::cross(right, forward));

    // Apply roll about the forward axis
    if (std::fabs(Roll) > 1e-8f) {
        right = RotateAroundAxis(right, forward, Roll);
        up    = RotateAroundAxis(up,    forward, Roll);
    }

    glm::mat4 V = ViewFromBasisRH(right, up, forward, Pos);
    glm::mat4 P = PerspectiveRH(FOVy, Ar, nearPlane, farPlane);

    // FIX: If your pipeline is Vulkan/D3D-style, you need this Y flip
    P[1][1] *= -1.0f;

    return P * V;
}

glm::mat4 MakeWorld(glm::vec3 Pos, float Yaw, float Pitch, float Roll) {
    // zxy convention: roll(Z), pitch(X), yaw(Y)
    // Apply in order: roll -> pitch -> yaw  => R = Ryaw * Rpitch * Rroll
    glm::mat4 R = RotY(Yaw) * RotX(Pitch) * RotZ(Roll);

    glm::mat4 T(1.0f);
    T[3][0] = Pos.x;
    T[3][1] = Pos.y;
    T[3][2] = Pos.z;

    return T * R;
}
