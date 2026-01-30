// ===============================
// A03 – City Traffic Simulation
// main.cpp (Explore / Orbit / Pan / Zoom)
// ===============================

#define JSON_DIAGNOSTICS 1

#include "../include/modules/Starter.hpp"
#include "../include/modules/TextMaker.hpp"
#include "../include/modules/Scene.hpp"
#include "../include/modules/UBO.hpp"
#include "WVP.hpp"

#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <vector>
#include <string>

// -------------------------------
// TEXT
// -------------------------------
std::vector<SingleText> outText = {
    {1, {"City Traffic Simulation",
         "WASD move | QE up/down | Arrows/Mouse orbit | IJKL pan | R reset | ESC quit",
         "", ""}, 0, 0}
};

struct Vertex {
    glm::vec3 pos;
    glm::vec2 UV;
    glm::vec3 norm;
};

class A03 : public BaseProject {
protected:
    DescriptorSetLayout DSL;
    VertexDescriptor VD;
    Pipeline P;
    Scene SC;
    TextMaker txt;

    float Ar = 4.0f / 3.0f;

    glm::vec3 CamTarget = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 CamPos    = glm::vec3(0.0f, 80.0f, 120.0f);

    float CamYaw   = glm::radians(45.0f);
    float CamPitch = glm::radians(45.0f);
    float CamDist  = 80.0f;
    float CamRoll  = 0.0f;

    float FOVdeg     = 45.0f;
    float zoomSpeed  = 500.0f;
    float orbitSpeed = glm::radians(120.0f);
    float panSpeed   = 120.0f;
    float flySpeed   = 140.0f;

    // collision control
    bool triggerCollision = false;  // set when T is pressed
    bool collisionMode = false; // optional toggle
    float hitDistance = 1.2f;   // how close cars need to be to "hit"
    // =========================
    // TRAFFIC
    // =========================
    struct Route {
        std::vector<glm::vec3> wp;
        std::vector<float> segLen;
        std::vector<float> cumLen;
        float total = 0.0f;
        bool isHorizontalFirst = true; // used for traffic light phase
    };

    struct CarState {
        int inst = -1;
        int routeId = 0;
        float s = 0.0f;        // distance along route [0..total)
        float speed = 10.0f;   // units/sec
        float yawOffset = 0.0f;
        float length = 3.0f;   // approximate car length
    };

    std::vector<CarState> Cars;
    std::vector<Route> Routes;

    float trafficTime = 0.0f;
    float lightPeriod = 10.0f;   // seconds per phase
    float amberTime   = 1.0f;    // last 1s is "yellow" -> we still treat as red for simplicity

    static glm::mat4 MakeCarTRS(const glm::vec3& pos, float yawRad, float yawOffsetRad) {
        glm::mat4 M(1.0f);
        M = glm::translate(M, pos);
        M = glm::rotate(M, yawRad + yawOffsetRad, glm::vec3(0, 1, 0));
        return M;
    }

    static float wrapPos(float s, float total) {
        if (total <= 0.0f) return 0.0f;
        s = std::fmod(s, total);
        if (s < 0.0f) s += total;
        return s;
    }

    static void buildRoute(Route& R) {
        R.segLen.clear();
        R.cumLen.clear();
        R.total = 0.0f;
        int n = (int)R.wp.size();
        R.segLen.resize(n);
        R.cumLen.resize(n + 1);
        R.cumLen[0] = 0.0f;
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            glm::vec3 d = R.wp[j] - R.wp[i];
            float L = glm::length(glm::vec2(d.x, d.z));
            R.segLen[i] = L;
            R.total += L;
            R.cumLen[i + 1] = R.total;
        }
    }

    static glm::vec3 posOnRoute(const Route& R, float s, int* outSeg = nullptr) {
        s = wrapPos(s, R.total);
        int n = (int)R.wp.size();
        int seg = 0;
        while (seg < n - 1 && s >= R.cumLen[seg + 1]) seg++;
        if (seg == n - 1 && s >= R.cumLen[n]) seg = n - 1;

        float segStart = R.cumLen[seg];
        float t = (R.segLen[seg] > 0.0001f) ? (s - segStart) / R.segLen[seg] : 0.0f;

        int j = (seg + 1) % n;
        glm::vec3 a = R.wp[seg];
        glm::vec3 b = R.wp[j];
        glm::vec3 p = a + (b - a) * t;

        if (outSeg) *outSeg = seg;
        return p;
    }

    static float yawAlongRoute(const Route& R, float s) {
        int seg = 0;
        glm::vec3 p = posOnRoute(R, s, &seg);
        int j = (seg + 1) % (int)R.wp.size();
        glm::vec3 b = R.wp[j];
        glm::vec2 dir(b.x - p.x, b.z - p.z);
        if (glm::length(dir) < 0.0001f) return 0.0f;
        dir = glm::normalize(dir);
        return std::atan2(dir.x, dir.y);
    }

    static float distAhead(const Route& R, float sFrom, float sTo) {
        // distance going forward along route from sFrom to sTo
        float a = wrapPos(sFrom, R.total);
        float b = wrapPos(sTo,   R.total);
        float d = b - a;
        if (d < 0.0f) d += R.total;
        return d;
    }

    bool lightGreenForRoute(const Route& R) const {
        // Phase 0: horizontal green, Phase 1: vertical green
        // We'll decide "horizontal" if the first segment is along X more than Z.
        float phaseT = std::fmod(trafficTime, 2.0f * lightPeriod);
        bool horizontalGreen = phaseT < lightPeriod;

        // treat last amberTime as red (more stable)
        float local = std::fmod(trafficTime, lightPeriod);
        bool inAmber = local > (lightPeriod - amberTime);

        bool green = (R.isHorizontalFirst == horizontalGreen) && !inAmber;
        return green;
    }

    void updateCars(float dt) {
        if (Cars.empty()) return;
        trafficTime += dt;

        // Because we want stable velocity per car, we keep it in a member array.
        // Easiest: create it lazily with same size as Cars.
        static std::vector<float> vel;
        if (vel.size() != Cars.size()) {
            vel.assign(Cars.size(), 0.0f);
            for (size_t i = 0; i < Cars.size(); i++) vel[i] = Cars[i].speed;
        }

        if (triggerCollision) {
            triggerCollision = false;  // reset immediately

            if (Cars.size() >= 2) {
                // Pick a route to force collision (e.g., route 0)
                int rid = 0;
                std::vector<int> idxs;
                for (int i = 0; i < (int)Cars.size(); i++)
                    if (Cars[i].routeId == rid)
                        idxs.push_back(i);

                if (idxs.size() >= 2) {
                    // sort cars along route
                    std::sort(idxs.begin(), idxs.end(), [&](int a, int b){
                        return Cars[a].s < Cars[b].s;
                    });

                    int rearIdx  = idxs[0];  // rear car
                    int frontIdx = idxs[1];  // front car

                    // push rear car close to front car
                    Cars[rearIdx].s = wrapPos(Cars[frontIdx].s - 1.5f, Routes[rid].total);

                    // adjust velocities
                    vel[frontIdx] += 8.0f; // front car speeds up
                    vel[rearIdx]  = std::max(0.0f, vel[rearIdx] - 6.0f);

                    // clamp front car speed
                    vel[frontIdx] = glm::min(vel[frontIdx], Cars[frontIdx].speed + 12.0f);
                }
            }
        }


        // Group cars by route
        std::vector<std::vector<int>> byRoute(Routes.size());
        for (int i = 0; i < (int)Cars.size(); i++) byRoute[Cars[i].routeId].push_back(i);

        // For each route: sort by s, then do "follow the leader" + stop at junctions
        for (int rid = 0; rid < (int)Routes.size(); rid++) {
            auto& idxs = byRoute[rid];
            if (idxs.empty()) continue;

            const Route& R = Routes[rid];

            std::sort(idxs.begin(), idxs.end(), [&](int a, int b){
                return Cars[a].s < Cars[b].s;
            });

            bool green = lightGreenForRoute(R);

            // Car-following params
            const float minGap = 2.5f;        // extra gap
            const float lookStop = 6.0f;      // start slowing when closer than this to stopline
            const float maxAccel = 6.0f;      // speed-up rate
            const float maxBrake = 10.0f;     // braking rate

            // For each car, compute desired speed considering the car in front (cyclic)
            for (int k = 0; k < (int)idxs.size(); k++) {
                CarState& c = Cars[idxs[k]];
                CarState& front = Cars[idxs[(k + 1) % idxs.size()]];

                float gap = distAhead(R, c.s, front.s) - front.length;
                float safe = c.length + minGap;

                float targetSpeed = c.speed;

                if (gap < safe) {
                    targetSpeed = 0.0f;
                } else if (gap < safe + 10.0f) {
                    float t = (gap - safe) / 10.0f;
                    targetSpeed = c.speed * glm::clamp(t, 0.0f, 1.0f);
                }

                // Stop at "intersections": we stop at each waypoint (corners) if red.
                // Compute distance to next waypoint ahead
                int seg = 0;
                posOnRoute(R, c.s, &seg);
                float nextWpS = R.cumLen[seg + 1];
                float dToWp = nextWpS - c.s;
                if (dToWp < 0.0f) dToWp += R.total;

                if (!green) {
                    // stop slightly before the waypoint
                    float stopLine = 2.0f;
                    float dToStop = dToWp - stopLine;

                    if (dToStop < lookStop) {
                        float t = glm::clamp(dToStop / lookStop, 0.0f, 1.0f);
                        targetSpeed = std::min(targetSpeed, c.speed * t);
                    }
                    if (dToStop <= 0.2f) {
                        targetSpeed = 0.0f;
                    }
                }

                // Smooth speed change via accel/brake
                float currV = c.speed; // we store base speed here, but we need current velocity separately
                // We'll store current velocity in c.speed? No, keep base speed. Use a static array:
            }

            // We need per-car current velocity; store it in a parallel array (kept across frames)
        }


        for (int rid = 0; rid < (int)Routes.size(); rid++) {
            auto& idxs = byRoute[rid];
            if (idxs.empty()) continue;

            const Route& R = Routes[rid];
            bool green = lightGreenForRoute(R);

            const float minGap = 2.5f;
            const float lookStop = 6.0f;
            const float maxAccel = 6.0f;
            const float maxBrake = 10.0f;
            const float stopLine = 2.0f;

            std::sort(idxs.begin(), idxs.end(), [&](int a, int b){
                return Cars[a].s < Cars[b].s;
            });

            for (int k = 0; k < (int)idxs.size(); k++) {
                int ci = idxs[k];
                int fi = idxs[(k + 1) % idxs.size()];

                CarState& c = Cars[ci];
                CarState& front = Cars[fi];

                float baseMax = c.speed;

                float gap = distAhead(R, c.s, front.s) - front.length;
                float safe = c.length + minGap;

                float desired = baseMax;

                if (gap < safe) {
                    desired = 0.0f;
                } else if (gap < safe + 10.0f) {
                    float t = (gap - safe) / 10.0f;
                    desired = baseMax * glm::clamp(t, 0.0f, 1.0f);
                }

                int seg = 0;
                posOnRoute(R, c.s, &seg);
                float nextWpS = R.cumLen[seg + 1];
                float dToWp = nextWpS - c.s;
                if (dToWp < 0.0f) dToWp += R.total;

                if (!green) {
                    float dToStop = dToWp - stopLine;
                    if (dToStop < lookStop) {
                        float t = glm::clamp(dToStop / lookStop, 0.0f, 1.0f);
                        desired = std::min(desired, baseMax * t);
                    }
                    if (dToStop <= 0.2f) desired = 0.0f;
                }

                float v = vel[ci];
                if (v < desired) v = std::min(desired, v + maxAccel * dt);
                else             v = std::max(desired, v - maxBrake * dt);
                vel[ci] = v;

                c.s = wrapPos(c.s + v * dt, R.total);

                glm::vec3 p = posOnRoute(R, c.s);
                float yaw = yawAlongRoute(R, c.s);

                SC.I[c.inst].Wm = MakeCarTRS(p, yaw, c.yawOffset);
            }
        }
    }

    void initTraffic() {
        Cars.clear();
        Routes.clear();
        trafficTime = 0.0f;
        const float offset = 3.0f;
        // === LANE-CORRECT loops (the ones you said work) ===
        // Inner is around +/-20, Outer around +/-68
        Route innerCW;
        innerCW.wp = { {-20,0.3f,-20}, {20,0.3f,-20}, {20,0.3f,20}, {-20,0.3f,20} };
        innerCW.isHorizontalFirst = true;
        buildRoute(innerCW);

        Route innerCCW = innerCW;
        innerCCW.wp={{-20.0f-offset,0.3f,-20.0f-offset},
            {-20.0f-offset,0.3f,20.0f+offset},
            {20.0f+offset,0.3f,20.0f+offset},
            {20.0f+offset,0.3f,-20.0f-offset}};
        innerCCW.isHorizontalFirst = true;
        buildRoute(innerCCW);

        Route outerCW;
        outerCW.wp = { {-68,0.3f,-68}, {68,0.3f,-68}, {68,0.3f,68}, {-68,0.3f,68} };
        outerCW.isHorizontalFirst = true;
        buildRoute(outerCW);

        Route outerCCW = outerCW;
        outerCCW.wp={{-68.0f-offset,0.3f,-68.0f-offset},
            {-68.0f-offset,0.3f,68.0f+offset},
            {68.0f+offset,0.3f,68.0f+offset},
            {68.0f+offset,0.3f,-68.0f-offset}};
        outerCCW.isHorizontalFirst = true;
        buildRoute(outerCCW);

        Routes.push_back(innerCW);   // routeId 0
        Routes.push_back(innerCCW);  // routeId 1
        Routes.push_back(outerCW);   // routeId 2
        Routes.push_back(outerCCW);  // routeId 3

        float yawOffset = 0.0f; // if cars face wrong, set +90/-90/180

        std::vector<std::string> carModelIds = {
            "aid_truck","ambulance","black_car","blue_jeep","croll_car","fire_truck",
            "green_jeep","limo","monst_car","old_cyan","orange_van","police_car",
            "red_car","RV_van","school_bus","tall_car","taxi","white_conv"
        };

        std::unordered_set<int> carMids;
        for (const auto& mid : carModelIds) {
            auto it = SC.MeshIds.find(mid);
            if (it != SC.MeshIds.end()) carMids.insert(it->second);
        }

        for (int i = 0; i < SC.InstanceCount; i++) {
            if (!carMids.count(SC.I[i].Mid)) continue;

            glm::vec3 p = glm::vec3(SC.I[i].Wm[3]);

            int ring = (std::abs(p.x) > 45.0f || std::abs(p.z) > 45.0f) ? 1 : 0; // 0 inner, 1 outer
            int dir = ((i * 37) % 2); // 0 cw, 1 ccw

            int routeId = (ring == 0) ? (dir == 0 ? 0 : 1) : (dir == 0 ? 2 : 3);
            const Route& R = Routes[routeId];

            CarState c;
            c.inst = i;
            c.routeId = routeId;
            c.yawOffset = yawOffset;

            // base speed variety
            c.speed = 7.0f + float((i * 53) % 70) / 10.0f; // 7..14
            c.length = 3.0f;

            // project car to nearest point (approx): pick closest waypoint segment start via closest s among waypoints
            // simple & stable: find nearest waypoint s and use it as start, plus a stagger
            int bestWp = 0;
            float bestD = 1e30f;
            for (int w = 0; w < (int)R.wp.size(); w++) {
                glm::vec2 d(R.wp[w].x - p.x, R.wp[w].z - p.z);
                float dd = glm::dot(d, d);
                if (dd < bestD) { bestD = dd; bestWp = w; }
            }
            c.s = R.cumLen[bestWp];

            // stagger so they don’t stack at start
            c.s = wrapPos(c.s + float((i * 19) % 40), R.total);

            Cars.push_back(c);
        }
    }

    // -------------------------------
    void setWindowParameters() override {
        windowWidth  = 1280;
        windowHeight = 800;
        windowTitle  = "A03 – City Traffic (Explore)";
        windowResizable = GLFW_TRUE;
        initialBackgroundColor = {0.55f, 0.75f, 0.95f, 1.0f};

        uniformBlocksInPool = 8000;
        texturesInPool      = 1024;
        setsInPool          = 8000;

        Ar = float(windowWidth) / float(windowHeight);
    }

    void onWindowResize(int w, int h) override {
        Ar = (h > 0) ? float(w) / float(h) : Ar;
    }

    void localInit() override {
        DSL.init(this, {
            {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL_GRAPHICS},
            {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT},
            {2, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL_GRAPHICS}
        });

        VD.init(this,
            {{0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX}},
            {
                {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos),  sizeof(glm::vec3), POSITION},
                {0, 1, VK_FORMAT_R32G32_SFLOAT,    offsetof(Vertex, UV),   sizeof(glm::vec2), UV},
                {0, 2, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, norm), sizeof(glm::vec3), NORMAL}
            }
        );

        P.init(this, &VD,
               "shaders/PhongVert.spv",
               "shaders/PhongFrag.spv",
               {&DSL});
        P.setAdvancedFeatures(VK_COMPARE_OP_LESS_OR_EQUAL,
                              VK_POLYGON_MODE_FILL,
                              VK_CULL_MODE_NONE,
                              false);

        SC.init(this, &VD, DSL, P, "assets/models/city_scene.json");

        if (SC.InstanceIds.count("prm")) {
            int iprm = SC.InstanceIds["prm"];
            CamTarget = glm::vec3(SC.I[iprm].Wm[3]);
        } else if (SC.InstanceCount > 0) {
            CamTarget = glm::vec3(SC.I[0].Wm[3]);
        }

        CamYaw   = glm::radians(45.0f);
        CamPitch = glm::radians(50.0f);
        CamDist  = 80.0f;

        initTraffic();

        txt.init(this, &outText);
    }

    void pipelinesAndDescriptorSetsInit() override {
        P.create();
        SC.pipelinesAndDescriptorSetsInit(DSL);
        txt.pipelinesAndDescriptorSetsInit();
    }

    void pipelinesAndDescriptorSetsCleanup() override {
        P.cleanup();
        SC.pipelinesAndDescriptorSetsCleanup();
        txt.pipelinesAndDescriptorSetsCleanup();
    }

    void localCleanup() override {
        DSL.cleanup();
        P.destroy();
        SC.localCleanup();
        txt.localCleanup();
    }

    void populateCommandBuffer(VkCommandBuffer cmd, int currentImage) override {
        P.bind(cmd);
        SC.populateCommandBuffer(cmd, currentImage, P);
        txt.populateCommandBuffer(cmd, currentImage, 0);
    }

    void cameraBasis(glm::vec3 &forward, glm::vec3 &right, glm::vec3 &up) {
        forward = glm::normalize(glm::vec3(
            cos(CamPitch) * sin(CamYaw),
            sin(CamPitch),
            cos(CamPitch) * cos(CamYaw)
        ));
        right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
        up    = glm::normalize(glm::cross(right, forward));
    }

    void updateUniformBuffer(uint32_t currentImage) override {
        float deltaT;
        glm::vec3 m(0.0f), r(0.0f);
        bool fire = false;
        getSixAxis(deltaT, m, r, fire);
        //accident
        static bool tWasDown = false;
        bool tDown = glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS;

        if (tDown && !tWasDown) {
            triggerCollision = true;  // set flag for updateCars
        }
        tWasDown = tDown;



        updateCars(deltaT);

        if (glfwGetKey(window, GLFW_KEY_LEFT))  CamYaw  -= orbitSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_RIGHT)) CamYaw  += orbitSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_UP))    CamPitch += orbitSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_DOWN))  CamPitch -= orbitSpeed * deltaT;

        CamYaw   += orbitSpeed * deltaT * r.y;
        CamPitch -= orbitSpeed * deltaT * r.x;

        CamPitch = glm::clamp(CamPitch, glm::radians(8.0f), glm::radians(85.0f));

        CamDist -= zoomSpeed * deltaT * m.y;
        if (glfwGetKey(window, GLFW_KEY_Q)) CamDist -= zoomSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_E)) CamDist += zoomSpeed * deltaT;

        CamDist = glm::clamp(CamDist, 3.0f, 2000.0f);

        glm::vec3 forward, right, up;
        cameraBasis(forward, right, up);
        glm::vec3 groundF = glm::normalize(glm::vec3(forward.x, 0.0f, forward.z));
        glm::vec3 groundR = glm::normalize(glm::vec3(right.x,   0.0f, right.z));

        if (glfwGetKey(window, GLFW_KEY_I)) CamTarget += groundF * panSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_K)) CamTarget -= groundF * panSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_J)) CamTarget -= groundR * panSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_L)) CamTarget += groundR * panSpeed * deltaT;

        if (glfwGetKey(window, GLFW_KEY_S)) CamTarget += groundF * flySpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_W)) CamTarget -= groundF * flySpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_D)) CamTarget -= groundR * flySpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_A)) CamTarget += groundR * flySpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_Z)) CamTarget.y -= flySpeed * 0.8f * deltaT;
        if (glfwGetKey(window, GLFW_KEY_X)) CamTarget.y += flySpeed * 0.8f * deltaT;

        if (glfwGetKey(window, GLFW_KEY_R)) {
            if (SC.InstanceIds.count("prm")) {
                int iprm = SC.InstanceIds["prm"];
                CamTarget = glm::vec3(SC.I[iprm].Wm[3]);
            } else {
                CamTarget = glm::vec3(0, 0, 0);
            }
            CamYaw   = glm::radians(45.0f);
            CamPitch = glm::radians(50.0f);
            CamDist  = 80.0f;
        }

        glm::vec3 offset;
        offset.x = CamDist * cos(CamPitch) * sin(CamYaw);
        offset.y = CamDist * sin(CamPitch);
        offset.z = CamDist * cos(CamPitch) * cos(CamYaw);
        CamPos = CamTarget + offset;

        glm::mat4 VP = MakeViewProjectionLookAt(
            CamPos,
            CamTarget,
            glm::vec3(0, 1, 0),
            CamRoll,
            glm::radians(FOVdeg),
            Ar,
            0.1f,
            4000.0f
        );

        GlobalUniformBufferObject gubo{};
        gubo.lightDir   = glm::normalize(glm::vec3(-1.0f, -1.0f, -0.5f));
        gubo.lightColor = glm::vec4(1, 1, 1, 1);
        gubo.eyePos     = CamPos;
        gubo.eyeDir     = glm::vec4(glm::normalize(CamTarget - CamPos), 1.0f);

        GlobalUniformBufferObject guboLocal = gubo;
        UniformBufferObject ubo{};

        // ===== UPDATE ANIMATION TIME FOR ALL INSTANCES =====
        std::unordered_set<int> updatedSkinnedModels;
        auto sampleTrack = [](const AnimationTrack& track, float time) -> AnimationKeyframe {
            AnimationKeyframe out{};
            out.translation = glm::vec3(0.0f);
            out.rotation = glm::quat(1, 0, 0, 0);
            out.scale = glm::vec3(1.0f);

            if (track.keyframes.empty()) return out;

            int idx0 = 0;
            for (int i = 0; i < (int)track.keyframes.size() - 1; i++) {
                if (track.keyframes[i].time <= time && time <= track.keyframes[i + 1].time) {
                    idx0 = i;
                    break;
                }
            }
            const auto& kf0 = track.keyframes[idx0];
            const auto& kf1 = track.keyframes[std::min(idx0 + 1, (int)track.keyframes.size() - 1)];
            float t = 0.0f;
            if (kf1.time > kf0.time) {
                t = (time - kf0.time) / (kf1.time - kf0.time);
            }
            out.translation = glm::mix(kf0.translation, kf1.translation, t);
            out.rotation = glm::slerp(kf0.rotation, kf1.rotation, t);
            out.scale = glm::mix(kf0.scale, kf1.scale, t);
            return out;
        };

        for (int i = 0; i < SC.InstanceCount; i++) {
            // Check if this instance has an animated model
            int modelIdx = SC.I[i].Mid;
            if (modelIdx >= 0 && modelIdx < SC.ModelCount && 
                !SC.M[modelIdx]->animations.empty()) {
				
                // Increment animation time
                SC.I[i].animTime += deltaT;
				
                // Loop animation
                float duration = SC.M[modelIdx]->animations[0].duration;
                if (SC.I[i].animTime > duration) {
                    SC.I[i].animTime = std::fmod(SC.I[i].animTime, duration);
                }

                // CPU skinning update (only once per model per frame)
                Model* model = SC.M[modelIdx];
                if (model->hasSkinning && !updatedSkinnedModels.count(modelIdx)) {
                    const auto& clip = model->animations[0];
                    const int nodeCount = (int)model->nodeParents.size();
                    if (nodeCount > 0 &&
                        model->nodeBaseTranslation.size() == (size_t)nodeCount &&
                        model->nodeBaseRotation.size() == (size_t)nodeCount &&
                        model->nodeBaseScale.size() == (size_t)nodeCount) {

                        std::vector<glm::vec3> t = model->nodeBaseTranslation;
                        std::vector<glm::quat> r = model->nodeBaseRotation;
                        std::vector<glm::vec3> s = model->nodeBaseScale;

                        for (const auto& track : clip.tracks) {
                            int node = track.jointIndex;
                            if (node < 0 || node >= nodeCount) continue;
                            AnimationKeyframe kf = sampleTrack(track, SC.I[i].animTime);
                            switch (track.path) {
                                case AnimationTrack::Path::Translation:
                                    t[node] = kf.translation;
                                    break;
                                case AnimationTrack::Path::Rotation:
                                    r[node] = kf.rotation;
                                    break;
                                case AnimationTrack::Path::Scale:
                                    s[node] = kf.scale;
                                    break;
                            }
                        }

                        std::vector<glm::mat4> local(nodeCount);
                        for (int n = 0; n < nodeCount; n++) {
                            glm::mat4 M(1.0f);
                            M = glm::translate(M, t[n]);
                            M = M * glm::mat4_cast(r[n]);
                            M = glm::scale(M, s[n]);
                            local[n] = M;
                        }

                        std::vector<glm::mat4> global(nodeCount, glm::mat4(1.0f));
                        std::vector<char> computed(nodeCount, 0);
                        auto computeGlobal = [&](auto&& self, int idx) -> glm::mat4 {
                            if (computed[idx]) return global[idx];
                            int parent = model->nodeParents[idx];
                            if (parent >= 0 && parent < nodeCount) {
                                global[idx] = self(self, parent) * local[idx];
                            } else {
                                global[idx] = local[idx];
                            }
                            computed[idx] = 1;
                            return global[idx];
                        };
                        for (int n = 0; n < nodeCount; n++) {
                            computeGlobal(computeGlobal, n);
                        }

                        const size_t jointCount = model->skinData.jointIndices.size();
                        std::vector<glm::mat4> jointMatrices(jointCount, glm::mat4(1.0f));
                        for (size_t j = 0; j < jointCount; j++) {
                            int node = model->skinData.jointIndices[j];
                            if (node < 0 || node >= nodeCount) continue;
                            if (j < model->skinData.inverseBindMatrices.size()) {
                                jointMatrices[j] = global[node] * model->skinData.inverseBindMatrices[j];
                            }
                        }

                        model->updateSkinnedVertices(jointMatrices);
                        updatedSkinnedModels.insert(modelIdx);
                    }
                }

                static bool logged = false;
                if (!logged) {
                    std::cout << "[Animation] Model " << modelIdx << " animation time: " 
                              << SC.I[i].animTime << "s / " << duration << "s\n";
                    logged = true;
                }
            }
        }

        for (int i = 0; i < SC.InstanceCount; i++) {
            ubo.mMat   = SC.I[i].Wm;
            ubo.mvpMat = VP * ubo.mMat;
            ubo.nMat   = glm::inverse(glm::transpose(ubo.mMat));

            SC.DS[i]->map(currentImage, &ubo, sizeof(ubo), 0);
            SC.DS[i]->map(currentImage, &guboLocal, sizeof(guboLocal), 2);
        }

        if (glfwGetKey(window, GLFW_KEY_ESCAPE)) {
            glfwSetWindowShouldClose(window, GL_TRUE);
        }
    }
};

int main() {
    A03 app;
    try {
        app.run();
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
