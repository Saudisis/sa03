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
#include <iostream>
#include <filesystem>

#ifdef _WIN32
#include <windows.h>
#endif

// --------------------------------------------------
// PATH UTILITIES (WINDOWS – RELATIVE TO EXECUTABLE)
// --------------------------------------------------
static std::filesystem::path GetExeDir() {
#ifdef _WIN32
    wchar_t buffer[MAX_PATH];
    DWORD len = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    if (len == 0 || len == MAX_PATH) {
        return std::filesystem::current_path(); // fallback
    }
    return std::filesystem::path(buffer).parent_path();
#else
    return std::filesystem::current_path();
#endif
}

static std::string PathRelToExe(const std::string& rel) {
    return (GetExeDir() / rel).generic_string();
}

// -------------------------------
// TEXT
// -------------------------------
std::vector<SingleText> outText = {
    {1, {"City Traffic Simulation - [TAB] Change Mode",
         "🖱️  Right Click: Rotate | Wheel: Zoom | Middle: Pan",
         "⌨️  WASD: Move | QE: Height | Shift: Fast | Ctrl: Slow | R: Reset",
         ""}, 0, 0}
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

    // Camera modes
    enum CameraMode { ORBIT, FREE, BIRD };
    CameraMode cameraMode = ORBIT;
    const char* cameraModeNames[3] = {"Orbit", "Free", "Bird"};

    // Smooth transitions
    glm::vec3 smoothCamPos = CamPos;
    glm::vec3 smoothCamTarget = CamTarget;
    float smoothYaw = CamYaw;
    float smoothPitch = CamPitch;
    float smoothDist = CamDist;
    float smoothFactor = 8.0f; // Higher = more responsive

    // Mouse control
    double lastMouseX = 0.0;
    double lastMouseY = 0.0;
    bool rightMousePressed = false;
    bool middleMousePressed = false;
    float mouseSensitivity = 0.003f;
    float mouseZoomSpeed = 10.0f;

    // collision control
    bool collisionMode = false; // optional toggle
    float hitDistance = 1.2f;   // how close cars need to be to "hit"

    //traffic light
    int trafficstate = 0; // 0=RED, 1=YELLOW, 2=GREEN

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

    // ===========================
    // PEDESTRIAN
    // ===========================
    struct PedestrianState {
        int instId = -1;        // instance index
        Route walkRoute;        // walking path
        float s = 0.0f;         // distance along route [0..total)
        float speed = 1.8f;     // walking speed (units/sec)
    };

    std::vector<CarState> Cars;
    std::vector<Route> Routes;
    std::vector<PedestrianState> Pedestrians;

    float trafficTime = 0.0f;
    float lightPeriod = 10.0f;   // seconds per phase
    float amberTime   = 2.0f;    // last 1s is "yellow"

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
        float a = wrapPos(sFrom, R.total);
        float b = wrapPos(sTo,   R.total);
        float d = b - a;
        if (d < 0.0f) d += R.total;
        return d;
    }

    bool lightGreenForRoute(const Route& R) const {
        float phaseT = std::fmod(trafficTime, 2.0f * lightPeriod);
        bool horizontalGreen = phaseT < lightPeriod;

        float local = std::fmod(trafficTime, lightPeriod);
        bool inAmber = local > (lightPeriod - amberTime);

        bool green = (R.isHorizontalFirst == horizontalGreen) && !inAmber;
        return green;
    }

    void updateCars(float dt) {
        if (Cars.empty()) return;
        trafficTime += dt;
        int colroute = 1;

        static std::vector<float> vel;
        if (vel.size() != Cars.size()) {
            vel.assign(Cars.size(), 0.0f);
            for (size_t i = 0; i < Cars.size(); i++) vel[i] = Cars[i].speed;
        }

        std::vector<std::vector<int>> byRoute(Routes.size());
        for (int i = 0; i < (int)Cars.size(); i++) byRoute[Cars[i].routeId].push_back(i);

        for (int rid = 0; rid < (int)Routes.size(); rid++) {
            auto& idxs = byRoute[rid];
            if (idxs.empty()) continue;

            const Route& R = Routes[rid];
            bool green = (trafficstate != 0);

            const float minGap = 2.5f;
            const float lookStop = 6.0f;
            const float maxAccel = 6.0f;
            const float maxBrake = 10.0f;
            const float stopLine = 2.0f;

            std::sort(idxs.begin(), idxs.end(), [&](int a, int b){
                return Cars[a].s < Cars[b].s;
            });

            if (rid==colroute && collisionMode) continue;

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
                    desired = 0.0f;
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

        // crush logic
        if (collisionMode) {
            auto& idxscrush = byRoute[colroute];
            if (idxscrush.size() >= 2) {
                std::sort(idxscrush.begin(), idxscrush.end(), [&](int a, int b){
                    return Cars[a].s < Cars[b].s;
                });

                int rearIdx = idxscrush[0];
                int frontIdx = idxscrush[1];
                CarState& rear = Cars[rearIdx];
                CarState& front = Cars[frontIdx];
                const Route& R = Routes[colroute];

                float gap = distAhead(R, rear.s, front.s) - front.length;

                float desiredRear = rear.speed;
                float desiredFront = front.speed;

                if (gap > 20.0f && gap < 60.0f) {
                    rear.s = wrapPos(front.s - (front.length + 5.0f), R.total);
                    desiredRear = front.speed;
                }
                else if (gap < front.length / 2) {
                    desiredFront = front.speed + 20.0f;
                    desiredRear  -= 5.0f;
                }
                else {
                    desiredRear = rear.speed + 15.0f;
                    desiredFront -= 5.0f;
                }

                float vR = vel[rearIdx];
                if (vR < desiredRear) vR = std::min(desiredRear, vR + 50.0f * dt);
                else                  vR = std::max(desiredRear, vR - 50.0f * dt);
                vel[rearIdx] = vR;

                float vF = vel[frontIdx];
                if (vF < desiredFront) vF = std::min(desiredFront, vF + 50.0f * dt);
                else                   vF = std::max(desiredFront, vF - 50.0f * dt);
                vel[frontIdx] = vF;

                rear.s = wrapPos(rear.s + vel[rearIdx] * dt, R.total);
                front.s = wrapPos(front.s + vel[frontIdx] * dt, R.total);

                for (int ci : {rearIdx, frontIdx}) {
                    glm::vec3 p = posOnRoute(R, Cars[ci].s);
                    float yaw = yawAlongRoute(R, Cars[ci].s);
                    SC.I[Cars[ci].inst].Wm = MakeCarTRS(p, yaw, Cars[ci].yawOffset);
                }
            }
        }
    }

    void updatePedestrian(float dt) {
        for (auto& ped : Pedestrians) {
            if (ped.instId < 0 || ped.walkRoute.total <= 0.0001f) continue;

            ped.s = wrapPos(ped.s + ped.speed * dt, ped.walkRoute.total);

            glm::vec3 pos = posOnRoute(ped.walkRoute, ped.s);
            float yaw = yawAlongRoute(ped.walkRoute, ped.s);

            glm::mat4 M(1.0f);
            M = glm::translate(M, pos);
            M = glm::rotate(M, yaw, glm::vec3(0, 1, 0));

            SC.I[ped.instId].Wm = M;
        }
    }

    void initTraffic() {
        Cars.clear();
        Routes.clear();
        trafficTime = 0.0f;
        const float offset = 3.0f;

        Route innerCWS;
        innerCWS.wp = { {-20,0.3f,-20},
            {20,0.3f,-20},
            {20,0.3f,20},
            {-20,0.3f,20} };
        innerCWS.isHorizontalFirst = true;
        buildRoute(innerCWS);

        Route threeCWF;
        threeCWF.wp = {
            {-68-offset,0.3f,-20-offset},
            {68+offset,0.3f,-20-offset},
            {68+offset,0.3f,20+offset},
            {-68-offset,0.3f,20+offset}
        };
        threeCWF.isHorizontalFirst = true;
        buildRoute(threeCWF);

        Route innerCCWS;
        innerCCWS.wp={{-20.0f-2*offset,0.3f,-20.0f-2*offset},
            {-20.0f-2*offset,0.3f,20.0f+2*offset},
            {20.0f+2*offset,0.3f,20.0f+2*offset},
            {20.0f+2*offset,0.3f,-20.0f-2*offset}};
        innerCCWS.isHorizontalFirst = true;
        buildRoute(innerCCWS);

        Route innerCCWF;
        innerCCWF.wp={{-20.0f-3*offset,0.3f,-20.0f-3*offset},
            {-20.0f-3*offset,0.3f,20.0f+3*offset},
            {20.0f+3*offset,0.3f,20.0f+3*offset},
            {20.0f+3*offset,0.3f,-20.0f-3*offset}};
        innerCCWF.isHorizontalFirst = true;
        buildRoute(innerCCWF);

        Route outerCWS;
        outerCWS.wp = { {-68,0.3f,-68},
            {68,0.3f,-68},
            {68,0.3f,68},
            {-68,0.3f,68} };
        outerCWS.isHorizontalFirst = true;
        buildRoute(outerCWS);

        Route HCWF;
        HCWF.wp = {
            {-68-offset,0.3f,-68-offset},
            {-20-offset,0.3f,-68-offset},
            {-20-offset,0.3f,-20-offset},
            {20+offset,0.3f,-20-offset},
            {20+offset,0.3f,-68-offset},
            {68+offset,0.3f,-68-offset},
            {68+offset,0.3f,68+offset},
            {20+offset,0.3f,68+offset},
            {20+offset,0.3f,20+offset},
            {-20-offset,0.3f,20+offset},
            {-20-offset,0.3f,68+offset},
            {-68-offset,0.3f,68+offset}
        };
        HCWF.isHorizontalFirst = true;
        buildRoute(HCWF);

        Route outerCCWS;
        outerCCWS.wp={{-68.0f-2*offset,0.3f,-68.0f-2*offset},
            {-68.0f-2*offset,0.3f,68.0f+2*offset},
            {68.0f+2*offset,0.3f,68.0f+2*offset},
            {68.0f+2*offset,0.3f,-68.0f-2*offset}};
        outerCCWS.isHorizontalFirst = true;
        buildRoute(outerCCWS);

        Route outerCCWF;
        outerCCWF.wp={{-68.0f-3*offset,0.3f,-68.0f-3*offset},
            {-68.0f-3*offset,0.3f,68.0f+3*offset},
            {68.0f+3*offset,0.3f,68.0f+3*offset},
            {68.0f+3*offset,0.3f,-68.0f-3*offset}};
        outerCCWF.isHorizontalFirst = true;
        buildRoute(outerCCWF);

        Routes.push_back(innerCWS);   // 0
        Routes.push_back(threeCWF);   // 1
        Routes.push_back(innerCCWS);  // 2
        Routes.push_back(innerCCWF);  // 3
        Routes.push_back(outerCWS);   // 4
        Routes.push_back(HCWF);       // 5
        Routes.push_back(outerCCWS);  // 6
        Routes.push_back(outerCCWF);  // 7

        float yawOffset = 0.0f;

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

            int routeId = i % 7;
            const Route& R = Routes[routeId];

            CarState c;
            c.inst = i;
            c.routeId = routeId;
            c.yawOffset = yawOffset;

            c.speed = 7.0f + float((i * 53) % 70) / 10.0f; // 7..14
            c.length = 3.0f;

            int bestWp = 0;
            float bestD = 1e30f;
            for (int w = 0; w < (int)R.wp.size(); w++) {
                glm::vec2 d(R.wp[w].x - p.x, R.wp[w].z - p.z);
                float dd = glm::dot(d, d);
                if (dd < bestD) { bestD = dd; bestWp = w; }
            }
            c.s = R.cumLen[bestWp];
            c.s = wrapPos(c.s + float((i * 19) % 40), R.total);

            Cars.push_back(c);
        }
    }

    void initPedestrian() {
        Pedestrians.clear();

        for (int i = 0; i < SC.InstanceCount; i++) {
            if (SC.I[i].id != nullptr) {
                std::string instId = *SC.I[i].id;
                if (instId.find("pedestrians") != std::string::npos) {
                    PedestrianState ped;
                    ped.instId = i;
                    ped.speed = 1.8f;
                    ped.s = 0.0f;

                    if (instId.find("center_up") != std::string::npos) {
                        ped.walkRoute.wp = {
                            glm::vec3(-17.0f, 0.3f, -65.0f),
                            glm::vec3( 17.0f, 0.3f, -65.0f),
                            glm::vec3( 17.0f, 0.3f, -32.0f),
                            glm::vec3(-17.0f, 0.3f, -32.0f)
                        };
                    } else if (instId.find("center_left") != std::string::npos) {
                        ped.walkRoute.wp = {
                            glm::vec3(-70.0f, 0.3f, -17.0f),
                            glm::vec3(-45.0f, 0.3f, -17.0f),
                            glm::vec3(-45.0f, 0.3f,  17.0f),
                            glm::vec3(-70.0f, 0.3f,  17.0f)
                        };
                    } else if (instId.find("crossing_right") != std::string::npos) {
                        ped.walkRoute.wp = {
                            glm::vec3( 31.0f, 0.3f, -17.0f),
                            glm::vec3( 65.0f, 0.3f, -17.0f),
                            glm::vec3( 65.0f, 0.3f,  65.0f),
                            glm::vec3( 31.0f, 0.3f,  65.0f)
                        };                    
                    } else if (instId.find("crossing_up") != std::string::npos) {
                        // Crossing up: walks through up_left and center_up
                        ped.walkRoute.wp = {
                            glm::vec3(-70.0f, 0.3f, -65.0f),   // NW corner (up_left)
                            glm::vec3( 17.0f, 0.3f, -65.0f),   // NE corner (center_up)
                            glm::vec3( 17.0f, 0.3f, -31.0f),   // SE corner (center_up)
                            glm::vec3(-70.0f, 0.3f, -31.0f)    // SW corner (up_left)
                        };                    } 
                        else {
                        ped.walkRoute.wp = {
                            glm::vec3(-17.0f, 0.3f, -17.0f),
                            glm::vec3( 17.0f, 0.3f, -17.0f),
                            glm::vec3( 17.0f, 0.3f,  17.0f),
                            glm::vec3(-17.0f, 0.3f,  17.0f)
                        };
                    }

                    buildRoute(ped.walkRoute);
                    Pedestrians.push_back(ped);

                    std::cout << "[Pedestrian] Initialized \"" << instId << "\" at instance "
                              << ped.instId << " with route length " << ped.walkRoute.total << " units\n";
                }
            }
        }

        if (Pedestrians.empty()) {
            std::cout << "[Pedestrian] Warning: No pedestrian instances found\n";
        }
    }

    // -------------------------------
    void setWindowParameters() override {
        windowWidth  = 1280;
        windowHeight = 800;
        windowTitle  = "A03 – City Traffic (Explore)";
        windowResizable = GLFW_TRUE;
        initialBackgroundColor = {0.90f, 0.70f, 0.55f, 1.0f};

        uniformBlocksInPool = 8000;
        texturesInPool      = 1024;
        setsInPool          = 8000;

        Ar = float(windowWidth) / float(windowHeight);
    }

    void onWindowResize(int w, int h) override {
        Ar = (h > 0) ? float(w) / float(h) : Ar;
    }

    void localInit() override {
        // Just to be sure where we load from
        std::cout << "Executable dir: " << GetExeDir().string() << std::endl;

        // Setup mouse callbacks
        glfwSetWindowUserPointer(window, this);
        glfwSetScrollCallback(window, [](GLFWwindow* w, double xoff, double yoff) {
            auto* app = static_cast<A03*>(glfwGetWindowUserPointer(w));
            app->smoothDist -= (float)yoff * app->mouseZoomSpeed;
            app->smoothDist = glm::clamp(app->smoothDist, 3.0f, 250.0f);
        });

        DSL.init(this, {
  {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL_GRAPHICS},
  {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT}, // base
  {2, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL_GRAPHICS},

});


        VD.init(this,
            {{0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX}},
            {
                {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos),  sizeof(glm::vec3), POSITION},
                {0, 1, VK_FORMAT_R32G32_SFLOAT,    offsetof(Vertex, UV),   sizeof(glm::vec2), UV},
                {0, 2, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, norm), sizeof(glm::vec3), NORMAL}
            }
        );

        const std::string vert = PathRelToExe("shaders/HalfLambertVert.vert.spv");
        const std::string frag = PathRelToExe("shaders/HalfLambertFrag.frag.spv");

        P.init(this, &VD, vert.c_str(), frag.c_str(), {&DSL});
        P.setAdvancedFeatures(VK_COMPARE_OP_LESS_OR_EQUAL,
                              VK_POLYGON_MODE_FILL,
                              VK_CULL_MODE_NONE,
                              false);

        const std::string scene = PathRelToExe("assets/models/city_scene.json");
        SC.init(this, &VD, DSL, P, scene.c_str());

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
        initPedestrian();

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
        // Use smooth values for more fluid calculations
        forward = glm::normalize(glm::vec3(
            cos(smoothPitch) * sin(smoothYaw),
            sin(smoothPitch),
            cos(smoothPitch) * cos(smoothYaw)
        ));
        right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
        up    = glm::normalize(glm::cross(right, forward));
    }

    void updateUniformBuffer(uint32_t currentImage) override {
        float deltaT;
        glm::vec3 m(0.0f), r(0.0f);
        bool fire = false;
        getSixAxis(deltaT, m, r, fire);

        // Toggle collision mode with T
        static bool tWasDown = false;
        bool tDown = glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS;
        if (tDown && !tWasDown) collisionMode = !collisionMode;
        tWasDown = tDown;

        // Toggle camera mode with TAB
        static bool tabWasDown = false;
        bool tabDown = glfwGetKey(window, GLFW_KEY_TAB) == GLFW_PRESS;
        if (tabDown && !tabWasDown) {
            cameraMode = (CameraMode)((cameraMode + 1) % 3);
            std::cout << "📹 Camera Mode: " << cameraModeNames[cameraMode] << std::endl;
        }
        tabWasDown = tabDown;

        // Speed modifiers
        float speedMultiplier = 1.0f;
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)) {
            speedMultiplier = 2.5f; // Fast mode
        }
        if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL)) {
            speedMultiplier = 0.3f; // Slow mode
        }

        // Mouse input
        double mouseX, mouseY;
        glfwGetCursorPos(window, &mouseX, &mouseY);
        double mouseDX = mouseX - lastMouseX;
        double mouseDY = mouseY - lastMouseY;
        lastMouseX = mouseX;
        lastMouseY = mouseY;

        // Right mouse button for orbit
        bool rightMouseNow = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        if (rightMouseNow && rightMousePressed) {
            smoothYaw += (float)mouseDX * mouseSensitivity;
            smoothPitch -= (float)mouseDY * mouseSensitivity;
        }
        rightMousePressed = rightMouseNow;

        // Middle mouse button for pan
        bool middleMouseNow = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        if (middleMouseNow && middleMousePressed) {
            glm::vec3 forward, right, up;
            cameraBasis(forward, right, up);
            glm::vec3 groundR = glm::normalize(glm::vec3(right.x, 0.0f, right.z));
            glm::vec3 groundF = glm::normalize(glm::vec3(forward.x, 0.0f, forward.z));
            smoothCamTarget -= groundR * (float)mouseDX * panSpeed * 0.02f * speedMultiplier;
            smoothCamTarget -= groundF * (float)mouseDY * panSpeed * 0.02f * speedMultiplier;
        }
        middleMousePressed = middleMouseNow;

        updateCars(deltaT);
        updatePedestrian(deltaT);

        // Keyboard orbit controls
        if (glfwGetKey(window, GLFW_KEY_LEFT))  smoothYaw  -= orbitSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_RIGHT)) smoothYaw  += orbitSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_UP))    smoothPitch += orbitSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_DOWN))  smoothPitch -= orbitSpeed * deltaT * speedMultiplier;

        smoothYaw   += orbitSpeed * deltaT * r.y;
        smoothPitch -= orbitSpeed * deltaT * r.x;

        smoothPitch = glm::clamp(smoothPitch, glm::radians(8.0f), glm::radians(85.0f));

        // Zoom controls
        smoothDist -= zoomSpeed * deltaT * m.y;
        if (glfwGetKey(window, GLFW_KEY_Q)) smoothDist -= zoomSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_E)) smoothDist += zoomSpeed * deltaT * speedMultiplier;

        smoothDist = glm::clamp(smoothDist, 3.0f, 250.0f);

        glm::vec3 forward, right, up;
        cameraBasis(forward, right, up);
        glm::vec3 groundF = glm::normalize(glm::vec3(forward.x, 0.0f, forward.z));
        glm::vec3 groundR = glm::normalize(glm::vec3(right.x,   0.0f, right.z));

        // Pan controls (IJKL)
        if (glfwGetKey(window, GLFW_KEY_I)) smoothCamTarget += groundF * panSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_K)) smoothCamTarget -= groundF * panSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_J)) smoothCamTarget -= groundR * panSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_L)) smoothCamTarget += groundR * panSpeed * deltaT * speedMultiplier;

        // Movement controls (WASD)
        float moveSpeed = (cameraMode == FREE) ? flySpeed * 1.5f : flySpeed;
        if (glfwGetKey(window, GLFW_KEY_W)) smoothCamTarget -= groundF * moveSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_S)) smoothCamTarget += groundF * moveSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_A)) smoothCamTarget += groundR * moveSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_D)) smoothCamTarget -= groundR * moveSpeed * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_Z)) smoothCamTarget.y -= flySpeed * 0.8f * deltaT * speedMultiplier;
        if (glfwGetKey(window, GLFW_KEY_X)) smoothCamTarget.y += flySpeed * 0.8f * deltaT * speedMultiplier;

        // Camera mode specific constraints
        if (cameraMode == BIRD) {
            smoothPitch = glm::clamp(smoothPitch, glm::radians(60.0f), glm::radians(89.0f));
            smoothCamTarget.y = glm::max(smoothCamTarget.y, 0.0f);
        }
        
        smoothCamTarget.x = glm::clamp(smoothCamTarget.x, -90.0f, 90.0f);
        smoothCamTarget.z = glm::clamp(smoothCamTarget.z, -90.0f, 90.0f);


        // Reset camera
        if (glfwGetKey(window, GLFW_KEY_R)) {
            if (SC.InstanceIds.count("prm")) {
                int iprm = SC.InstanceIds["prm"];
                smoothCamTarget = glm::vec3(SC.I[iprm].Wm[3]);
            } else {
                smoothCamTarget = glm::vec3(0, 0, 0);
            }
            smoothYaw   = glm::radians(45.0f);
            smoothPitch = glm::radians(50.0f);
            smoothDist  = 80.0f;
        }

        // Apply smooth transitions
        float smoothSpeed = smoothFactor * deltaT;
        CamYaw = glm::mix(CamYaw, smoothYaw, smoothSpeed);
        CamPitch = glm::mix(CamPitch, smoothPitch, smoothSpeed);
        CamDist = glm::mix(CamDist, smoothDist, smoothSpeed);
        CamTarget = glm::mix(CamTarget, smoothCamTarget, smoothSpeed);

        // Calculate camera position based on mode
        glm::vec3 offset;
        if (cameraMode == FREE) {
            // First-person style: camera closer to target
            float freeDist = glm::min(CamDist * 0.3f, 15.0f);
            offset.x = freeDist * cos(CamPitch) * sin(CamYaw);
            offset.y = freeDist * sin(CamPitch);
            offset.z = freeDist * cos(CamPitch) * cos(CamYaw);
        } else {
            // Orbit/Bird mode: normal distance
            offset.x = CamDist * cos(CamPitch) * sin(CamYaw);
            offset.y = CamDist * sin(CamPitch);
            offset.z = CamDist * cos(CamPitch) * cos(CamYaw);
        }
        CamPos = CamTarget + offset;

        // Prevent camera from going underground
        if (CamPos.y < 0.5f) {
            CamPos.y = 0.5f;
        }

        // Adjust FOV based on camera mode
        float targetFOV = 45.0f;
        if (cameraMode == BIRD) targetFOV = 60.0f;  // Wider view for bird mode
        if (cameraMode == FREE) targetFOV = 75.0f;  // Wide for first-person feel
        FOVdeg = glm::mix(FOVdeg, targetFOV, deltaT * 3.0f);

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
        gubo.lightDir   = glm::vec4(glm::normalize(glm::vec3(-1.0f, -0.2f, -0.8f)), 0.0f);
        gubo.lightColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
        gubo.eyePos     = CamPos;
        gubo.eyeDir     = glm::vec4(glm::normalize(CamTarget - CamPos), 0.0f);

        GlobalUniformBufferObject guboLocal = gubo;

        // -------------------------------
        // TRAFFIC LIGHT STATE (GLOBAL)
        // -------------------------------


        float phaseT = std::fmod(trafficTime, 2.0f * lightPeriod);
        bool horizontalGreen = phaseT < lightPeriod;

        float localT = std::fmod(trafficTime, lightPeriod);
        bool inAmber = localT > (lightPeriod - amberTime);

        if (inAmber) trafficstate = 1;                 // yellow
        else         trafficstate = horizontalGreen ? 2 : 0;  // green or red

        guboLocal.traffic = glm::ivec4(trafficstate, 0, 0, 0);


        UniformBufferObject ubo{};

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
            int modelIdx = SC.I[i].Mid;
            if (modelIdx >= 0 && modelIdx < SC.ModelCount &&
                !SC.M[modelIdx]->animations.empty()) {

                SC.I[i].animTime += deltaT;

                float duration = SC.M[modelIdx]->animations[0].duration;
                if (SC.I[i].animTime > duration) {
                    SC.I[i].animTime = std::fmod(SC.I[i].animTime, duration);
                }

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
