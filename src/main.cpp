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

// -------------------------------
// TEXT
// -------------------------------
std::vector<SingleText> outText = {
    {1, {"City Traffic Simulation",
         "WASD move | QE up/down | Arrows/Mouse orbit | IJKL pan | R reset | ESC quit",
         "", ""}, 0, 0}
};

// -------------------------------
// VERTEX
// -------------------------------
struct Vertex {
    glm::vec3 pos;
    glm::vec2 UV;
    glm::vec3 norm;
};

// =======================================================
// MAIN APPLICATION
// =======================================================
class A03 : public BaseProject {
protected:
    DescriptorSetLayout DSL;
    VertexDescriptor VD;
    Pipeline P;
    Scene SC;
    TextMaker txt;

    float Ar = 4.0f / 3.0f;

    // -------- Camera (Explorer) --------
    glm::vec3 CamTarget = glm::vec3(0.0f, 0.0f, 0.0f);
    glm::vec3 CamPos    = glm::vec3(0.0f, 80.0f, 120.0f);

    float CamYaw   = glm::radians(45.0f);
    float CamPitch = glm::radians(45.0f);
    float CamDist  = 80.0f;     // start closer
    float CamRoll  = 0.0f;

    float FOVdeg     = 45.0f;                 // tighter FOV -> feels closer
    float zoomSpeed  = 500.0f;                // fast zoom
    float orbitSpeed = glm::radians(120.0f);  // rad/sec
    float panSpeed   = 120.0f;                // units/sec
    float flySpeed   = 140.0f;                // WASD speed

    // -------------------------------
    void setWindowParameters() override {
        windowWidth  = 1280;
        windowHeight = 800;
        windowTitle  = "A03 – City Traffic (Explore)";
        windowResizable = GLFW_TRUE;
        initialBackgroundColor = {0.55f, 0.75f, 0.95f, 1.0f};

        // Pools for big scenes
        uniformBlocksInPool = 8000;
        texturesInPool      = 1024;
        setsInPool          = 8000;

        Ar = float(windowWidth) / float(windowHeight);
    }

    void onWindowResize(int w, int h) override {
        Ar = (h > 0) ? float(w) / float(h) : Ar;
    }

    // -------------------------------
    void localInit() override {
        // Descriptor layout
        DSL.init(this, {
            {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL_GRAPHICS},
            {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT},
            {2, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL_GRAPHICS}
        });

        // Vertex format
        VD.init(this,
            {{0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX}},
            {
                {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos),  sizeof(glm::vec3), POSITION},
                {0, 1, VK_FORMAT_R32G32_SFLOAT,    offsetof(Vertex, UV),   sizeof(glm::vec2), UV},
                {0, 2, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, norm), sizeof(glm::vec3), NORMAL}
            }
        );

        // Pipeline
        P.init(this, &VD,
               "shaders/PhongVert.spv",
               "shaders/PhongFrag.spv",
               {&DSL});
        P.setAdvancedFeatures(VK_COMPARE_OP_LESS_OR_EQUAL,
                              VK_POLYGON_MODE_FILL,
                              VK_CULL_MODE_NONE,
                              false);

        // Load scene (your file)
        SC.init(this, &VD, DSL, P, "assets/models/city_scene.json");

        // Center camera on your city instance if it exists
        if (SC.InstanceIds.count("prm")) {
            int iprm = SC.InstanceIds["prm"];
            CamTarget = glm::vec3(SC.I[iprm].Wm[3]);
        } else if (SC.InstanceCount > 0) {
            CamTarget = glm::vec3(SC.I[0].Wm[3]);
        }

        // Start a bit above and behind target
        CamYaw   = glm::radians(45.0f);
        CamPitch = glm::radians(50.0f);
        CamDist  = 80.0f;

        txt.init(this, &outText);
    }

    // -------------------------------
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

    // -------------------------------
    void populateCommandBuffer(VkCommandBuffer cmd, int currentImage) override {
        P.bind(cmd);
        SC.populateCommandBuffer(cmd, currentImage, P);
        txt.populateCommandBuffer(cmd, currentImage, 0);
    }

    // Helper: basis vectors from yaw/pitch (camera orientation)
    void cameraBasis(glm::vec3 &forward, glm::vec3 &right, glm::vec3 &up) {
        // forward from yaw/pitch (looking direction)
        forward = glm::normalize(glm::vec3(
            cos(CamPitch) * sin(CamYaw),
            sin(CamPitch),
            cos(CamPitch) * cos(CamYaw)
        ));
        right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
        up    = glm::normalize(glm::cross(right, forward));
    }

    // -------------------------------
    void updateUniformBuffer(uint32_t currentImage) override {
        // Delta time + 6-axis input (mouse/stick)
        float deltaT;
        glm::vec3 m(0.0f), r(0.0f);
        bool fire = false;
        getSixAxis(deltaT, m, r, fire);

        // --- ORBIT: use r from device + arrow keys ---
        if (glfwGetKey(window, GLFW_KEY_LEFT))  CamYaw  -= orbitSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_RIGHT)) CamYaw  += orbitSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_UP))    CamPitch += orbitSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_DOWN))  CamPitch -= orbitSpeed * deltaT;

        CamYaw   += orbitSpeed * deltaT * r.y;
        CamPitch -= orbitSpeed * deltaT * r.x;

        CamPitch = glm::clamp(CamPitch, glm::radians(8.0f), glm::radians(85.0f));

        // --- ZOOM: mouse wheel / axis m.y + Q/E backup ---
        CamDist -= zoomSpeed * deltaT * m.y;
        if (glfwGetKey(window, GLFW_KEY_Q)) CamDist -= zoomSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_E)) CamDist += zoomSpeed * deltaT;

        CamDist = glm::clamp(CamDist, 3.0f, 2000.0f);

        // --- PAN target: IJKL on ground plane ---
        // I/K = forward/back on ground, J/L = left/right
        glm::vec3 forward, right, up;
        cameraBasis(forward, right, up);
        glm::vec3 groundF = glm::normalize(glm::vec3(forward.x, 0.0f, forward.z));
        glm::vec3 groundR = glm::normalize(glm::vec3(right.x,   0.0f, right.z));

        if (glfwGetKey(window, GLFW_KEY_I)) CamTarget += groundF * panSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_K)) CamTarget -= groundF * panSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_J)) CamTarget -= groundR * panSpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_L)) CamTarget += groundR * panSpeed * deltaT;

        // --- FLY move (WASD + R/F vertical) by moving target (explore like drone) ---
        if (glfwGetKey(window, GLFW_KEY_S)) CamTarget += groundF * flySpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_W)) CamTarget -= groundF * flySpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_D)) CamTarget -= groundR * flySpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_A)) CamTarget += groundR * flySpeed * deltaT;
        if (glfwGetKey(window, GLFW_KEY_Z)) CamTarget.y -= flySpeed * 0.8f * deltaT;
        if (glfwGetKey(window, GLFW_KEY_X)) CamTarget.y += flySpeed * 0.8f * deltaT;

        // Reset
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

        // Compute camera position from orbit params
        glm::vec3 offset;
        offset.x = CamDist * cos(CamPitch) * sin(CamYaw);
        offset.y = CamDist * sin(CamPitch);
        offset.z = CamDist * cos(CamPitch) * cos(CamYaw);
        CamPos = CamTarget + offset;

        // VIEW-PROJ
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

        // GLOBAL LIGHT
        GlobalUniformBufferObject gubo{};
        gubo.lightDir   = glm::normalize(glm::vec3(-1.0f, -1.0f, -0.5f));
        gubo.lightColor = glm::vec4(1, 1, 1, 1);
        gubo.eyePos     = CamPos;
        gubo.eyeDir     = glm::vec4(glm::normalize(CamTarget - CamPos), 1.0f);

        // map() needs void* -> use non-const local copies
        GlobalUniformBufferObject guboLocal = gubo;
        UniformBufferObject ubo{};

        // UPDATE ALL INSTANCES
        for (int i = 0; i < SC.InstanceCount; i++) {
            ubo.mMat   = SC.I[i].Wm;
            ubo.mvpMat = VP * ubo.mMat;
            ubo.nMat   = glm::inverse(glm::transpose(ubo.mMat));

            SC.DS[i]->map(currentImage, &ubo, sizeof(ubo), 0);
            SC.DS[i]->map(currentImage, &guboLocal, sizeof(guboLocal), 2);
        }

        // Quit
        if (glfwGetKey(window, GLFW_KEY_ESCAPE)) {
            glfwSetWindowShouldClose(window, GL_TRUE);
        }
    }
};

// -------------------------------
// MAIN
// -------------------------------
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
