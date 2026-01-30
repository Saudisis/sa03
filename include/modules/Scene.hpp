#pragma once

#include <unordered_map>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "../json.hpp"
#include "UBO.hpp"

// =====================================================
// Instance struct
// =====================================================
typedef struct {
    std::string *id;
    int Mid;        // Model index
    int Tid;        // Texture index
    glm::mat4 Wm;   // World matrix (compatible with your loader convention)
    float animTime; // Animation time for this instance (0..duration)
} Instance;

// =====================================================
// Scene class
// =====================================================
class Scene {
public:
    VertexDescriptor *VD = nullptr;
    BaseProject *BP = nullptr;

    // Models
    int ModelCount = 0;
    Model **M = nullptr;
    std::unordered_map<std::string, int> MeshIds;

    // Textures
    int TextureCount = 0;
    Texture **T = nullptr;
    std::unordered_map<std::string, int> TextureIds;

    // Instances
    int InstanceCount = 0;
    DescriptorSet **DS = nullptr;
    Instance *I = nullptr;
    std::unordered_map<std::string, int> InstanceIds;

private:
    // -------------------------------------------------
    // Read transform[16] exactly as your original convention
    // -------------------------------------------------
    static glm::mat4 ReadTransform16_Compatible(const nlohmann::json &inst) {
        auto &t = inst["transform"];
        float m[16];
        for (int k = 0; k < 16; k++) m[k] = t[k].get<float>();

        return glm::mat4(
            m[0],  m[4],  m[8],  m[12],
            m[1],  m[5],  m[9],  m[13],
            m[2],  m[6],  m[10], m[14],
            m[3],  m[7],  m[11], m[15]
        );
    }

    static glm::vec3 ReadVec3(const nlohmann::json &j, const char *key, const glm::vec3 &def) {
        if (!j.contains(key)) return def;
        auto v = j[key];
        return glm::vec3(v[0].get<float>(), v[1].get<float>(), v[2].get<float>());
    }

    static glm::mat4 MakeTRS(const glm::vec3 &pos, float rotY_deg, const glm::vec3 &scale) {
        glm::mat4 M(1.0f);
        M = glm::translate(M, pos);
        M = glm::rotate(M, glm::radians(rotY_deg), glm::vec3(0, 1, 0));
        M = glm::scale(M, scale);
        return M;
    }

    static std::string PickModelRoundRobin(const nlohmann::json &models, int &rr) {
        int n = (int)models.size();
        if (n <= 0) return "";
        std::string s = models[rr % n].get<std::string>();
        rr++;
        return s;
    }

    static void PrintIds(const std::unordered_map<std::string, int> &mp, const char *title) {
        std::cout << title << ":\n";
        for (auto &kv : mp) std::cout << "  " << kv.first << " -> " << kv.second << "\n";
    }

    // Interpolate animation keyframe
    static AnimationKeyframe SampleAnimationTrack(const AnimationTrack& track, float time) {
        AnimationKeyframe out{};
        out.translation = glm::vec3(0.0f);
        out.rotation = glm::quat(1, 0, 0, 0);
        out.scale = glm::vec3(1.0f);

        if (track.keyframes.empty()) {
            return out;
        }

        // Find the two keyframes to interpolate between
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

        // Interpolate components
        out.translation = glm::mix(kf0.translation, kf1.translation, t);
        out.rotation = glm::slerp(kf0.rotation, kf1.rotation, t);
        out.scale = glm::mix(kf0.scale, kf1.scale, t);

        return out;
    }

public:
    // =================================================
    // INIT
    // =================================================
    void init(BaseProject *_BP,
              VertexDescriptor *_VD,
              DescriptorSetLayout &DSL,
              Pipeline &P,
              const std::string &file) {

        BP = _BP;
        VD = _VD;

        nlohmann::json js;
        std::ifstream ifs(file);
        if (!ifs.is_open()) {
            std::cout << "❌ Scene file not found: " << file << "\n" << std::flush;
            exit(-1);
        }

        try {
            ifs >> js;
            ifs.close();
        } catch (...) {
            std::cout << "❌ Failed parsing JSON: " << file << "\n" << std::flush;
            exit(-10);
        }

        // -------------------------------------------------
        // MODELS
        // -------------------------------------------------
        auto ms = js["models"];
        ModelCount = (int)ms.size();

        M = (Model **)calloc(ModelCount, sizeof(Model *));
        for (int i = 0; i < ModelCount; i++) {
            std::string id = ms[i]["id"].get<std::string>();
            MeshIds[id] = i;

            std::string fmt = ms[i]["format"].get<std::string>();
            M[i] = new Model();
            M[i]->init(BP, VD, ms[i]["model"].get<std::string>(),
                       (fmt[0] == 'O') ? OBJ :
                       (fmt[0] == 'G') ? GLTF : MGCG);
        }

        // -------------------------------------------------
        // TEXTURES
        // -------------------------------------------------
        auto ts = js["textures"];
        TextureCount = (int)ts.size();

        T = (Texture **)calloc(TextureCount, sizeof(Texture *));
        for (int i = 0; i < TextureCount; i++) {
            std::string id = ts[i]["id"].get<std::string>();
            TextureIds[id] = i;

            T[i] = new Texture();
            T[i]->init(BP, ts[i]["texture"].get<std::string>());
        }

        // -------------------------------------------------
        // INSTANCES (base + placements)
        // -------------------------------------------------
        std::vector<Instance> tempInstances;
        tempInstances.reserve(4096);

        // A) explicit instances
        auto is = js["instances"];
        for (int i = 0; i < (int)is.size(); i++) {
            Instance inst;
            inst.id  = new std::string(is[i]["id"].get<std::string>());
            inst.animTime = 0.0f;

            std::string modelId = is[i]["model"].get<std::string>();
            std::string texId   = is[i]["texture"].get<std::string>();

            if (!MeshIds.count(modelId)) {
                std::cout << "❌ Unknown modelId in instances: '" << modelId
                          << "' (instance id=" << *inst.id << ")\n" << std::flush;
                PrintIds(MeshIds, "Known model ids");
                exit(-2);
            }
            if (!TextureIds.count(texId)) {
                std::cout << "❌ Unknown textureId in instances: '" << texId
                          << "' (instance id=" << *inst.id << ")\n" << std::flush;
                PrintIds(TextureIds, "Known texture ids");
                exit(-3);
            }

            inst.Mid = MeshIds[modelId];
            inst.Tid = TextureIds[texId];
            inst.Wm  = ReadTransform16_Compatible(is[i]);

            InstanceIds[*inst.id] = (int)tempInstances.size();
            tempInstances.push_back(inst);
        }

        // B) placements: points / row / grid
        if (js.contains("placements")) {
            auto ps = js["placements"];

            for (int pidx = 0; pidx < (int)ps.size(); pidx++) {
                auto &p = ps[pidx];

                std::string baseId = p.value("id", std::string("plc"));
                std::string texId  = p.value("texture", std::string("t1"));

                if (!TextureIds.count(texId)) {
                    std::cout << "❌ Unknown textureId in placement '" << baseId
                              << "': '" << texId << "'\n" << std::flush;
                    PrintIds(TextureIds, "Known texture ids");
                    exit(-3);
                }

                float rotY = p.value("rotY", 0.0f);
                glm::vec3 scale = ReadVec3(p, "scale", glm::vec3(1.0f));

                int rr = 0;

                auto emitInstance = [&](const glm::vec3 &pos, const std::string &modelId, int idx) {
                    if (!MeshIds.count(modelId)) {
                        std::cout << "❌ Unknown modelId in placement '" << baseId
                                  << "': '" << modelId << "'\n" << std::flush;
                        PrintIds(MeshIds, "Known model ids");
                        exit(-2);
                    }

                    Instance inst;
                    inst.id = new std::string(baseId + "_" + std::to_string(idx));
                    inst.animTime = 0.0f;
                    inst.Mid = MeshIds[modelId];
                    inst.Tid = TextureIds[texId];

                    // Match the same convention as transform[16]
                    glm::mat4 Mtrs = MakeTRS(pos, rotY, scale);
                    inst.Wm = Mtrs;

                    InstanceIds[*inst.id] = (int)tempInstances.size();
                    tempInstances.push_back(inst);
                };

                // points
                if (p.contains("points")) {
                    std::string modelId = p.value("model", std::string(""));
                    if (modelId.empty()) {
                        std::cout << "❌ placement '" << baseId << "' has points but no 'model'\n" << std::flush;
                        exit(-11);
                    }
                    int k = 0;
                    for (auto &pt : p["points"]) {
                        glm::vec3 pos(pt[0].get<float>(), pt[1].get<float>(), pt[2].get<float>());
                        emitInstance(pos, modelId, k++);
                    }
                    continue;
                }

                // row
                if (p.contains("start") && p.contains("dir") && p.contains("count")) {
                    glm::vec3 start = ReadVec3(p, "start", glm::vec3(0));
                    glm::vec3 dir   = ReadVec3(p, "dir", glm::vec3(1,0,0));
                    int count       = p.value("count", 0);
                    float step      = p.value("step", 1.0f);

                    for (int i = 0; i < count; i++) {
                        glm::vec3 pos = start + dir * (step * (float)i);

                        std::string modelId;
                        if (p.contains("model")) modelId = p["model"].get<std::string>();
                        else modelId = PickModelRoundRobin(p["models"], rr);

                        emitInstance(pos, modelId, i);
                    }
                    continue;
                }

                // grid
                if (p.contains("origin") && p.contains("grid")) {
                    glm::vec3 origin = ReadVec3(p, "origin", glm::vec3(0));
                    int nx = p["grid"].value("nx", 0);
                    int nz = p["grid"].value("nz", 0);
                    float dx = p["grid"].value("dx", 1.0f);
                    float dz = p["grid"].value("dz", 1.0f);

                    int idx = 0;
                    for (int z = 0; z < nz; z++) {
                        for (int x = 0; x < nx; x++) {
                            glm::vec3 pos = origin + glm::vec3(dx * x, 0.0f, dz * z);

                            std::string modelId;
                            if (p.contains("model")) modelId = p["model"].get<std::string>();
                            else modelId = PickModelRoundRobin(p["models"], rr);

                            emitInstance(pos, modelId, idx++);
                        }
                    }
                    continue;
                }

                std::cout << "⚠️ placement '" << baseId
                          << "' ignored (needs points OR row(start/dir/count) OR grid(origin+grid))\n";
            }
        }

        // -------------------------------------------------
        // FINAL ALLOCATION
        // -------------------------------------------------
        InstanceCount = (int)tempInstances.size();
        I  = (Instance *)calloc(InstanceCount, sizeof(Instance));
        DS = (DescriptorSet **)calloc(InstanceCount, sizeof(DescriptorSet *));

        for (int i = 0; i < InstanceCount; i++) I[i] = tempInstances[i];

        std::cout << "✅ Scene loaded. FINAL InstanceCount = " << InstanceCount << "\n" << std::flush;
    }

    // =================================================
    // DESCRIPTORS
    // =================================================
    void pipelinesAndDescriptorSetsInit(DescriptorSetLayout &DSL) {
        for (int i = 0; i < InstanceCount; i++) {
            DS[i] = new DescriptorSet();
            DS[i]->init(BP, &DSL, {
                {0, UNIFORM, sizeof(UniformBufferObject), nullptr},
                {1, TEXTURE, 0, T[I[i].Tid]},
                {2, UNIFORM, sizeof(GlobalUniformBufferObject), nullptr}
            });
        }
    }

    void pipelinesAndDescriptorSetsCleanup() {
        for (int i = 0; i < InstanceCount; i++) {
            DS[i]->cleanup();
            delete DS[i];
        }
    }

    // =================================================
    // CLEANUP
    // =================================================
    void localCleanup() {
        for (int i = 0; i < TextureCount; i++) {
            T[i]->cleanup();
            delete T[i];
        }
        free(T);

        for (int i = 0; i < ModelCount; i++) {
            M[i]->cleanup();
            delete M[i];
        }
        free(M);

        for (int i = 0; i < InstanceCount; i++) {
            delete I[i].id;
        }
        free(I);
        free(DS);
    }

    // =================================================
    // DRAW
    // =================================================
    void populateCommandBuffer(VkCommandBuffer cmd, int img, Pipeline &P) {
        for (int i = 0; i < InstanceCount; i++) {
            M[I[i].Mid]->bind(cmd);
            DS[i]->bind(cmd, P, 0, img);
            vkCmdDrawIndexed(
                cmd,
                static_cast<uint32_t>(M[I[i].Mid]->indices.size()),
                1, 0, 0, 0
            );
        }
    }
};
