#version 450

layout(binding = 2, std140) uniform GlobalUniformBufferObject
{
    vec3 lightDir;
    vec4 lightColor;
    vec3 eyePos;
    vec4 eyeDir;
} gubo;

layout(binding = 0, std140) uniform UniformBufferObject
{
    mat4 mvpMat;
    mat4 mMat;
    mat4 nMat;
} ubo;

layout(binding = 1) uniform sampler2D texSampler;

layout(location = 1) in vec3 fragNorm;
layout(location = 0) in vec3 fragPos;
layout(location = 2) in vec2 fragTexCoord;
layout(location = 0) out vec4 outColor;

void main() {
    // 1. Re-normalize vectors to account for length changes during interpolation
    vec3 N = normalize(fragNorm);
    vec3 L = normalize(gubo.lightDir.xyz);
    vec3 V = normalize(gubo.eyePos.xyz - fragPos);
    // 2. Sample the texture
    vec3 albedo = texture(texSampler, fragTexCoord).rgb;
    // 3. Half-Lambert Diffuse (prevents back-lit areas from becoming pitch black, creating a more translucent look)
    float halfLambert = dot(N, L) * 0.5 + 0.5;
    vec3 diffuse = albedo * (halfLambert * halfLambert) * gubo.lightColor.rgb;
    // 4. Simple Ambient Mixing (simulates sky light and ground light)
    // Upward-facing surfaces (sky) get a blue tint; downward-facing surfaces (ground) get dark grey
    vec3 skyColor = vec3(0.2, 0.3, 0.4);
    vec3 groundColor = vec3(0.1, 0.1, 0.1);
    vec3 ambient = mix(groundColor, skyColor, N.y * 0.5 + 0.5) * albedo;
    // 5. Subtle Specular (Blinn-Phong)
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 32.0);
    vec3 specular = vec3(spec * 0.2); // 建筑不需要太亮的高光
    // Final Composition
    outColor = vec4(diffuse + ambient + specular, 1.0);
}

