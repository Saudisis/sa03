#version 450

layout(binding = 2, std140) uniform GlobalUniformBufferObject
{
    vec3 lightDir;
    vec4 lightColor;
    vec3 eyePos;
    vec4 eyeDir;

    ivec4 traffic; // traffic.x = 0 red, 1 yellow, 2 green
} gubo;

layout(binding = 0, std140) uniform UniformBufferObject
{
    mat4 mvpMat;
    mat4 mMat;
    mat4 nMat;
} ubo;

layout(binding = 1) uniform sampler2D texBase;

layout(location = 1) in vec3 fragNorm;
layout(location = 0) in vec3 fragPos;
layout(location = 2) in vec2 fragTexCoord;

layout(location = 0) out vec4 outColor;

float nearColor(vec3 a, vec3 b, float th) {
    return step(distance(a, b), th); // 1 if close, 0 if far
}

void main() {
    // Normalize vectors
    vec3 N = normalize(fragNorm);
    vec3 L = normalize(gubo.lightDir.xyz);
    vec3 V = normalize(gubo.eyePos.xyz - fragPos);

    // Base albedo from palette texture
    vec3 albedo = texture(texBase, fragTexCoord).rgb;

    // -----------------------------
    // TRAFFIC LIGHT OVERRIDE (PALETTE)
    // -----------------------------
    // Ajusta estos "target colors" si tu paleta tiene otros tonos.
    // (son aproximaciones, por eso usamos threshold)
    vec3 targetRed    = vec3(0.75, 0.20, 0.20);
    vec3 targetYellow = vec3(0.75, 0.75, 0.20);
    vec3 targetGreen  = vec3(0.20, 0.75, 0.20);

    float th = 0.18; // tolerancia

    float isRed    = nearColor(albedo, targetRed, th);
    float isYellow = nearColor(albedo, targetYellow, th);
    float isGreen  = nearColor(albedo, targetGreen, th);

    float isTraffic = max(isRed, max(isYellow, isGreen));

    if (isTraffic > 0.5) {
        // encendidos (más "vivos")
        vec3 onRed    = vec3(1.00, 0.10, 0.10);
        vec3 onYellow = vec3(1.00, 1.00, 0.10);
        vec3 onGreen  = vec3(0.10, 1.00, 0.10);

        // apagado
        vec3 off = albedo * 0.15;

        // cuál canal se enciende según estado global
        float sRed    = (gubo.traffic.x == 0) ? isRed    : 0.0;
        float sYellow = (gubo.traffic.x == 1) ? isYellow : 0.0;
        float sGreen  = (gubo.traffic.x == 2) ? isGreen  : 0.0;

        vec3 lit = off;
        //lit = mix(lit, onRed,    sRed);
        //lit = mix(lit, onYellow, sYellow);
        //lit = mix(lit, onGreen,  sGreen);
        lit = (gubo.traffic.x == 0) ? onRed : ((gubo.traffic.x == 1) ? onYellow : onGreen);
        albedo = lit;
    }

    // Half-Lambert diffuse
    float halfLambert = dot(N, L) * 0.5 + 0.5;
    vec3 diffuse = albedo * (halfLambert * halfLambert) * gubo.lightColor.rgb;

    // Ambient
    vec3 skyColor = vec3(0.2, 0.3, 0.4);
    vec3 groundColor = vec3(0.1, 0.1, 0.1);
    vec3 ambient = mix(groundColor, skyColor, N.y * 0.5 + 0.5) * albedo;

    // Specular
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 32.0);
    vec3 specular = vec3(spec * 0.2);

    outColor = vec4(diffuse + ambient + specular, 1.0);
}
