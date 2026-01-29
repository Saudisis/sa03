#include <iostream>
#include <vector>
#include <set>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

struct Mat4RM { // row-major 4x4
  float m[16];
};

static Mat4RM makeTRS_RotY(float x, float y, float z, int rotDeg) {
  // rotDeg: 0, 90, 180, 270 (Y axis)
  // Row-major. This matches the style you have in JSON.
  float c=1.f, s=0.f;
  int r = ((rotDeg % 360) + 360) % 360;
  if (r == 0)      { c = 1; s = 0; }
  else if (r == 90)  { c = 0; s = 1; }
  else if (r == 180) { c = -1; s = 0; }
  else if (r == 270) { c = 0; s = -1; }

  // Standard Y-rotation (right-handed):
  // [  c, 0, s ]
  // [  0, 1, 0 ]
  // [ -s, 0, c ]
  Mat4RM T{};
  T.m[0]  =  c;  T.m[1]  = 0; T.m[2]  =  s;  T.m[3]  = x;
  T.m[4]  =  0;  T.m[5]  = 1; T.m[6]  =  0;  T.m[7]  = y;
  T.m[8]  = -s;  T.m[9]  = 0; T.m[10] =  c;  T.m[11] = z;
  T.m[12] =  0;  T.m[13] = 0; T.m[14] =  0;  T.m[15] = 1;
  return T;
}

static std::string matToJsonArray(const Mat4RM& M) {
  std::ostringstream ss;
  ss << "[ ";
  ss << std::fixed << std::setprecision(2);
  for (int i = 0; i < 16; i++) {
    ss << M.m[i];
    if (i != 15) ss << ", ";
  }
  ss << " ]";
  return ss.str();
}

static void printInstance(const std::string& id,
                          const std::string& model,
                          const std::string& texture,
                          const Mat4RM& tr)
{
  std::cout
    << "    { \"id\": \"" << id << "\", "
    << "\"model\": \"" << model << "\", "
    << "\"texture\": \"" << texture << "\", "
    << "\"transform\": " << matToJsonArray(tr) << " }";
}

int main() {
  // ====== AJUSTA ESTO SI QUIERES ======
  const float yRoad = 0.02f;

  // Este "half" lo estás usando ya: tus esquinas están a +/-24 desde el centro de cada manzana
  const float half = 24.f;

  // Este es el paso real entre piezas straight en tu escena (entre -8 y 8, etc.)
  // Si tu modelo straight mide distinto, cambia este valor.
  const float tile = 16.f;

  // Centros de manzanas que ya se ven en tu layout
  const std::vector<float> blockX = { -80.f, 0.f, 80.f };
  const std::vector<float> blockZ = {   0.f, -88.f };
  // ====================================

  // Líneas de calle (bordes de manzanas)
  std::set<float> streetX, streetZ;
  for (float cx : blockX) { streetX.insert(cx - half); streetX.insert(cx + half); }
  for (float cz : blockZ) { streetZ.insert(cz - half); streetZ.insert(cz + half); }

  std::vector<float> xs(streetX.begin(), streetX.end());
  std::vector<float> zs(streetZ.begin(), streetZ.end());
  std::sort(xs.begin(), xs.end());
  std::sort(zs.begin(), zs.end());

  auto hasX = [&](float x) {
    return std::find(xs.begin(), xs.end(), x) != xs.end();
  };
  auto hasZ = [&](float z) {
    return std::find(zs.begin(), zs.end(), z) != zs.end();
  };

  // Helper para decidir qué pieza va en cada intersección
  auto pickNode = [&](float x, float z, std::string& modelOut, int& rotOut) {
    bool L = hasX(x - (2 * half)); // distancia entre líneas X consecutivas en tu ciudad es 48 (=2*half)
    bool R = hasX(x + (2 * half));
    bool U = hasZ(z + (2 * half)); // +z (en tu escena: arriba suele ser +24, +z)
    bool D = hasZ(z - (2 * half));

    int deg = 0;

    int cnt = int(L)+int(R)+int(U)+int(D);
    if (cnt == 4) {
      modelOut = "r_cross"; rotOut = 0; return;
    }
    if (cnt == 3) {
      modelOut = "r_T";
      // OJO: La orientación base del modelo T puede variar.
      // Aquí asumimos: rot=0 deja el "hueco" hacia +Z (arriba). Si no coincide, rota 90 hasta que encaje.
      // Elegimos rot para que el lado FALTANTE apunte hacia el que NO existe.
      if (!U) deg = 0;
      else if (!R) deg = 90;
      else if (!D) deg = 180;
      else if (!L) deg = 270;
      rotOut = deg; return;
    }
    if (cnt == 2) {
      // Recto o esquina
      if ((L && R) || (U && D)) {
        modelOut = "r_straight";
        // Horizontal (L-R) = 0, Vertical (U-D) = 90
        rotOut = (L && R) ? 0 : 90;
        return;
      } else {
        modelOut = "r_corner";
        // Tu r_corner base (rot=0) en tu escena coincide con la esquina SE (como tu C_cSE con identidad).
        // Vamos a asignar rotaciones según qué dos direcciones existan.
        // Existentes: (L,U)=NW, (R,U)=NE, (R,D)=SE, (L,D)=SW
        if (R && D) rotOut = 0;       // SE
        else if (L && D) rotOut = 90; // SW
        else if (L && U) rotOut = 180;// NW
        else if (R && U) rotOut = 270;// NE
        else rotOut = 0;
        return;
      }
    }

    // cnt 0/1 (bordes raros): no colocamos nada
    modelOut = ""; rotOut = 0;
  };

  std::cout << "[\n";

  bool first = true;
  int idCounter = 0;

  auto emit = [&](const std::string& model, float x, float y, float z, int rotDeg) {
    if (model.empty()) return;
    if (!first) std::cout << ",\n";
    first = false;

    std::ostringstream id;
    id << "RD_" << model << "_" << idCounter++;

    Mat4RM tr = makeTRS_RotY(x, y, z, rotDeg);
    printInstance(id.str(), model, "tRoad", tr);
  };

  // 1) Intersecciones (nodos): corner/T/cross/straight según conectividad
  for (float z : zs) {
    for (float x : xs) {
      std::string m; int rot=0;
      pickNode(x, z, m, rot);
      emit(m, x, yRoad, z, rot);
    }
  }

  // 2) Relleno de segmentos entre nodos con straight repetidos (esto elimina los huecos)
  // Horizontal: para cada z fijo, entre xs[i] y xs[i+1] coloca straights a x0+tile ... x1-tile
  for (float z : zs) {
    for (int i = 0; i < (int)xs.size() - 1; i++) {
      float x0 = xs[i], x1 = xs[i+1];
      // rellenar interior
      for (float x = x0 + tile; x <= x1 - tile + 0.001f; x += tile) {
        emit("r_straight", x, yRoad, z, 0); // horizontal
      }
    }
  }

  // Vertical: para cada x fijo, entre zs[j] y zs[j+1] coloca straights a z0+tile ... z1-tile
  for (float x : xs) {
    for (int j = 0; j < (int)zs.size() - 1; j++) {
      float z0 = zs[j], z1 = zs[j+1];
      for (float z = z0 + tile; z <= z1 - tile + 0.001f; z += tile) {
        emit("r_straight", x, yRoad, z, 90); // vertical
      }
    }
  }

  std::cout << "\n]\n";
  return 0;
}
