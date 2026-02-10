# Dokumentacja basic.cc - Podstawowy przykład C++

## Przegląd

Plik `basic.cc` to **minimalny przykład** użycia MuJoCo w C++. Pokazuje:
- Jak załadować model XML
- Jak stworzyć okno z wizualizacją (OpenGL + GLFW)
- Jak uruchomić pętlę symulacji
- Jak obsługiwać interakcję użytkownika (mysz, klawiatura)

Jest to najbardziej podstawowy przykład - dobry punkt startowy do zrozumienia niskopoziomowego API MuJoCo.

---

## Struktura programu

### 1. Zmienne globalne

```cpp
// STRUKTURY DANYCH MUJOCO
mjModel* m = NULL;          // Model - opis robota (NIEZMIENNY podczas symulacji)
mjData* d = NULL;           // Data - stan symulacji (ZMIENNY w każdym kroku)

// STRUKTURY WIZUALIZACJI
mjvCamera cam;              // Kamera - punkt widzenia użytkownika
mjvOption opt;              // Opcje wizualizacji (co pokazywać: kontakty, stawy, itp.)
mjvScene scn;               // Scena - struktura do renderowania
mjrContext con;             // Kontekst GPU - zarządza zasobami graficznymi

// INTERAKCJA Z MYSZĄ
bool button_left = false;   // Czy lewy przycisk myszy jest wciśnięty
bool button_middle = false; // Czy środkowy przycisk myszy jest wciśnięty
bool button_right = false;  // Czy prawy przycisk myszy jest wciśnięty
double lastx = 0;           // Ostatnia pozycja X kursora
double lasty = 0;           // Ostatnia pozycja Y kursora
```

**Dlaczego zmienne globalne?**
- API GLFW wymaga przekazywania funkcji callback
- Callbacki nie mogą mieć kontekstu (nie da się przekazać dodatkowych parametrów)
- Dlatego używamy zmiennych globalnych do współdzielenia stanu

---

## Funkcje callback (interakcja użytkownika)

### 1. `keyboard()` - Obsługa klawiatury

```cpp
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // BACKSPACE: Reset symulacji
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);  // Przywróć stan początkowy
    mj_forward(m, d);    // Przelicz kinematykę dla nowego stanu
  }
}
```

**Co się dzieje:**
1. `mj_resetData(m, d)` - resetuje wszystkie wartości w `d` do stanu początkowego
2. `mj_forward(m, d)` - przelicza pozycje ciał, orientacje, itp. (kinematyka bezpośrednia)

**Możliwe rozszerzenia:**
```cpp
// Dodaj inne klawisze:
if (act==GLFW_PRESS && key==GLFW_KEY_SPACE) {
  paused = !paused;  // Pauza/wznów symulację
}

if (act==GLFW_PRESS && key==GLFW_KEY_R) {
  // Losowa perturbacja
  for (int i=0; i<m->nv; i++) {
    d->qvel[i] += (rand()/RAND_MAX - 0.5) * 0.1;
  }
}
```

---

### 2. `mouse_button()` - Wykrywanie kliknięć myszy

```cpp
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // Aktualizuj stan przycisków
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // Zapisz aktualną pozycję myszy
  glfwGetCursorPos(window, &lastx, &lasty);
}
```

**Co się dzieje:**
- Sprawdza które przyciski są wciśnięte
- Zapisuje pozycję myszy (potrzebne do obliczenia ruchu w `mouse_move`)

---

### 3. `mouse_move()` - Poruszanie kamerą

```cpp
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // Jeśli żaden przycisk nie jest wciśnięty - nic nie rób
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // Oblicz przesunięcie myszy
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // Pobierz rozmiar okna (do normalizacji ruchu)
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // Sprawdź czy Shift jest wciśnięty
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // Określ typ akcji na podstawie przycisku myszy
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;  // Przesuwanie
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;  // Obracanie
  } else {
    action = mjMOUSE_ZOOM;  // Zoom (środkowy przycisk)
  }

  // Wykonaj ruch kamery
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}
```

**Sterowanie kamerą:**

| Akcja | Opis |
|-------|------|
| **Lewy przycisk + ruch** | Obróć kamerę wokół punktu patrzenia |
| **Lewy + Shift + ruch** | Obróć kamerę wokół osi horyzontalnej |
| **Prawy przycisk + ruch** | Przesuń kamerę w pionie |
| **Prawy + Shift + ruch** | Przesuń kamerę w poziomie |
| **Środkowy przycisk + ruch** | Zoom (przybliż/oddal) |
| **Scroll** | Zoom (przybliż/oddal) |

---

### 4. `scroll()` - Zoom scrollem

```cpp
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // Emuluj pionowy ruch myszy = 5% wysokości okna
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}
```

**Co się dzieje:**
- Każdy obrót kółka = 5% wysokości okna ruchu pionowego
- Minus przed `yoffset` - scroll w górę = zoom in

---

## Funkcja main - Główny program

### Krok 1: Weryfikacja argumentów

```cpp
if (argc!=2) {
  std::printf(" USAGE:  basic modelfile\n");
  return EXIT_FAILURE;
}
```

**Użycie:**
```bash
./basic model/humanoid/humanoid.xml
```

---

### Krok 2: Załadowanie modelu

```cpp
char error[1000] = "Could not load binary model";

// Sprawdź rozszerzenie pliku
if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
  // Plik binarny (.mjb)
  m = mj_loadModel(argv[1], 0);
} else {
  // Plik XML (.xml)
  m = mj_loadXML(argv[1], 0, error, 1000);
}

if (!m) {
  mju_error("Load model error: %s", error);
}
```

**Typy plików:**
- **`.xml`** - Format tekstowy MJCF (MuJoCo Modeling Format)
  - Czytelny dla człowieka
  - Można edytować w edytorze tekstu
  - Wolniejsze ładowanie
  
- **`.mjb`** - Format binarny MuJoCo
  - Znacznie szybsze ładowanie
  - Mniejszy rozmiar pliku
  - Nie można edytować ręcznie

**Konwersja XML → MJB:**
```bash
# W MuJoCo można zapisać model jako binarny:
# (wymaga użycia API)
mj_saveModel(m, "model.mjb", NULL, 0);
```

---

### Krok 3: Inicjalizacja danych

```cpp
// Stwórz strukturę danych dla symulacji
d = mj_makeData(m);
```

**Co się dzieje:**
- Alokuje pamięć dla stanu symulacji
- Inicjalizuje wszystkie pola (pozycje, prędkości, siły, etc.)
- Rozmiar zależy od modelu (więcej DOF = więcej pamięci)

---

### Krok 4: Inicjalizacja GLFW (biblioteka okien)

```cpp
// Zainicjalizuj GLFW
if (!glfwInit()) {
  mju_error("Could not initialize GLFW");
}

// Stwórz okno 1200x900
GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
glfwMakeContextCurrent(window);  // Użyj tego okna dla OpenGL
glfwSwapInterval(1);             // V-Sync (synchronizacja z odświeżaniem)
```

**GLFW** to biblioteka do tworzenia okien i obsługi wejścia (mysz, klawiatura)

**V-Sync:**
- `glfwSwapInterval(1)` - synchronizuj z odświeżaniem monitora (60Hz → 60 FPS)
- `glfwSwapInterval(0)` - brak ograniczenia FPS (maksymalna prędkość)

---

### Krok 5: Inicjalizacja wizualizacji MuJoCo

```cpp
// Zainicjalizuj struktury domyślnymi wartościami
mjv_defaultCamera(&cam);   // Domyślna kamera
mjv_defaultOption(&opt);   // Domyślne opcje wizualizacji
mjv_defaultScene(&scn);    // Domyślna scena
mjr_defaultContext(&con);  // Domyślny kontekst GPU

// Stwórz scenę (max 2000 obiektów wizualnych)
mjv_makeScene(m, &scn, 2000);

// Stwórz kontekst GPU (font 150%)
mjr_makeContext(m, &con, mjFONTSCALE_150);
```

**Parametry:**
- `2000` w `mjv_makeScene` - maksymalna liczba geometrii do renderowania
  - Za mało → niektóre obiekty nie będą widoczne
  - Za dużo → marnowanie pamięci GPU
  
- `mjFONTSCALE_150` - rozmiar czcionki (50% większy niż standardowy)

---

### Krok 6: Podłączenie callbacków

```cpp
glfwSetKeyCallback(window, keyboard);              // Klawiatura
glfwSetCursorPosCallback(window, mouse_move);      // Ruch myszy
glfwSetMouseButtonCallback(window, mouse_button);  // Kliknięcia
glfwSetScrollCallback(window, scroll);             // Scroll
```

---

### Krok 7: Główna pętla symulacji

```cpp
// Pętla - dopóki okno jest otwarte
while (!glfwWindowShouldClose(window)) {
  // Pobierz czas początkowy symulacji
  mjtNum simstart = d->time;

  // PĘTLA KROKÓW SYMULACJI
  // Wykonaj wiele kroków symulacji (żeby nadążyć za czasem rzeczywistym)
  while (d->time - simstart < 1.0/60.0) {  // 1/60s = jeden frame przy 60 FPS
    mj_step(m, d);  // WYKONAJ KROK SYMULACJI
  }

  // Pobierz rozmiar framebuffera (może być różny od rozmiaru okna)
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);

  // RENDEROWANIE
  mjrRect viewport = {0, 0, width, height};
  
  // Zaktualizuj scenę (oblicz co trzeba narysować)
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
  
  // Narysuj scenę
  mjr_render(viewport, &scn, &con);

  // OVERLAY - teksty na ekranie (opcjonalnie)
  // Można tu dodać informacje (FPS, czas symulacji, etc.)

  // Zamień bufory (pokaż narysowany obraz)
  glfwSwapBuffers(window);
  
  // Obsłuż eventy (mysz, klawiatura)
  glfwPollEvents();
}
```

**Kluczowe funkcje:**

#### `mj_step(m, d)` - Krok symulacji
```cpp
// Wykonuje jeden krok symulacji (timestep określony w modelu)
// Sekwencja:
// 1. Oblicz siły (grawitacja, siłowniki, sprężyny, etc.)
// 2. Wykryj kolizje
// 3. Rozwiąż ograniczenia (constraints)
// 4. Zaktualizuj pozycje i prędkości
mj_step(m, d);
```

#### `mjv_updateScene()` - Aktualizacja sceny
```cpp
// Przelicza co trzeba narysować
// - Pozycje wszystkich geometrii
// - Kolory
// - Dodatkowe elementy (kontakty, siły, stawy)
mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
```

**Parametry:**
- `&opt` - opcje wizualizacji (co pokazywać)
- `NULL` - perturbacje (można pokazać siły zewnętrzne)
- `&cam` - kamera
- `mjCAT_ALL` - kategorie do pokazania (wszystkie)
- `&scn` - scena wyjściowa

#### `mjr_render()` - Renderowanie
```cpp
// Rysuje scenę na GPU
mjr_render(viewport, &scn, &con);
```

---

### Krok 8: Sprzątanie

```cpp
// Zwolnij zasoby MuJoCo
mjv_freeScene(&scn);
mjr_freeContext(&con);

// Zwolnij dane i model
mj_deleteData(d);
mj_deleteModel(m);

// Zamknij GLFW
glfwTerminate();

return EXIT_SUCCESS;
```

**Ważne:** Zawsze zwalniaj zasoby w odwrotnej kolejności niż je tworzyłeś!

---

## Kompilacja i uruchomienie

### Linux/macOS:

```bash
# Kompilacja
g++ -std=c++11 \
    sample/basic.cc \
    -I include \
    -L lib \
    -lmujoco \
    -lglfw \
    -o basic

# Uruchomienie
./basic model/humanoid/humanoid.xml
```

### Windows (Visual Studio):

```batch
cl /std:c++17 ^
   /I include ^
   sample\basic.cc ^
   lib\mujoco.lib ^
   lib\glfw3.lib ^
   /Fe:basic.exe

basic.exe model\humanoid\humanoid.xml
```

### CMake (uniwersalne):

```cmake
cmake_minimum_required(VERSION 3.16)
project(basic)

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(basic sample/basic.cc)
target_link_libraries(basic mujoco::mujoco glfw)
```

```bash
mkdir build && cd build
cmake ..
cmake --build .
./basic ../model/humanoid/humanoid.xml
```

---

## Rozszerzenia dla studentów

### 1. Dodaj własne sterowanie

```cpp
// W głównej pętli, przed mj_step():
void my_controller(const mjModel* m, mjData* d) {
  // Prosty regulator PD
  double Kp = 100.0;
  double Kd = 10.0;
  
  for (int i = 0; i < m->nu; i++) {
    // Oblicz błąd (dla uproszczenia: cel = 0)
    double error = 0 - d->qpos[m->jnt_qposadr[i]];
    double derror = 0 - d->qvel[m->jnt_dofadr[i]];
    
    // Sygnał sterujący
    d->ctrl[i] = Kp * error + Kd * derror;
  }
}

// W pętli:
while (d->time - simstart < 1.0/60.0) {
  my_controller(m, d);  // Dodaj kontroler
  mj_step(m, d);
}
```

### 2. Wyświetl informacje na ekranie

```cpp
// Po mjr_render(), dodaj:
char text[100];
snprintf(text, 100, "Time: %.2f\nHeight: %.2f", d->time, d->qpos[2]);

mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, text, NULL, &con);
```

### 3. Zapisz wideo

```cpp
// Alokuj bufor dla pikseli
unsigned char* rgb = (unsigned char*)malloc(3 * width * height);

// W pętli, po renderowaniu:
mjr_readPixels(rgb, NULL, viewport, &con);

// Zapisz do pliku (np. używając stb_image_write)
stbi_write_png("frame.png", width, height, 3, rgb, 3 * width);

free(rgb);
```

---

## Typowe błędy

### 1. Czarny ekran

**Przyczyna:** Model nie został załadowany lub kamera jest źle ustawiona

**Rozwiązanie:**
```cpp
// Sprawdź czy model się załadował
if (!m) printf("Model failed to load!\n");

// Zresetuj kamerę
cam.lookat[0] = 0;
cam.lookat[1] = 0;
cam.lookat[2] = 1;
cam.distance = 5.0;
cam.azimuth = 90;
cam.elevation = -20;
```

### 2. Segmentation fault

**Przyczyna:** Próba dostępu do niezainicjalizowanych struktur

**Rozwiązanie:** Sprawdź kolejność inicjalizacji:
```cpp
// POPRAWNA kolejność:
m = mj_loadXML(...);        // 1. Model
d = mj_makeData(m);         // 2. Data (wymaga m)
mjv_makeScene(m, &scn, ...); // 3. Scena (wymaga m)
mjr_makeContext(m, &con, ...); // 4. Kontekst (wymaga m)
```

### 3. Symulacja jest zbyt wolna/szybka

**Rozwiązanie:** Dostosuj krok czasowy w XML:
```xml
<option timestep="0.002"/>  <!-- 2ms per step -->
```

Lub w głównej pętli zmień cel FPS:
```cpp
while (d->time - simstart < 1.0/30.0) { // 30 FPS zamiast 60
  mj_step(m, d);
}
```

---

## Podsumowanie

### Kluczowe koncepty:

✅ **mjModel** - opis robota (niezmienny)  
✅ **mjData** - stan symulacji (zmienny)  
✅ **mj_step()** - główna funkcja symulacji  
✅ **Pętla render** - renderuj szybciej niż symuluj (60 FPS render, wiele kroków sim)  
✅ **Callbacki** - obsługa interakcji użytkownika  

### Przepływ programu:

```
Załaduj model → Stwórz dane → Inicjalizuj GLFW
     ↓
Stwórz okno → Inicjalizuj wizualizację
     ↓
┌─────────────────┐
│  Główna pętla   │
│  while (okno)   │
│    ├─ Symuluj   │ ← mj_step()
│    ├─ Renderuj  │ ← mjr_render()
│    └─ Events    │ ← glfwPollEvents()
└─────────────────┘
     ↓
Zwolnij zasoby → Zakończ
```

---

**Dokumentacja przygotowana dla studentów Politechniki Rzeszowskiej** 🎓

**Następne kroki:**
- Zmodyfikuj program aby dodać własny kontroler
- Spróbuj załadować różne modele
- Dodaj wyświetlanie dodatkowych informacji
- Zobacz `sample/` katalog dla więcej przykładów
