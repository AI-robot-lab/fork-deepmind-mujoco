# Przewodnik Studenta - MuJoCo dla Politechniki Rzeszowskiej

## Spis treści
1. [Wprowadzenie](#wprowadzenie)
2. [Jak zacząć - krok po kroku](#jak-zacząć---krok-po-kroku)
3. [Podstawowe koncepcje](#podstawowe-koncepcje)
4. [Pierwszy program](#pierwszy-program)
5. [Praca z modelami](#praca-z-modelami)
6. [Symulacja i sterowanie](#symulacja-i-sterowanie)
7. [Wizualizacja](#wizualizacja)
8. [Typowe błędy i jak ich unikać](#typowe-błędy-i-jak-ich-unikać)
9. [Zasoby do nauki](#zasoby-do-nauki)

## Wprowadzenie

Witaj w świecie symulacji robotycznych! Ten przewodnik został przygotowany specjalnie dla studentów Politechniki Rzeszowskiej, aby ułatwić naukę i pracę z biblioteką MuJoCo.

### Czym jest MuJoCo?

MuJoCo (Multi-Joint dynamics with Contact) to profesjonalny silnik fizyki używany przez wiodące ośrodki badawcze na całym świecie, w tym:
- Google DeepMind
- OpenAI
- Uniwersytet Stanford
- MIT

**Dlaczego jest ważny?**
- Pozwala symulować roboty przed ich zbudowaniem
- Umożliwia testowanie algorytmów sterowania bez ryzyka uszkodzenia sprzętu
- Jest znacznie szybszy niż testy na prawdziwym robocie
- Używany w najnowocześniejszych badaniach nad robotyką i AI

### Wymagania wstępne

**Wiedza:**
- Podstawy programowania w Pythonie lub C++
- Podstawy fizyki (mechanika, kinematyka)
- Podstawy algebry liniowej (wektory, macierze)

**Oprogramowanie:**
- Python 3.9 lub nowszy
- System operacyjny: Linux, Windows, lub macOS
- Zalecane: Środowisko Jupyter Notebook dla eksperymentów

## Jak zacząć - krok po kroku

### Krok 1: Instalacja

#### Metoda A: Instalacja Python (zalecana dla początkujących)

```bash
# Utwórz wirtualne środowisko (opcjonalnie, ale zalecane)
python -m venv mujoco_env
source mujoco_env/bin/activate  # Na Windows: mujoco_env\Scripts\activate

# Zainstaluj MuJoCo
pip install mujoco

# Zainstaluj dodatkowe narzędzia
pip install numpy matplotlib ipython jupyter
```

#### Metoda B: Pobranie binarek

1. Przejdź do https://github.com/google-deepmind/mujoco/releases
2. Pobierz najnowszą wersję dla swojego systemu
3. Rozpakuj do wybranego katalogu

### Krok 2: Weryfikacja instalacji

Utwórz plik `test_instalacji.py`:

```python
import mujoco
import numpy as np

print(f"MuJoCo version: {mujoco.__version__}")
print("Instalacja przebiegła pomyślnie!")

# Prosty test - stworzenie pustego modelu
model = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody>
    <body>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
""")

print(f"Model utworzony: {model.nbody} bodies")
```

Uruchom: `python test_instalacji.py`

Jeśli zobaczysz komunikat o sukcesie, wszystko działa poprawnie!

### Krok 3: Pierwsza symulacja

Uruchom aplikację `simulate` (jeśli używasz binarek) lub użyj Python viewera:

```python
import mujoco
import mujoco.viewer

# Załaduj przykładowy model humanoid
model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# Uruchom interaktywny viewer
mujoco.viewer.launch(model, data)
```

## Podstawowe koncepcje

### 1. Model vs Data - kluczowa różnica

#### MjModel (Model)
```
┌─────────────────────┐
│   MjModel           │
│  (NIEZMIENNY)       │
├─────────────────────┤
│ - Geometria         │
│ - Masy              │
│ - Współczynniki     │
│   tarcia            │
│ - Parametry         │
│   siłowników        │
│ - Ograniczenia      │
└─────────────────────┘
```

**Analogia:** Model to jak instrukcja budowy robota - opisuje CO jest zbudowane, ale nie opisuje jak robot się OBECNIE porusza.

#### MjData (Data)
```
┌─────────────────────┐
│   MjData            │
│  (ZMIENNY)          │
├─────────────────────┤
│ - Pozycje (qpos)    │
│ - Prędkości (qvel)  │
│ - Siły kontrolne    │
│ - Siły kontaktowe   │
│ - Energie           │
└─────────────────────┘
```

**Analogia:** Data to jak "stan" robota w danym momencie - gdzie są jego części, jak szybko się poruszają, jakie siły działają.

### 2. Podstawowe struktury danych

```python
# STRUKTURA MODELU
model.nq          # Liczba współrzędnych pozycji (q - position)
model.nv          # Liczba współrzędnych prędkości (v - velocity)
model.nu          # Liczba sterowania (u - control/actuation)
model.nbody       # Liczba ciał sztywnych
model.njnt        # Liczba stawów (joints)

# STRUKTURA DATA
data.qpos         # Wektor pozycji [nq]
data.qvel         # Wektor prędkości [nv]
data.ctrl         # Wektor sterowania [nu]
data.qacc         # Wektor przyspieszeń [nv]
data.xpos         # Pozycje ciał w przestrzeni kartezjańskiej [nbody x 3]
data.xmat         # Orientacje ciał (macierze obrotu) [nbody x 9]
```

### 3. Cykl symulacji

```
┌──────────────┐
│   START      │
└──────┬───────┘
       │
       ▼
┌──────────────────┐
│ 1. Ustaw ctrl    │  ◄── Tutaj ustawiasz sygnały sterujące
│    (data.ctrl)   │
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│ 2. mj_step()     │  ◄── Główna funkcja symulacji
│    - Oblicza     │      (jeden krok czasowy)
│      dynamikę    │
│    - Wykrywa     │
│      kontakty    │
│    - Aktualizuje │
│      stan        │
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│ 3. Odczyt stanu  │  ◄── Odczytujesz nowe pozycje, prędkości
│    (data.qpos,   │
│     data.qvel)   │
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│ 4. Renderowanie  │  ◄── Opcjonalnie: wyświetl lub zapisz
└──────┬───────────┘
       │
       ▼
    Powtórz
```

## Pierwszy program

### Program 1: Spadająca kula (grawitacja)

Cel: Zrozumienie podstawowej symulacji fizycznej

```python
"""
Program demonstrujący podstawową symulację spadającej kuli.
Pokazuje jak:
1. Stworzyć prosty model XML
2. Zainicjalizować symulację
3. Wykonać kroki symulacji
4. Odczytać wyniki
"""

import mujoco
import numpy as np
import matplotlib.pyplot as plt

# KROK 1: Definicja modelu w formacie XML
# ----------------------------------------
# XML to format, w którym opisujemy robota/scenę
model_xml = """
<mujoco>
  <!-- Opcje symulacji -->
  <option gravity="0 0 -9.81" timestep="0.001"/>
  
  <!-- Świat - zawiera wszystkie obiekty -->
  <worldbody>
    <!-- Podłoże -->
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
    
    <!-- Spadająca kula -->
    <body name="ball" pos="0 0 2">  <!-- Początkowa pozycja: 2m nad ziemią -->
      <freejoint/>  <!-- Swobodny ruch w przestrzeni 3D -->
      <geom name="ball_geom" type="sphere" size="0.1" rgba="1 0 0 1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

# KROK 2: Stworzenie modelu i danych
# -----------------------------------
model = mujoco.MjModel.from_xml_string(model_xml)
data = mujoco.MjData(model)

# KROK 3: Przygotowanie do zbierania danych
# ------------------------------------------
n_steps = 1000  # Liczba kroków symulacji
timestep = model.opt.timestep  # Krok czasowy z modelu (0.001s)

# Tablice do zapisywania historii
time_history = []
height_history = []
velocity_history = []

# KROK 4: Główna pętla symulacji
# -------------------------------
print("Rozpoczynam symulację spadającej kuli...")
print(f"Krok czasowy: {timestep}s")
print(f"Całkowity czas: {n_steps * timestep}s")

for i in range(n_steps):
    # Wykonaj jeden krok symulacji
    mujoco.mj_step(model, data)
    
    # Zbierz dane (co 10 kroków, żeby nie zbierać za dużo)
    if i % 10 == 0:
        time_history.append(data.time)
        # data.qpos[2] to wysokość Z kuli (trzecia współrzędna pozycji)
        height_history.append(data.qpos[2])
        # data.qvel[2] to prędkość w osi Z
        velocity_history.append(data.qvel[2])

# KROK 5: Analiza wyników
# ------------------------
print("\nWyniki symulacji:")
print(f"Początkowa wysokość: {height_history[0]:.3f}m")
print(f"Końcowa wysokość: {height_history[-1]:.3f}m")
print(f"Maksymalna prędkość: {min(velocity_history):.3f}m/s")

# Obliczenia teoretyczne dla weryfikacji
# Dla swobodnego spadku: v = sqrt(2 * g * h)
h = height_history[0] - height_history[-1]
v_theoretical = np.sqrt(2 * 9.81 * h)
print(f"Teoretyczna prędkość uderzenia: {v_theoretical:.3f}m/s")

# KROK 6: Wizualizacja
# --------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Wykres wysokości
ax1.plot(time_history, height_history, 'b-', linewidth=2)
ax1.set_xlabel('Czas [s]')
ax1.set_ylabel('Wysokość [m]')
ax1.set_title('Wysokość kuli w czasie')
ax1.grid(True)

# Wykres prędkości
ax2.plot(time_history, velocity_history, 'r-', linewidth=2)
ax2.set_xlabel('Czas [s]')
ax2.set_ylabel('Prędkość [m/s]')
ax2.set_title('Prędkość kuli w czasie')
ax2.grid(True)

plt.tight_layout()
plt.savefig('spadajaca_kula.png')
print("\nWykres zapisany jako 'spadajaca_kula.png'")
```

### Program 2: Sterowanie siłownikiem

Cel: Zrozumienie jak kontrolować robota

```python
"""
Program demonstrujący sterowanie prostym wahadłem.
Pokazuje jak:
1. Dodać siłownik (actuator) do modelu
2. Wysyłać sygnały sterujące
3. Obserwować odpowiedź systemu
"""

import mujoco
import numpy as np
import matplotlib.pyplot as plt

# Model wahadła z silnikiem
model_xml = """
<mujoco>
  <option timestep="0.001"/>
  
  <worldbody>
    <!-- Punkt mocowania -->
    <body name="pendulum" pos="0 0 1">
      <!-- Staw obrotowy wokół osi Y -->
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.1"/>
      
      <!-- Ramię wahadła -->
      <geom name="arm" type="capsule" fromto="0 0 0 0 0 -0.5" 
            size="0.02" rgba="0 0 1 1" mass="1"/>
      
      <!-- Ciężarek na końcu -->
      <body pos="0 0 -0.5">
        <geom name="weight" type="sphere" size="0.05" 
              rgba="1 0 0 1" mass="0.5"/>
      </body>
    </body>
  </worldbody>
  
  <!-- SIŁOWNIK: Urządzenie wykonawcze generujące moment obrotowy -->
  <actuator>
    <motor name="motor" joint="hinge" gear="1" ctrllimited="true" 
           ctrlrange="-2 2"/>
  </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(model_xml)
data = mujoco.MjData(model)

# Parametry symulacji
n_steps = 5000
time_history = []
angle_history = []
velocity_history = []
control_history = []

print("Symulacja sterowanego wahadła:")
print(f"Liczba siłowników: {model.nu}")
print(f"Zakres sterowania: {model.actuator_ctrlrange[0]}")

for i in range(n_steps):
    # STRATEGIA STEROWANIA: Oscylacja sinusoidalna
    # ctrl[0] bo mamy jeden siłownik (motor)
    data.ctrl[0] = 1.0 * np.sin(2 * np.pi * data.time)
    
    # Wykonaj krok symulacji
    mujoco.mj_step(model, data)
    
    # Zbierz dane
    if i % 10 == 0:
        time_history.append(data.time)
        angle_history.append(data.qpos[0])  # Kąt wahadła
        velocity_history.append(data.qvel[0])  # Prędkość kątowa
        control_history.append(data.ctrl[0])  # Sygnał sterujący

# Wizualizacja
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

ax1.plot(time_history, control_history, 'g-', linewidth=2, label='Sygnał sterujący')
ax1.set_ylabel('Moment [Nm]')
ax1.set_title('Sygnał sterujący')
ax1.grid(True)
ax1.legend()

ax2.plot(time_history, np.rad2deg(angle_history), 'b-', linewidth=2, label='Kąt')
ax2.set_ylabel('Kąt [°]')
ax2.set_title('Kąt wychylenia wahadła')
ax2.grid(True)
ax2.legend()

ax3.plot(time_history, velocity_history, 'r-', linewidth=2, label='Prędkość kątowa')
ax3.set_xlabel('Czas [s]')
ax3.set_ylabel('Prędkość [rad/s]')
ax3.set_title('Prędkość kątowa')
ax3.grid(True)
ax3.legend()

plt.tight_layout()
plt.savefig('sterowane_wahadlo.png')
print("Wykres zapisany jako 'sterowane_wahadlo.png'")
```

## Praca z modelami

### Format XML (MJCF)

MuJoCo używa formatu XML (MJCF - MuJoCo Modeling Format) do definiowania modeli.

#### Podstawowa struktura:

```xml
<mujoco>
  <!-- 1. OPCJE GLOBALNE -->
  <option timestep="0.001" gravity="0 0 -9.81"/>
  
  <!-- 2. ZASOBY (tekstury, siatki 3D, materiały) -->
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"/>
    <material name="grid_mat" texture="grid"/>
  </asset>
  
  <!-- 3. ŚWIAT I OBIEKTY -->
  <worldbody>
    <!-- Podłoże -->
    <geom name="floor" type="plane" material="grid_mat"/>
    
    <!-- Robot/obiekt -->
    <body name="robot" pos="0 0 1">
      <!-- Geometria wizualna i kolizji -->
      <geom type="box" size="0.1 0.1 0.1"/>
      
      <!-- Stawy -->
      <joint name="joint1" type="hinge"/>
      
      <!-- Zagnieżdżone części -->
      <body name="link2" pos="0 0 0.2">
        <geom type="cylinder" size="0.05 0.1"/>
      </body>
    </body>
  </worldbody>
  
  <!-- 4. SIŁOWNIKI -->
  <actuator>
    <motor joint="joint1" gear="100"/>
  </actuator>
  
  <!-- 5. SENSORY -->
  <sensor>
    <accelerometer site="imu_site"/>
    <gyro site="imu_site"/>
  </sensor>
</mujoco>
```

### Najważniejsze typy geometrii (geom)

| Typ | Opis | Parametr size |
|-----|------|---------------|
| `plane` | Nieskończona płaszczyzna | [3 wartości] |
| `sphere` | Kula | [promień] |
| `capsule` | Kapsuła (cylinder z półkulami) | [promień wysokość] |
| `box` | Prostopadłościan | [pół-szerokość x y z] |
| `cylinder` | Cylinder | [promień wysokość] |
| `mesh` | Dowolna siatka 3D | - (używa asset) |

### Najważniejsze typy stawów (joint)

| Typ | Opis | Stopnie swobody |
|-----|------|-----------------|
| `hinge` | Staw obrotowy (1D) | 1 |
| `slide` | Staw przesuwny (1D) | 1 |
| `ball` | Staw kulowy (3D) | 3 |
| `free` | Swobodny ruch (6D) | 6 |

## Symulacja i sterowanie

### Podstawowe funkcje symulacji

```python
# 1. FORWARD DYNAMICS - Najbardziej podstawowa symulacja
mujoco.mj_step(model, data)
# Co robi: ctrl → forces → accelerations → velocities → positions
# Używaj: Zawsze, gdy chcesz symulować krok w czasie

# 2. FORWARD KINEMATICS - Oblicz pozycje bez dynamiki
mujoco.mj_forward(model, data)
# Co robi: positions → compute all derived quantities (xpos, xmat, etc.)
# Używaj: Gdy zmieniasz ręcznie qpos i chcesz przeliczyć pozycje

# 3. INVERSE KINEMATICS - Znajdź konfigurację dla zadanej pozycji
mujoco.mj_inverse(model, data)
# Co robi: desired end-effector position → joint angles
# Używaj: Gdy znasz gdzie ma być ręka, a chcesz wiedzieć jakie ustawić kąty

# 4. RESET - Zresetuj symulację do stanu początkowego
mujoco.mj_resetData(model, data)
# Co robi: Przywraca wszystkie wartości w data do stanu początkowego
# Używaj: Gdy chcesz zacząć symulację od nowa
```

### Strategia PD (Proportional-Derivative) - podstawowy regulator

```python
"""
Regulator PD to najprostszy i najczęściej używany kontroler.
Wzór: u = Kp * (q_desired - q_actual) + Kd * (0 - qvel_actual)
      |__________________|   |___________________|
       Człon proporcjonalny   Człon różniczkowy
       (jak daleko jesteśmy)  (jak szybko się poruszamy)
"""

def pd_controller(q_desired, q_actual, qvel_actual, Kp, Kd):
    """
    Regulator PD dla pozycji stawów.
    
    Args:
        q_desired: Pożądana pozycja
        q_actual: Aktualna pozycja
        qvel_actual: Aktualna prędkość
        Kp: Wzmocnienie proporcjonalne (jak mocno ciągnąć do celu)
        Kd: Wzmocnienie różniczkowe (jak mocno tłumić ruch)
    
    Returns:
        Sygnał sterujący
    """
    error_position = q_desired - q_actual
    error_velocity = 0 - qvel_actual  # Chcemy by prędkość była 0
    
    control = Kp * error_position + Kd * error_velocity
    return control

# Przykład użycia
for i in range(1000):
    # Steruj pierwszym stawem
    q_desired = np.sin(data.time)  # Sinusoidalna trajektoria
    q_actual = data.qpos[0]
    qvel_actual = data.qvel[0]
    
    data.ctrl[0] = pd_controller(q_desired, q_actual, qvel_actual, 
                                  Kp=10.0, Kd=1.0)
    
    mujoco.mj_step(model, data)
```

## Wizualizacja

### Metoda 1: Interaktywny viewer (najłatwiejsza)

```python
import mujoco.viewer

# Prosta wizualizacja
model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
```

### Metoda 2: Viewer z niestandardowym sterowaniem

```python
import mujoco.viewer

model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# Funkcja kontrolna wywoływana w każdym kroku
def controller(model, data):
    # Twoje sterowanie tutaj
    data.ctrl[0] = np.sin(data.time)

# Uruchom viewer z kontrolerem
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        controller(model, data)
        mujoco.mj_step(model, data)
        viewer.sync()
```

### Metoda 3: Renderowanie offline (do filmów/zdjęć)

```python
import mujoco

model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# Inicjalizuj renderer
renderer = mujoco.Renderer(model, height=480, width=640)

frames = []
for i in range(100):
    mujoco.mj_step(model, data)
    
    # Renderuj klatkę
    renderer.update_scene(data)
    pixels = renderer.render()
    frames.append(pixels)

# Zapisz jako film (wymaga imageio)
import imageio
imageio.mimsave('symulacja.mp4', frames, fps=30)
```

## Typowe błędy i jak ich unikać

### Błąd 1: Niestabilna symulacja (model "eksploduje")

**Objawy:** Model zaczyna dziko skakać, wartości pozycji rosną do nieskończoności

**Przyczyny:**
1. Za duży krok czasowy (`timestep`)
2. Za małe tłumienie
3. Zbyt sztywne sprężyny

**Rozwiązanie:**
```xml
<!-- Zmniejsz krok czasowy -->
<option timestep="0.001"/>  <!-- Zamiast 0.01 -->

<!-- Dodaj tłumienie do stawów -->
<joint damping="0.5"/>

<!-- Zmień parametry kontaktu (bardziej miękkie) -->
<option>
  <flag contact="enable"/>
</option>
<geom solimp="0.9 0.95 0.001" solref="0.02 1"/>
```

### Błąd 2: Model przechodzi przez podłogę

**Przyczyny:**
1. Brak kolizji dla podłoża
2. Nieprawidłowe parametry kontaktu

**Rozwiązanie:**
```xml
<!-- Upewnij się, że podłoże ma geom -->
<geom name="floor" type="plane" size="10 10 0.1"/>

<!-- Sprawdź czy kolizje są włączone -->
<option>
  <flag contact="enable"/>
</option>
```

### Błąd 3: "qpos is out of bounds"

**Przyczyny:**
Próba ustawienia pozycji stawu poza jego ograniczeniami

**Rozwiązanie:**
```python
# Sprawdź ograniczenia
print("Zakres stawu:", model.jnt_range[joint_id])

# Przytnij wartości
q_desired = np.clip(q_desired, model.jnt_range[joint_id, 0], 
                               model.jnt_range[joint_id, 1])
```

### Błąd 4: Siłownik nie działa

**Przyczyny:**
1. Brak połączenia siłownika ze stawem
2. Zbyt małe `gear` (przełożenie)
3. Sterowanie poza zakresem

**Rozwiązanie:**
```xml
<!-- Sprawdź czy actuator jest połączony z joint -->
<actuator>
  <motor name="motor1" joint="joint1" gear="100"/>
  <!-- gear="100" oznacza że 1 jednostka ctrl = 100 Nm momentu -->
</actuator>
```

## Zasoby do nauki

### Dokumentacja
1. **Oficjalna dokumentacja:** https://mujoco.readthedocs.io
2. **Forum użytkowników:** https://github.com/google-deepmind/mujoco/discussions
3. **Przykłady kodu:** https://github.com/google-deepmind/mujoco/tree/main/python

### Tutoriale wideo
1. "MuJoCo Basics" - Tutorial DeepMind
2. Colab notebooks (linki w README_PL.md)

### Książki i artykuły
1. "Synthesis and stabilization of complex behaviors through online trajectory optimization" - Todorov et al. (oryginalny artykuł o MuJoCo)
2. "DeepMind Control Suite" - Tassa et al.

### Powiązane technologie warte poznania
1. **dm_control** - wrapper MuJoCo od DeepMind
2. **Gymnasium** - standardowe środowiska RL (zawiera envs z MuJoCo)
3. **Stable Baselines3** - gotowe algorytmy uczenia przez wzmacnianie

## Podsumowanie dla studenta

### Co powinieneś umieć po przeczytaniu tego przewodnika:

✅ Zainstalować MuJoCo  
✅ Stworzyć prosty model w XML  
✅ Uruchomić symulację  
✅ Wysłać sygnały sterujące do siłowników  
✅ Odczytać stan robota (pozycje, prędkości)  
✅ Wizualizować wyniki  
✅ Debugować podstawowe problemy  

### Następne kroki:

1. **Tydzień 1-2:** Przećwicz przykłady z tego przewodnika
2. **Tydzień 3-4:** Stwórz własny prosty model (np. robot na kołach)
3. **Tydzień 5-6:** Zaimplementuj kontroler (PD, MPC, lub RL)
4. **Tydzień 7-8:** Przejdź do przewodnika Unitree G1 (UNITREE_G1_PRZEWODNIK.md)

### Pytania? Problemy?

1. Sprawdź [typowe błędy](#typowe-błędy-i-jak-ich-unikać)
2. Przejrzyj dokumentację: https://mujoco.readthedocs.io
3. Zapytaj na forum: https://github.com/google-deepmind/mujoco/discussions

**Powodzenia w nauce robotyki!** 🤖
