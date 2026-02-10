# Przewodnik Unitree G1 EDU-U6 - Zastosowanie MuJoCo

## Spis treści
1. [Wprowadzenie do Unitree G1](#wprowadzenie-do-unitree-g1)
2. [Dlaczego symulacja jest ważna](#dlaczego-symulacja-jest-ważna)
3. [Przygotowanie środowiska](#przygotowanie-środowiska)
4. [Model Unitree G1 w MuJoCo](#model-unitree-g1-w-mujoco)
5. [Podstawowe operacje](#podstawowe-operacje)
6. [Zaawansowane aplikacje](#zaawansowane-aplikacje)
7. [Workflow: od symulacji do prawdziwego robota](#workflow-od-symulacji-do-prawdziwego-robota)
8. [Projekty przykładowe](#projekty-przykładowe)

## Wprowadzenie do Unitree G1

### Czym jest Unitree G1 EDU-U6?

**Unitree G1** to zaawansowany robot humanoidalny zaprojektowany dla badań i edukacji. Wersja **EDU-U6** oferuje pełny dostęp do wszystkich systemów robota.

**Specyfikacja:**
- **Wysokość:** ~130 cm
- **Waga:** ~35 kg
- **Stopnie swobody:** ~40 DOF (Degrees of Freedom)
  - Tułów: 3 DOF (pitch, roll, yaw)
  - Ręce: 2 × 7 DOF (ramię + dłoń)
  - Nogi: 2 × 6 DOF (biodro, kolano, stopa)
  - Głowa: 2 DOF (pan, tilt)
- **Siłowniki:** Wysokopowtarzalne motory elektryczne z reduktorami
- **Sensory:**
  - IMU (Inertial Measurement Unit) - akcelerometr, żyroskop
  - Enkodery w każdym stawie
  - Czujniki siły w stopach
  - Kamery (opcjonalne)

### Kluczowe możliwości

1. **Lokomocja dwunożna** - chodzenie, bieganie, wchodzenie po schodach
2. **Manipulacja obiektami** - chwytanie, przenoszenie
3. **Interakcja człowiek-robot** - bezpieczna współpraca
4. **Uczenie maszynowe** - trenowanie zachowań przez RL

## Dlaczego symulacja jest ważna

### Korzyści z symulacji przed pracą z prawdziwym robotem:

| Aspekt | Prawdziwy robot | Symulacja |
|--------|----------------|-----------|
| **Bezpieczeństwo** | Ryzyko uszkodzenia | Całkowicie bezpieczne |
| **Koszt** | Zużycie baterii, części | Darmowe |
| **Szybkość** | Czas rzeczywisty | Można przyspieszyć |
| **Iteracje** | Ograniczone | Nieskończone |
| **Debug** | Trudny | Pełny dostęp do stanu |
| **Równoległość** | Jeden robot | Wiele instancji naraz |

### Typowy przepływ pracy (workflow):

```
1. PROJEKTOWANIE         2. SYMULACJA          3. WALIDACJA         4. DEPLOYMENT
   ┌─────────┐              ┌─────────┐           ┌─────────┐          ┌─────────┐
   │ Pomysł  │──────────▶   │ MuJoCo  │──────────▶│ Testy   │─────────▶│ Robot   │
   │ na ruch │              │ Sim     │           │ w sim   │          │ rzeczy- │
   └─────────┘              └─────────┘           └─────────┘          │ wisty   │
                                 │                                      └─────────┘
                                 │
                                 ▼
                            Iteracje
                           (szybkie!)
```

### Przykłady zastosowań:

✅ **Bezpieczne testowanie nowych algorytmów**  
   Zanim puścisz regulator na prawdziwym robocie, sprawdź czy działa w symulacji

✅ **Uczenie przez wzmacnianie (RL)**  
   Trenuj polityki sterowania przez miliony iteracji (niemożliwe na prawdziwym robocie)

✅ **Planowanie trajektorii**  
   Znajdź optymalną ścieżkę ruchu, a potem wykonaj na robocie

✅ **Testowanie odporności**  
   Symuluj zakłócenia (popchnięcia, śliskie podłoże) i sprawdź czy robot przetrwa

✅ **Edukacja i demonstracje**  
   Pokaż działanie bez dostępu do prawdziwego sprzętu

## Przygotowanie środowiska

### Instalacja podstawowa

```bash
# 1. Utwórz dedykowane środowisko Python
python -m venv unitree_g1_env
source unitree_g1_env/bin/activate  # Windows: unitree_g1_env\Scripts\activate

# 2. Zainstaluj MuJoCo
pip install mujoco

# 3. Zainstaluj dodatkowe biblioteki
pip install numpy matplotlib scipy
pip install mediapy  # Do zapisywania wideo
pip install gymnasium  # Standardowe RL environments (opcjonalne)
```

### Struktura projektu (zalecana)

```
unitree_g1_project/
├── models/
│   ├── unitree_g1.xml           # Model MJCF robota
│   ├── unitree_g1_simplified.xml # Uproszczony model (szybsza symulacja)
│   └── scenes/
│       ├── flat_ground.xml      # Scena: płaski teren
│       ├── stairs.xml           # Scena: schody
│       └── obstacles.xml        # Scena: przeszkody
├── controllers/
│   ├── pd_controller.py         # Regulator PD
│   ├── walking_controller.py   # Kontroler chodzenia
│   └── balance_controller.py   # Kontroler równowagi
├── simulations/
│   ├── test_balance.py          # Test równowagi
│   ├── test_walking.py          # Test chodzenia
│   └── test_manipulation.py    # Test manipulacji
├── utils/
│   ├── kinematics.py            # Funkcje kinematyczne
│   ├── visualization.py         # Narzędzia wizualizacji
│   └── data_logger.py           # Zapis danych
└── notebooks/
    ├── 01_basic_control.ipynb   # Tutorial podstaw
    └── 02_walking_gait.ipynb    # Tutorial chodu
```

## Model Unitree G1 w MuJoCo

### Tworzenie uproszczonego modelu Unitree G1

Na potrzeby edukacji, stwórzmy uproszczony model humanoidalny inspirowany Unitree G1:

```xml
<!-- unitree_g1_simplified.xml -->
<mujoco model="Unitree_G1_Simplified">
  
  <!-- OPCJE SYMULACJI -->
  <option timestep="0.002" iterations="50" solver="Newton" tolerance="1e-10"/>
  
  <!-- KOMPILATOR - ustawienia parsowania -->
  <compiler angle="radian" meshdir="meshes/"/>
  
  <!-- USTAWIENIA WIZUALNE -->
  <visual>
    <map force="0.1" zfar="30"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <quality shadowsize="4096"/>
    <global offwidth="1920" offheight="1080"/>
  </visual>
  
  <!-- ZASOBY -->
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="body_tex" type="cube" builtin="flat" mark="cross" width="128" height="128" 
             rgb1="0.9 0.9 0.9" rgb2="0.9 0.9 0.9" markrgb="0.2 0.2 0.2"/>
    <material name="body_mat" texture="body_tex" texuniform="true" rgba="0.9 0.9 0.9 1"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.1 0.1 0.1" rgb2="0.2 0.2 0.2"/>
    <material name="grid_mat" texture="grid" texrepeat="1 1" texuniform="true" reflectance="0.2"/>
  </asset>
  
  <!-- WARTOŚCI DOMYŚLNE -->
  <default>
    <geom contype="1" conaffinity="1" condim="3" friction="0.9 0.1 0.1" 
          solimp="0.9 0.95 0.001 0.5 2" solref="0.002 1"/>
    <joint damping="0.5" armature="0.01"/>
    <motor ctrlrange="-1 1" ctrllimited="true"/>
    
    <!-- Domyślne ustawienia dla tułowia -->
    <default class="torso">
      <geom type="capsule" material="body_mat"/>
    </default>
    
    <!-- Domyślne ustawienia dla kończyn -->
    <default class="limb">
      <geom type="capsule" material="body_mat"/>
      <joint type="hinge" limited="true"/>
    </default>
  </default>
  
  <!-- ŚWIAT -->
  <worldbody>
    <!-- PODŁOŻE -->
    <light directional="true" pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="20 20 0.1" material="grid_mat"/>
    
    <!-- ROBOT UNITREE G1 -->
    <body name="torso" pos="0 0 1.0">
      <!-- TUŁÓW - główne ciało robota -->
      <freejoint name="root"/>  <!-- 6 DOF: pozycja + orientacja w przestrzeni -->
      <inertial pos="0 0 0" mass="15" diaginertia="0.5 0.5 0.3"/>
      <geom name="torso_geom" type="box" size="0.15 0.1 0.25" rgba="0.3 0.3 0.9 1"/>
      
      <!-- Sensor IMU (akcelerometr i żyroskop) -->
      <site name="imu" pos="0 0 0" size="0.01"/>
      
      <!-- GŁOWA -->
      <body name="head" pos="0 0 0.3">
        <joint name="head_yaw" type="hinge" axis="0 0 1" range="-1.57 1.57" damping="0.1"/>
        <inertial pos="0 0 0.05" mass="1" diaginertia="0.01 0.01 0.01"/>
        <geom name="head_geom" type="sphere" size="0.08" rgba="0.9 0.9 0.9 1"/>
      </body>
      
      <!-- PRAWA NOGA -->
      <body name="right_hip" pos="0 -0.1 -0.1">
        <!-- Biodro: zginanie/prostowanie -->
        <joint name="right_hip_pitch" class="limb" axis="0 1 0" range="-1.57 1.57"/>
        <inertial pos="0 0 -0.15" mass="1.5" diaginertia="0.02 0.02 0.005"/>
        <geom name="right_thigh" class="limb" fromto="0 0 0 0 0 -0.3" size="0.04" rgba="0.3 0.3 0.9 1"/>
        
        <!-- Kolano -->
        <body name="right_knee" pos="0 0 -0.3">
          <joint name="right_knee_pitch" class="limb" axis="0 1 0" range="-2.4 0"/>
          <inertial pos="0 0 -0.15" mass="1.2" diaginertia="0.015 0.015 0.003"/>
          <geom name="right_shin" class="limb" fromto="0 0 0 0 0 -0.3" size="0.035" rgba="0.4 0.4 0.9 1"/>
          
          <!-- Stopa -->
          <body name="right_foot" pos="0 0 -0.3">
            <joint name="right_ankle_pitch" class="limb" axis="0 1 0" range="-0.7 0.7"/>
            <inertial pos="0.05 0 -0.02" mass="0.5" diaginertia="0.005 0.01 0.01"/>
            <geom name="right_foot_geom" type="box" size="0.1 0.05 0.02" 
                  pos="0.05 0 -0.02" rgba="0.2 0.2 0.2 1"/>
            <!-- Sensor siły w stopie -->
            <site name="right_foot_sensor" pos="0.05 0 -0.04" size="0.01"/>
          </body>
        </body>
      </body>
      
      <!-- LEWA NOGA (symetrycznie) -->
      <body name="left_hip" pos="0 0.1 -0.1">
        <joint name="left_hip_pitch" class="limb" axis="0 1 0" range="-1.57 1.57"/>
        <inertial pos="0 0 -0.15" mass="1.5" diaginertia="0.02 0.02 0.005"/>
        <geom name="left_thigh" class="limb" fromto="0 0 0 0 0 -0.3" size="0.04" rgba="0.3 0.3 0.9 1"/>
        
        <body name="left_knee" pos="0 0 -0.3">
          <joint name="left_knee_pitch" class="limb" axis="0 1 0" range="-2.4 0"/>
          <inertial pos="0 0 -0.15" mass="1.2" diaginertia="0.015 0.015 0.003"/>
          <geom name="left_shin" class="limb" fromto="0 0 0 0 0 -0.3" size="0.035" rgba="0.4 0.4 0.9 1"/>
          
          <body name="left_foot" pos="0 0 -0.3">
            <joint name="left_ankle_pitch" class="limb" axis="0 1 0" range="-0.7 0.7"/>
            <inertial pos="0.05 0 -0.02" mass="0.5" diaginertia="0.005 0.01 0.01"/>
            <geom name="left_foot_geom" type="box" size="0.1 0.05 0.02" 
                  pos="0.05 0 -0.02" rgba="0.2 0.2 0.2 1"/>
            <site name="left_foot_sensor" pos="0.05 0 -0.04" size="0.01"/>
          </body>
        </body>
      </body>
      
      <!-- PRAWA RĘKA -->
      <body name="right_shoulder" pos="0 -0.25 0.15">
        <joint name="right_shoulder_pitch" class="limb" axis="0 1 0" range="-3.14 3.14"/>
        <inertial pos="0 0 -0.15" mass="0.8" diaginertia="0.01 0.01 0.002"/>
        <geom name="right_upper_arm" class="limb" fromto="0 0 0 0 0 -0.25" size="0.03" rgba="0.3 0.3 0.9 1"/>
        
        <body name="right_elbow" pos="0 0 -0.25">
          <joint name="right_elbow_pitch" class="limb" axis="0 1 0" range="0 2.4"/>
          <inertial pos="0 0 -0.12" mass="0.6" diaginertia="0.008 0.008 0.002"/>
          <geom name="right_forearm" class="limb" fromto="0 0 0 0 0 -0.2" size="0.025" rgba="0.4 0.4 0.9 1"/>
          
          <!-- Prosta dłoń (chwytarka) -->
          <body name="right_hand" pos="0 0 -0.2">
            <inertial pos="0 0 -0.03" mass="0.3" diaginertia="0.003 0.003 0.001"/>
            <geom name="right_hand_geom" type="box" size="0.04 0.03 0.06" 
                  pos="0 0 -0.03" rgba="0.2 0.2 0.2 1"/>
            <site name="right_hand_tip" pos="0 0 -0.06" size="0.01"/>
          </body>
        </body>
      </body>
      
      <!-- LEWA RĘKA (symetrycznie) -->
      <body name="left_shoulder" pos="0 0.25 0.15">
        <joint name="left_shoulder_pitch" class="limb" axis="0 1 0" range="-3.14 3.14"/>
        <inertial pos="0 0 -0.15" mass="0.8" diaginertia="0.01 0.01 0.002"/>
        <geom name="left_upper_arm" class="limb" fromto="0 0 0 0 0 -0.25" size="0.03" rgba="0.3 0.3 0.9 1"/>
        
        <body name="left_elbow" pos="0 0 -0.25">
          <joint name="left_elbow_pitch" class="limb" axis="0 1 0" range="0 2.4"/>
          <inertial pos="0 0 -0.12" mass="0.6" diaginertia="0.008 0.008 0.002"/>
          <geom name="left_forearm" class="limb" fromto="0 0 0 0 0 -0.2" size="0.025" rgba="0.4 0.4 0.9 1"/>
          
          <body name="left_hand" pos="0 0 -0.2">
            <inertial pos="0 0 -0.03" mass="0.3" diaginertia="0.003 0.003 0.001"/>
            <geom name="left_hand_geom" type="box" size="0.04 0.03 0.06" 
                  pos="0 0 -0.03" rgba="0.2 0.2 0.2 1"/>
            <site name="left_hand_tip" pos="0 0 -0.06" size="0.01"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  
  <!-- SIŁOWNIKI (ACTUATORS) -->
  <actuator>
    <!-- Głowa -->
    <motor name="motor_head_yaw" joint="head_yaw" gear="20" ctrlrange="-1 1"/>
    
    <!-- Prawa noga -->
    <motor name="motor_right_hip_pitch" joint="right_hip_pitch" gear="150" ctrlrange="-1 1"/>
    <motor name="motor_right_knee_pitch" joint="right_knee_pitch" gear="150" ctrlrange="-1 1"/>
    <motor name="motor_right_ankle_pitch" joint="right_ankle_pitch" gear="100" ctrlrange="-1 1"/>
    
    <!-- Lewa noga -->
    <motor name="motor_left_hip_pitch" joint="left_hip_pitch" gear="150" ctrlrange="-1 1"/>
    <motor name="motor_left_knee_pitch" joint="left_knee_pitch" gear="150" ctrlrange="-1 1"/>
    <motor name="motor_left_ankle_pitch" joint="left_ankle_pitch" gear="100" ctrlrange="-1 1"/>
    
    <!-- Prawa ręka -->
    <motor name="motor_right_shoulder_pitch" joint="right_shoulder_pitch" gear="100" ctrlrange="-1 1"/>
    <motor name="motor_right_elbow_pitch" joint="right_elbow_pitch" gear="80" ctrlrange="-1 1"/>
    
    <!-- Lewa ręka -->
    <motor name="motor_left_shoulder_pitch" joint="left_shoulder_pitch" gear="100" ctrlrange="-1 1"/>
    <motor name="motor_left_elbow_pitch" joint="left_elbow_pitch" gear="80" ctrlrange="-1 1"/>
  </actuator>
  
  <!-- SENSORY -->
  <sensor>
    <!-- IMU w tułowiu -->
    <accelerometer name="accel" site="imu"/>
    <gyro name="gyro" site="imu"/>
    <framequat name="orientation" objtype="site" objname="imu"/>
    
    <!-- Czujniki siły w stopach -->
    <force name="right_foot_force" site="right_foot_sensor"/>
    <force name="left_foot_force" site="left_foot_sensor"/>
    
    <!-- Pozycje końcówek rąk (do manipulacji) -->
    <framepos name="right_hand_pos" objtype="site" objname="right_hand_tip"/>
    <framepos name="left_hand_pos" objtype="site" objname="left_hand_tip"/>
  </sensor>
  
</mujoco>
```

### Zapisz model

Zapisz powyższy kod jako `/home/runner/work/deepmind-mujoco/deepmind-mujoco/model/unitree_g1/unitree_g1_simplified.xml`

## Podstawowe operacje

### 1. Załadowanie i wyświetlenie modelu

```python
"""
Podstawowa wizualizacja modelu Unitree G1.
"""
import mujoco
import mujoco.viewer

# Załaduj model
model = mujoco.MjModel.from_xml_path('model/unitree_g1/unitree_g1_simplified.xml')
data = mujoco.MjData(model)

# Wyświetl informacje o modelu
print("=== Informacje o modelu Unitree G1 ===")
print(f"Liczba ciał (bodies): {model.nbody}")
print(f"Liczba stawów (joints): {model.njnt}")
print(f"Liczba siłowników (actuators): {model.nu}")
print(f"Liczba sensorów: {model.nsensor}")
print(f"Stopnie swobody (DOF): {model.nv}")
print(f"\nNazwy siłowników:")
for i in range(model.nu):
    print(f"  {i}: {model.actuator(i).name}")

# Uruchom viewer
print("\nUruchamiam viewer...")
mujoco.viewer.launch(model, data)
```

### 2. Test równowagi (balansowanie w miejscu)

```python
"""
Test równowagi - robot stoi nieruchomo przy włączonym sterrowaniu PD.
Demonstracja: podstawowy regulator utrzymujący pozycję początkową.
"""
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path('model/unitree_g1/unitree_g1_simplified.xml')
data = mujoco.MjData(model)

# Pozycja docelowa - lekko zgięte kolana (stabilna pozycja stojąca)
target_qpos = np.zeros(model.nq)
# Indeksy dla stawów nóg (zależnie od modelu, sprawdź kolejność!)
# Załóżmy że: right_hip_pitch=7, right_knee_pitch=8, right_ankle_pitch=9
#             left_hip_pitch=10, left_knee_pitch=11, left_ankle_pitch=12
target_qpos[8] = -0.3   # Prawe kolano lekko zgięte
target_qpos[11] = -0.3  # Lewe kolano lekko zgięte

# Wzmocnienia regulatora PD
Kp = 50.0  # Wzmocnienie proporcjonalne
Kd = 5.0   # Wzmocnienie różniczkowe

def balance_controller(model, data):
    """
    Prosty regulator PD dla wszystkich stawów.
    Cel: utrzymanie pozycji docelowej.
    """
    # Dla każdego siłownika
    for i in range(model.nu):
        # Znajdź odpowiadający staw
        joint_id = model.actuator_trnid[i, 0]
        
        # Pobierz aktualny stan
        qpos_actual = data.qpos[joint_id + 7]  # +7 bo pierwsze 7 to freejoint (pos+quat)
        qvel_actual = data.qvel[joint_id + 6]  # +6 bo freejoint to 6 DOF
        
        # Pozycja docelowa
        qpos_desired = target_qpos[joint_id + 7] if joint_id + 7 < len(target_qpos) else 0
        
        # Regulator PD
        error_pos = qpos_desired - qpos_actual
        error_vel = 0 - qvel_actual  # Docelowa prędkość = 0
        
        control_signal = Kp * error_pos + Kd * error_vel
        
        # Ogranicz sygnał do zakresu siłownika
        ctrl_min, ctrl_max = model.actuator_ctrlrange[i]
        data.ctrl[i] = np.clip(control_signal, ctrl_min, ctrl_max)

# Uruchom viewer z kontrolerem
print("Uruchamiam symulację równowagi...")
print("Robot powinien stać stabilnie w miejscu.")
print("Spróbuj go popchnąć myszką (Ctrl + prawy przycisk myszy)!")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        balance_controller(model, data)
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 3. Test podnoszenia nogi (przygotowanie do chodzenia)

```python
"""
Test podnoszenia nogi - przygotowanie do implementacji chodu.
"""
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path('model/unitree_g1/unitree_g1_simplified.xml')
data = mujoco.MjData(model)

class LegLiftController:
    """
    Kontroler podnoszący naprzemiennie lewą i prawą nogę.
    """
    def __init__(self):
        self.phase = 0  # Faza ruchu (0-2π)
        self.frequency = 0.5  # Hz (0.5 cyklu na sekundę)
        
    def update(self, model, data):
        # Aktualizuj fazę
        self.phase = (2 * np.pi * self.frequency * data.time) % (2 * np.pi)
        
        # Pozycja docelowa zależy od fazy
        # Pierwsza połowa cyklu: podnoś prawą nogę
        # Druga połowa: podnoś lewą nogę
        
        Kp, Kd = 100.0, 10.0
        
        for i in range(model.nu):
            actuator_name = model.actuator(i).name
            joint_id = model.actuator_trnid[i, 0]
            
            qpos_actual = data.qpos[joint_id + 7]
            qvel_actual = data.qvel[joint_id + 6]
            
            # Domyślna pozycja: lekko zgięte kolana
            qpos_desired = 0
            
            # Logika podnoszenia nóg
            if 'knee' in actuator_name:
                qpos_desired = -0.3  # Domyślne zgięcie
                
                if 'right' in actuator_name and self.phase < np.pi:
                    # Podnoś prawą nogę (bardziej zegnij kolano)
                    qpos_desired = -1.0
                elif 'left' in actuator_name and self.phase >= np.pi:
                    # Podnoś lewą nogę
                    qpos_desired = -1.0
            
            # Regulator PD
            error_pos = qpos_desired - qpos_actual
            error_vel = 0 - qvel_actual
            control = Kp * error_pos + Kd * error_vel
            
            data.ctrl[i] = np.clip(control, -1, 1)

# Uruchom
controller = LegLiftController()

print("Test podnoszenia nóg...")
print("Robot będzie podnosił naprzemiennie prawą i lewą nogę.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        controller.update(model, data)
        mujoco.mj_step(model, data)
        viewer.sync()
```

## Zaawansowane aplikacje

### 1. Implementacja prostego chodu

Pełna implementacja chodu jest złożona i wykracza poza ten przewodnik, ale oto podstawowa struktura:

```python
"""
Szkielet dla implementacji chodu dwunożnego.
"""

class WalkingGait:
    """
    Prosty generator chodu oparty na sinusoidach (CPG - Central Pattern Generator).
    """
    def __init__(self, step_frequency=1.0, step_height=0.05):
        self.freq = step_frequency
        self.height = step_height
        
    def get_target_positions(self, time):
        """
        Oblicza docelowe pozycje stawów dla danego czasu.
        
        Zwraca:
            Dict z docelowymi pozycjami dla każdego stawu.
        """
        phase = 2 * np.pi * self.freq * time
        
        targets = {}
        
        # PRAWA NOGA
        # Biodro: ruch do przodu/tyłu
        targets['right_hip_pitch'] = 0.3 * np.sin(phase)
        # Kolano: zgięcie podczas podnoszenia
        targets['right_knee_pitch'] = -0.5 - 0.3 * max(0, np.sin(phase))
        # Kostka: kompensacja
        targets['right_ankle_pitch'] = 0.1 * np.sin(phase)
        
        # LEWA NOGA (przesunięta o pół fazy - przeciwfaza)
        targets['left_hip_pitch'] = 0.3 * np.sin(phase + np.pi)
        targets['left_knee_pitch'] = -0.5 - 0.3 * max(0, np.sin(phase + np.pi))
        targets['left_ankle_pitch'] = 0.1 * np.sin(phase + np.pi)
        
        # RĘCE: przeciwfazowy ruch (jak przy naturalnym chodzeniu)
        targets['right_shoulder_pitch'] = -0.5 * np.sin(phase + np.pi)
        targets['left_shoulder_pitch'] = -0.5 * np.sin(phase)
        
        return targets

# W praktyce: wymaga tuning parametrów, balansu, i detekcji kontaktu ze stopami!
```

### 2. Zbieranie danych sensorycznych

```python
"""
Zbieranie i analiza danych z sensorów robota.
"""
import mujoco
import numpy as np
import matplotlib.pyplot as plt

model = mujoco.MjModel.from_xml_path('model/unitree_g1/unitree_g1_simplified.xml')
data = mujoco.MjData(model)

# Listy do przechowywania historii
imu_data = {'accel': [], 'gyro': [], 'orientation': []}
foot_forces = {'left': [], 'right': []}
times = []

# Symulacja z zapisem danych
for i in range(1000):
    mujoco.mj_step(model, data)
    
    if i % 10 == 0:  # Zapisz co 10 kroków
        times.append(data.time)
        
        # Odczyt IMU
        # Indeksy sensorów zależą od kolejności w XML
        # Można je sprawdzić przez model.sensor(name).id
        imu_data['accel'].append(data.sensordata[0:3].copy())
        imu_data['gyro'].append(data.sensordata[3:6].copy())
        
        # Odczyt sił w stopach
        foot_forces['right'].append(data.sensordata[9:12].copy())  # Przykładowe indeksy
        foot_forces['left'].append(data.sensordata[12:15].copy())

# Wizualizacja
fig, axes = plt.subplots(2, 1, figsize=(12, 8))

# Przyspieszenia
accel_array = np.array(imu_data['accel'])
axes[0].plot(times, accel_array[:, 2], label='Przyspieszenie Z')
axes[0].set_ylabel('Przyspieszenie [m/s²]')
axes[0].set_title('Dane z IMU')
axes[0].grid(True)
axes[0].legend()

# Siły w stopach
right_force = np.array(foot_forces['right'])
left_force = np.array(foot_forces['left'])
axes[1].plot(times, np.linalg.norm(right_force, axis=1), label='Prawa stopa')
axes[1].plot(times, np.linalg.norm(left_force, axis=1), label='Lewa stopa')
axes[1].set_xlabel('Czas [s]')
axes[1].set_ylabel('Siła [N]')
axes[1].set_title('Siły kontaktowe w stopach')
axes[1].grid(True)
axes[1].legend()

plt.tight_layout()
plt.savefig('sensor_data.png')
print("Dane zapisane jako 'sensor_data.png'")
```

## Workflow: od symulacji do prawdziwego robota

### Krok 1: Rozwój w symulacji

```python
# 1. Zaprojektuj kontroler w symulacji
# 2. Przetestuj w różnych scenariuszach
# 3. Optymalizuj parametry
```

### Krok 2: Walidacja robustness

```python
# Test różnych warunków:
# - Różne pozycje startowe
# - Zakłócenia (pchnięcia)
# - Różne podłoża (friction)
# - Różne prędkości
```

### Krok 3: Sim-to-Real Transfer

**Kluczowe kroki:**

1. **Domain Randomization** - trenuj z losowymi parametrami
   ```python
   # Losuj parametry w każdej iteracji
   model.opt.timestep = np.random.uniform(0.001, 0.003)
   model.geom_friction[floor_id] = np.random.uniform(0.5, 1.5)
   ```

2. **System Identification** - zmierz prawdziwe parametry robota
   - Masy ogniw
   - Długości
   - Przełożenia silników
   - Opóźnienia

3. **Testowanie stopniowe**
   - Najpierw testuj pojedyncze stawy
   - Potem proste ruchy
   - Na końcu złożone zachowania

4. **Safety Checks** - zawsze miej:
   - Wyłącznik awaryjny
   - Ograniczenia prędkości
   - Ograniczenia momentów
   - Detekcję kolizji

## Projekty przykładowe

### Projekt 1: Stabilizacja pozycji stojącej

**Cel:** Robot utrzymuje równowagę pomimo zakłóceń

**Kroki:**
1. Implementuj regulator PD dla wszystkich stawów
2. Dodaj feedback z IMU (orientacja tułowia)
3. Test na równej powierzchni
4. Test z zakłóceniami (pchnięcia)

### Projekt 2: Proste przemieszczenie

**Cel:** Robot wykonuje jeden krok do przodu

**Kroki:**
1. Zaprojektuj trajektorię dla jednego kroku
2. Implementuj generator trajektorii
3. Dodaj detekcję kontaktu stopy z ziemią
4. Testuj stabilność

### Projekt 3: Chodzenie w linii prostej

**Cel:** Ciągłe, cykliczne kroki

**Kroki:**
1. Rozszerz Projekt 2 do cyklu kroków
2. Dodaj regulator równowagi (kompensacja przechyleń)
3. Optymalizuj parametry chodu (długość kroku, wysokość, tempo)
4. Testuj na różnych powierzchniach

### Projekt 4: Manipulacja obiektami

**Cel:** Robot podnosi i przenosi obiekt

**Kroki:**
1. Dodaj obiekt do sceny
2. Użyj kinematyki odwrotnej (IK) do planowania ruchu ręki
3. Implementuj chwytanie (kontrola dłoni/chwytarki)
4. Zaplanuj trajektorię przeniesienia

## Podsumowanie

### Co osiągnąłeś:

✅ Zrozumienie budowy modelu Unitree G1 w MuJoCo  
✅ Umiejętność podstawowej symulacji i sterowania  
✅ Znajomość workflow od symulacji do wdrożenia  
✅ Praktyczne przykłady aplikacji  

### Następne kroki:

1. **Eksperymentuj** - modyfikuj parametry, testuj różne podejścia
2. **Ucz się** - studiuj literaturę z zakresu robotyki dwunożnej
3. **Implementuj** - wybierz jeden z projektów i zrealizuj go w pełni
4. **Przejdź do prawdziwego robota** - jeśli masz dostęp do Unitree G1

### Zasoby dodatkowe:

- **Artykuły naukowe o chodzie dwunożnym**
  - "Passive Dynamic Walking" - McGeer
  - "Learning Agile Robotic Locomotion Skills" - Hwangbo et al.
  
- **Kursy online**
  - "Underactuated Robotics" - MIT (Russ Tedrake)
  - "Modern Robotics" - Northwestern University

- **Narzędzia**
  - Stable-Baselines3 (Reinforcement Learning)
  - PyBullet (alternatywny symulator)
  - ROS2 (system robotyczny)

**Powodzenia w pracy z Unitree G1!** 🤖
