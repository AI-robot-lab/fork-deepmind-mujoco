# Humanoid - Model Robota Humanoidalnego

## Wprowadzenie

Ten uproszczony model humanoidalny, wprowadzony w [1], został zaprojektowany do badań nad lokomocją dwunożną. Chociaż istnieje kilka jego wariantów, ta wersja jest oparta na modelu z DeepMind Control Suite [2], który ma dość realistyczne wzmocnienia siłowników.

* **Stopnie swobody (DOF):** 27
* **Siłowniki:** 21

<p float="left">
  <img src="humanoid.png" width="400">
</p>

## Opis modelu

### Struktura kinematyczna

Model humanoid składa się z następujących głównych części:

1. **Tułów (Torso)** - główne ciało robota
   - Zawiera większość masy
   - Punkt odniesienia dla innych części
   - Posiada sensor IMU (akcelerometr + żyroskop)

2. **Głowa (Head)** 
   - Połączona z tułowiem stawem obrotowym
   - Może obracać się w lewo/prawo

3. **Ręce (Arms)** - 2 × ramię
   - Bark (shoulder) - 3 DOF
   - Łokieć (elbow) - 1 DOF
   - Każda ręka ma 4 siłowniki

4. **Nogi (Legs)** - 2 × noga  
   - Biodro (hip) - 3 DOF
   - Kolano (knee) - 1 DOF
   - Stopa (foot) - 2 DOF
   - Każda noga ma 6 siłowników

### Właściwości fizyczne

- **Wysokość:** ~1.4m (w pozycji stojącej)
- **Masa całkowita:** ~76 kg
- **Współczynnik tarcia:** 0.7 (stopa-podłoże)
- **Tłumienie stawów:** Realistyczne wartości dla ruchu ludzkiego

### Siłowniki

Model używa 21 silników z ograniczeniami:
- Zakres sterowania: [-1, 1] dla większości stawów
- Realistyczne przełożenia (gear) dopasowane do anatomii
- Limity stawów zapobiegające nadmiernym wygięciom

## Zastosowania

### 1. Nauka chodzenia (Locomotion)

Model jest idealny do:
- Syntezy chodów (gait generation)
- Uczenia przez wzmacnianie (Reinforcement Learning)
- Optymalizacji trajektorii
- Badań nad równowagą

**Przykładowe zadania:**
- Stanie w miejscu
- Chodzenie w linii prostej
- Chodzenie po schodach
- Pokonywanie przeszkód
- Bieganie

### 2. Sterowanie całociałowe (Whole-body control)

- Kontrola wszystkich 27 DOF jednocześnie
- Koordynacja rąk i nóg
- Zadania manipulacyjne podczas chodzenia

### 3. Badania nad równowagą

- Odporność na pchnięcia
- Reakcja na nierówne podłoże
- Balansowanie na jednej nodze

## Jak używać tego modelu

### Podstawowa symulacja

```python
import mujoco
import mujoco.viewer

# Załaduj model
model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# Uruchom viewer
mujoco.viewer.launch(model, data)
```

### Z własnym kontrolerem

```python
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

def my_controller(model, data):
    # Twój kod sterujący tutaj
    # Przykład: prosta stabilizacja
    data.ctrl[:] = -0.1 * data.qvel[6:]  # Tłumienie prędkości

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        my_controller(model, data)
        mujoco.mj_step(model, data)
        viewer.sync()
```

## Warianty modelu

W tym katalogu znajdują się różne warianty modelu:

### `humanoid.xml` - Podstawowy model
Standardowy model z 1 robotem. Użyj tego do nauki i testowania kontrolerów.

### `humanoid100.xml` - Model zoptymalizowany
Wariant z ulepszonymi parametrami dla lepszej wydajności symulacji.

### `100_humanoids.xml` - 100 robotów
Scena ze 100 robotami humanoidalnymi. Użyteczna do:
- Równoległego testowania różnych kontrolerów
- Wizualizacji wyników uczenia maszynowego
- Demonstracji skalowania wydajności MuJoCo

### `22_humanoids.xml` - 22 roboty
Średnia scena - kompromis między wydajnością a liczbą instancji.

## Tipsy dla studentów

### 1. Zrozumienie struktury

```python
# Wyświetl informacje o modelu
print(f"Liczba ciał: {model.nbody}")
print(f"Liczba stawów: {model.njnt}")
print(f"Liczba siłowników: {model.nu}")

# Nazwy siłowników
for i in range(model.nu):
    print(f"{i}: {model.actuator(i).name}")
```

### 2. Kontrola pozycji stawów

```python
# Proste sterowanie PD (Proportional-Derivative)
Kp = 100.0  # Wzmocnienie proporcjonalne
Kd = 10.0   # Wzmocnienie różniczkowe

target_pos = np.zeros(model.nv)  # Docelowa pozycja

for i in range(model.nu):
    joint_id = model.actuator_trnid[i, 0]
    q_actual = data.qpos[7 + joint_id]  # +7 bo freejoint
    qvel_actual = data.qvel[6 + joint_id]  # +6 bo freejoint
    
    error_pos = target_pos[joint_id] - q_actual
    error_vel = 0 - qvel_actual
    
    data.ctrl[i] = Kp * error_pos + Kd * error_vel
```

### 3. Monitoring wysokości (wykrywanie upadku)

```python
# Sprawdź czy robot upadł
height = data.qpos[2]  # Wysokość centrum masy (Z)

if height < 0.8:  # Próg upadku
    print("Robot upadł!")
    mujoco.mj_resetData(model, data)  # Reset
```

### 4. Czytanie sensorów

```python
# Model ma wbudowane sensory
# Sprawdź jakie są dostępne:
for i in range(model.nsensor):
    sensor_name = model.sensor(i).name
    sensor_data = data.sensordata[i]
    print(f"{sensor_name}: {sensor_data}")
```

## Znane problemy i rozwiązania

### Problem: Robot natychmiast upada

**Rozwiązanie:**
- Sprawdź czy wysyłasz sygnały sterujące (data.ctrl)
- Użyj regulatora PD z odpowiednimi wzmocnieniami
- Upewnij się, że docelowa pozycja jest osiągalna

### Problem: Niestabilna symulacja

**Rozwiązanie:**
```xml
<!-- W pliku XML, zmniejsz timestep -->
<option timestep="0.002"/>  <!-- Zamiast 0.005 -->
```

### Problem: Zbyt wolna symulacja

**Rozwiązanie:**
- Użyj uproszczonego modelu (mniej geometrii)
- Wyłącz wizualizację kontaktów
- Zmniejsz liczbę kroków solwera

## Historia zmian (Changelog)

* **08-10-2024:** Przeniesiono światło śledzące z tułowia do świata.
* **20-02-2024:** Posortowano siłowniki w tej samej kolejności co stawy.
* **02-01-2024:** Dodano więcej klatek kluczowych (keyframes).
* **27-11-2023:** Przeniesiono geometrie humanoid do grupy 1.
* **05-04-2023:** Poprawiono literówkę w rozmiarze tekstury.
* **20-09-2022:** Użyto klasy domyślnej dla geometrii left_upper_arm.
* **17-09-2022:** Zwiększono rozdzielczość bufora renderowania offscreen do 2560x1440.
* **12-09-2022:**
  * Zwiększono maksymalną fleksję biodra.
  * Zsymetryzowano stawy ramienia i kostki.
  * Dodano ścięgna podkolanowe łączące biodro i kolano przy dużych wartościach fleksji.
  * Przeniesiono zduplikowane wartości do domyślnych (defaults).
  * Dodano dwie klatki kluczowe.
  * Poprawiono oświetlenie.
  * Zmieniono konwencję nazewnictwa.

## Referencje

[1] [Synthesis and Stabilization of Complex Behaviors through Online Trajectory Optimization](https://doi.org/10.1109/IROS.2012.6386025).

[2] [DeepMind Control Suite](https://arxiv.org/abs/1801.00690).

## Dalsze materiały edukacyjne

- **PRZEWODNIK_STUDENTA.md** - Kompleksowy przewodnik dla początkujących
- **UNITREE_G1_PRZEWODNIK.md** - Zastosowanie z robotem Unitree G1
- **VIEWER_PL.md** - Dokumentacja wizualizatora
- **ROLLOUT_PL.md** - Dokumentacja modułu rollout

---

**Materiał przygotowany dla studentów Politechniki Rzeszowskiej** 🎓
