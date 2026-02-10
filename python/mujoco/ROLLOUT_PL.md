# Dokumentacja Rollout - MuJoCo dla Studentów

## Przegląd modułu `rollout.py`

Moduł `rollout.py` dostarcza **wielowątkowy mechanizm do wykonywania wielu symulacji równolegle**. Jest to zaawansowane narzędzie wykorzystywane głównie w:
- Uczeniu maszynowym (Machine Learning)
- Optymalizacji trajektorii
- Testowaniu robustness (odporności) kontrolerów
- Symulacjach Monte Carlo

---

## Po co rollout?

### Scenariusz: Chcesz przetestować kontroler

**Bez rollout (sekwencyjnie):**
```
Test 1: [████████] 10s
Test 2: [████████] 10s  
Test 3: [████████] 10s
Test 4: [████████] 10s
────────────────────────
Razem:  40 sekund
```

**Z rollout (równolegle, 4 wątki):**
```
Test 1: [████████] 
Test 2: [████████]  } Równocześnie
Test 3: [████████]  
Test 4: [████████] 
────────────────────────
Razem:  ~10 sekund (4x szybciej!)
```

---

## Podstawy użycia

### 1. Prosty rollout - jedna trajektoria

**Cel:** Wykonanie symulacji z otwartą pętlą (open-loop) - z góry określone sterowanie

**Przykład:**
```python
import mujoco
from mujoco import rollout
import numpy as np

# Załaduj model
model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# Przygotuj rollout object (z 4 wątkami)
with rollout.Rollout(nthread=4) as r:
    # Stan początkowy (pozycje i prędkości)
    # Dla humanoid: 28 pozycji (qpos) + 27 prędkości (qvel)
    initial_state = np.zeros((1, model.nq + model.nv))  # 1 trajektoria
    # Możesz ustawić własny stan początkowy:
    # initial_state[0, :model.nq] = custom_qpos
    # initial_state[0, model.nq:] = custom_qvel
    
    # Sygnały sterujące (otwarta pętla)
    # Wymiar: [liczba_trajektorii, liczba_kroków, liczba_siłowników]
    nsteps = 100  # 100 kroków symulacji
    control = np.zeros((1, nsteps, model.nu))  # 1 trajektoria, 100 kroków, nu siłowników
    
    # Przykład: sinusoidalne sterowanie pierwszym siłownikiem
    for step in range(nsteps):
        control[0, step, 0] = np.sin(2 * np.pi * step / 50)
    
    # WYKONAJ ROLLOUT
    # Funkcja zwraca: (state, sensordata)
    # state: [nbatch, nstep, nstate] - wszystkie stany w trajektorii
    # sensordata: [nbatch, nstep, nsensordata] - dane z sensorów
    state, sensordata = r.rollout(
        model=model,
        data=data,
        initial_state=initial_state,
        control=control
    )
    
    print(f"Kształt state: {state.shape}")  # (1, 100, nstate)
    print(f"Kształt sensordata: {sensordata.shape}")  # (1, 100, nsensordata)
```

**Co się stało:**
1. Rollout wykonał 100 kroków symulacji
2. W każdym kroku zastosował odpowiedni sygnał sterujący z `control`
3. Zapisał stan (pozycje, prędkości) w każdym kroku
4. Zapisał dane z sensorów w każdym kroku
5. Zwrócił kompletną trajektorię

---

### 2. Wiele trajektorii równolegle (batch rollout)

**Cel:** Wykonanie wielu różnych symulacji jednocześnie

**Przykład: Testowanie 100 różnych warunków początkowych**

```python
import mujoco
from mujoco import rollout
import numpy as np

model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# PRZYPADEK UŻYCIA: Sprawdź jak robot radzi sobie z różnymi pozycjami startowymi

nbatch = 100  # 100 różnych trajektorii
nsteps = 200  # Każda po 200 kroków

with rollout.Rollout(nthread=8) as r:  # Użyj 8 wątków
    # Stwórz 100 różnych stanów początkowych
    initial_states = np.zeros((nbatch, model.nq + model.nv))
    
    for i in range(nbatch):
        # Losowe małe perturbacje pozycji początkowej
        initial_states[i, :model.nq] = np.random.randn(model.nq) * 0.1
        initial_states[i, model.nq:] = np.random.randn(model.nv) * 0.1
    
    # Taki sam kontroler dla wszystkich (można też różne!)
    # Wymiar: [nbatch, nsteps, nu]
    # Jeśli podasz [1, nsteps, nu], zostanie automatycznie powielone (broadcast)
    control = np.zeros((1, nsteps, model.nu))
    
    # Wykonaj wszystkie 100 trajektorii równolegle
    states, sensordatas = r.rollout(
        model=model,
        data=data,
        initial_state=initial_states,  # [100, nstate]
        control=control                # [1, nsteps, nu] -> auto-broadcast do [100, nsteps, nu]
    )
    
    print(f"Wykonano {nbatch} trajektorii")
    print(f"Kształt wyników: {states.shape}")  # (100, 200, nstate)
    
    # ANALIZA: Ile trajektorii zakończyło się "sukcesem"?
    # Definicja sukcesu: robot nie upadł (wysokość > 0.8m w ostatnim kroku)
    final_heights = states[:, -1, 2]  # Wysokość Z w ostatnim kroku każdej trajektorii
    success_count = np.sum(final_heights > 0.8)
    success_rate = success_count / nbatch * 100
    
    print(f"Współczynnik sukcesu: {success_rate:.1f}%")
    print(f"Średnia końcowa wysokość: {final_heights.mean():.2f}m")
```

**Zalety:**
- ✅ 100 symulacji w czasie niewiele dłuższym niż 1 symulacja
- ✅ Statystyczna walidacja kontrolera
- ✅ Znajdowanie edge cases (sytuacje graniczne)

---

## Zaawansowane użycie

### 3. Różne kontrolery dla każdej trajektorii

**Przykład: Porównanie różnych parametrów kontrolera PD**

```python
import mujoco
from mujoco import rollout
import numpy as np

model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# Cel: Znajdź optymalne wzmocnienia PD dla stabilnego stania

# Test 50 różnych kombinacji Kp i Kd
nbatch = 50
nsteps = 500

# Różne wartości Kp i Kd do przetestowania
Kp_values = np.linspace(10, 200, nbatch)
Kd_values = np.linspace(1, 20, nbatch)

with rollout.Rollout(nthread=8) as r:
    # Stan początkowy (ten sam dla wszystkich)
    initial_state = np.zeros((1, model.nq + model.nv))
    initial_state[0, :7] = [0, 0, 1, 1, 0, 0, 0]  # Pozycja i orientacja tułowia
    
    # Przygotuj sterowanie dla każdej trajektorii
    controls = np.zeros((nbatch, nsteps, model.nu))
    
    # Prosta symulacja regulatora PD (uproszczona - brak pełnego feedback)
    # W praktyce: trzeba by użyć callbacks, tu tylko demonstracja konceptu
    target_pos = np.zeros(model.nv)  # Docelowa pozycja (stanie prosto)
    
    for batch_idx in range(nbatch):
        Kp = Kp_values[batch_idx]
        Kd = Kd_values[batch_idx]
        
        # Dla uproszczenia: stałe sterowanie (w rzeczywistości trzeba feedback)
        # To tylko ilustracja - prawdziwy PD wymaga odczytu stanu w każdym kroku
        controls[batch_idx, :, :] = 0.1 * Kp  # Uproszczenie!
    
    # Wykonaj rollout
    states, _ = r.rollout(
        model=model,
        data=data,
        initial_state=initial_state,
        control=controls  # [nbatch, nsteps, nu]
    )
    
    # Analiza: który zestaw parametrów dał najlepsze wyniki?
    # Metryka: średnia wysokość przez cały czas
    avg_heights = states[:, :, 2].mean(axis=1)  # [nbatch]
    
    best_idx = np.argmax(avg_heights)
    print(f"Najlepsze parametry:")
    print(f"  Kp = {Kp_values[best_idx]:.1f}")
    print(f"  Kd = {Kd_values[best_idx]:.1f}")
    print(f"  Średnia wysokość: {avg_heights[best_idx]:.2f}m")
```

---

### 4. Użycie wielu modeli (różne roboty równocześnie)

**Przykład: Porównanie dwóch różnych projektów robota**

```python
import mujoco
from mujoco import rollout
import numpy as np

# Załaduj dwa różne modele
model1 = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
model2 = mujoco.MjModel.from_xml_path('model/humanoid/humanoid100.xml')  # Wariant

# UWAGA: Modele muszą mieć tę samą "sygnaturę rozmiaru" 
# (te same nq, nv, nu, etc.)

# Jeśli modele są różne, musisz użyć sequence:
models = [model1, model2, model1, model2]  # 4 trajektorie: 2x model1, 2x model2
nbatch = len(models)

# Dane dla każdego modelu
datas = [mujoco.MjData(m) for m in models]

nsteps = 100

with rollout.Rollout(nthread=4) as r:
    initial_states = np.zeros((nbatch, model1.nq + model1.nv))
    controls = np.zeros((1, nsteps, model1.nu))
    
    states, _ = r.rollout(
        model=models,   # Lista modeli
        data=datas,     # Lista danych
        initial_state=initial_states,
        control=controls
    )
    
    print("Porównanie modeli:")
    print(f"Model 1, traj 0: końcowa wysokość = {states[0, -1, 2]:.2f}m")
    print(f"Model 2, traj 1: końcowa wysokość = {states[1, -1, 2]:.2f}m")
```

---

## Najważniejsze parametry

### `rollout()` - parametry funkcji

```python
state, sensordata = r.rollout(
    model=model,                    # MjModel lub lista MjModel
    data=data,                      # MjData lub lista MjData
    initial_state=initial_state,    # [nbatch, nstate] lub [1, nstate]
    control=control,                # [nbatch, nstep, nu] lub [1, nstep, nu]
    
    # Opcjonalne:
    control_spec=mujoco.mjtState.mjSTATE_CTRL.value,  # Co oznacza 'control'
    skip_checks=False,              # Pomiń sprawdzanie kształtów (szybsze, ale ryzykowne)
    nstep=None,                     # Liczba kroków (auto z control jeśli None)
    initial_warmstart=None,         # Początkowe qfrc_warmstart [nbatch, nv]
    state=None,                     # Bufor wyjściowy (alokuj sam jeśli None)
    sensordata=None,                # Bufor wyjściowy dla sensorów
    chunk_size=None,                # Wielkość chunków dla threadpool
)
```

**Wyjaśnienie parametrów:**

#### `initial_state` - Stan początkowy
```python
# Składa się z: [qpos, qvel]
# qpos: pozycje stawów [nq]
# qvel: prędkości stawów [nv]

initial_state = np.zeros((nbatch, model.nq + model.nv))
initial_state[:, :model.nq] = starting_qpos   # Ustaw pozycje
initial_state[:, model.nq:] = starting_qvel   # Ustaw prędkości
```

#### `control` - Sygnały sterujące
```python
# Wymiar: [nbatch lub 1, nstep, nu]
# Dla każdej trajektorii, dla każdego kroku, dla każdego siłownika

# Przykład 1: Stałe sterowanie
control = np.ones((1, 100, model.nu)) * 0.5  # Wszystkie siłowniki na 0.5

# Przykład 2: Różne dla każdej trajektorii
control = np.random.randn(nbatch, nsteps, model.nu)

# Przykład 3: Zmienne w czasie
control = np.zeros((1, nsteps, model.nu))
for t in range(nsteps):
    control[0, t, :] = np.sin(2*np.pi*t/nsteps)  # Sinusoida
```

#### `control_spec` - Interpretacja sterowania
```python
# Domyślnie: mjSTATE_CTRL (sterowanie siłownikami)
# Możliwe opcje:
# - mjSTATE_CTRL: control to wartości dla actuators (domyślne)
# - mjSTATE_QFRC_APPLIED: control to bezpośrednie siły w stawach
```

---

## Praktyczne zastosowania dla studentów

### 1. Uczenie przez wzmacnianie (Reinforcement Learning)

```python
"""
Rollout używany do zbierania doświadczeń (experiences) dla RL.
"""

# Generuj trajektorie z losowym sterowaniem (exploration)
nbatch = 1000  # 1000 trajektorii
nsteps = 50

initial_states = sample_initial_states(nbatch)  # Twoja funkcja
random_actions = np.random.uniform(-1, 1, (nbatch, nsteps, model.nu))

with rollout.Rollout(nthread=16) as r:
    states, sensors = r.rollout(model, data, initial_states, random_actions)
    
    # Oblicz nagrody (rewards)
    rewards = compute_rewards(states)  # Twoja funkcja
    
    # Zapisz do replay buffer
    add_to_buffer(states, random_actions, rewards)
```

### 2. Optymalizacja trajektorii

```python
"""
Znajdź najlepszą trajektorię metodą próbkowania (sampling-based).
"""

def optimize_trajectory(model, data, target_position, n_iterations=10):
    """
    Prosta optymalizacja: próbkuj losowe sterowania i wybierz najlepsze.
    """
    best_control = None
    best_cost = float('inf')
    
    with rollout.Rollout(nthread=8) as r:
        for iteration in range(n_iterations):
            # Próbkuj 100 kandydatów
            candidate_controls = np.random.randn(100, 50, model.nu)
            
            # Ewaluuj wszystkie
            states, _ = r.rollout(
                model, data,
                initial_state=np.zeros((1, model.nq + model.nv)),
                control=candidate_controls
            )
            
            # Oblicz koszty
            final_positions = states[:, -1, :3]  # Końcowe pozycje XYZ
            costs = np.linalg.norm(final_positions - target_position, axis=1)
            
            # Znajdź najlepszego
            min_idx = np.argmin(costs)
            if costs[min_idx] < best_cost:
                best_cost = costs[min_idx]
                best_control = candidate_controls[min_idx]
                print(f"Iteracja {iteration}: Nowy najlepszy koszt = {best_cost:.3f}")
    
    return best_control

# Użycie
target = np.array([1.0, 0.0, 1.0])  # Docelowa pozycja XYZ
optimal_control = optimize_trajectory(model, data, target)
```

### 3. Testowanie odporności (robustness testing)

```python
"""
Sprawdź jak kontroler radzi sobie z perturbacjami.
"""

# Test: jak wpływają małe zmiany w masie robota?
nbatch = 20
mass_variations = np.linspace(0.8, 1.2, nbatch)  # ±20% masy

# Stwórz warianty modelu
models = []
for mass_factor in mass_variations:
    m = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
    m.body_mass[:] *= mass_factor  # Przeskaluj masy
    models.append(m)

datas = [mujoco.MjData(m) for m in models]

# Taki sam kontroler dla wszystkich
controller_output = np.zeros((1, 200, model.nu))  # Twój kontroler

with rollout.Rollout(nthread=8) as r:
    states, _ = r.rollout(models, datas, 
                           initial_state=np.zeros((1, model.nq + model.nv)),
                           control=controller_output)
    
    # Analiza: jak masa wpływa na stabilność?
    final_heights = states[:, -1, 2]
    
    import matplotlib.pyplot as plt
    plt.plot(mass_variations, final_heights, 'o-')
    plt.xlabel('Współczynnik masy')
    plt.ylabel('Końcowa wysokość [m]')
    plt.title('Wpływ masy na stabilność')
    plt.grid(True)
    plt.savefig('robustness_test.png')
```

---

## Różnice: rollout vs normalna pętla symulacji

### Normalna pętla (zamknięta):
```python
# PĘTLA ZAMKNIĘTA (closed-loop)
# Kontroler ma dostęp do stanu w każdym kroku

for step in range(nsteps):
    # 1. Odczytaj aktualny stan
    current_state = data.qpos, data.qvel
    
    # 2. Oblicz sterowanie NA PODSTAWIE aktualnego stanu
    data.ctrl[:] = controller(current_state)  # Feedback!
    
    # 3. Krok symulacji
    mujoco.mj_step(model, data)
```

### Rollout (otwarta):
```python
# PĘTLA OTWARTA (open-loop)  
# Sterowanie jest z góry określone, bez feedback

# 1. Przygotuj WSZYSTKIE sygnały sterujące z góry
all_controls = np.zeros((1, nsteps, model.nu))
for step in range(nsteps):
    all_controls[0, step, :] = precomputed_control(step)  # Brak feedback!

# 2. Wykonaj całą trajektorię na raz
states, _ = rollout.rollout(model, data, initial_state, all_controls)
```

**Kiedy co używać?**
- **Normalna pętla**: Gdy potrzebujesz feedback (większość kontrolerów)
- **Rollout**: Gdy:
  - Testujesz wielu kandydatów równocześnie
  - Masz z góry określoną trajektorię
  - Optymalizujesz open-loop sterowanie
  - Zbierasz dane do ML

---

## Optymalizacja wydajności

### Tipsy dla maksymalnej szybkości:

```python
# 1. Użyj odpowiedniej liczby wątków
import os
n_cores = os.cpu_count()
with rollout.Rollout(nthread=n_cores) as r:  # Wykorzystaj wszystkie rdzenie
    ...

# 2. Pomiń sprawdzanie kształtów (jeśli jesteś pewien)
states, _ = r.rollout(..., skip_checks=True)

# 3. Pre-alokuj bufory wyjściowe
state_buffer = np.zeros((nbatch, nsteps, model.nq + model.nv))
sensor_buffer = np.zeros((nbatch, nsteps, model.nsensordata))

states, sensors = r.rollout(
    ...,
    state=state_buffer,      # Użyj istniejącego bufora
    sensordata=sensor_buffer
)

# 4. Użyj większych chunk_size dla dużych batch
# Domyślnie: chunk_size = max(1, nbatch / (nthread * 10))
# Dla bardzo dużych nbatch, zwiększ:
states, _ = r.rollout(..., chunk_size=100)
```

---

## Typowe błędy

### 1. Złe wymiary tablicy control

```python
# BŁĄD: Zapomniałeś o wymiarze batch
control = np.zeros((nsteps, model.nu))  # ❌ Brak wymiaru batch!

# POPRAWNIE:
control = np.zeros((1, nsteps, model.nu))  # ✅ [batch, steps, nu]
# lub
control = np.zeros((nbatch, nsteps, model.nu))
```

### 2. Niezgodne rozmiary między model a initial_state

```python
# BŁĄD: Źle policzone nstate
initial_state = np.zeros((nbatch, model.nq))  # ❌ Brak qvel!

# POPRAWNIE:
nstate = model.nq + model.nv  # qpos + qvel
initial_state = np.zeros((nbatch, nstate))  # ✅
```

### 3. Próba użycia feedback w rollout

```python
# To NIE ZADZIAŁA - rollout to open-loop!
# Nie możesz użyć aktualnego stanu do obliczenia kontroli

for step in range(nsteps):
    control[0, step, :] = pd_controller(current_state)  # ❌ Nie masz current_state!
```

---

## Podsumowanie

### Kluczowe punkty:

✅ **Rollout = wielowątkowa symulacja open-loop**  
✅ **Idealny do: ML, optymalizacji, testowania robustness**  
✅ **NIE do: kontrolerów wymagających feedback w każdym kroku**  
✅ **Znaczne przyśpieszenie przy wielu trajektoriach**  

### Zalecane ścieżki nauki:

1. **Początkujący:** Zacznij od 1 trajektorii, zrozum wymiary tablic
2. **Średni:** Użyj wielu trajektorii do testowania różnych warunków
3. **Zaawansowany:** Zintegruj z algorytmami ML (RL, optymalizacja)

---

**Dokumentacja przygotowana dla studentów Politechniki Rzeszowskiej** 🎓
