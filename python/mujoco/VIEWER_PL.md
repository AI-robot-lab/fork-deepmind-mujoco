# Dokumentacja Viewer - MuJoCo dla Studentów

## Przegląd modułu `viewer.py`

Moduł `viewer.py` dostarcza **interaktywny wizualizator 3D** dla symulacji MuJoCo. Jest to kluczowe narzędzie do:
- Wizualizacji robotów w czasie rzeczywistym
- Debugowania zachowań symulacji
- Demonstracji wyników
- Interaktywnego eksperymentowania z kontrolerami

---

## Główne funkcje

### 1. `launch(model, data)` - Prosty viewer

**Cel:** Uruchomienie podstawowego interaktywnego viewera

**Kiedy używać:** Gdy chcesz szybko zobaczyć model bez dodatkowego kodu sterującego

**Przykład:**
```python
import mujoco
import mujoco.viewer

# Załaduj model
model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# Uruchom viewer - otworzy się okno interaktywne
mujoco.viewer.launch(model, data)
```

**Co się dzieje:**
1. Tworzone jest okno 3D z renderowaną sceną
2. Symulacja uruchamia się automatycznie
3. Możesz poruszać kamerą myszką
4. Możesz wstrzymać/wznowić symulację (klawisz Spacja)
5. Możesz zresetować symulację (Backspace)

**Sterowanie myszką:**
- **Lewy przycisk + ruch:** Obróć kamerę wokół punktu
- **Prawy przycisk + ruch:** Powiększ/pomniejsz (zoom)
- **Środkowy przycisk + ruch:** Przesuń kamerę (pan)
- **Ctrl + prawy przycisk:** Zastosuj siłę do obiektu (pchnij robota!)

---

### 2. `launch_passive(model, data)` - Viewer z własnym sterowaniem

**Cel:** Uruchomienie viewera z pełną kontrolą nad pętlą symulacji

**Kiedy używać:** Gdy chcesz implementować własny kontroler lub zbierać dane

**Przykład:**
```python
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
data = mujoco.MjData(model)

# Funkcja kontrolera (wywoływana w każdym kroku)
def my_controller(model, data):
    """
    Twój własny kontroler - wykona się w każdej iteracji.
    
    Tutaj możesz:
    - Odczytać stan robota (data.qpos, data.qvel)
    - Obliczyć sygnały sterujące
    - Ustawić data.ctrl
    """
    # Przykład: sinusoidalne sterowanie pierwszym siłownikiem
    data.ctrl[0] = np.sin(data.time)

# Uruchom viewer w trybie pasywnym
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Główna pętla symulacji
    while viewer.is_running():
        # 1. Wywołaj kontroler
        my_controller(model, data)
        
        # 2. Wykonaj krok symulacji
        mujoco.mj_step(model, data)
        
        # 3. Synchronizuj wizualizację z symulacją
        viewer.sync()
```

**Dlaczego to jest ważne:**
- **Pełna kontrola:** Ty decydujesz kiedy wykonać krok symulacji
- **Możliwość zbierania danych:** Możesz zapisywać stan w każdym kroku
- **Niestandardowe sterowanie:** Możesz implementować dowolne algorytmy
- **Debugowanie:** Możesz dodać print() lub breakpoints w pętli

---

## Klasa `Handle` - Uchwyt do viewera

**Cel:** Zapewnia dostęp do parametrów viewera podczas działania

**Najważniejsze właściwości:**

### `handle.cam` - Kamera
```python
# Dostęp do parametrów kamery
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Ustaw pozycję kamery
    viewer.cam.azimuth = 90     # Kąt azymutu (obrót wokół osi Z)
    viewer.cam.elevation = -20  # Kąt elewacji (wysokość)
    viewer.cam.distance = 3.0   # Odległość od obiektu
    viewer.cam.lookat[:] = [0, 0, 1]  # Punkt na który patrzy kamera
```

### `handle.opt` - Opcje wizualizacji
```python
# Włącz/wyłącz różne elementy wizualizacji
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True   # Pokaż punkty kontaktu
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True   # Pokaż siły kontaktowe
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False   # Wyłącz przezroczystość
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True          # Pokaż stawy
```

### `handle.perturb` - Zakłócenia
```python
# Programowo zastosuj siłę do robota
viewer.perturb.active = True
viewer.perturb.select = body_id  # ID ciała do zakłócenia
viewer.perturb.refpos[:] = [0, 0, 0]  # Punkt przyłożenia siły
```

---

## Dodatkowe funkcje Handle

### `handle.set_texts()` - Wyświetlanie tekstu na ekranie

**Cel:** Pokazanie informacji na ekranie viewera (np. wartości z sensorów, stanu kontrolera)

**Przykład:**
```python
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        my_controller(model, data)
        mujoco.mj_step(model, data)
        
        # Wyświetl informacje na ekranie
        info_text = f"Czas: {data.time:.2f}s\n"
        info_text += f"Energia: {data.energy[0]:.2f}J\n"
        info_text += f"Wysokość: {data.qpos[2]:.2f}m"
        
        viewer.set_texts(
            (mujoco.mjtFontScale.mjFONTSCALE_150,      # Rozmiar czcionki
             mujoco.mjtGridPos.mjGRID_TOPLEFT,         # Pozycja (lewy górny róg)
             info_text,                                 # Tekst lewy
             "")                                        # Tekst prawy (pusty)
        )
        
        viewer.sync()
```

**Pozycje tekstu (mjtGridPos):**
- `mjGRID_TOPLEFT` - Lewy górny róg
- `mjGRID_TOPRIGHT` - Prawy górny róg
- `mjGRID_BOTTOMLEFT` - Lewy dolny róg
- `mjGRID_BOTTOMRIGHT` - Prawy dolny róg

### `handle.set_figures()` - Wykresy na żywo

**Cel:** Wyświetlanie wykresów w czasie rzeczywistym (np. trajektorie, wartości sensorów)

**Przykład:**
```python
import mujoco

# Przygotuj strukturę MjvFigure
figure = mujoco.MjvFigure()
figure.title = "Wysokość robota"

# Utwórz viewport (prostokąt na ekranie)
viewport = mujoco.MjrRect(
    left=10,      # Lewy margines (px)
    bottom=10,    # Dolny margines (px) 
    width=300,    # Szerokość (px)
    height=200    # Wysokość (px)
)

history = []
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        
        # Zbieraj dane
        history.append(data.qpos[2])  # Wysokość Z
        
        # Aktualizuj wykres (co 10 kroków)
        if len(history) % 10 == 0:
            # Dane do wykresu
            figure.linedata[0][:len(history)] = history
            figure.linepnt[0] = len(history)
            
            # Wyświetl
            viewer.set_figures((viewport, figure))
        
        viewer.sync()
```

---

## Tipsy i triki dla studentów

### 1. Zapis wideo z symulacji

```python
import mediapy as media

frames = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Stwórz renderer do przechwytywania klatek
    renderer = mujoco.Renderer(model)
    
    for i in range(1000):
        mujoco.mj_step(model, data)
        
        # Renderuj klatkę
        renderer.update_scene(data, camera="tracking")
        pixels = renderer.render()
        frames.append(pixels)
        
        viewer.sync()

# Zapisz jako wideo
media.write_video('symulacja.mp4', frames, fps=30)
```

### 2. Pauza symulacji na określonym warunku

```python
paused = False

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        if not paused:
            my_controller(model, data)
            mujoco.mj_step(model, data)
            
            # Zatrzymaj jeśli robot upadnie
            if data.qpos[2] < 0.5:  # Wysokość < 0.5m
                paused = True
                print("Robot upadł! Symulacja zatrzymana.")
        
        viewer.sync()
```

### 3. Automatyczne resetowanie po czasie

```python
RESET_TIME = 10.0  # Reset co 10 sekund

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        my_controller(model, data)
        mujoco.mj_step(model, data)
        
        # Reset po czasie
        if data.time >= RESET_TIME:
            mujoco.mj_resetData(model, data)
            print(f"Reset symulacji po {RESET_TIME}s")
        
        viewer.sync()
```

### 4. Różne tryby kamery

```python
# Kamera śledząca (tracking)
viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
viewer.cam.trackbodyid = 1  # ID ciała do śledzenia

# Kamera wolna
viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE

# Kamera stała
viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
```

---

## Typowe problemy i rozwiązania

### Problem: Viewer zamyka się natychmiast

**Przyczyna:** Pętla `while viewer.is_running()` kończy się zbyt szybko

**Rozwiązanie:**
```python
# ŹLE - viewer zamknie się natychmiast
with mujoco.viewer.launch_passive(model, data) as viewer:
    pass  # Brak pętli!

# DOBRZE
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

### Problem: Symulacja jest zbyt wolna

**Przyczyna:** Za dużo obliczeń w pętli lub za wolny komputer

**Rozwiązanie:**
```python
# Przyspiesz symulację - wykonuj więcej kroków na klatkę
STEPS_PER_FRAME = 10

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        for _ in range(STEPS_PER_FRAME):
            my_controller(model, data)
            mujoco.mj_step(model, data)
        
        viewer.sync()  # Tylko jedna synchronizacja na wiele kroków
```

### Problem: Nie widzę robota/sceny

**Przyczyna:** Kamera jest źle ustawiona

**Rozwiązanie:**
```python
# Resetuj kamerę do domyślnej pozycji
viewer.cam.azimuth = 90
viewer.cam.elevation = -20
viewer.cam.distance = 5.0
viewer.cam.lookat[:] = [0, 0, 1]
```

---

## Podsumowanie

### Kiedy używać którego trybu?

| Scenariusz | Funkcja | Uzasadnienie |
|------------|---------|--------------|
| Szybki podgląd modelu | `launch()` | Najmniej kodu, automatyczna symulacja |
| Implementacja kontrolera | `launch_passive()` | Pełna kontrola nad pętlą |
| Zbieranie danych | `launch_passive()` | Dostęp do stanu w każdym kroku |
| Debugowanie | `launch_passive()` | Możliwość pauz i printów |
| Demonstracja | `launch()` lub `launch_passive()` | Zależnie od potrzeb |

### Kluczowe pojęcia do zapamiętania

✅ **Viewer** = Okno 3D do wizualizacji  
✅ **Handle** = Uchwyt do kontroli viewera  
✅ **sync()** = Synchronizacja wizualizacji z danymi symulacji  
✅ **launch()** vs **launch_passive()** = Automatyczny vs manualny tryb  

---

**Dokumentacja przygotowana dla studentów Politechniki Rzeszowskiej** 🎓
