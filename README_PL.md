# MuJoCo - Wersja dla Studentów Politechniki Rzeszowskiej

<h1>
  <a href="#"><img alt="MuJoCo" src="banner.png" width="100%"/></a>
</h1>

## Wprowadzenie

**MuJoCo** to skrót od **Mu**lti-**Jo**int dynamics with **Co**ntact (Dynamika Wielu Stawów z Kontaktem). Jest to zaawansowany silnik fizyki ogólnego przeznaczenia, który ułatwia badania i rozwój w dziedzinach takich jak:
- Robotyka
- Biomechanika
- Grafika komputerowa i animacja
- Uczenie maszynowe
- Inne obszary wymagające szybkiej i dokładnej symulacji struktur przegubowych wchodzących w interakcję z otoczeniem

To repozytorium jest utrzymywane przez [Google DeepMind](https://www.deepmind.com/).

## Dlaczego MuJoCo?

MuJoCo został zaprojektowany z myślą o badaczach i programistach. Oferuje:
- **API w języku C** - niskopoziomowy dostęp do silnika fizyki
- **Wydajność** - moduł symulacji w czasie rzeczywistym jest zoptymalizowany pod kątem maksymalnej wydajności
- **Wizualizacja** - interaktywna wizualizacja z natywnym GUI renderowanym w OpenGL
- **Funkcje użytkowe** - bogaty zestaw funkcji do obliczania wielkości związanych z fizyką
- **Wiązania Python** - wygodny dostęp dla programistów Python
- **Wtyczka Unity** - integracja z silnikiem gier Unity

## Zastosowanie w Projekcie z Robotem Unitree G1 EDU-U6

Ten pakiet oprogramowania jest szczególnie przydatny w pracy z robotem humanoidalnym **Unitree G1 EDU-U6**. MuJoCo umożliwia:

1. **Symulację ruchu robota** przed wdrożeniem na prawdziwym urządzeniu
2. **Testowanie algorytmów sterowania** w bezpiecznym środowisku wirtualnym
3. **Optymalizację trajektorii** ruchu dla złożonych zadań manipulacyjnych
4. **Naukę przez wzmacnianie** (Reinforcement Learning) dla zaawansowanych zachowań
5. **Walidację modeli kinematycznych i dynamicznych**

Zobacz szczegółowy przewodnik [UNITREE_G1_PRZEWODNIK.md](UNITREE_G1_PRZEWODNIK.md) aby dowiedzieć się więcej o zastosowaniu MuJoCo z robotem Unitree G1.

## Dokumentacja

Pełna dokumentacja MuJoCo dostępna jest pod adresem [mujoco.readthedocs.io](https://mujoco.readthedocs.io). Nadchodzące funkcje planowane w następnym wydaniu można znaleźć w [changelog](https://mujoco.readthedocs.io/en/latest/changelog.html) w gałęzi "latest".

**Dla studentów Politechniki Rzeszowskiej:**
- Zobacz [PRZEWODNIK_STUDENTA.md](PRZEWODNIK_STUDENTA.md) - szczegółowy przewodnik dla studentów
- Wszystkie kluczowe pliki kodu zawierają polskie komentarze wyjaśniające działanie

## Pierwsze Kroki

Istnieją dwa proste sposoby, aby rozpocząć pracę z MuJoCo:

### 1. Uruchomienie aplikacji `simulate` na własnym komputerze

Aplikacja `simulate` to natywny interaktywny wizualizator MuJoCo. Aby go uruchomić:

1. Pobierz skompilowane binaria z [strony releases](https://github.com/google-deepmind/mujoco/releases)
2. Rozpakuj archiwum
3. Uruchom aplikację `simulate` (lub `simulate.exe` na Windows)
4. Otwórz dowolny plik modelu XML (np. `model/humanoid/humanoid.xml`)

[Ten film](https://www.youtube.com/watch?v=P83tKA1iz2Y) pokazuje zapis ekranu działania aplikacji `simulate`.

### 2. Eksploracja interaktywnych notebooków IPython

Jeśli jesteś użytkownikiem Pythona, możesz rozpocząć od naszych notebooków tutorialowych działających na Google Colab:

- **Tutorial wprowadzający** uczy podstaw MuJoCo:
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb)

- **Tutorial edycji modeli** pokazuje jak tworzyć i edytować modele programowo:
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb)

- **Tutorial rollout** pokazuje jak używać wielowątkowego modułu `rollout`:
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/rollout.ipynb)

- **Tutorial LQR** syntetyzuje kontroler liniowo-kwadratowy, równoważąc humanoidę na jednej nodze:
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/LQR.ipynb)

- **Tutorial najmniejszych kwadratów** wyjaśnia jak używać solvera nieliniowych najmniejszych kwadratów w Pythonie:
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/least_squares.ipynb)

- **Tutorial MJX** dostarcza przykłady użycia [MuJoCo XLA](https://mujoco.readthedocs.io/en/stable/mjx.html), gałęzi MuJoCo napisanej w JAX:
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/tutorial.ipynb)

- **Tutorial fizyki różniczkowalnej** trenuje polityki lokomocji z analitycznymi gradientami automatycznie wyprowadzonymi z kroku fizycznego MuJoCo:
  [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/training_apg.ipynb)

## Instalacja

### Wstępnie skompilowane binaria

Wydania z numerami wersji są dostępne jako wstępnie skompilowane binaria na [stronie releases](https://github.com/google-deepmind/mujoco/releases) na GitHubie, zbudowane dla:
- Linux (x86-64 i AArch64)
- Windows (tylko x86-64)
- macOS (universal)

To jest **zalecany sposób** korzystania z oprogramowania.

### Instalacja Python (>= 3.9)

Natywne wiązania Pythona, które zawierają wstępnie spakowaną kopię MuJoCo, można zainstalować z [PyPI](https://pypi.org/project/mujoco/) za pomocą:

```bash
pip install mujoco
```

**Uwaga:** Wstępnie skompilowane koła (wheels) dla Linuxa są targetowane na `manylinux2014`. Zobacz [tutaj](https://github.com/pypa/manylinux) listę kompatybilnych dystrybucji.

### Kompilacja ze źródeł

Użytkownicy, którzy chcą zbudować MuJoCo ze źródeł, powinni zapoznać się z sekcją [build from source](https://mujoco.readthedocs.io/en/latest/programming#building-from-source) w dokumentacji. Należy jednak pamiętać, że commit na końcu gałęzi `main` może być niestabilny.

## Struktura Repozytorium

```
mujoco/
├── doc/           # Dokumentacja
├── include/       # Pliki nagłówkowe C/C++
├── model/         # Przykładowe modele (w tym humanoid)
├── python/        # Wiązania Python i przykłady
├── sample/        # Przykładowe programy w C++
├── src/           # Kod źródłowy silnika
├── simulate/      # Kod aplikacji wizualizatora
└── test/          # Testy jednostkowe
```

## Kluczowe Koncepcje

### Model (MjModel)
Struktura danych zawierająca opis modelu - definicję robota, jego geometrię, właściwości fizyczne, siłowniki itp. **Model jest niezmienny** podczas symulacji.

### Data (MjData)
Struktura danych zawierająca stan symulacji - pozycje, prędkości, siły, wyniki obliczeń fizycznych. **Data zmienia się** w każdym kroku symulacji.

### Krok symulacji (mj_step)
Główna funkcja wykonująca jeden krok symulacji fizycznej. Aktualizuje strukturę `data` zgodnie z modelami fizycznymi zdefiniowanymi w `model`.

### Wizualizacja
MuJoCo oferuje kilka metod wizualizacji:
- Natywny GUI (`simulate`)
- Renderowanie offscreen (Python: `Renderer`)
- Interaktywny viewer (Python: `viewer`)

## Przykłady Użycia

### Przykład w C++ (basic.cc)
Zobacz plik `sample/basic.cc` z polskimi komentarzami wyjaśniającymi każdy krok.

### Przykład w Python
Zobacz pliki w katalogu `python/mujoco/` - wszystkie kluczowe moduły zawierają szczegółowe polskie komentarze:
- `viewer.py` - interaktywny viewer
- `rollout.py` - wielowątkowe rollouts
- `renderer.py` - renderowanie offline
- `minimize.py` - optymalizacja

## Wsparcie i Pomoc

- **Pytania:** Użyj [GitHub Discussions](https://github.com/google-deepmind/mujoco/discussions/categories/asking-for-help)
- **Błędy i Feature Requests:** Użyj [GitHub Issues](https://github.com/google-deepmind/mujoco/issues)
- **Przewodnik dla Kontrybutorów:** Zobacz [CONTRIBUTING.md](CONTRIBUTING.md)

## Powiązane Oprogramowanie

### Wiązania do innych języków

#### Wiązania pierwszopartyjne:
- [Wiązania Python](https://mujoco.readthedocs.io/en/stable/python.html)
- [Wiązania JavaScript i wsparcie WebAssembly](/wasm/README.md)
- [Wiązania C# i wtyczka Unity](https://mujoco.readthedocs.io/en/stable/unity.html)

#### Wiązania innych dostawców:
- **MATLAB Simulink**: [Simulink Blockset for MuJoCo Simulator](https://github.com/mathworks-robotics/mujoco-simulink-blockset)
- **Swift**: [swift-mujoco](https://github.com/liuliu/swift-mujoco)
- **Java**: [mujoco-java](https://github.com/CommonWealthRobotics/mujoco-java)
- **Julia**: [MuJoCo.jl](https://github.com/JamieMair/MuJoCo.jl)
- **Rust**: [MuJoCo-rs](https://github.com/davidhozic/mujoco-rs)

### Konwertery

- **OpenSim**: [MyoConverter](https://github.com/MyoHub/myoconverter) konwertuje modele OpenSim do MJCF
- **SDFormat**: [gz-mujoco](https://github.com/gazebosim/gz-mujoco/) - dwukierunkowe narzędzie konwersji SDFormat <-> MJCF
- **OBJ**: [obj2mjcf](https://github.com/kevinzakka/obj2mjcf) - skrypt konwertujący złożone pliki OBJ do modeli MJCF
- **Onshape**: [Onshape to Robot](https://github.com/rhoban/onshape-to-robot) - konwertuje assemblies CAD z [onshape](https://www.onshape.com/en/) do MJCF

## Cytowanie

Jeśli używasz MuJoCo w publikowanych badaniach, prosimy o cytowanie:

```
@inproceedings{todorov2012mujoco,
  title={MuJoCo: A physics engine for model-based control},
  author={Todorov, Emanuel and Erez, Tom and Tassa, Yuval},
  booktitle={2012 IEEE/RSJ International Conference on Intelligent Robots and Systems},
  pages={5026--5033},
  year={2012},
  organization={IEEE},
  doi={10.1109/IROS.2012.6386109}
}
```

## Licencja

Copyright 2021 DeepMind Technologies Limited.

Kod źródłowy jest licencjonowany na podstawie Apache License, Version 2.0. Kopię licencji można uzyskać pod adresem https://www.apache.org/licenses/LICENSE-2.0.

Dokumenty ReStructuredText, obrazy i filmy w katalogu `doc` są udostępniane na warunkach licencji Creative Commons Attribution 4.0 (CC BY 4.0).

To nie jest oficjalnie wspierany produkt Google.

---

## Materiały dla Studentów

- [PRZEWODNIK_STUDENTA.md](PRZEWODNIK_STUDENTA.md) - Kompleksowy przewodnik dla studentów
- [UNITREE_G1_PRZEWODNIK.md](UNITREE_G1_PRZEWODNIK.md) - Przewodnik specyficzny dla robota Unitree G1
- Wszystkie kluczowe pliki Python zawierają szczegółowe polskie komentarze
