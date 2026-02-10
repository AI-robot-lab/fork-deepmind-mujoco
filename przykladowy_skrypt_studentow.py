#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Przykładowy skrypt demonstracyjny MuJoCo dla studentów
=====================================================

Ten skrypt pokazuje podstawowe operacje z biblioteką MuJoCo:
1. Ładowanie modelu robota (humanoid)
2. Inicjalizacja symulacji
3. Implementacja prostego kontrolera PD
4. Wizualizacja w czasie rzeczywistym
5. Zbieranie i analiza danych

Autor: Przygotowane dla studentów Politechniki Rzeszowskiej
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

# =============================================================================
# SEKCJA 1: KONFIGURACJA I ŁADOWANIE MODELU
# =============================================================================

print("=" * 70)
print("PRZYKŁADOWA SYMULACJA MUJOCO - Robot Humanoid")
print("=" * 70)

# Ścieżka do modelu XML
# Model humanoid to standardowy model robota dwunożnego dostarczany z MuJoCo
MODEL_PATH = 'model/humanoid/humanoid.xml'

try:
    # Załaduj model z pliku XML
    # MjModel zawiera OPIS robota (geometrię, masy, siłowniki, etc.)
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    print(f"✓ Model załadowany pomyślnie: {MODEL_PATH}")
except Exception as e:
    print(f"✗ Błąd ładowania modelu: {e}")
    print("  Upewnij się, że uruchamiasz skrypt z głównego katalogu repozytorium")
    exit(1)

# Utwórz strukturę danych dla symulacji
# MjData zawiera STAN symulacji (pozycje, prędkości, siły, etc.)
data = mujoco.MjData(model)

# =============================================================================
# SEKCJA 2: INFORMACJE O MODELU
# =============================================================================

print("\n" + "-" * 70)
print("INFORMACJE O MODELU:")
print("-" * 70)
print(f"Liczba ciał (bodies):           {model.nbody}")
print(f"Liczba stawów (joints):         {model.njnt}")
print(f"Liczba siłowników (actuators):  {model.nu}")
print(f"Liczba sensorów (sensors):      {model.nsensor}")
print(f"Stopnie swobody pozycji (nq):   {model.nq}")
print(f"Stopnie swobody prędkości (nv): {model.nv}")
print(f"Krok czasowy (timestep):        {model.opt.timestep}s")

# Wyświetl nazwy siłowników
print("\nSiłowniki w modelu:")
for i in range(min(model.nu, 10)):  # Pokaż pierwsze 10
    actuator_name = model.actuator(i).name
    ctrl_range = model.actuator_ctrlrange[i]
    print(f"  [{i:2d}] {actuator_name:30s} zakres: [{ctrl_range[0]:6.2f}, {ctrl_range[1]:6.2f}]")
if model.nu > 10:
    print(f"  ... i {model.nu - 10} więcej")

# =============================================================================
# SEKCJA 3: REGULATOR PD (PROPORTIONAL-DERIVATIVE)
# =============================================================================

class PDController:
    """
    Regulator PD (Proportional-Derivative) dla kontroli pozycji stawów.
    
    Regulator PD to najprostszy typ kontrolera, który:
    1. Mierzy BŁĄD POZYCJI (jak daleko jesteśmy od celu)
    2. Mierzy BŁĄD PRĘDKOŚCI (jak szybko się poruszamy)
    3. Generuje SIŁĘ korygującą proporcjonalną do tych błędów
    
    Wzór: u = Kp * (q_desired - q_actual) + Kd * (0 - qvel_actual)
          |__________________________|   |_____________________|
           człon proporcjonalny           człon różniczkowy
           (poprawia pozycję)             (tłumi oscylacje)
    """
    
    def __init__(self, model, Kp=100.0, Kd=10.0):
        """
        Inicjalizacja regulatora PD.
        
        Args:
            model: Model MuJoCo
            Kp: Wzmocnienie proporcjonalne (im większe, tym mocniejsza korekcja pozycji)
            Kd: Wzmocnienie różniczkowe (im większe, tym silniejsze tłumienie)
        """
        self.model = model
        self.Kp = Kp  # Wzmocnienie proporcjonalne
        self.Kd = Kd  # Wzmocnienie różniczkowe
        
        # Domyślna pozycja docelowa - naturalnie stojąca postawa
        # (lekko zgięte kolana dla stabilności)
        self.target_qpos = np.zeros(model.nv)
        
        # Dla humanoid: ustaw kolana w lekkim zgięciu
        # Indeksy zależą od struktury modelu - to są przykładowe wartości
        # W praktyce trzeba sprawdzić dokładną strukturę modelu
        
    def set_target(self, target_qpos):
        """
        Ustaw docelową pozycję dla wszystkich stawów.
        
        Args:
            target_qpos: Tablica docelowych pozycji [nv]
        """
        self.target_qpos = np.array(target_qpos)
    
    def compute_control(self, data):
        """
        Oblicz sygnały sterujące na podstawie aktualnego stanu.
        
        Args:
            data: Dane symulacji MjData
            
        Returns:
            Tablica sygnałów sterujących [nu]
        """
        # Dla każdego siłownika oblicz sygnał sterujący
        control = np.zeros(self.model.nu)
        
        for i in range(self.model.nu):
            # Znajdź staw połączony z tym siłownikiem
            # actuator_trnid zawiera indeksy transmission (połączenie actuator->joint)
            joint_id = self.model.actuator_trnid[i, 0]
            
            # UWAGA: qpos i qvel mają różne indeksowanie!
            # Używamy dedykowanych tablic adresów z modelu (poprawna metoda)
            # jnt_qposadr - tablica adresów startowych pozycji dla każdego stawu
            # jnt_dofadr - tablica adresów startowych DOF dla każdego stawu
            
            # Pobierz poprawne indeksy z modelu
            if joint_id < self.model.njnt:
                qpos_idx = self.model.jnt_qposadr[joint_id]
                qvel_idx = self.model.jnt_dofadr[joint_id]
            else:
                continue  # Pomiń jeśli joint_id jest nieprawidłowy
            
            if qpos_idx < len(data.qpos) and qvel_idx < len(data.qvel):
                # Aktualny stan stawu
                q_actual = data.qpos[qpos_idx]
                qvel_actual = data.qvel[qvel_idx]
                
                # Docelowy stan
                q_desired = self.target_qpos[joint_id] if joint_id < len(self.target_qpos) else 0
                
                # OBLICZ SYGNAŁ STERUJĄCY - wzór regulatora PD
                # Człon P: proporcjonalny do błędu pozycji
                error_position = q_desired - q_actual
                
                # Człon D: proporcjonalny do prędkości (chcemy ją zmniejszyć do 0)
                error_velocity = 0 - qvel_actual
                
                # Sygnał wynikowy
                control_signal = self.Kp * error_position + self.Kd * error_velocity
                
                # Ogranicz do zakresu siłownika
                ctrl_min, ctrl_max = self.model.actuator_ctrlrange[i]
                control[i] = np.clip(control_signal, ctrl_min, ctrl_max)
        
        return control


# =============================================================================
# SEKCJA 4: GŁÓWNA PĘTLA SYMULACJI
# =============================================================================

def run_simulation_with_viewer():
    """
    Uruchom symulację z interaktywnym viewerem.
    
    Ta funkcja:
    1. Tworzy regulator PD
    2. Uruchamia viewer w trybie pasywnym (mamy kontrolę nad pętlą)
    3. W każdej iteracji:
       - Oblicza sygnały sterujące
       - Wykonuje krok symulacji
       - Aktualizuje wizualizację
    """
    print("\n" + "-" * 70)
    print("URUCHAMIANIE SYMULACJI")
    print("-" * 70)
    print("Instrukcje:")
    print("  - Używaj myszki do obrotu kamery (lewy przycisk)")
    print("  - Scrolluj do przybliżenia/oddalenia")
    print("  - Spacja: pauza/wznów")
    print("  - Backspace: reset symulacji")
    print("  - Ctrl + prawy przycisk: pchnij robota!")
    print("  - Zamknij okno aby zakończyć")
    print("-" * 70)
    
    # Utwórz regulator PD
    # Parametry Kp i Kd są dobrane eksperymentalnie dla modelu humanoid
    controller = PDController(model, Kp=100.0, Kd=10.0)
    
    # Licznik kroków (do statystyk)
    step_count = 0
    start_time = time.time()
    
    # Zmienne do zbierania danych
    heights = []  # Historia wysokości centrum masy
    
    # Uruchom viewer w trybie pasywnym
    # Tryb pasywny daje nam pełną kontrolę nad pętlą symulacji
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # Ustaw początkową pozycję kamery dla lepszego widoku
        viewer.cam.azimuth = 90      # Kąt obrotu wokół osi Z
        viewer.cam.elevation = -15   # Kąt elewacji (wysokość)
        viewer.cam.distance = 5.0    # Odległość od robota
        viewer.cam.lookat[:] = [0, 0, 1]  # Punkt na który patrzy kamera
        
        # Główna pętla symulacji
        # Działa dopóki użytkownik nie zamknie okna
        while viewer.is_running():
            
            # KROK 1: OBLICZ STEROWANIE
            # Regulator PD oblicza sygnały dla wszystkich siłowników
            data.ctrl[:] = controller.compute_control(data)
            
            # KROK 2: WYKONAJ KROK SYMULACJI
            # mj_step() to główna funkcja MuJoCo - wykonuje jeden krok czasowy:
            #   - Oblicza dynamikę (siły, momenty)
            #   - Wykrywa kolizje
            #   - Aktualizuje pozycje i prędkości
            mujoco.mj_step(model, data)
            
            # KROK 3: ZBIERAJ DANE (opcjonalnie)
            # qpos[2] to wysokość Z centrum masy (dla freejoint)
            if step_count % 10 == 0:  # Zbieraj co 10 kroków
                heights.append(data.qpos[2])
            
            # KROK 4: SYNCHRONIZUJ WIZUALIZACJĘ
            # Aktualizuje obraz w viewerze zgodnie z aktualnym stanem
            viewer.sync()
            
            # KROK 5: WYŚWIETL INFORMACJE (co 100 kroków)
            if step_count % 100 == 0 and step_count > 0:
                elapsed = time.time() - start_time
                fps = step_count / elapsed
                
                # Wyświetl tekst na ekranie viewera
                info_text = f"Krok: {step_count}\n"
                info_text += f"Czas: {data.time:.2f}s\n"
                info_text += f"FPS: {fps:.1f}\n"
                info_text += f"Wysokość: {data.qpos[2]:.2f}m\n"
                info_text += f"Energia: {data.energy[0] + data.energy[1]:.2f}J"
                
                viewer.set_texts(
                    (mujoco.mjtFontScale.mjFONTSCALE_150,
                     mujoco.mjtGridPos.mjGRID_TOPLEFT,
                     info_text,
                     "")
                )
            
            step_count += 1
    
    # Podsumowanie po zakończeniu
    print("\n" + "=" * 70)
    print("PODSUMOWANIE SYMULACJI")
    print("=" * 70)
    print(f"Całkowity czas symulacji: {data.time:.2f}s")
    print(f"Liczba kroków: {step_count}")
    print(f"Średnia wysokość robota: {np.mean(heights):.2f}m")
    print(f"Min wysokość: {np.min(heights):.2f}m")
    print(f"Max wysokość: {np.max(heights):.2f}m")


# =============================================================================
# SEKCJA 5: URUCHOMIENIE PROGRAMU
# =============================================================================

if __name__ == "__main__":
    """
    Punkt wejścia programu.
    
    Uruchom ten skrypt poleceniem:
        python przykladowy_skrypt_studentow.py
    
    Upewnij się, że jesteś w głównym katalogu repozytorium MuJoCo.
    """
    try:
        # Uruchom symulację
        run_simulation_with_viewer()
        
        print("\n✓ Symulacja zakończona pomyślnie!")
        print("\nNastępne kroki:")
        print("  1. Zmień parametry Kp i Kd w kontrolerze i zobacz jak to wpływa na stabilność")
        print("  2. Zmodyfikuj target_qpos aby robot wykonywał różne ruchy")
        print("  3. Dodaj własne sensory i zbieraj więcej danych")
        print("  4. Zobacz przewodniki: PRZEWODNIK_STUDENTA.md i UNITREE_G1_PRZEWODNIK.md")
        
    except KeyboardInterrupt:
        print("\n\n✓ Symulacja przerwana przez użytkownika (Ctrl+C)")
    except Exception as e:
        print(f"\n✗ Wystąpił błąd: {e}")
        import traceback
        traceback.print_exc()
