# ----------------------------------------
# Abschnitt: Bibliotheken importieren
# ----------------------------------------

import cv2
import cvzone
from cvzone.HandTrackingModule import HandDetector
from picamera2 import Picamera2
import lgpio
import time
import numpy as np

# ----------------------------------------
# Abschnitt: Servo-Einstellungen
# ----------------------------------------

# Bestehende Servos
SERVO_PIN_1 = 18  # Erster Servo an Pin 18
SERVO_PIN_2 = 6   # Zweiter Servo an Pin 6
SERVO_PIN_3 = 13  # Dritter Servo an Pin 13
PWM_FREQUENCY = 333  # 333 Hz für Servos 1-3

# Neuer Servo 4 Einstellungen
SERVO_PIN_4 = 19  # Vierter Servo an Pin 19
SERVO_FREQUENCY_4 = 252  # 252 Hz für Servo 4

# Erweiterter Pulsbreitenbereich für Servos 1-3
MIN_PULSE_WIDTH = 900   # Ermöglicht Bewegung leicht unter 0 Grad
MAX_PULSE_WIDTH = 2100  # Ermöglicht Bewegung leicht über 180 Grad

# Pulsbreitenparameter für Servo 4
SERVO_MIN_PULSE_WIDTH_MS_4 = 1.0   # Pulsbreite in Millisekunden für 0 Grad
SERVO_MAX_PULSE_WIDTH_MS_4 = 2.0   # Pulsbreite in Millisekunden für 180 Grad

# PWM-Periode für Servo 4 berechnen
PWM_PERIOD_MS_4 = (1 / SERVO_FREQUENCY_4) * 1000  # In Millisekunden umrechnen

# Tastverhältnisse für Servo 4 berechnen
SERVO_MIN_DUTY_4 = (SERVO_MIN_PULSE_WIDTH_MS_4 / PWM_PERIOD_MS_4) * 100
SERVO_MAX_DUTY_4 = (SERVO_MAX_PULSE_WIDTH_MS_4 / PWM_PERIOD_MS_4) * 100

# Bewegungsparameter
STEP_SIZE = 4  # Schrittgröße für sanftere Bewegung
DELAY_BETWEEN_STEPS = 0.06  # Verzögerung, um Servo zu stabilisieren

# ----------------------------------------
# Abschnitt: Schrittmotor-Steuerung
# ----------------------------------------

DIR_PIN = 20    # Pin für Schrittmotorrichtung
STEP_PIN = 21   # Pin für Schrittmotorschritte
ON_TIME = 0.01  # Einschaltzeit pro Schritt
OFF_TIME = 0.01 # Ausschaltzeit pro Schritt
CALIBRATION_PIN = 22  # Pin für Kalibrierung (Endschalter)

# Schrittmotorparameter
STEPPER_STEPS_PER_REV = 200  # Schritte pro Umdrehung (z.B. 200 für 1,8-Grad-Schrittmotor)

# ----------------------------------------
# Abschnitt: Taster-Steuerung
# ----------------------------------------

BUTTON_PIN_1 = 24  # Taster eins (Exit-Taste)
BUTTON_PIN_2 = 27  # Taster zwei (Ruheposition)
BUTTON_PIN_3 = 25  # Taster drei (Startposition)

# ----------------------------------------
# Abschnitt: GPIO-Initialisierung
# ----------------------------------------

# GPIO-Chip initialisieren und Pins konfigurieren
chip = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(chip, SERVO_PIN_1)
lgpio.gpio_claim_output(chip, SERVO_PIN_2)
lgpio.gpio_claim_output(chip, SERVO_PIN_3)
lgpio.gpio_claim_output(chip, SERVO_PIN_4)
lgpio.gpio_claim_output(chip, DIR_PIN)
lgpio.gpio_claim_output(chip, STEP_PIN)
lgpio.gpio_claim_input(chip, CALIBRATION_PIN)  # Kalibrierungspin als Eingang konfigurieren
lgpio.gpio_claim_input(chip, BUTTON_PIN_1)     # Taster eins als Eingang
lgpio.gpio_claim_input(chip, BUTTON_PIN_2)     # Taster zwei als Eingang
lgpio.gpio_claim_input(chip, BUTTON_PIN_3)     # Taster drei als Eingang

# ----------------------------------------
# Abschnitt: Hilfsfunktionen
# ----------------------------------------

def pulse_width_to_duty_cycle(pulse_width, frequency):
    """Konvertiert Pulsbreite in Tastverhältnis."""
    period_us = (1 / frequency) * 1e6  # Periode in Mikrosekunden
    return (pulse_width / period_us) * 100  # Pulsbreite in Tastverhältnis umrechnen

def set_servo_angle(servo_pin, angle):
    """Setzt den Servo auf den angegebenen Winkel (zwischen 0 und 180 Grad)."""
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    duty_cycle = pulse_width_to_duty_cycle(pulse_width, PWM_FREQUENCY)
    
    # Servo bewegen
    lgpio.tx_pwm(chip, servo_pin, PWM_FREQUENCY, duty_cycle)
    time.sleep(DELAY_BETWEEN_STEPS)  # Servo Zeit zum Bewegen geben
    lgpio.tx_pwm(chip, servo_pin, PWM_FREQUENCY, 0)  # PWM-Signal stoppen

def set_servo_angle_4(angle):
    """Setzt Servo 4 auf den angegebenen Winkel (zwischen 0 und 180 Grad) und hält die Position."""
    duty_cycle = SERVO_MIN_DUTY_4 + (angle / 180.0) * (SERVO_MAX_DUTY_4 - SERVO_MIN_DUTY_4)
    
    # Servo bewegen
    lgpio.tx_pwm(chip, SERVO_PIN_4, SERVO_FREQUENCY_4, duty_cycle)
    time.sleep(0.5)  # Verzögerung anpassen
    lgpio.tx_pwm(chip, SERVO_PIN_4, SERVO_FREQUENCY_4, 0)  # PWM-Signal stoppen

def rotate_stepper(direction):
    """Bewegt den Schrittmotor basierend auf der Richtung."""
    lgpio.gpio_write(chip, DIR_PIN, direction)  # Richtung setzen
    lgpio.gpio_write(chip, STEP_PIN, 1)
    time.sleep(ON_TIME)
    lgpio.gpio_write(chip, STEP_PIN, 0)
    time.sleep(OFF_TIME)

def rotate_stepper_angle(angle_degrees, direction):
    """Dreht den Schrittmotor um einen bestimmten Winkel in Grad."""
    steps_needed = int((STEPPER_STEPS_PER_REV / 360.0) * angle_degrees)
    lgpio.gpio_write(chip, DIR_PIN, direction)  # Richtung setzen
    print(f"Schrittmotor dreht {angle_degrees} Grad nach {'links' if direction == 0 else 'rechts'}...")
    for _ in range(steps_needed):
        lgpio.gpio_write(chip, STEP_PIN, 1)
        time.sleep(ON_TIME)
        lgpio.gpio_write(chip, STEP_PIN, 0)
        time.sleep(OFF_TIME)

def calibrate_stepper():
    """Kalibriert den Schrittmotor."""
    print("Kalibriere Schrittmotor...")
    # Überprüfen, ob der Kalibrierungspin aktiv ist
    if lgpio.gpio_read(chip, CALIBRATION_PIN) == 1:
        # Vom Nullpunkt wegbewegen
        lgpio.gpio_write(chip, DIR_PIN, 1)  # 1 bewegt sich vom Nullpunkt weg
        steps_taken = 0
        max_steps = 1000
        while lgpio.gpio_read(chip, CALIBRATION_PIN) == 1 and steps_taken < max_steps:
            lgpio.gpio_write(chip, STEP_PIN, 1)
            time.sleep(ON_TIME)
            lgpio.gpio_write(chip, STEP_PIN, 0)
            time.sleep(OFF_TIME)
            steps_taken += 1
        if steps_taken >= max_steps:
            print("Kalibrierung fehlgeschlagen: Konnte sich nicht vom Nullpunkt entfernen")
            return
    # Zum Nullpunkt bewegen
    lgpio.gpio_write(chip, DIR_PIN, 0)  # 0 bewegt sich zum Nullpunkt
    steps_taken = 0
    max_steps = 1000
    while lgpio.gpio_read(chip, CALIBRATION_PIN) == 0 and steps_taken < max_steps:
        lgpio.gpio_write(chip, STEP_PIN, 1)
        time.sleep(ON_TIME)
        lgpio.gpio_write(chip, STEP_PIN, 0)
        time.sleep(OFF_TIME)
        steps_taken += 1
    if steps_taken >= max_steps:
        print("Kalibrierung fehlgeschlagen: Konnte Nullpunkt nicht finden")
    else:
        print("Schrittmotor-Kalibrierung abgeschlossen.")

# ----------------------------------------
# Abschnitt: Kamera-Initialisierung
# ----------------------------------------

# Picamera2 initialisieren
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# ----------------------------------------
# Abschnitt: Hauptfunktion zur Steuerung
# ----------------------------------------

def start_camera_and_control_servos():
    """Startet die Kamera und steuert Servos und Schrittmotor basierend auf Handgesten."""
    # Handdetektor initialisieren
    detector = HandDetector(maxHands=2, detectionCon=0.5, minTrackCon=0.5)
    
    # Anfangspositionen der Servos
    current_angle_1 = 90
    current_angle_2 = 90
    current_angle_3 = 90
    set_servo_angle(SERVO_PIN_1, current_angle_1)
    set_servo_angle(SERVO_PIN_2, current_angle_2)
    set_servo_angle(SERVO_PIN_3, current_angle_3)
    
    # Servo 4 initialisieren
    servo_4_position = 'open'
    current_angle_4 = 50
    set_servo_angle_4(current_angle_4)
    
    # Schrittmotor kalibrieren
    calibrate_stepper()
    
    moving_servo = None  # Verfolgt, welcher Servo sich bewegt
    
    # Vorherige Fingeranzahl für jede Hand
    prev_num_fingers = {'Left': 0, 'Right': 0}
    
    # Tasterzustände initialisieren
    button_pressed_1 = False  # Für Taster eins (Exit)
    button_pressed_2 = False
    button_pressed_3 = False
    
    # Flag zum Deaktivieren des Schrittmotors
    stepper_disabled = False

    # ----------------------------------------
    # Abschnitt: Fenster im Vollbildmodus einstellen
    # ----------------------------------------

    # Fenster "im" im Vollbildmodus öffnen
    cv2.namedWindow("im", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("im", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    
    while True:
        # Tasterzustände lesen
        button_state_1 = lgpio.gpio_read(chip, BUTTON_PIN_1)
        button_state_2 = lgpio.gpio_read(chip, BUTTON_PIN_2)
        button_state_3 = lgpio.gpio_read(chip, BUTTON_PIN_3)
    
        # Taster eins (Exit-Taste) behandeln
        if button_state_1 == 1 and not button_pressed_1:
            button_pressed_1 = True
            print("Exit-Taste (Taster eins) gedrückt")
            break  # Schleife verlassen
    
        elif button_state_1 == 0 and button_pressed_1:
            button_pressed_1 = False  # Taster losgelassen
    
        # Taster zwei behandeln
        if button_state_2 == 1 and not button_pressed_2:
            button_pressed_2 = True
            print("Taster zwei gedrückt")
    
            # Schrittmotor kalibrieren
            calibrate_stepper()
    
            # Schrittmotor um 135 Grad nach links drehen
            rotate_stepper_angle(135, direction=0)  # 0 für links
    
            # Servos auf bestimmte Positionen bewegen
            target_angle_1 = 135
            target_angle_2 = 175
            target_angle_3 = 7
    
            # Schritte für jeden Servo berechnen
            steps_1 = int(abs(current_angle_1 - target_angle_1) / STEP_SIZE)
            steps_2 = int(abs(current_angle_2 - target_angle_2) / STEP_SIZE)
            steps_3 = int(abs(current_angle_3 - target_angle_3) / STEP_SIZE)
            max_steps = max(steps_1, steps_2, steps_3)
    
            for _ in range(max_steps):
                if current_angle_1 != target_angle_1:
                    if current_angle_1 < target_angle_1:
                        current_angle_1 = min(current_angle_1 + STEP_SIZE, target_angle_1)
                    else:
                        current_angle_1 = max(current_angle_1 - STEP_SIZE, target_angle_1)
                    set_servo_angle(SERVO_PIN_1, current_angle_1)
    
                if current_angle_2 != target_angle_2:
                    if current_angle_2 < target_angle_2:
                        current_angle_2 = min(current_angle_2 + STEP_SIZE, target_angle_2)
                    else:
                        current_angle_2 = max(current_angle_2 - STEP_SIZE, target_angle_2)
                    set_servo_angle(SERVO_PIN_2, current_angle_2)
    
                if current_angle_3 != target_angle_3:
                    if current_angle_3 < target_angle_3:
                        current_angle_3 = min(current_angle_3 + STEP_SIZE, target_angle_3)
                    else:
                        current_angle_3 = max(current_angle_3 - STEP_SIZE, target_angle_3)
                    set_servo_angle(SERVO_PIN_3, current_angle_3)
    
                time.sleep(DELAY_BETWEEN_STEPS)
    
            # Weitere Schrittmotorbewegung deaktivieren
            stepper_disabled = True
            print("Servos auf Position bewegt. Schrittmotor deaktiviert.")
    
        elif button_state_2 == 0 and button_pressed_2:
            button_pressed_2 = False  # Taster losgelassen
    
        # Taster drei behandeln
        if button_state_3 == 1 and not button_pressed_3:
            button_pressed_3 = True
            print("Taster drei gedrückt")
    
            # Servos 1-3 langsam auf 90 Grad bewegen (Servo 4 ausgenommen)
            target_angle_1 = 90
            target_angle_2 = 90
            target_angle_3 = 90
    
            # Schritte für jeden Servo berechnen
            steps_1 = int(abs(current_angle_1 - target_angle_1) / STEP_SIZE)
            steps_2 = int(abs(current_angle_2 - target_angle_2) / STEP_SIZE)
            steps_3 = int(abs(current_angle_3 - target_angle_3) / STEP_SIZE)
            max_steps = max(steps_1, steps_2, steps_3)
    
            for _ in range(max_steps):
                if current_angle_1 != target_angle_1:
                    if current_angle_1 < target_angle_1:
                        current_angle_1 = min(current_angle_1 + STEP_SIZE, target_angle_1)
                    else:
                        current_angle_1 = max(current_angle_1 - STEP_SIZE, target_angle_1)
                    set_servo_angle(SERVO_PIN_1, current_angle_1)
    
                if current_angle_2 != target_angle_2:
                    if current_angle_2 < target_angle_2:
                        current_angle_2 = min(current_angle_2 + STEP_SIZE, target_angle_2)
                    else:
                        current_angle_2 = max(current_angle_2 - STEP_SIZE, target_angle_2)
                    set_servo_angle(SERVO_PIN_2, current_angle_2)
    
                if current_angle_3 != target_angle_3:
                    if current_angle_3 < target_angle_3:
                        current_angle_3 = min(current_angle_3 + STEP_SIZE, target_angle_3)
                    else:
                        current_angle_3 = max(current_angle_3 - STEP_SIZE, target_angle_3)
                    set_servo_angle(SERVO_PIN_3, current_angle_3)
    
                time.sleep(DELAY_BETWEEN_STEPS)
    
            # Schrittmotor kalibrieren
            calibrate_stepper()
            print("Servos 1-3 auf 90 Grad bewegt und Schrittmotor kalibriert.")
    
        elif button_state_3 == 0 and button_pressed_3:
            button_pressed_3 = False  # Taster losgelassen
    
        # Bild von der Kamera erfassen
        im = picam2.capture_array()
        im = cv2.flip(im, 1)  # Bild horizontal spiegeln für korrekte Links-Rechts-Ausrichtung
    
        # Hände im Bild erkennen, ohne Beschriftungen zu zeichnen
        hands, _ = detector.findHands(im, draw=False)
    
        # Handumrisse und Verbindungen manuell zeichnen, ohne die Labels "Left" und "Right"
        if hands:
            for hand in hands:
                lmList = hand['lmList']  # Landmarkenliste
                bbox = hand['bbox']      # Begrenzungsrahmen
                # Handumriss zeichnen
                cv2.rectangle(im, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), (255, 0, 255), 2)
                # Punkte (Landmarken) zeichnen
                for lm in lmList:
                    cv2.circle(im, (lm[0], lm[1]), 5, (0, 255, 0), cv2.FILLED)
                # Verbindungen zwischen den Punkten zeichnen
                connections = [
                    (0, 1), (1, 2), (2, 3), (3, 4),        # Daumen
                    (0, 5), (5, 6), (6, 7), (7, 8),        # Zeigefinger
                    (5, 9), (9, 10), (10, 11), (11, 12),   # Mittelfinger
                    (9, 13), (13, 14), (14, 15), (15, 16), # Ringfinger
                    (13, 17), (17, 18), (18, 19), (19, 20),# Kleiner Finger
                    (0, 17)  # Handfläche zu kleinem Finger Basis
                ]
                for connection in connections:
                    start_point = lmList[connection[0]]
                    end_point = lmList[connection[1]]
                    cv2.line(im, (start_point[0], start_point[1]), (end_point[0], end_point[1]), (0, 255, 0), 2)
    
        # Wenn Hände erkannt wurden
        if hands:
            for hand in hands:
                handType = hand['type']  # Linke oder rechte Hand
                fingers = detector.fingersUp(hand)
    
                # Handtyp für Klarheit umbenennen
                if handType == 'Left':
                    labeled_hand_type = 'Left'
                else:
                    labeled_hand_type = 'Right'
                
                # Anzahl der gestreckten Finger zählen
                num_fingers = fingers.count(1)
                print(f"Erkannter Handtyp: {labeled_hand_type}, Gestreckte Finger: {num_fingers}")
    
                # Servo 4 (Greifer) Bewegung behandeln
                if num_fingers == 5 and prev_num_fingers[labeled_hand_type] != 5:
                    # Geste hat sich zu 5 Fingern geändert
                    if labeled_hand_type == 'Left':
                        # Servo 4 auf 180 Grad bewegen (schließen)
                        if servo_4_position != 'closed':
                            current_angle_4 = 190
                            set_servo_angle_4(current_angle_4)
                            servo_4_position = 'closed'
                            print("Servo 4 auf 180 Grad bewegt (geschlossen)")
                    elif labeled_hand_type == 'Right':
                        # Servo 4 auf 135 Grad bewegen (öffnen)
                        if servo_4_position != 'open':
                            current_angle_4 = 142  # Angepasste Greifposition
                            set_servo_angle_4(current_angle_4)
                            servo_4_position = 'open'
                            print("Servo 4 auf 135 Grad bewegt (geöffnet)")
    
                # Vorherige Fingeranzahl aktualisieren
                prev_num_fingers[labeled_hand_type] = num_fingers
    
                # Servo 1: Gesteuert durch 1 Finger (Daumen)
                if labeled_hand_type == 'Left' and num_fingers == 1 and moving_servo in [None, 1]:
                    current_angle_1 = max(current_angle_1 - STEP_SIZE, -10)  # Nach links drehen
                    moving_servo = 1
                elif labeled_hand_type == 'Right' and num_fingers == 1 and moving_servo in [None, 1]:
                    current_angle_1 = min(current_angle_1 + STEP_SIZE, 190)  # Nach rechts drehen
                    moving_servo = 1
    
                # Servo 2: Gesteuert durch 2 Finger (Zeige- und Mittelfinger)
                elif labeled_hand_type == 'Right' and num_fingers == 2 and moving_servo in [None, 2]:
                    current_angle_2 = max(current_angle_2 - STEP_SIZE, -10)  # Nach links drehen
                    moving_servo = 2
                elif labeled_hand_type == 'Left' and num_fingers == 2 and moving_servo in [None, 2]:
                    current_angle_2 = min(current_angle_2 + STEP_SIZE, 190)  # Nach rechts drehen
                    moving_servo = 2
    
                # Servo 3: Gesteuert durch 3 Finger (Zeige-, Mittel- und Ringfinger)
                elif labeled_hand_type == 'Left' and num_fingers == 3 and moving_servo in [None, 3]:
                    current_angle_3 = max(current_angle_3 - STEP_SIZE, -10)  # Nach links drehen
                    moving_servo = 3
                elif labeled_hand_type == 'Right' and num_fingers == 3 and moving_servo in [None, 3]:
                    current_angle_3 = min(current_angle_3 + STEP_SIZE, 190)  # Nach rechts drehen
                    moving_servo = 3
    
                # Schrittmotor: Gesteuert durch 4 Finger, nur wenn nicht deaktiviert
                elif num_fingers == 4 and not stepper_disabled:
                    direction = 0 if labeled_hand_type == 'Left' else 1
                    rotate_stepper(direction)
                    moving_servo = None  # moving_servo zurücksetzen
    
            # Nur den aktuell aktiven Servo aktualisieren
            if moving_servo == 1:
                set_servo_angle(SERVO_PIN_1, current_angle_1)
            elif moving_servo == 2:
                set_servo_angle(SERVO_PIN_2, current_angle_2)
            elif moving_servo == 3:
                set_servo_angle(SERVO_PIN_3, current_angle_3)
    
            time.sleep(DELAY_BETWEEN_STEPS)
    
            # moving_servo nach der Bewegung zurücksetzen
            moving_servo = None
    
        # Bild anzeigen (Bildübertragung wird auch ohne Handgesten aktualisiert)
        cv2.imshow("im", im)
    
        # Schleife beenden, wenn 'ESC' gedrückt wird
        key = cv2.waitKey(1)
        if key == 27:  # ESC-Taste
            break
    
    # Servos bleiben in ihrer aktuellen Position
    print("Servos bleiben in ihrer Endposition.")
    
    # Aufräumen
    picam2.stop()
    cv2.destroyAllWindows()

# ----------------------------------------
# Abschnitt: Hauptprogramm-Ausführung
# ----------------------------------------

try:
    print("Starte Kamera und steuere Servos und Schrittmotor mit Handgesten...")
    start_camera_and_control_servos()

except KeyboardInterrupt:
    print("Programm wird beendet")

# ----------------------------------------
# Abschnitt: GPIO-Aufräumen
# ----------------------------------------

# GPIO aufräumen - Servos und Schrittmotor bleiben in ihrer aktuellen Position
lgpio.gpiochip_close(chip)  # Chip sicher schließen



# 4. Oktober 2024, Lars Nussbaumer, überarbeitet von GPT-4