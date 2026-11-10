#!/usr/bin/env python3
"""
Visualisateur LiDAR LD06 en temps réel avec Pygame.
Projet d'apprentissage SLAM & Robotique.

Ce script :
1. Lit le flux binaire série du LD06 à 230 400 bauds.
2. Décode les paquets de 47 octets (angles, distances, intensité).
3. Transforme les coordonnées polaires (r, θ) en coordonnées cartésiennes (x, y) [Trigonométrie].
4. Projette les coordonnées métriques vers les pixels de l'écran (avec inversion de l'axe Y).
5. Affiche le nuage de points et les repères métriques en temps réel à 60+ FPS.
"""

import sys
import math
import serial
import pygame

# ==============================================================================
# 1. PARAMÈTRES MATÉRIELS DU LD06
# ==============================================================================
SERIAL_PORT = "/dev/ttyUSB0"  # Port sur lequel le convertisseur USB-UART est branché
BAUD_RATE = 230400            # Vitesse de communication standard du LD06
PKG_HEADER = 0x54             # Octet magique marquant le début de chaque paquet
PKG_VER_LEN = 0x2C            # Longueur du paquet (0x2C = 44 octets après le header)
POINT_COUNT = 12              # Chaque trame contient exactement 12 mesures de distance

# ==============================================================================
# 2. PARAMÈTRES D'AFFICHAGE PYGAME
# ==============================================================================
WINDOW_SIZE = 800             # Fenêtre carrée de 800 x 800 pixels
CENTER_X = WINDOW_SIZE // 2   # Centre horizontal (400 px)
CENTER_Y = WINDOW_SIZE // 2   # Centre vertical (400 px)
SCALE = 90                    # Facteur d'échelle : 90 pixels = 1 mètre dans le monde réel

# Palette de couleurs (format RGB : Rouge, Vert, Bleu de 0 à 255)
COLOR_BG = (15, 17, 26)       # Fond bleu nuit très sombre
COLOR_GRID = (50, 60, 80)     # Grille et cercles gris-bleu discret
COLOR_TEXT = (200, 210, 225)  # Texte blanc cassé
COLOR_LIDAR = (0, 255, 180)   # Points d'obstacles vert émeraude fluo
COLOR_ROBOT = (255, 75, 75)   # Position du LiDAR au centre en rouge vif

# ==============================================================================
# 3. VÉRIFICATION DU CRC8 (Somme de contrôle)
# ==============================================================================
# Table de correspondance polynomiale spécifique au constructeur du LD06.
# Elle sert à vérifier qu'aucun bit n'a été corrompu pendant le transfert USB.
CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
]

def verify_crc(packet_bytes):
    """Calcule le CRC8 sur les 46 premiers octets et compare avec le 47ème."""
    crc = 0
    for byte in packet_bytes[:46]:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc == packet_bytes[46]


# ==============================================================================
# 4. DÉCODAGE DES DONNÉES DU LD06 (Protocole binaire)
# ==============================================================================

def decode_packet(packet):
    """
    Extrait les angles et les 12 mesures de distance d'un paquet de 47 octets.
    """
    # 1. Lecture de l'angle de départ et de fin (encodés en centièmes de degré)
    # En Little-Endian : valeur = octet_bas + (octet_haut * 256)
    start_angle = (packet[4] | (packet[5] << 8)) / 100.0
    end_angle = (packet[42] | (packet[43] << 8)) / 100.0

    # Gestion de la transition 360° -> 0° (quand le laser franchit l'axe zéro)
    diff_angle = end_angle - start_angle
    if diff_angle < 0:
        diff_angle += 360.0

    # L'écart angulaire entre deux points consécutifs parmi les 12 points
    step_angle = diff_angle / (POINT_COUNT - 1)

    points = []
    for i in range(POINT_COUNT):
        offset = 6 + i * 3
        # 2 octets pour la distance en millimètres
        distance_mm = packet[offset] | (packet[offset + 1] << 8)
        # 1 octet pour la confiance (qualité de la réflexion du laser sur la cible)
        confidence = packet[offset + 2]

        # Interpolation linéaire de l'angle pour le i-ème point
        angle_deg = (start_angle + i * step_angle) % 360.0

        # Conversion de millimètres en mètres (standard en robotique)
        distance_m = distance_mm / 1000.0

        points.append((angle_deg, distance_m, confidence))

    return points


# ==============================================================================
# 5. MATHÉMATIQUES : TRIGONOMÉTRIE & CHANGEMENT DE REPÈRE
# ==============================================================================

def polar_to_cartesian(distance_m, angle_deg):
    """
    CONVERSION POLAIRE -> CARTÉSIEN
    
    Prend :
      - distance_m : distance en mètres (norme du vecteur r)
      - angle_deg  : angle en degrés (orientation θ)

    Formules :
      x = r * cos(θ)
      y = r * sin(θ)
    
    Attention : math.cos et math.sin en Python attendent des RADIANS !
    """
    angle_rad = math.radians(angle_deg)
    x = distance_m * math.cos(angle_rad)
    y = distance_m * math.sin(angle_rad)
    return x, y


def world_to_screen(x_m, y_m):
    """
    TRANSFORMATION MONDE RÉEL -> ÉCRAN PIXELS
    
    Monde réel (mètres) :
      - (0, 0) est le robot au centre.
      - X va vers la droite (+X).
      - Y va vers l'avant (+Y).

    Écran d'ordinateur (pixels) :
      - (0, 0) est en haut à gauche.
      - X va vers la droite (pareil).
      - Y va vers le bas (INVERSÉ !).

    On applique l'échelle (SCALE) et on décale au centre de l'écran (CENTER_X, CENTER_Y).
    """
    screen_x = int(CENTER_X + x_m * SCALE)
    screen_y = int(CENTER_Y - y_m * SCALE)  # Signe moins car l'axe Y écran est inversé
    return screen_x, screen_y


# ==============================================================================
# 6. INTERFACE GRAPHIQUE (Dessin de la grille et des repères)
# ==============================================================================

def draw_grid(screen, font):
    """
    Dessine la mire radar avec les cercles métriques et les axes.
    """
    # 1. Cercles concentriques de distance (1m, 2m, 3m, 4m)
    for r_meter in range(1, 5):
        radius_px = int(r_meter * SCALE)
        # Cercle fin de repère
        pygame.draw.circle(screen, COLOR_GRID, (CENTER_X, CENTER_Y), radius_px, 1)
        # Texte indiquant le métrage
        label = font.render(f"{r_meter}m", True, COLOR_TEXT)
        screen.blit(label, (CENTER_X + radius_px + 4, CENTER_Y - 12))

    # 2. Axes orthogonaux (croix centrale X / Y)
    pygame.draw.line(screen, COLOR_GRID, (CENTER_X, 0), (CENTER_X, WINDOW_SIZE), 1)
    pygame.draw.line(screen, COLOR_GRID, (0, CENTER_Y), (WINDOW_SIZE, CENTER_Y), 1)

    # 3. Position centrale du robot / LiDAR (cercle rouge plein)
    pygame.draw.circle(screen, COLOR_ROBOT, (CENTER_X, CENTER_Y), 6)


# ==============================================================================
# 7. BOUCLE PRINCIPALE D'ACQUISITION ET DE RENDU
# ==============================================================================

def main():
    print("=" * 60)
    print(f"Connexion au LiDAR LD06 sur {SERIAL_PORT} ({BAUD_RATE} bauds)...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print("-> Connecté avec succès !")
    except Exception as e:
        print(f"ERREUR : Impossible d'ouvrir le port série : {e}")
        print("Astuce : As-tu exécuté 'sudo chmod 666 /dev/ttyUSB0' ?")
        sys.exit(1)

    # Initialisation du moteur Pygame
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("Visualiseur LiDAR LD06 - Apprentissage SLAM")
    font = pygame.font.SysFont("monospace", 14, bold=True)
    clock = pygame.time.Clock()

    # Tampon circulaire conservant les ~450 derniers points (1 tour complet à 10 Hz)
    scan_points = []
    MAX_POINTS_BUFFER = 480

    print("Interface démarrée ! Appuie sur Échap ou ferme la fenêtre pour quitter.")
    print("=" * 60)

    running = True
    while running:
        # 1. Gestion des événements utilisateur (clic sur la croix ou touche Échap)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        # 2. Lecture et décodage du flux série
        # On lit un octet pour chercher le marqueur de synchronisation 0x54
        b = ser.read(1)
        if len(b) > 0 and b[0] == PKG_HEADER:
            # Vérification du second octet de longueur (0x2C)
            b2 = ser.read(1)
            if len(b2) > 0 and b2[0] == PKG_VER_LEN:
                # Lecture des 45 octets restants du paquet (total = 47)
                rest = ser.read(45)
                if len(rest) == 45:
                    packet = bytes([PKG_HEADER, PKG_VER_LEN]) + rest

                    # Contrôle d'intégrité via CRC8
                    if verify_crc(packet):
                        new_points = decode_packet(packet)
                        for angle_deg, dist_m, conf in new_points:
                            # Filtrage du bruit :
                            # - distance > 5 cm (on ignore les reflets internes du boîtier)
                            # - distance < 12 m (portée max du capteur)
                            # - confiance > 120 (point net et fiable)
                            if 0.05 < dist_m < 12.0 and conf > 120:
                                # Trigonométrie : (distance, angle) -> (x, y)
                                x, y = polar_to_cartesian(dist_m, angle_deg)
                                # Projection écran : (x, y en mètres) -> (pixel_x, pixel_y)
                                sx, sy = world_to_screen(x, y)

                                scan_points.append((sx, sy))
                                # On maintient une taille maximale pour n'afficher que le scan frais
                                if len(scan_points) > MAX_POINTS_BUFFER:
                                    scan_points.pop(0)

        # 3. Rendu graphique
        screen.fill(COLOR_BG)          # Effacer l'écran avec la couleur de fond
        draw_grid(screen, font)        # Dessiner la mire radar

        # Dessiner chaque point d'obstacle détecté par le laser
        for px, py in scan_points:
            if 0 <= px < WINDOW_SIZE and 0 <= py < WINDOW_SIZE:
                pygame.draw.circle(screen, COLOR_LIDAR, (px, py), 2)

        # 4. Affichage des statistiques (HUD en haut à gauche)
        fps = int(clock.get_fps())
        hud_points = font.render(f"Points actifs : {len(scan_points)}", True, COLOR_TEXT)
        hud_fps = font.render(f"FPS : {fps}", True, COLOR_TEXT)
        hud_scale = font.render(f"Echelle : {SCALE} px/m", True, COLOR_TEXT)
        screen.blit(hud_points, (15, 15))
        screen.blit(hud_fps, (15, 35))
        screen.blit(hud_scale, (15, 55))

        # Rafraîchir l'écran
        pygame.display.flip()
        clock.tick(120)  # Boucle cadencée à haute fréquence pour vider le port série sans latence

    # Nettoyage propre en quittant
    ser.close()
    pygame.quit()
    print("Visualiseur fermé proprement.")


if __name__ == "__main__":
    main()
