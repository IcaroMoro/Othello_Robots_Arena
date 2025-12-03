# config.py - A Fonte da Verdade do Projeto Othello
import os
import math

# =================================================================
# --- REDE E CONEXÃO ---
# =================================================================

BRIDGE_HOST = "0.0.0.0"
BRIDGE_PORT = 8000
BRIDGE_URL_LOCAL = f"http://127.0.0.1:{BRIDGE_PORT}"

BRIDGE_URL_REMOTE = os.getenv("BRIDGE_URL", "https://viperish-pressuringly-janessa.ngrok-free.dev")
BRIDGE_TOKEN = os.getenv("BRIDGE_TOKEN", "33yYBVeHa0tHRcwskgckgGKPLSz_5cLRTcRXkWs1Rdh4Zr4JF")

DEFAULT_RID = int(os.environ.get("RID", "1"))

ROBOT1_IP = os.getenv("ROBOT1_IP", "192.168.1.201")
ROBOT2_IP = os.getenv("ROBOT2_IP", "192.168.1.202")

# =================================================================
# --- CINEMÁTICA E MOVIMENTO ---
# =================================================================

# --- VELOCIDADES (Nomes alinhados com bridge_server.py) ---
SPEED_PTP = 60          
SPEED_LINEAR = 120     
STAGING_SPEED = 60   

# --- ACELERAÇÕES ---
ACCEL_PTP = 60        
ACCEL_LINEAR = 60      

# Aliases para compatibilidade com verificador.py (se ele usar os nomes antigos)
MOVE_SPEED = SPEED_PTP
MOVE_SPEED_LINEAR = SPEED_LINEAR

# =================================================================
# --- CALIBRAÇÃO DOS ROBÔS ---
# =================================================================

# --- ROBÔ 1 (PRETAS) ---
CFG_R1 = {
    "BOARD_ORIGIN_X": -203.653,
    "BOARD_ORIGIN_Y": 189.311,
    "BOARD_ANGLE_RAD": 0.08551212,
    "SQUARE_SIZE_MM": 49.9035,
    
    "ESTOJO_OFFSET_X": 74.0,
    "ESTOJO_OFFSET_Y": -150.0,
    "ESTOJO_SPACING_MM": 12.6,
    
    "ORIENTATION_ESTOJO": (180.0, 0.1, math.degrees(0.06082656)), 
    "ORIENTATION_TABULEIRO_ESQ": (115.0, 1.0, 0.0),
    "ORIENTATION_TABULEIRO_DIR": (115.0, -1.0, 0.0),
    
    "Z_OPERATION": 10.5,
    "Z_OPERATION_ESTOJO": 23.0,
    "Z_APPROACH_MARGIN": 40.0, 
    
    "Z_FLIP_ASCEND_DELTA": 60.0,
    "Z_FLIP_DESCEND_DELTA": 60.0,
    "Z_FLIP_RETRACT_DELTA": 60.0,

    "Z_TRAVEL": 12.5 + 60.0,
    "Z_TRAVEL_ESTOJO": 23.0 + 60.0,

    "LADO_ESTOJO": "esq",
    "STAGING_ESQ": [70.0, -105.0, 110.0, -95.0, 90.0, -20.0],
    "STAGING_DIR": [70.0, -70.0, 70.0, 30.0, 250.0, 170.0]
}

# --- ROBÔ 2 (BRANCAS) ---
CFG_R2 = {
    "BOARD_ORIGIN_X": -176.5960,
    "BOARD_ORIGIN_Y": 207.2400,
    "BOARD_ANGLE_RAD": -0.02500313,
    "SQUARE_SIZE_MM": 49.9035,
    
    "ESTOJO_OFFSET_X": 74.0,
    "ESTOJO_OFFSET_Y": -150.0+12.6,
    "ESTOJO_SPACING_MM": 12.6,
    
    "ORIENTATION_ESTOJO": (180.0, 0.1, math.degrees(-0.02876740)),
    "ORIENTATION_TABULEIRO_ESQ": (115.0, 0.1, 0.0),
    "ORIENTATION_TABULEIRO_DIR": (115.0, -0.1, 180.0),
    
    "Z_OPERATION": 11.0,
    "Z_OPERATION_ESTOJO": 27.0,
    "Z_APPROACH_MARGIN": 40.0,
    
    "Z_FLIP_ASCEND_DELTA": 60.0,
    "Z_FLIP_DESCEND_DELTA": 60.0,
    "Z_FLIP_RETRACT_DELTA": 60.0,

    "Z_TRAVEL": 14.0 + 60.0,
    "Z_TRAVEL_ESTOJO": 27.0 + 60.0,

    "LADO_ESTOJO": "dir",
    "STAGING_ESQ": [70.0, -70.0, 70.0, 30.0, 250.0, 170.0],
    "STAGING_DIR": [70.0, -105.0, 110.0, -95.0, 90.0, -20.0]
}

ROBOT_CONFIGS = {1: CFG_R1, 2: CFG_R2}