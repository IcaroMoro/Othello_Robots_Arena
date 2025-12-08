import os
import sys
import time
import threading
import json
import requests
import math

# =================================================================
# --- IMPORTAÇÃO DO CONFIG.PY ---
# =================================================================

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(current_dir) 
sys.path.append(parent_dir)  

try:
    from config import (
        ROBOT_CONFIGS, 
        STAGING_SPEED,
        BRIDGE_URL_REMOTE, 
        BRIDGE_TOKEN, 
        MOVE_SPEED 
    )
except ImportError:
    print("❌ ERRO CRÍTICO: Não foi possível encontrar o arquivo 'config.py'.")
    print("Certifique-se de que ele está na raiz do projeto e que você está rodando este script corretamente.")
    sys.exit(1)

# =================================================================
# --- CONFIG DE CONEXÃO ---
# =================================================================

BASE = BRIDGE_URL_REMOTE
TOKEN = BRIDGE_TOKEN
RID = int(os.environ.get("RID", "1"))

# ---------- HTTP ----------
session = requests.Session()
if TOKEN: session.headers.update({"Authorization": f"Bearer {TOKEN}"})
session.headers.update({"Content-Type": "application/json"})

def http_post(path, json=None, **params):
    global RID
    url = f"{BASE.rstrip('/')}{path}"
    all_params = {"rid": RID, **params}
    try:
        r = session.post(url, json=json, params=all_params, timeout=600)
        return r.json()
    except Exception as e:
        print(f"[ERRO HTTP] {e}")
        return {"ok": False}

def http_get(path, **params):
    global RID
    url = f"{BASE.rstrip('/')}{path}"
    all_params = {"rid": RID, **params}
    try:
        r = session.get(url, params=all_params, timeout=5)
        return r.json()
    except: return {}

# ---------- UTILITÁRIOS ----------
def get_current_config():
    return ROBOT_CONFIGS.get(RID)

def normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180) % 360 - 180

# ---------- STATE MONITOR ----------
current_operation_side = 'esq' 
robot_status = {}
status_lock = threading.Lock()
stop_monitor = threading.Event()

def monitor_status():
    while not stop_monitor.is_set():
        try:
            st = http_get("/status")
            with status_lock:
                if isinstance(st, dict) and st.get('ok'): robot_status.update(st)
        except: pass
        time.sleep(0.5)

# --- COMANDOS BÁSICOS ---
def set_gripper_cmd(open_gripper: bool):
    r = http_post("/gripper", open=open_gripper); return r.get("ok", False)

def move_joints_cmd(joints: list, speed=MOVE_SPEED):
    r = http_post("/move/joints", json={"j1":joints[0], "j2":joints[1], "j3":joints[2], "j4":joints[3], "j5":joints[4], "j6":joints[5], "speed": speed})
    return r.get("ok", False)

def move_pose_cmd(x,y,z,rx,ry,rz, speed=MOVE_SPEED):
    r = http_post("/move/pose", json={"x":x,"y":y,"z":z,"rx":rx,"ry":ry,"rz":rz,"speed":speed})
    return r.get("ok", False)

def move_relative_cmd(dx=0,dy=0,dz=0,drx=0,dry=0,drz=0, speed=MOVE_SPEED):
    r = http_post("/move/relative", json={"dx":dx,"dy":dy,"dz":dz,"drx":drx,"dry":dry,"drz":drz,"speed":speed})
    return r.get("ok", False)

def rotate_gripper_cmd(angle, speed=MOVE_SPEED):
    r = http_post("/rotate/gripper", json={"angle": angle, "speed": speed}); return r.get("ok", False)

def move_to_home(): 
    global current_operation_side
    print("🏠 Indo para Home...")
    
    cfg = get_current_config()
    
    r = http_post("/home")
    if r.get("ok"): 
        current_operation_side = 'home' 
        return True
    return False

def habilitar_servos():
    print("⚡ Habilitando servos...")
    http_post("/habilitar")

def get_target_side(j: int) -> str:
    # Colunas 0,1,2,3 são sempre o lado "base" (RET) do algoritmo
    return 'ret' if j <= 3 else 'est'

def get_pose_from_grid(i: int, j: int, z_height: float):
    cfg = get_current_config()
    
    x0 = cfg["BOARD_ORIGIN_X"]; y0 = cfg["BOARD_ORIGIN_Y"]
    angle = cfg["BOARD_ANGLE_RAD"]; size = cfg["SQUARE_SIZE_MM"]
    cos_t = math.cos(angle); sin_t = math.sin(angle)
    
    x_final = x0 + (i * size * cos_t) - (j * size * sin_t)
    y_final = y0 + (i * size * sin_t) + (j * size * cos_t)
    
    is_ret = (j <= 3)
    
    if is_ret:
        base_ori = cfg["ORIENTATION_TABULEIRO_RET"] 
        
        rz_rad = math.atan2(x_final, y_final)
        final_rz = math.degrees(rz_rad)
    else:
        base_ori = cfg["ORIENTATION_TABULEIRO_EST"]
        
        rz_rad = math.atan2(x_final, y_final)
        rz_deg = math.degrees(rz_rad)
        final_rz = 180.0 - rz_deg 

    rx, ry = base_ori[0], base_ori[1]
    
    return (x_final, y_final, z_height, rx, ry, normalize_angle_deg(final_rz))

def get_pose_from_estojo(num_peca: int, z_height: float):
    """
    Calcula a pose exata usando a MESMA lógica do bridge_server.py.
    Usa a casa (7,0) como âncora e aplica o offset rotacionado.
    """
    cfg = get_current_config()
    
    x0 = cfg["BOARD_ORIGIN_X"]
    y0 = cfg["BOARD_ORIGIN_Y"]
    angle = cfg["BOARD_ANGLE_RAD"]
    size = cfg["SQUARE_SIZE_MM"]
    
    cos_t = math.cos(angle)
    sin_t = math.sin(angle)
    
    i_ref, j_ref = 7, 0    
    x_70 = x0 + (i_ref * size * cos_t) - (j_ref * size * sin_t)
    y_70 = y0 + (i_ref * size * sin_t) + (j_ref * size * cos_t)
    
    dx_local = cfg["ESTOJO_OFFSET_X"]
    dy_local = cfg["ESTOJO_OFFSET_Y"] + (num_peca * cfg["ESTOJO_SPACING_MM"])
    
    x_final = x_70 + (dx_local * cos_t) - (dy_local * sin_t)    
    y_final = y_70 + (dx_local * sin_t) + (dy_local * cos_t)
    
    ori = cfg["ORIENTATION_ESTOJO"]
    
    return (x_final, y_final, z_height, ori[0], ori[1], ori[2])

# --- TRANSIÇÃO ---
def handle_safe_transition_cmd(target_side: str) -> bool:
    global current_operation_side
    
    if current_operation_side == target_side: 
        return False
        
    print(f"↷ Transição {current_operation_side} -> {target_side}")
    
    cfg = get_current_config()
    
    if target_side == 'estojo': 
        target_side = cfg["LADO_ESTOJO"] # Pode ser 'ret' ou 'est' no config

    # O código não quer saber se é esquerda ou direita, só quer a chave certa
    if target_side == 'ret':
        juntas = cfg["STAGING_RET"]
        print(f"   -> Via Staging RETRAÍDO (J1={juntas[0]})")
    else: # est
        juntas = cfg["STAGING_EST"]
        print(f"   -> Via Staging ESTICADO (J1={juntas[0]})")

    if move_joints_cmd(juntas, speed=STAGING_SPEED):
        current_operation_side = target_side
        return True
        
    print("❌ Falha na transição!")
    return False

# --- MACROS ---

def macro_colocar_cmd(i: int, j: int):
    global current_operation_side
    cfg = get_current_config()
    
    target = get_target_side(j)
    handle_safe_transition_cmd(target)
    
    z_travel = cfg["Z_TRAVEL"] 
    z_op = cfg["Z_OPERATION"]
    ascend = cfg["Z_FLIP_ASCEND_DELTA"]
    descend = cfg["Z_FLIP_DESCEND_DELTA"]
    retract = cfg["Z_FLIP_RETRACT_DELTA"]

    print(f"🤖 Colocando peça em ({i}, {j})...")

    # 1. Aproximação
    pose = get_pose_from_grid(i, j, z_travel)
    if not move_pose_cmd(*pose, speed=MOVE_SPEED): return False
    
    # 2. Desce, Solta, Sobe
    if not move_relative_cmd(dz=-(z_travel - z_op)): return False
    if not set_gripper_cmd(True): return False; time.sleep(0.3)
    if not move_relative_cmd(dz=ascend): return False
    
    return True

def macro_trocar_cmd(i: int, j: int):
    global current_operation_side
    cfg = get_current_config()
    
    target = get_target_side(j)
    handle_safe_transition_cmd(target)
    
    z_travel = cfg["Z_TRAVEL"] 
    z_op = cfg["Z_OPERATION"]
    ascend = cfg["Z_FLIP_ASCEND_DELTA"]
    descend = cfg["Z_FLIP_DESCEND_DELTA"]
    retract = cfg["Z_FLIP_RETRACT_DELTA"]

    print(f"🤖 Trocando peça em ({i}, {j})...")

    # 1. Aproximação
    pose = get_pose_from_grid(i, j, z_travel)
    if not move_pose_cmd(*pose, speed=MOVE_SPEED): return False
    
    # 2. Desce, Pega, Sobe
    if not move_relative_cmd(dz=-(z_travel - z_op)): return False
    if not set_gripper_cmd(False): return False; time.sleep(0.3)
    if not move_relative_cmd(dz=ascend): return False
    
    # 3. Gira 180
    if not rotate_gripper_cmd(180): return False
    
    # 4. Desce, Solta, Sobe
    if not move_relative_cmd(dz=-descend): return False
    if not set_gripper_cmd(True): return False; time.sleep(0.3)
    if not move_relative_cmd(dz=retract): return False
    
    return True

def macro_pegar_estojo_cmd(num_peca: int):
    """Simula a ação de pegar uma peça específica no estojo."""
    global current_operation_side
    cfg = get_current_config()
    
    # Força transição. Como target é 'estojo', a função acima usará STAGING_ESQ
    handle_safe_transition_cmd('estojo') 
    
    z_op = cfg["Z_OPERATION_ESTOJO"]
    z_travel = cfg["Z_TRAVEL_ESTOJO"]
    
    print(f"📦 Indo pegar PEÇA N°{num_peca} no estojo...")

    # 1. Calcula pose exata da peça N
    pose = get_pose_from_estojo(num_peca, z_travel)
    
    # 2. Aproximação
    if not move_pose_cmd(*pose, speed=MOVE_SPEED): return False
    
    # 3. Abre Garra
    if not set_gripper_cmd(True): return False; time.sleep(0.2)
    
    # 4. Desce
    if not move_relative_cmd(dz=-(z_travel - z_op)): return False
    
    # 5. Fecha Garra
    if not set_gripper_cmd(False): return False; time.sleep(0.5)
    
    # 6. Sobe
    if not move_relative_cmd(dz=(z_travel - z_op)): return False
    
    current_operation_side = 'estojo'
    print(f"✅ Peça {num_peca} capturada (está na garra).")
    return True

# --- NOVA MACRO DE TESTE: DANCINHA ---
def macro_danca_vitoria_cmd():
    """Chama a macro de dancinha no bridge."""
    print("🕺 Enviando comando de Dancinha da Vitória...")
    r = http_post("/macro/vitoria")
    if r.get("ok"):
        print("✅ Dancinha executada com sucesso!")
    else:
        print(f"❌ Falha na dancinha: {r.get('error', 'Erro desconhecido')}")

# --- ROTINA DE VERIFICAÇÃO COMPLETA ---
def executar_rotina_verificacao():
    global RID
    
    print("\n=== CONFIGURAR ROTINA ===")
    rid_input = input("Qual robô usar (1 ou 2)? [1]: ").strip()
    RID = int(rid_input) if rid_input in ['1', '2'] else 1
    
    cfg = get_current_config()
    print(f"\n--- INICIANDO ROTINA COM ROBÔ {RID} ---")
    print(f"Origem: ({cfg['BOARD_ORIGIN_X']:.2f}, {cfg['BOARD_ORIGIN_Y']:.2f})")
    
    casas_para_testar = [
        (0, 0), (0, 7), (7, 0), (7, 7), 
        (3, 3), (4, 4), (3, 4), (4, 3)
    ]
    
    habilitar_servos()
    if not move_to_home():
        print("❌ Falha ao ir para Home. Abortando.")
        return

    for idx, (i, j) in enumerate(casas_para_testar):
        print(f"\n[{idx+1}/{len(casas_para_testar)}] >>> Verificando Casa ({i}, {j})...")
        sucesso = macro_trocar_cmd(i, j)
        if not sucesso:
            print(f"❌ Erro na casa ({i}, {j}). Parando rotina.")
            break
        print(f"✅ Casa ({i}, {j}) ok.")
        time.sleep(0.5)

    print("\n🏁 Rotina finalizada. Voltando para Home.")
    move_to_home()

# --- MENU ---
def main():
    global RID
    print(f"Console Verificação | Bridge: {BASE}")
    print(f"Usando Configuração Centralizada.")
    threading.Thread(target=monitor_status, daemon=True).start()
    
    while True:
        try:
            cmd_raw = input(f"\n(R{RID})> ").strip().lower()
            if not cmd_raw: continue
            cmd = cmd_raw.split()
            
            if cmd[0] == "verificar":
                executar_rotina_verificacao()
                
            elif cmd[0] == "usar" and len(cmd)>1:
                RID = int(cmd[1]); print(f"Set: Robô {RID}")
            
            elif cmd[0] == "habilitar": 
                habilitar_servos()
            
            elif cmd[0] == "home": move_to_home()
            elif cmd[0] == "garra": set_gripper_cmd(True)
            elif cmd[0] == "ungarra": set_gripper_cmd(False)
            
            # --- COMANDO ESTOJO ---
            elif cmd[0] == "pegar" and len(cmd)==2:
                try:
                    num = int(cmd[1])
                    macro_pegar_estojo_cmd(num)
                except ValueError:
                    print("Erro: Digite um número inteiro. Ex: pegar 0")
            
            elif cmd[0] == "trocar" and len(cmd)==3:
                macro_trocar_cmd(int(cmd[1]), int(cmd[2]))

            elif cmd[0] == "colocar" and len(cmd)==3:
                macro_colocar_cmd(int(cmd[1]), int(cmd[2]))

            # --- COMANDO DANCINHA ---
            elif cmd[0] == "dancar":
                macro_danca_vitoria_cmd()
            
            elif cmd[0] in ["sair", "exit"]: break
            else: print("Comandos: verificar, habilitar, usar 1|2, home, garra, ungarra, trocar i j, pegar N")
            
        except Exception as e: print(f"Erro: {e}")

if __name__ == "__main__": main()