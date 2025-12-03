import os
import time
import threading
from typing import Optional, Dict, List, Tuple
import traceback
import math
from fastapi import FastAPI, HTTPException, Depends, Query, Body
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, Field
import logging

# --- IMPORTAÇÕES DO CONFIG.PY ---
try:
    from config import (
        ROBOT1_IP, ROBOT2_IP, BRIDGE_TOKEN,
        SPEED_PTP, SPEED_LINEAR, ACCEL_PTP, ACCEL_LINEAR,
        STAGING_SPEED, ROBOT_CONFIGS
    )
except ImportError:
    raise ImportError("Arquivo config.py não encontrado na raiz do projeto!")

try:
    from pmr_elirobots_sdk import EC
except Exception:
    try:
        from eliterobot.ec import EC
    except ImportError:
        raise ImportError("Não foi possível importar 'EC' nem de 'pmr_elirobots_sdk' nem de 'eliterobot'")

# --- CONFIGURAÇÕES LOCAIS (Ainda não movidas para config.py) ---
DIGITAL_GARRA1 = os.getenv("DIGITAL_GARRA1", "Y0")
DIGITAL_GARRA2 = os.getenv("DIGITAL_GARRA2", "Y0")
HOME_POSITION_JOINTS = [90.0, -155.0, 100.0, -35.0, 90.0, 0.0, 0.0, 0.0]
HOME_SPEED = 80

# --- FILTRO DE LOGS ---
class EndpointFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        log_msg = record.getMessage()
        blocklist = ["/vis/update_state", "/dashboard/data", "/status"]
        return not any(x in log_msg for x in blocklist)

# === ESTADO GLOBAL DO BRIDGE ===
robot_states = { 1: {'side': 'home'}, 2: {'side': 'home'} }
g_ultima_jogada_humano: Optional[Dict] = None


# === LÓGICA DE GEOMETRIA (Movida de othello_robo_ponte) ===
def normalize_angle_deg(angle_deg: float) -> float:
    return (angle_deg + 180) % 360 - 180

def get_physical_coords(rid: int, i_logic: int, j_logic: int) -> Tuple[int, int]:
    if i_logic == -1 and j_logic == -1: return -1, -1
    if rid == 1: return i_logic, j_logic
    elif rid == 2: return 7 - i_logic, 7 - j_logic
    else: raise ValueError(f"RID inválido: {rid}")

def get_pose_from_grid(rid: int, i_phys: int, j_phys: int, z_height: float) -> Dict[str, float]:
    cfg = ROBOT_CONFIGS[rid]
    x0 = cfg["BOARD_ORIGIN_X"]
    y0 = cfg["BOARD_ORIGIN_Y"]
    angle = cfg["BOARD_ANGLE_RAD"]
    size = cfg["SQUARE_SIZE_MM"]
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)    
    x_final_robo = x0 + (i_phys * size * cos_theta) - (j_phys * size * sin_theta)    
    y_final_robo = y0 + (i_phys * size * sin_theta) + (j_phys * size * cos_theta)
    if j_phys <= 3: base_orientation = cfg["ORIENTATION_TABULEIRO_ESQ"]
    else: base_orientation = cfg["ORIENTATION_TABULEIRO_DIR"]
    tool_rx, tool_ry = base_orientation[0], base_orientation[1]
    rz_rad = math.atan2(x_final_robo, y_final_robo); rz_deg = math.degrees(rz_rad)
    if j_phys <= 3: final_rz_deg = rz_deg
    else: final_rz_deg = 180.0 - rz_deg
    rz_normalizado = normalize_angle_deg(final_rz_deg)
    return {"x": x_final_robo, "y": y_final_robo, "z": z_height, "rx": tool_rx, "ry": tool_ry, "rz": rz_normalizado}

def get_pose_from_estojo(rid: int, z_height: float, num_pecas_ja_usadas: int) -> Dict[str, float]:
    cfg = ROBOT_CONFIGS[rid]    
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
    dy_local = cfg["ESTOJO_OFFSET_Y"] + (num_pecas_ja_usadas * cfg["ESTOJO_SPACING_MM"])
    x_final = x_70 + (dx_local * cos_t) - (dy_local * sin_t)    
    y_final = y_70 + (dx_local * sin_t) + (dy_local * cos_t)
    ori = cfg["ORIENTATION_ESTOJO"]
    return {"x": x_final,"y": y_final,"z": z_height,"rx": ori[0],"ry": ori[1],"rz": ori[2]}

def get_target_side(rid: int, i_phys: int, j_phys: int) -> str:
    cfg = ROBOT_CONFIGS[rid]
    if i_phys == -1 and j_phys == -1: return 'estojo'
    if rid == 1: return 'esq' if j_phys <= 3 else 'dir'
    elif rid == 2: return 'dir' if j_phys <= 3 else 'esq'
    raise ValueError(f"RID inválido para get_target_side: {rid}")


# === LÓGICA DE CONEXÃO E LOCKS (Original do bridge.py) ===

bearer = HTTPBearer(auto_error=False)
def require_token(cred: Optional[HTTPAuthorizationCredentials] = Depends(bearer)):
    if not BRIDGE_TOKEN: return
    if not cred or cred.scheme.lower() != "bearer" or cred.credentials != BRIDGE_TOKEN:
        raise HTTPException(status_code=401, detail="Invalid or missing token")

class RobotSlot:
    def __init__(self, ip: str, io_garra: str):
        self.ip = ip
        self.io_garra = io_garra
        self.ec: Optional[EC] = None
        self.connect_lock = threading.Lock()
        self.command_lock = threading.Lock()
robots: Dict[int, RobotSlot] = { 1: RobotSlot(ROBOT1_IP, DIGITAL_GARRA1), 2: RobotSlot(ROBOT2_IP, DIGITAL_GARRA2), }

def connect_robot(rid: int) -> bool:
    slot = robots.get(rid)
    if not slot: return False
    with slot.connect_lock:
        if slot.ec and slot.ec.alive: return True
        if slot.ec:
            try: slot.ec.disconnect_ETController()
            except Exception: pass
            finally: slot.ec = None
        try:
            print(f"[BRIDGE] Tentando conectar ao Robô {rid} em {slot.ip}...")
            ec_instance = EC(slot.ip, enable_log=False)
            ok, info = ec_instance.connect_ETController(slot.ip)
            if ok: slot.ec = ec_instance; print(f"[BRIDGE] Robô {rid} conectado."); return True
            else: print(f"[BRIDGE][WARN] Conexão falhou: {info}"); slot.ec = None; return False
        except BaseException as e: print(f"[BRIDGE][ERR] Exceção connect_robot RID {rid}:"); traceback.print_exc(); slot.ec = None; return False

def ensure_robot(rid: int) -> EC:
    slot = robots.get(rid)
    if not slot: raise HTTPException(status_code=404, detail=f"Robô {rid} não config.")
    if slot.ec and slot.ec.alive: 
        return slot.ec
    if connect_robot(rid) and slot.ec: 
        return slot.ec
    raise HTTPException(status_code=503, detail=f"Robô {rid} indisponível ({slot.ip})")

def spawn_reconnector(rid: int, interval: float = 5.0):
    def loop():
        while True:
            time.sleep(interval)
            slot = robots.get(rid); should_reconnect = False
            if slot:
                with slot.command_lock:
                    try: 
                        if not slot.ec or not slot.ec.alive: should_reconnect = True
                    except: should_reconnect = True
            if should_reconnect: connect_robot(rid)
    t = threading.Thread(target=loop, daemon=True, name=f"Reconnector_R{rid}"); t.start()

def check_other_robot_is_stopped(rid_to_move: int):
    """
    Verifica se o *outro* robô está parado (RobotState.STOP).
    Levanta HTTPException 423 (Locked) se o outro robô estiver ativo.
    """
    other_rid = 2 if rid_to_move == 1 else 1
    other_slot = robots.get(other_rid)
    
    if other_slot and other_slot.ec and other_slot.ec.alive:
        other_state = None
        try:
            with other_slot.command_lock:
                other_state = other_slot.ec.state
        except Exception:
             print(f"[BRIDGE][WARN] Falha ao ler estado do Robô {other_rid} durante cross-check.")
             return
        
        if other_state is not None and other_state != EC.RobotState.STOP:
            print(f"[BRIDGE][BLOCK] Robô {rid_to_move} bloqueado. Robô {other_rid} está ativo (Estado: {other_state.name}).")
            raise HTTPException(
                status_code=423, 
                detail=f"Operação bloqueada: Robô {other_rid} está em movimento ou não está em estado STOP (Estado: {other_state.name})."
            )
    
    print(f"[BRIDGE][OK] Cross-check para R{rid_to_move} liberado (R{other_rid} está parado ou offline).")


# === FUNÇÕES HELPER DE MOVIMENTO SÍNCRONO (Internas do Bridge) ===

def _exec_move_joint_sync(ec: EC, joints: list, speed: int, accel: int) -> bool:
    """Função helper para executar move_joint e checar o resultado."""
    cmd = joints + [0] * (8 - len(joints))
    resp = ec.move_joint(target_joint=cmd, speed=speed, acc=accel, dec=accel, block=True)
    if not resp.success:
        print(f"[BRIDGE][MACRO_ERR] Falha no move_joint: {resp.result}")
    return resp.success

def _exec_move_line_sync(ec: EC, pose: dict, speed: int, accel: int) -> bool:
    """Função helper para executar move_line (calculando IK antes)."""
    j_resp = ec.get_joint()
    if not j_resp.success: 
        print(f"[BRIDGE][MACRO_ERR] Falha get_joint (mov_lin): {j_resp.result}")
        return False
    
    target_p = [pose['x'], pose['y'], pose['z'], pose['rx'], pose['ry'], pose['rz']]
    ik_resp = ec.get_inverse_kinematic(pose=target_p, ref_joint=j_resp.result, unit_type=0)
    
    if not ik_resp.success:
        print(f"[BRIDGE][MACRO_ERR] Falha IK (mov_lin): {ik_resp.result}")
        return False
        
    cmd = ik_resp.result + [0] * (8 - len(ik_resp.result))
    resp = ec.move_line(target_joint=cmd, speed=speed, acc=accel, dec=accel, block=True)
    
    if not resp.success:
        print(f"[BRIDGE][MACRO_ERR] Falha no move_line: {resp.result}")
    return resp.success

def _exec_move_ptp_sync(ec: EC, pose: dict, speed: int, accel: int) -> bool:
    """Função helper para executar move_joint (PTP) para uma pose (calculando IK antes)."""
    j_resp = ec.get_joint()
    if not j_resp.success: 
        print(f"[BRIDGE][MACRO_ERR] Falha get_joint (mov_ptp): {j_resp.result}")
        return False

    target_p = [pose['x'], pose['y'], pose['z'], pose['rx'], pose['ry'], pose['rz']]
    ik_resp = ec.get_inverse_kinematic(pose=target_p, ref_joint=j_resp.result, unit_type=0)
    
    if not ik_resp.success:
        print(f"[BRIDGE][MACRO_ERR] Falha IK (mov_ptp): {ik_resp.result}")
        return False
        
    return _exec_move_joint_sync(ec, ik_resp.result, speed, accel)

def _exec_move_relative_sync(ec: EC, deltas: list, speed: int, accel: int) -> bool:
    """Função helper para executar um move_joint relativo."""
    pose_resp = ec.get_tcp_pose(unit_type=0)
    if not pose_resp.success:
        print(f"[BRIDGE][MACRO_ERR] Falha ao ler pose para mov_rel: {pose_resp.result}")
        return False
                
    joint_resp = ec.get_joint()
    if not joint_resp.success:
        print(f"[BRIDGE][MACRO_ERR] Falha ao ler juntas para mov_rel: {joint_resp.result}")
        return False
    
    current_pose = pose_resp.result
    ref_joint = joint_resp.result
    target_pose = [current + delta for current, delta in zip(current_pose, deltas)]
    
    ik_resp = ec.get_inverse_kinematic(pose=target_pose, ref_joint=ref_joint, unit_type=0)
    if not ik_resp.success:
        print(f"[BRIDGE][MACRO_ERR] Falha no IK do mov_rel: {ik_resp.result}")
        return False
                
    return _exec_move_joint_sync(ec, ik_resp.result, speed, accel)

def _handle_safe_transition(ec: EC, rid: int, target_side: str) -> bool:
    global robot_states
    current_side = robot_states[rid]['side']
    cfg = ROBOT_CONFIGS[rid]

    # Se veio do Home, assume que é seguro
    if current_side == 'home': 
        robot_states[rid]['side'] = target_side
        return False 

    # Otimização: Se já está no lado certo E não envolve o estojo, pode pular.
    # (Sempre forçamos staging se envolver estojo para garantir altura)
    if current_side == target_side and current_side != 'estojo' and target_side != 'estojo':
        return False
    
    print(f"↷ [R{rid}][Transição] {current_side} → {target_side}")
    
    # --- LÓGICA DE SELEÇÃO ---
    if target_side == 'dir':
        # Vai para Direita
        juntas_alvo = list(cfg['STAGING_DIR'])
        print(f"[R{rid}] Usando Staging DIREITA")
        
    elif target_side == 'esq' or target_side == 'estojo':
        # MUDANÇA AQUI: Se for Esquerda OU Estojo -> Usa Staging ESQ (Retraído)
        juntas_alvo = list(cfg['STAGING_ESQ'])
        print(f"[R{rid}] Usando Staging ESQUERDA (Para Alvo: {target_side})")
    
    else:
        # Fallback (não deve acontecer)
        juntas_alvo = list(cfg['STAGING_ESQ'])
    
    _exec_move_joint_sync(ec, juntas_alvo, STAGING_SPEED, ACCEL_PTP)
    robot_states[rid]['side'] = target_side
    return True


# === LÓGICA DAS MACROS (Internas do Bridge) ===

def _macro_pegar_no_estojo_sync(ec: EC, rid: int, num_peca: int, speed: int, accel: int):
    cfg = ROBOT_CONFIGS[rid] 
    z_travel_estojo = cfg["Z_TRAVEL_ESTOJO"]
    z_op_estojo = cfg["Z_OPERATION_ESTOJO"]
    io_garra = robots[rid].io_garra
    print(f"[BRIDGE][MACRO] R{rid} Exec: Pegar Peça (N°{num_peca})")
    ec.set_digital_io(io_garra, 0); time.sleep(0.1)
    if not _exec_move_relative_sync(ec, [0,0, -(z_travel_estojo-z_op_estojo), 0,0,0], speed, accel): return
    ec.set_digital_io(io_garra, 1); time.sleep(0.1)
    if not _exec_move_relative_sync(ec, [0,0, (z_travel_estojo-z_op_estojo), 0,0,0], speed, accel): return
    print(f"[BRIDGE][MACRO] R{rid} Fim: Pegar Peça")

def _macro_colocar_na_casa_sync(ec: EC, rid: int, speed: int, accel: int):
    cfg = ROBOT_CONFIGS[rid]
    z_op = cfg["Z_OPERATION"]
    z_travel = cfg["Z_TRAVEL"]
    io_garra = robots[rid].io_garra
    print(f"[BRIDGE][MACRO] R{rid} Exec: Colocar Peça")
    if not _exec_move_relative_sync(ec, [0,0, -(z_travel - z_op), 0,0,0], speed, accel): return
    ec.set_digital_io(io_garra, 0); time.sleep(0.1)
    if not _exec_move_relative_sync(ec, [0,0, (z_travel - z_op), 0,0,0], speed, accel): return
    print(f"[BRIDGE][MACRO] R{rid} Fim: Colocar Peça")

def _macro_virar_na_casa_sync(ec: EC, rid: int, speed: int, accel: int):
    cfg = ROBOT_CONFIGS[rid]
    z_op = cfg["Z_OPERATION"]
    z_travel = cfg["Z_TRAVEL"]
    io_garra = robots[rid].io_garra
    Z_FLIP_ASCEND_DELTA = cfg["Z_FLIP_ASCEND_DELTA"]
    Z_FLIP_DESCEND_DELTA = cfg["Z_FLIP_DESCEND_DELTA"]
    Z_FLIP_RETRACT_DELTA = cfg["Z_FLIP_RETRACT_DELTA"]
    print(f"[BRIDGE][MACRO] R{rid} Exec: Virar Peça")
    if not _exec_move_relative_sync(ec, [0,0, -(z_travel - z_op), 0,0,0], speed, accel): return
    ec.set_digital_io(io_garra, 1); time.sleep(0.1)
    if not _exec_move_relative_sync(ec, [0,0, Z_FLIP_ASCEND_DELTA, 0,0,0], speed, accel): return
    
    # Girar Garra
    j_resp = ec.get_joint()
    if not j_resp.success: print("[BRIDGE][MACRO_ERR] Falha get_joint (girar)"); return
    target_j = list(j_resp.result[:6])
    target_j[5] = (target_j[5] + 180 + 180) % 360 - 180 # Gira 180
    if not _exec_move_joint_sync(ec, target_j, speed, accel): return

    if not _exec_move_relative_sync(ec, [0,0, -Z_FLIP_DESCEND_DELTA, 0,0,0], speed, accel): return
    ec.set_digital_io(io_garra, 0); time.sleep(0.1)
    if not _exec_move_relative_sync(ec, [0,0, Z_FLIP_RETRACT_DELTA, 0,0,0], speed, accel): return
    print(f"[BRIDGE][MACRO] R{rid} Fim: Virar Peça")

# === MODELOS PYDANTIC (Originais + Novos) ===
class Joints(BaseModel): 
    j1: float; j2: float; j3: float; j4: float; j5: float; j6: float
    speed: int = Field(20, ge=1, le=100)
    accel: int = Field(80, ge=1, le=100)

class Pose(BaseModel): 
    x: float; y: float; z: float; rx: float; ry: float; rz: float
    speed: int = Field(20, ge=1, le=100)
    accel: int = Field(80, ge=1, le=100)

class LinePose(BaseModel): 
    x: float; y: float; z: float; rx: float; ry: float; rz: float
    speed: int = Field(100, ge=1, le=3000)
    accel: int = Field(80, ge=1, le=100)

class RelativeMove(BaseModel): 
    dx: float = 0.0; dy: float = 0.0; dz: float = 0.0
    drx: float = 0.0; dry: float = 0.0; drz: float = 0.0
    speed: int = Field(20, ge=1, le=100)
    accel: int = Field(80, ge=1, le=100)

class RotateGripper(BaseModel): 
    angle: float
    speed: int = Field(20, ge=1, le=100)
    accel: int = Field(80, ge=1, le=100)

class JogadaHumano(BaseModel):
    jogada: List[int] = Field(..., min_items=2, max_items=2)

class MacroPtpPayload(BaseModel):
    speed: int = Field(SPEED_PTP, ge=1, le=100)
    accel: int = Field(ACCEL_PTP, ge=1, le=100)

class MacroPegaPayload(BaseModel):
    num_peca: int = Field(0, ge=0)
    speed_ptp: int = Field(SPEED_PTP, ge=1, le=100)
    accel_ptp: int = Field(ACCEL_PTP, ge=1, le=100)

class LanceCompletoPayload(BaseModel):
    jogada: Tuple[int, int]
    capturas_por_direcao: List[List[Tuple[int, int]]]
    num_peca_atual: int
    speed_ptp: int = Field(SPEED_PTP, ge=1, le=100)
    speed_linear: int = Field(SPEED_LINEAR, ge=1, le=3000)
    accel_ptp: int = Field(ACCEL_PTP, ge=1, le=100)
    accel_linear: int = Field(ACCEL_LINEAR, ge=1, le=100)


# === INICIALIZAÇÃO DO APP ===
app = FastAPI(title="EC Bridge (2 robôs)", version="2.0.0")
@app.on_event("startup")
def _startup(): 
    logging.getLogger("uvicorn.access").addFilter(EndpointFilter())
    print("[BRIDGE] Iniciando..."); 
    connect_robot(1); 
    connect_robot(2); 
    spawn_reconnector(1); 
    spawn_reconnector(2); 
    print("[BRIDGE] Startup completo.")


# === ENDPOINTS DE STATUS E CONTROLE ===

@app.get("/status", dependencies=[Depends(require_token)])
def status(rid: int = Query(..., ge=1, le=2)):
    slot = robots.get(rid); 
    if not slot: raise HTTPException(404, f"Robô {rid} não config.")
    ec = None
    try: ec = ensure_robot(rid)
    except HTTPException as e: return {"ok": False, "connected": False, "ip": slot.ip, "error": f"Robô {rid} offline"}
    try:
        with slot.command_lock:
            if not ec.alive: raise HTTPException(503, "Robô desconectou")
            state_resp = ec.state; mode_resp = ec.mode; joints_resp = ec.get_joint()
            pose_resp = ec.get_tcp_pose(unit_type=0); servo_resp = ec.servo_status
            gripper_resp = ec.get_digital_io(slot.io_garra); is_collision_resp = ec.is_collision
            drag_enabled = None; 
            try: drag_enabled = ec.drag_status;
            except AttributeError: pass
        state_name = getattr(state_resp, 'name', str(state_resp)); mode_name = getattr(mode_resp, 'name', str(mode_resp))
        joints_val = joints_resp.result if joints_resp.success else None; pose_val = pose_resp.result if pose_resp.success else None
        servo_on = servo_resp.result if servo_resp.success else False; 
        gripper_open = gripper_resp.result == 0 if gripper_resp.success else None
        collision_state = is_collision_resp == 1 if isinstance(is_collision_resp, int) else None 
        return {"ok": True, "connected": True, "ip": slot.ip, "state": state_name, "mode": mode_name, "joints": joints_val, "pose": pose_val, "servo_on": servo_on, "gripper_open": gripper_open, "hand_drag_enabled": drag_enabled, "in_collision": collision_state}
    except HTTPException: raise
    except Exception as e: print(f"[BRIDGE][ERR] Status Robô {rid}:"); traceback.print_exc(); slot.ec = None; raise HTTPException(500, f"Exceção status Robô {rid}: {e}")

@app.post("/connect", dependencies=[Depends(require_token)])
def connect_now(rid: int = Query(..., ge=1, le=2)):
    slot = robots.get(rid); was_connected = False
    if slot and slot.ec and slot.ec.alive: was_connected = True
    success = connect_robot(rid); return {"ok": success, "previously_connected": was_connected}

@app.post("/disconnect", dependencies=[Depends(require_token)])
def disconnect_now(rid: int = Query(..., ge=1, le=2)):
    slot = robots.get(rid); disconnected = False
    if slot and slot.ec:
        with slot.connect_lock:
             try:
                 if slot.ec.alive: slot.ec.disconnect_ETController(); disconnected = True
             except Exception as e: print(f"[BRIDGE][WARN] Erro desconectar Robô {rid}: {e}")
             finally: slot.ec = None
    message = f"Robô {rid} desconectado." if disconnected else f"Robô {rid} já desconectado."
    return {"ok": True, "message": message}


@app.post("/habilitar", dependencies=[Depends(require_token)])
def habilitar_servos_endpoint(rid: int = Query(..., ge=1, le=2)):
    try:
        ec = ensure_robot(rid)
    except HTTPException:
        if connect_robot(rid):
             ec = robots[rid].ec
        else:
             raise HTTPException(503, "Robô não conecta.")

    slot = robots[rid]
    print(f"[BRIDGE] Habilitando servos Robô {rid}")
    
    with slot.command_lock:
        try:
            success = False
            try:
                success = ec.robot_servo_on(max_retries=5)
            except SystemExit:
                print(f"[BRIDGE][CRITICAL] Socket do Robô {rid} morreu (SystemExit). Tentando reconectar...")
                slot.ec = None
                if connect_robot(rid):
                    print(f"[BRIDGE] Reconectado. Tentando habilitar novamente...")
                    success = robots[rid].ec.robot_servo_on(max_retries=5)
                else:
                    raise HTTPException(503, "Conexão perdida e falha ao reconectar.")
            except Exception as e:
                 print(f"[BRIDGE][WARN] Erro ao habilitar: {e}")
                 slot.ec = None 
                 raise HTTPException(503, "Erro de comunicação. Tente novamente.")

            if success: 
                print(f"[BRIDGE] Servos Robô {rid} habilitados.")
                time.sleep(0.5) 
                return {"ok": True, "result": "Servos habilitados."}
            else:
                 return {"ok": False, "result": "Falha (SDK retornou False)."}
                 
        except Exception as e:
            print(f"[BRIDGE][ERR] Exceção fatal /habilitar Robô {rid}: {e}")
            slot.ec = None 
            raise HTTPException(500, f"Exceção: {e}")
        
@app.post("/servo", dependencies=[Depends(require_token)])
def set_servo_endpoint(on: bool, rid: int = Query(..., ge=1, le=2)):
    if on: raise HTTPException(status_code=400, detail="Use o endpoint /habilitar para ligar os servos.")
    ec = ensure_robot(rid); slot = robots[rid]; action_str = "desligar"
    print(f"[BRIDGE] {action_str.capitalize()} servos Robô {rid}")
    with slot.command_lock:
        try:
            resp = ec.set_servo_status(0) 
            if resp.success:
                 print(f"[BRIDGE] Comando {action_str} enviado. Aguardando..."); time.sleep(0.5)
                 current_servo_status = ec.servo_status.result
                 if not current_servo_status: print(f"[BRIDGE] Servo Robô {rid} confirmado desligado."); return {"ok": True, "result": "Servo desligado."}
                 else: print(f"[BRIDGE][WARN] Servo Robô {rid} não confirmou desligado."); return {"ok": False, "result": "Falha confirmação servo desligado."}
            else: print(f"[BRIDGE][ERR] Falha comando {action_str}: {resp.result}"); return {"ok": False, "result": f"Falha comando: {resp.result}"}
        except Exception as e: print(f"[BRIDGE][ERR] Exceção /servo Robô {rid}:"); traceback.print_exc(); slot.ec = None; raise HTTPException(500, f"Exceção {action_str} servo Robô {rid}: {e}")

@app.post("/drag", dependencies=[Depends(require_token)])
def toggle_drag_endpoint(enable: bool, rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]; action_str = "habilitar" if enable else "desabilitar"
    print(f"[BRIDGE] {action_str.capitalize()} modo drag Robô {rid}")
    with slot.command_lock:
        try:
            resp = ec.switch_drag_teach(1 if enable else 0) 
            print(f"[BRIDGE] Comando drag Robô {rid} enviado. SDK: {resp}"); return {"ok": resp.success, "result": resp.result}
        except AttributeError: raise HTTPException(500, "Método 'switch_drag_teach' não disponível no SDK.")
        except Exception as e: print(f"[BRIDGE][ERR] Exceção /drag Robô {rid}:"); traceback.print_exc(); slot.ec = None; raise HTTPException(500, f"Exceção {action_str} drag Robô {rid}: {e}")

@app.post("/recuperar", dependencies=[Depends(require_token)])
def recuperar_endpoint(rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]
    print(f"[BRIDGE] Recuperando (limpar alarme/colisão) Robô {rid}")
    with slot.command_lock:
        try:
            resp_col = ec.clear_collision_alarm(); print(f"[BRIDGE] clear_collision_alarm: {resp_col}"); time.sleep(0.2)
            resp_alm = ec.clear_alarm(); print(f"[BRIDGE] clear_alarm: {resp_alm}"); success = resp_col.success or resp_alm.success
            return {"ok": success, "result": "Comandos limpar alarme/colisão enviados."}
        except Exception as e: print(f"[BRIDGE][ERR] Exceção /recuperar Robô {rid}:"); traceback.print_exc(); slot.ec = None; raise HTTPException(500, f"Exceção recuperar Robô {rid}: {e}")

@app.post("/stop", dependencies=[Depends(require_token)])
def stop_endpoint(rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]
    print(f"[BRIDGE] STOP Robô {rid}")
    with slot.command_lock:
        try:
            resp = ec.stop(); print(f"[BRIDGE] Comando STOP Robô {rid} enviado. SDK: {resp}"); return {"ok": resp.success, "result": resp.result}
        except Exception as e: print(f"[BRIDGE][ERR] Exceção /stop Robô {rid}:"); traceback.print_exc(); slot.ec = None; raise HTTPException(500, f"Exceção stop Robô {rid}: {e}")

@app.post("/gripper", dependencies=[Depends(require_token)])
def gripper(open: bool, rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]; io = slot.io_garra
    val = 0 if open else 1
    action_str = "abrindo" if open else "fechando"
    print(f"[BRIDGE] {action_str.capitalize()} garra Robô {rid} (IO:{io}, Valor Enviado:{val})")
    with slot.command_lock:
        try:
            resp = ec.set_digital_io(io, val)
            print(f"[BRIDGE] Comando garra Robô {rid} enviado. SDK: {resp}"); time.sleep(0.1) 
            return {"ok": resp.success, "result": resp.result}
        except Exception as e: print(f"[BRIDGE][ERR] Exceção /gripper Robô {rid}:"); traceback.print_exc(); slot.ec = None; raise HTTPException(500, f"Exceção garra Robô {rid}: {e}")


# --- ENDPOINTS DE JOGADA HUMANA (Inalterados) ---
g_tabuleiro_fisico = [[0]*8 for _ in range(8)]
g_camera_conectada = False
g_last_camera_update = 0

# --- ESTADO DO JOGO E DASHBOARD ---
g_tabuleiro_logico = [[0]*8 for _ in range(8)]
g_tabuleiro_logico[3][3], g_tabuleiro_logico[4][4] = -1, -1
g_tabuleiro_logico[3][4], g_tabuleiro_logico[4][3] = 1, 1

g_placar = {"Pretas": 2, "Brancas": 2}

g_jogador_atual = "Aguardando..." 
g_acao_atual = "Ocioso"          
g_ultima_jogada_coord = None      
g_instrucoes_humano = []         
g_lista_passos = []

class BoardStatePayload(BaseModel):
    matrix: List[List[int]]  

@app.post("/vis/update_state")
def update_board_state(body: BoardStatePayload):
    global g_tabuleiro_fisico, g_camera_conectada, g_last_camera_update
    g_tabuleiro_fisico = body.matrix
    g_camera_conectada = True
    g_last_camera_update = time.time()
    return {"ok": True}

@app.post("/vis/validar_estado")
def validar_estado_fisico(esperado: List[List[int]] = Body(...)):
    global g_tabuleiro_logico, g_placar, g_instrucoes_humano, g_acao_atual
    
    g_tabuleiro_logico = esperado
    
    # Atualiza Placar
    flat = [item for sublist in esperado for item in sublist]
    g_placar = {"Pretas": flat.count(1), "Brancas": flat.count(-1)}
    
    # Limpa instruções anteriores
    g_instrucoes_humano = []
    erros = []

    for r in range(8):
        for c in range(8):
            visto = g_tabuleiro_fisico[r][c]
            logico = esperado[r][c]

            if logico == 0 and visto != 0:
                msg = f"Remover peça/mão em ({r},{c})"
                erros.append(msg); g_instrucoes_humano.append(msg)
            elif logico != 0 and visto == 0:
                msg = f"Colocar peça em ({r},{c})"
                erros.append(msg); g_instrucoes_humano.append(msg)
            elif logico != 0 and visto != 0 and logico != visto:
                cor_certa = "PRETA" if logico == 1 else "BRANCA"
                msg = f"Virar peça em ({r},{c}) para {cor_certa}"
                erros.append(msg); g_instrucoes_humano.append(msg)

    if erros:
        g_acao_atual = "Aguardando Correção Humana"
        # Retorna 409 como antes, mas agora o dashboard já sabe o que escrever
        raise HTTPException(status_code=409, detail={"erros": erros})

    g_acao_atual = "Tabuleiro Validado"
    g_instrucoes_humano = [] 
    return {"ok": True}

@app.post("/vis/notificar_jogada_humano", dependencies=[Depends(require_token)])
def notificar_jogada_humano(body: JogadaHumano):
    global g_ultima_jogada_humano
    jogada_tuple = tuple(body.jogada)
    print(f"[BRIDGE][VIS] Jogada humana detectada e armazenada: {jogada_tuple}")
    g_ultima_jogada_humano = {"jogada": [jogada_tuple[0], jogada_tuple[1]]}
    return {"ok": True, "jogada_armazenada": jogada_tuple}

g_tabuleiro_snapshot = [[0]*8 for _ in range(8)] # O tabuleiro ANTES do humano jogar

@app.post("/vis/preparar_jogada_humano")
def preparar_jogada_humano(tabuleiro_base: List[List[int]] = Body(...)):
    """
    Define o estado BASE (Lógico) para comparação.
    Agora recebemos a verdade absoluta do Python, não confiamos na câmera para o snapshot.
    """
    global g_tabuleiro_snapshot, g_ultima_jogada_humano
    
    # O Snapshot agora é o tabuleiro lógico limpo (antes da jogada)
    g_tabuleiro_snapshot = tabuleiro_base
    g_ultima_jogada_humano = None
    
    print("[BRIDGE][VIS] Snapshot Lógico definido. Aguardando nova peça em casas vazias...")
    return {"ok": True}

@app.get("/vis/get_jogada_humano")
def get_jogada_humano():
    """
    Compara o AGORA com o SNAPSHOT.
    Procura a casa que era 0 e virou 1 ou -1.
    """
    global g_tabuleiro_fisico, g_tabuleiro_snapshot, g_ultima_jogada_humano    
    if g_ultima_jogada_humano:
        return {"ok": True, "status": "JOGADA_PRONTA", "data": g_ultima_jogada_humano}

    candidatos = []
    for r in range(8):
        for c in range(8):
            antes = g_tabuleiro_snapshot[r][c]
            agora = g_tabuleiro_fisico[r][c]

            if antes == 0 and agora != 0:
                candidatos.append((r, c))
    
    if len(candidatos) == 0:
        return {"ok": True, "status": "ESPERANDO_MOVIMENTO", "data": None}
    
    elif len(candidatos) == 1:
        jogada = candidatos[0]
        g_ultima_jogada_humano = {"jogada": [jogada[0], jogada[1]]}
        print(f"[BRIDGE][VIS] Jogada detectada em {jogada}")
        return {"ok": True, "status": "JOGADA_PRONTA", "data": g_ultima_jogada_humano}
    
    else:
        return {"ok": True, "status": "ERRO_MULTIPLAS_PECAS", "candidatos": candidatos}

@app.post("/vis/limpar_jogada_humano", dependencies=[Depends(require_token)])
def limpar_jogada_humano():
    global g_ultima_jogada_humano
    print("[BRIDGE][VIS] Juiz (Colab) solicitou limpeza. Esperando nova jogada humana...")
    g_ultima_jogada_humano = None
    return {"ok": True, "message": "Jogada limpa, pronto para nova detecção."}

class GameStatusPayload(BaseModel):
    jogador: str # "Robô 1", "Robô 2", "Humano"
    acao: str    # "Jogando", "Aguardando", etc
    jogada: Optional[List[int]] = None # [r, c] se houver
    passos: Optional[List[str]] = None

@app.post("/game/set_status")
def set_game_status(body: GameStatusPayload):
    global g_jogador_atual, g_acao_atual, g_ultima_jogada_coord, g_lista_passos
    g_jogador_atual = body.jogador
    g_acao_atual = body.acao
    
    if body.jogada is not None:
        g_ultima_jogada_coord = tuple(body.jogada)
    
    if body.passos is not None:
        g_lista_passos = body.passos
        
    return {"ok": True}

# == ENDPOINTS INTERFACE ==
@app.get("/dashboard/data")
def get_dashboard_data():
    global g_tabuleiro_fisico, g_tabuleiro_logico, g_placar, g_status_jogo, g_ultima_jogada_humano
    
    diffs = []
    for r in range(8):
        for c in range(8):
            if g_tabuleiro_fisico[r][c] != g_tabuleiro_logico[r][c]:
                diffs.append((r, c))

    return {
        "fisico": g_tabuleiro_fisico,
        "logico": g_tabuleiro_logico,
        "placar": g_placar,        
        "jogador": g_jogador_atual,
        "acao": g_acao_atual,
        "destaque_verde": g_ultima_jogada_coord,
        "instrucoes": g_instrucoes_humano,
        "passos_robo": g_lista_passos,
        "diffs": diffs
    }

from fastapi.responses import HTMLResponse

@app.get("/dashboard", response_class=HTMLResponse)
def dashboard_ui():
    return """
<!DOCTYPE html>
<html>
<head>
    <title>Othello Command Center</title>
    <style>
        body { font-family: 'Segoe UI', sans-serif; background: #121212; color: #eee; margin: 0; padding: 0; overflow: hidden; height: 100vh; display: flex; flex-direction: column; }
        
        /* --- HEADER --- */
        .header-panel { 
            background: #1f1f1f; padding: 15px 30px; 
            display: flex; justify-content: space-between; align-items: center; 
            border-bottom: 1px solid #333; box-shadow: 0 2px 10px rgba(0,0,0,0.5);
            z-index: 10;
        }
        .app-title { font-size: 1.5em; color: #fff; font-weight: bold; letter-spacing: 1px; }
        .status-big { font-size: 1.6em; font-weight: bold; color: #4dabf7; }
        .status-sub { font-size: 1.0em; color: #888; margin-left: 10px; }
        .score { font-size: 1.4em; font-weight: bold; background: #333; padding: 8px 20px; border-radius: 5px; border: 1px solid #555; }

        /* --- LAYOUT PRINCIPAL (GRID) --- */
        .main-layout {
            display: grid;
            grid-template-columns: 280px 1fr; /* Barra Lateral | Conteúdo Principal */
            height: 100%;
        }

        /* --- SIDEBAR (Passos do Robô) --- */
        .sidebar {
            background: #181818;
            border-right: 1px solid #333;
            padding: 20px;
            overflow-y: auto;
        }
        .sidebar h3 { color: #aaa; border-bottom: 1px solid #444; padding-bottom: 10px; margin-top: 0; }
        .step-list { list-style: none; padding: 0; }
        .step-item { 
            padding: 10px; margin-bottom: 8px; 
            background: #252525; border-radius: 4px; border-left: 4px solid #444;
            font-size: 0.95em; color: #ccc;
            transition: all 0.3s;
        }
        .step-item.active { border-left-color: #4dabf7; background: #2a2a35; color: #fff; }
        .step-icon { margin-right: 8px; }

        /* --- CENTRO (Tabuleiros) --- */
        .center-stage {
            display: flex; flex-direction: column; align-items: center; justify-content: flex-start;
            padding-top: 30px; position: relative;
            background: radial-gradient(circle at center, #222 0%, #121212 100%);
        }

        /* Alerta Humano */
        #instruction-box { 
            display: none; background: #3a0b0b; color: #ffcccc; 
            padding: 15px 30px; margin-bottom: 30px; border-radius: 50px; 
            border: 2px solid #ff4444; font-size: 1.3em;
            box-shadow: 0 0 20px rgba(255, 0, 0, 0.2);
            animation: slideDown 0.3s ease;
        }
        @keyframes slideDown { from {transform: translateY(-20px); opacity:0;} to {transform: translateY(0); opacity:1;} }

        /* Tabuleiro Físico */
        .board { display: grid; grid-gap: 3px; background: #222; border: 8px solid #444; border-radius: 4px; box-shadow: 0 10px 30px rgba(0,0,0,0.5); }
        .cell { background: #006600; display: flex; align-items: center; justify-content: center; position: relative; box-sizing: border-box; }
        
        .board-big { grid-template-columns: repeat(8, 75px); }
        .board-big .cell { width: 75px; height: 75px; }
        .board-big .piece { width: 60px; height: 60px; border-radius: 50%; box-shadow: 2px 4px 8px rgba(0,0,0,0.6); }
        
        .piece.black { background: radial-gradient(circle at 30% 30%, #555, #000); border: 1px solid #000; }
        .piece.white { background: radial-gradient(circle at 30% 30%, #fff, #ccc); border: 1px solid #999; }

        /* Miniatura Lógica */
        .mini-stage {
            position: absolute; bottom: 30px; right: 30px;
            background: rgba(0,0,0,0.6); padding: 10px;
            border-radius: 8px; border: 1px solid #444;
            backdrop-filter: blur(5px);
        }
        .mini-label { font-size: 0.8em; color: #aaa; text-align: center; margin-bottom: 5px; }
        .board-small { grid-template-columns: repeat(8, 20px); grid-gap: 1px; border-width: 2px; }
        .board-small .cell { width: 20px; height: 20px; }
        .board-small .piece { width: 16px; height: 16px; }

        /* Destaques */
        .diff { border: 4px solid #ff3333 !important; background-color: rgba(255, 0, 0, 0.2); }
        .highlight-green { box-shadow: inset 0 0 20px 5px #00ff00; border: 3px solid #ccffcc !important; animation: pulse 1.5s infinite; }
        @keyframes pulse { 0% { box-shadow: inset 0 0 5px 2px #00ff00; } 50% { box-shadow: inset 0 0 25px 10px #00ff00; } 100% { box-shadow: inset 0 0 5px 2px #00ff00; } }

    </style>
</head>
<body>
    
    <div class="header-panel">
        <div class="app-title">🤖 Othello Arena</div>
        <div style="display: flex; align-items: baseline;">
            <div id="status-player" class="status-big">Inicializando...</div>
            <div id="status-action" class="status-sub"></div>
        </div>
        <div class="score">
            ⚫ <span id="s-black">0</span> &nbsp;|&nbsp; ⚪ <span id="s-white">0</span>
        </div>
    </div>

    <div class="main-layout">
        <div class="sidebar">
            <h3>PLANO DE AÇÃO</h3>
            <ul id="steps-list" class="step-list">
                <li class="step-item" style="opacity: 0.5;">Aguardando robô...</li>
            </ul>
        </div>

        <div class="center-stage">
            <div id="instruction-box"></div>

            <div id="board-fisico" class="board board-big"></div>

            <div class="mini-stage">
                <div class="mini-label">Lógica Interna</div>
                <div id="board-logico" class="board board-small"></div>
            </div>
        </div>
    </div>

    <script>
        function renderBoard(elId, matrix, diffs=[], highlightGreen=null) {
            const board = document.getElementById(elId);
            board.innerHTML = '';
            for (let r=0; r<8; r++) {
                for (let c=0; c<8; c++) {
                    const cell = document.createElement('div');
                    cell.className = 'cell';
                    
                    if (diffs.some(d => d[0]===r && d[1]===c)) cell.classList.add('diff');
                    
                    if (!cell.classList.contains('diff') && highlightGreen && highlightGreen[0]===r && highlightGreen[1]===c) {
                        cell.classList.add('highlight-green');
                    }

                    const val = matrix[r][c];
                    if (val !== 0) {
                        const p = document.createElement('div');
                        p.className = val === 1 ? 'piece black' : 'piece white';
                        cell.appendChild(p);
                    }
                    board.appendChild(cell);
                }
            }
        }

        async function loop() {
            try {
                const req = await fetch('/dashboard/data');
                const d = await req.json();

                // Status
                document.getElementById('status-player').innerText = d.jogador;
                document.getElementById('status-action').innerText = d.acao;
                document.getElementById('s-black').innerText = d.placar.Pretas;
                document.getElementById('s-white').innerText = d.placar.Brancas;

                // Lista de Passos (Sidebar)
                const stepsUl = document.getElementById('steps-list');
                if (d.passos_robo && d.passos_robo.length > 0) {
                    // Reconstrói a lista apenas se mudou (para não piscar, idealmente faria diff, mas ok)
                    // Aqui vamos simplificar e reconstruir
                    let html = '';
                    d.passos_robo.forEach((step, idx) => {
                        // O primeiro passo é o "atual" (simulação visual)
                        let activeClass = idx === 0 ? '' : ''; 
                        html += `<li class="step-item ${activeClass}">${step}</li>`;
                    });
                    if (stepsUl.innerHTML !== html) stepsUl.innerHTML = html;
                } else {
                     // Se não tem passos, mostra vazio ou mensagem padrão
                     if (!stepsUl.innerHTML.includes("Aguardando"))
                        stepsUl.innerHTML = '<li class="step-item" style="opacity: 0.5;">Aguardando plano...</li>';
                }

                // Instruções Humano
                const box = document.getElementById('instruction-box');
                if (d.instrucoes && d.instrucoes.length > 0) {
                    box.style.display = 'block';
                    box.innerHTML = '<strong>⚠️ AÇÃO:</strong> ' + d.instrucoes.join('<br>');
                } else {
                    box.style.display = 'none';
                }

                // Tabuleiros
                let verde = (d.instrucoes && d.instrucoes.length > 0) ? null : d.destaque_verde;
                renderBoard('board-fisico', d.fisico, d.diffs, verde);
                renderBoard('board-logico', d.logico, [], d.destaque_verde);

            } catch(e) { console.log(e); }
        }
        
        setInterval(loop, 300);
    </script>
</body>
</html>
    """

# === ENDPOINTS DE MOVIMENTO BÁSICO (Com Verificação Cruzada) ===

@app.post("/move/joints", dependencies=[Depends(require_token)])
def move_joints(body: Joints, rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]
    cmd = [body.j1, body.j2, body.j3, body.j4, body.j5, body.j6, 0, 0]
    check_other_robot_is_stopped(rid)
    safe_speed = min(body.speed, SPEED_PTP)
    safe_accel = min(body.accel, ACCEL_PTP)
    print(f"[BRIDGE] Movendo Robô {rid} para juntas: {cmd[:6]} @ {safe_speed}%")
    with slot.command_lock:
        try:
            resp = ec.move_joint(target_joint=cmd, speed=safe_speed, acc=safe_accel, dec=safe_accel, block=True)
            print(f"[BRIDGE] Mov Juntas Robô {rid} concluído. SDK: {resp}"); 
            return {"ok": resp.success, "result": resp.result}
        except Exception as e: 
            print(f"[BRIDGE][ERR] Exceção /move/joints Robô {rid}:"); traceback.print_exc(); slot.ec = None; 
            raise HTTPException(500, f"Exceção mov juntas Robô {rid}: {e}")

@app.post("/move/pose", dependencies=[Depends(require_token)])
def move_pose(body: Pose, rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]
    target_p = [body.x, body.y, body.z, body.rx, body.ry, body.rz]
    check_other_robot_is_stopped(rid)
    safe_speed = min(body.speed, SPEED_PTP)
    safe_accel = min(body.accel, ACCEL_PTP)
    print(f"[BRIDGE] Movendo Robô {rid} para pose (PTP): {target_p} @ {safe_speed}%")
    with slot.command_lock:
        try:
            if not _exec_move_ptp_sync(ec, body.dict(), safe_speed, safe_accel):
                raise RuntimeError("Falha no helper _exec_move_ptp_sync")
            print(f"[BRIDGE] Mov Pose (PTP) Robô {rid} concluído."); 
            return {"ok": True, "result": "Movimento PTP concluído."}
        except Exception as e: 
            print(f"[BRIDGE][ERR] Exceção /move/pose Robô {rid}:"); traceback.print_exc(); slot.ec = None; 
            raise HTTPException(500, f"Exceção mov pose Robô {rid}: {e}")

@app.post("/move/line", dependencies=[Depends(require_token)])
def move_line(body: LinePose, rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]
    check_other_robot_is_stopped(rid)
    safe_speed = min(body.speed, SPEED_LINEAR)
    safe_accel = min(body.accel, ACCEL_LINEAR)
    print(f"[BRIDGE] Movendo Robô {rid} para pose (LIN): {[body.x, body.y, body.z]} @ {safe_speed} mm/s")
    with slot.command_lock:
        try:
            if not _exec_move_line_sync(ec, body.dict(), safe_speed, safe_accel):
                 raise RuntimeError("Falha no helper _exec_move_line_sync")
            print(f"[BRIDGE] Mov Pose (LIN) Robô {rid} concluído."); 
            return {"ok": True, "result": "Movimento LIN concluído."}
        except Exception as e: 
            print(f"[BRIDGE][ERR] Exceção /move/line Robô {rid}:"); traceback.print_exc(); slot.ec = None; 
            raise HTTPException(500, f"Exceção mov line Robô {rid}: {e}")

@app.post("/home", dependencies=[Depends(require_token)])
def move_to_home(rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]
    check_other_robot_is_stopped(rid)
    print(f"[BRIDGE] Movendo Robô {rid} para Home (Físico) @{HOME_SPEED}%...")
    with slot.command_lock:
        try:
            # Usa as constantes de Home Físico (os 6 primeiros eixos)
            if not _exec_move_joint_sync(ec, HOME_POSITION_JOINTS[:6], HOME_SPEED, ACCEL_PTP):
                raise RuntimeError("Falha no helper _exec_move_joint_sync para HOME_POSITION_JOINTS")
            
            global robot_states
            # Atualiza o estado para sabermos que ele está no home físico, não no staging
            robot_states[rid]['side'] = 'home' 
            print(f"[BRIDGE] Mov Home Físico Robô {rid} concluído."); 
            return {"ok": True, "result": "Movimento Home Físico concluído."}
        except Exception as e: 
            print(f"[BRIDGE][ERR] Exceção /home Robô {rid}:"); traceback.print_exc(); slot.ec = None; 
            raise HTTPException(500, f"Exceção mov home Robô {rid}: {e}")

@app.post("/move/relative", dependencies=[Depends(require_token)])
def move_relative(body: RelativeMove, rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]
    deltas = [body.dx, body.dy, body.dz, body.drx, body.dry, body.drz]
    check_other_robot_is_stopped(rid)
    safe_speed = min(body.speed, SPEED_PTP)
    safe_accel = min(body.accel, ACCEL_PTP)
    print(f"[BRIDGE] Movendo Robô {rid} relativamente: {deltas} @ {safe_speed}%")
    with slot.command_lock:
        try:
            if not _exec_move_relative_sync(ec, deltas, safe_speed, safe_accel):
                raise RuntimeError("Falha no helper _exec_move_relative_sync")
            print(f"[BRIDGE] Mov Relativo Robô {rid} concluído."); 
            return {"ok": True, "result": "Movimento Relativo concluído."}
        except Exception as e: 
            print(f"[BRIDGE][ERR] Exceção /move/relative Robô {rid}:"); traceback.print_exc(); slot.ec = None; 
            raise HTTPException(500, f"Exceção mov relativo Robô {rid}: {e}")

@app.post("/rotate/gripper", dependencies=[Depends(require_token)])
def rotate_gripper_endpoint(body: RotateGripper, rid: int = Query(..., ge=1, le=2)):
    ec = ensure_robot(rid); slot = robots[rid]; angle_deg = body.angle
    check_other_robot_is_stopped(rid)
    safe_speed = min(body.speed, SPEED_PTP)
    safe_accel = min(body.accel, ACCEL_PTP)
    print(f"[BRIDGE] Girando garra (J6) Robô {rid} por {angle_deg} graus @ {safe_speed}%")
    with slot.command_lock:
        try:
            current_j_resp = ec.get_joint()
            if not current_j_resp.success: 
                return {"ok": False, "result": "Falha leitura juntas (rotação)."}
            current_j = current_j_resp.result
            target_j = list(current_j[:6])
            current_angle_j6 = target_j[5]
            target_angle_raw = current_angle_j6 + angle_deg
            target_angle_normalized = (target_angle_raw + 180) % 360 - 180
            target_j[5] = target_angle_normalized
            print(f"[BRIDGE] J6 atual: {current_angle_j6:.2f}, Alvo bruto: {target_angle_raw:.2f}, Alvo normalizado: {target_angle_normalized:.2f}")
            cmd = target_j + [0] * (8 - len(target_j))
            print(f"[BRIDGE] Juntas alvo rotação Robô {rid}: {cmd[:6]}. Enviando mov...")
            move_resp = ec.move_joint(target_joint=cmd, speed=safe_speed, acc=safe_accel, dec=safe_accel, block=True)
            print(f"[BRIDGE] Rotação garra Robô {rid} concluída. SDK: {move_resp}")
            return {"ok": move_resp.success, "result": move_resp.result}
        except Exception as e: 
            print(f"[BRIDGE][ERR] Exceção /rotate/gripper Robô {rid}:"); traceback.print_exc()
            slot.ec = None; raise HTTPException(500, f"Exceção rotação garra Robô {rid}: {e}")


# === ENDPOINTS DE MACRO (Síncronos / Bloqueantes) ===

@app.post("/macro/pegar_estojo", dependencies=[Depends(require_token)])
def macro_pegar_estojo(
    body: MacroPegaPayload, 
    rid: int = Query(..., ge=1, le=2)
):
    """
    Executa a macro de pegar peça (descer, fechar, subir).
    NOTA: Esta macro é "burra", ela NÃO calcula a pose.
    O Colab deve chamar /move/pose para a pose_aprox ANTES.
    """
    ec = ensure_robot(rid); slot = robots[rid]
    check_other_robot_is_stopped(rid)
    safe_s = min(body.speed_ptp, SPEED_PTP)
    safe_a = min(body.accel_ptp, ACCEL_PTP)
    with slot.command_lock:
        print(f"[BRIDGE][MACRO] R{rid} Exec: Pegar Peça (Macro Burra)")
        _macro_pegar_no_estojo_sync(ec, rid, 0, safe_s, safe_a)
        print(f"[BRIDGE][MACRO] R{rid} Fim: Pegar Peça (Macro Burra)")
        
    return {"ok": True, "message": f"R{rid} - Macro Pega Estojo concluída."}


@app.post("/macro/colocar_casa", dependencies=[Depends(require_token)])
def macro_colocar_casa(
    body: MacroPtpPayload,
    rid: int = Query(..., ge=1, le=2)
):
    """Executa apenas a macro de colocar (descer, abrir, subir)."""
    ec = ensure_robot(rid); slot = robots[rid]
    check_other_robot_is_stopped(rid)
    safe_s = min(body.speed_ptp, SPEED_PTP)
    safe_a = min(body.accel_ptp, ACCEL_PTP)
    
    with slot.command_lock:
        _macro_colocar_na_casa_sync(ec, rid, safe_s, safe_a)
        
    return {"ok": True, "message": f"R{rid} - Macro Colocar Casa concluída."}


@app.post("/macro/virar_casa", dependencies=[Depends(require_token)])
def macro_virar_casa(
    body: MacroPtpPayload,
    rid: int = Query(..., ge=1, le=2)
):
    """Executa apenas a macro de virar peça (descer, fechar, subir, girar, ...)."""
    ec = ensure_robot(rid); slot = robots[rid]
    check_other_robot_is_stopped(rid)
    safe_s = min(body.speed_ptp, SPEED_PTP)
    safe_a = min(body.accel_ptp, ACCEL_PTP)
    
    with slot.command_lock:
        _macro_virar_na_casa_sync(ec, rid, safe_s, safe_a)

    return {"ok": True, "message": f"R{rid} - Macro Virar Casa concluída."}


# === NOVO ENDPOINT: SUPER-MACRO DE LANCE COMPLETO ===

@app.post("/macro/executar_lance_completo", dependencies=[Depends(require_token)])
def macro_executar_lance_completo(
    body: LanceCompletoPayload,
    rid: int = Query(..., ge=1, le=2)
):
    """
    Executa o lance inteiro de Othello (Pegar, Colocar, Virar)
    de forma síncrona no lado do bridge para eliminar latência.
    """
    global robot_states
    
    cfg = ROBOT_CONFIGS[rid] 
    z_travel = cfg["Z_TRAVEL"]
    z_travel_estojo = cfg["Z_TRAVEL_ESTOJO"]

    ec = ensure_robot(rid)
    slot = robots[rid]
    
    # Mapeia RID (1, 2) para Jogador (1, -1)
    jogador_atual = 1 if rid == 1 else -1 
    
    # Pega os parâmetros do payload
    i_log, j_log = body.jogada
    run_s_ptp = min(body.speed_ptp, SPEED_PTP)
    run_s_lin = min(body.speed_linear, SPEED_LINEAR)
    run_a_ptp = min(body.accel_ptp, ACCEL_PTP)
    run_a_lin = min(body.accel_linear, ACCEL_LINEAR)
    num_peca_atual = body.num_peca_atual
    capturas_por_direcao = body.capturas_por_direcao
    
    # 1. Trava este robô
    check_other_robot_is_stopped(rid)
    
    with slot.command_lock:
        print(f"--- [R{rid}] INICIANDO SUPER-MACRO LANCE ---")
        try:
            # --- 2. PEGAR PEÇA NOVA ---
            print(f"--- [R{rid}] Tarefa 1: Pegar Peça (Estojo) ---")
            print(f"[R{rid}] Buscando peça N° {num_peca_atual + 1} do estojo.")
            i_phys_estojo, j_phys_estojo = get_physical_coords(rid, -1, -1)
            target_side = get_target_side(rid, i_phys_estojo, j_phys_estojo)
            _ = _handle_safe_transition(ec, rid, target_side)
            
            pose_aprox_estojo = get_pose_from_estojo(rid, z_travel_estojo, num_peca_atual)
            print(f"[R{rid}] Aproximação Estojo (PTP) @ {run_s_ptp}% (Peça {num_peca_atual})")
            if not _exec_move_ptp_sync(ec, pose_aprox_estojo, run_s_ptp, run_a_ptp):
                raise RuntimeError("Falha ao mover para aproximação do estojo")
                
            _macro_pegar_no_estojo_sync(ec, rid, num_peca_atual, run_s_ptp, run_a_ptp)
            print(f"[R{rid}] Saindo do Estojo. Forçando transição via Staging.")
            robot_states[rid]['side'] = "estojo"

            # --- 3. COLOCAR PEÇA NA CASA ---
            print(f"--- [R{rid}] Tarefa 2: Colocar Peça ({i_log},{j_log}) ---")
            i_phys, j_phys = get_physical_coords(rid, i_log, j_log)
            target_side = get_target_side(rid, i_phys, j_phys)
            did_transition = _handle_safe_transition(ec, rid, target_side)
            
            pose_aprox_colocar = get_pose_from_grid(rid, i_phys, j_phys, z_travel)
            if did_transition:
                if not _exec_move_ptp_sync(ec, pose_aprox_colocar, run_s_ptp, run_a_ptp):
                    raise RuntimeError("Falha ao mover (PTP) para aproximação de colocação")
            else:
                if not _exec_move_line_sync(ec, pose_aprox_colocar, run_s_lin, run_a_lin):
                     raise RuntimeError("Falha ao mover (LIN) para aproximação de colocação")
                     
            _macro_colocar_na_casa_sync(ec, rid, run_s_ptp, run_a_ptp)

            # --- 4. VIRAR PEÇAS CAPTURADAS ---
            alvos: List[Tuple[int, int]] = []
            total = 0
            for lst in capturas_por_direcao:
                total += len(lst)
                alvos.extend(lst)

            if total == 0:
                print(f"--- [R{rid}] Tarefa 3: Sem Flips ---")
            else:
                print(f"--- [R{rid}] Tarefa 3: Virar {total} Peças ---")
                i_phys_jogada, j_phys_jogada = get_physical_coords(rid, i_log, j_log)
                q0 = get_target_side(rid, i_phys_jogada, j_phys_jogada)

                if total <= 2:
                    modo = "direcao_primeiro"
                    ordem = [tuple(ij) for lst in capturas_por_direcao for ij in lst]
                else:
                    modo = "quadrante_primeiro"
                    baldes: Dict[str, List[Tuple[int, int]]] = {"esq": [], "dir": []}
                    for ij_log in alvos:
                        ij_log_tuple = tuple(ij_log)
                        i_phys_flip, j_phys_flip = get_physical_coords(rid, *ij_log_tuple)
                        baldes[get_target_side(rid, i_phys_flip, j_phys_flip)].append(ij_log_tuple)
                    
                    ordem_lados = [q0] + [q for q in ["esq", "dir"] if q != q0]
                    ordem = []
                    for q in ordem_lados:
                        ordem += sorted(baldes[q], key=lambda ij: (ij[0], ij[1]))

                print(f"[R{rid}] Iniciando {total} flips (modo: {modo})")

                q_atual = q0
                for (pi_log, pj_log) in ordem:
                    pi_phys, pj_phys = get_physical_coords(rid, pi_log, pj_log)
                    new_side = get_target_side(rid, pi_phys, pj_phys)
                    did_transition = _handle_safe_transition(ec, rid, new_side)
                    q_atual = new_side
                    
                    pose_aprox_flip = get_pose_from_grid(rid, pi_phys, pj_phys, z_travel)
                    
                    if did_transition:
                        if not _exec_move_ptp_sync(ec, pose_aprox_flip, run_s_ptp, run_a_ptp):
                            raise RuntimeError(f"Falha ao mover (PTP) para aproximação de flip em {(pi_log, pj_log)}")
                    else:
                        if not _exec_move_line_sync(ec, pose_aprox_flip, run_s_lin, run_a_lin):
                            raise RuntimeError(f"Falha ao mover (LIN) para aproximação de flip em {(pi_log, pj_log)}")
                            
                    _macro_virar_na_casa_sync(ec, rid, run_s_ptp, run_a_ptp)

                print(f"[R{rid}] Flips concluídos. Recuando para home {q_atual}.")

            # --- 5. RETORNAR AO HOME FÍSICO (Conforme solicitado) ---
            print(f"[R{rid}] Lance concluído. Recuando para Home Físico.")
            robot_states[rid]['side'] = 'home' # Define o estado
            
            # Usa as constantes de Home Físico e velocidade de Home
            if not _exec_move_joint_sync(ec, HOME_POSITION_JOINTS[:6], HOME_SPEED, run_a_ptp):
                raise RuntimeError("Falha ao retornar para Home Físico")
            
            print(f"--- [R{rid}] SUPER-MACRO LANCE CONCLUÍDA ---")

        except Exception as e:
            print(f"[BRIDGE][ERR] Falha na Super-Macro R{rid}: {e}")
            traceback.print_exc()
            try:
                ec.clear_collision_alarm()
                ec.clear_alarm()
            except: pass
            raise HTTPException(status_code=500, detail=f"Erro na macro R{rid}: {e}")

    return {"ok": True, "message": f"R{rid} - Lance completo executado."}