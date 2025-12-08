import math
import numpy as np

def calibrar_tabuleiro(p_origem, p_eixo_x, p_eixo_y):
    """
    p_origem: (x, y) da casa [0,0] (Físico)
    p_eixo_x: (x, y) da casa [7,0] (Físico - final da linha)
    p_eixo_y: (x, y) da casa [0,7] (Físico - final da coluna)
    """
    x0, y0 = p_origem
    xx, yx = p_eixo_x
    
    # 1. Calcular Ângulo (A inclinação da mesa em relação ao robô)
    # deltaY / deltaX
    angulo_rad = math.atan2(yx - y0, xx - x0)
    
    # 2. Calcular Tamanho Real da Casa (Check de sanidade)
    distancia_total = math.sqrt((xx - x0)**2 + (yx - y0)**2)
    tamanho_casa_calc = distancia_total / 7.0  # 7 casas de A1 a H1
    
    print(f"--- RESULTADO CALIBRAÇÃO ---")
    print(f"BOARD_ORIGIN_X = {x0:.4f}")
    print(f"BOARD_ORIGIN_Y = {y0:.4f}")
    print(f"BOARD_ANGLE_RAD = {angulo_rad:.8f}")
    print(f"SQUARE_SIZE_MM (Calculado) = {tamanho_casa_calc:.4f}")
    
    # 3. Calcular H8 (índice 7, 7)
    i_target = 7.0
    j_target = 7.0
    
    x_h8 = x0 + (i_target * tamanho_casa_calc * math.cos(angulo_rad)) - (j_target * tamanho_casa_calc * math.sin(angulo_rad))
    y_h8 = y0 + (i_target * tamanho_casa_calc * math.sin(angulo_rad)) + (j_target * tamanho_casa_calc * math.cos(angulo_rad))

    print(f"--- PONTO H8 (7,7) CALCULADO ---")
    print(f"H8_X = {x_h8:.4f}")
    print(f"H8_Y = {y_h8:.4f}")

    return x0, y0, angulo_rad

# --- EXEMPLO DE USO (Substitua pelos valores que você ler no teach pendant) ---
# Exemplo Robô 1:
# Leve o robô na A1 (0,0) e anote X,Y
p1 = (-203.231, 189.298)
# Leve o robô na H1 (7,0) e anote X,Y
p2 = (143.186, 219.210) 
# Leve o robô na A8 (0,7) (opcional, para validar ortogonalidade)
p3 = (-232.498, 538.794) 

calibrar_tabuleiro(p1, p2, p3)