import cv2
import numpy as np
import time
import requests
import threading

# --- CONFIGURAÇÃO DE REDE ---
BRIDGE_URL = "http://127.0.0.1:8000/vis/update_state"
keep_running = True
global_board_state = None

# --- CONFIGURAÇÃO VISÃO ---
rois_rects = [
    [399, 127, 25, 25],   
    [840, 123, 25, 25],  
    [938, 558, 25, 25],  
    [292, 563, 25, 25]    
]

selected_roi_index = None
drag_offset = (0, 0)
alpha = 0.15 
smoothed_rect = None 

# Configuração do CLAHE
clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))

# --- CLASSE DE ESTABILIZAÇÃO TEMPORAL ---
class StableBoard:
    def __init__(self, persistence=10):
        self.persistence = persistence
        self.stable_grid = np.zeros((8, 8), dtype=int)
        self.counters = np.zeros((8, 8), dtype=int)
        self.candidates = np.zeros((8, 8), dtype=int)

    def update(self, raw_grid):
        for r in range(8):
            for c in range(8):
                new_val = raw_grid[r][c]
                current_stable = self.stable_grid[r][c]
                
                if new_val == current_stable:
                    self.counters[r][c] = 0
                    self.candidates[r][c] = new_val 
                else:
                    if new_val == self.candidates[r][c]:
                        self.counters[r][c] += 1
                    else:
                        self.candidates[r][c] = new_val
                        self.counters[r][c] = 1
                    
                    if self.counters[r][c] >= self.persistence:
                        self.stable_grid[r][c] = new_val
                        self.counters[r][c] = 0 
        
        return self.stable_grid

stabilizer = StableBoard(persistence=15)

# --- FUNÇÕES AUXILIARES ---
def mouse_callback(event, x, y, flags, param):
    global selected_roi_index, drag_offset
    if event == cv2.EVENT_LBUTTONDOWN:
        for i, (rx, ry, rw, rh) in enumerate(rois_rects):
            if rx < x < rx + rw and ry < y < ry + rh:
                selected_roi_index = i
                drag_offset = (x - rx, y - ry)
                break
    elif event == cv2.EVENT_MOUSEMOVE:
        if selected_roi_index is not None:
            rois_rects[selected_roi_index][0] = x - drag_offset[0]
            rois_rects[selected_roi_index][1] = y - drag_offset[1]
    elif event == cv2.EVENT_LBUTTONUP:
        selected_roi_index = None

def nothing(x): pass

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def draw_dashboard(matrix_data):
    h, w = 400, 400
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:] = (50, 50, 50) 
    step = h // 8
    
    for r in range(8):
        for c in range(8):
            val = matrix_data[r][c]
            x1 = c * step; y1 = r * step; x2 = x1 + step; y2 = y1 + step
            
            bg_color = (60, 60, 60)
            if val == 1: bg_color = (20, 20, 20) 
            elif val == -1: bg_color = (100, 100, 100) 
            
            cv2.rectangle(img, (x1, y1), (x2, y2), bg_color, -1)
            cv2.rectangle(img, (x1, y1), (x2, y2), (200, 200, 200), 1)
            
            center_x = x1 + step // 2
            center_y = y1 + step // 2
            
            if val == 1: 
                cv2.circle(img, (center_x, center_y), 15, (0, 0, 0), -1)
                cv2.circle(img, (center_x, center_y), 15, (100, 100, 100), 1)
            elif val == -1:
                cv2.circle(img, (center_x, center_y), 15, (255, 255, 255), -1)
            
    return img

# --- THREAD DE REDE (ENVIO PARA O BRIDGE) ---
def network_worker():
    global global_board_state
    session = requests.Session()
    last_sent_time = 0
    print(f"[REDE] Iniciando envio para {BRIDGE_URL}...")
    
    while keep_running:
        if global_board_state is not None and (time.time() - last_sent_time > 0.2):
            try:
                matrix_to_send = [row[:] for row in global_board_state]
                payload = {"matrix": matrix_to_send}
                session.post(BRIDGE_URL, json=payload, timeout=0.5)
                last_sent_time = time.time()
            except Exception as e:
                time.sleep(1.0)
        time.sleep(0.05)

# Inicia a thread de rede
t_net = threading.Thread(target=network_worker)
t_net.daemon = True
t_net.start()

# --- SETUP DA CÂMERA (Cuidado: ID 2 Fixo como no Hough original) ---
cap = cv2.VideoCapture(2) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

cv2.namedWindow("Ajustes")
cv2.createTrackbar("Divisor Preto/Branco", "Ajustes", 100, 255, nothing)
cv2.createTrackbar("Sensibilidade (Hough)", "Ajustes", 30, 100, nothing)
cv2.createTrackbar("Raio Min Peca", "Ajustes", 35, 50, nothing)
cv2.createTrackbar("Raio Max Peca", "Ajustes", 45, 60, nothing)
cv2.createTrackbar("Persistencia (Frames)", "Ajustes", 15, 60, nothing)

cv2.namedWindow("Webcam Original")
cv2.setMouseCallback("Webcam Original", mouse_callback)

print("[SISTEMA] Iniciando Loop Principal de Visão...")

# --- LOOP PRINCIPAL ---
while True:
    ret, frame = cap.read()
    if not ret: break

    debug_view = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_eq = clahe.apply(gray)

    found_points = []

    for i, (x, y, w, h) in enumerate(rois_rects):
        color = (0, 255, 255) if i == selected_roi_index else (255, 0, 0)
        cv2.rectangle(debug_view, (x, y), (x+w, y+h), color, 2)
        roi_gray = gray[y:y+h, x:x+w]
        corners = cv2.goodFeaturesToTrack(roi_gray, 1, 0.01, 10)
        if corners is not None:
            lx, ly = corners.ravel()
            gx, gy = x + lx, y + ly
            found_points.append([gx, gy])
            cv2.circle(debug_view, (int(gx), int(gy)), 5, (0, 0, 255), -1)

    if len(found_points) == 4:
        pts = np.array(found_points, dtype="float32")
        rect = order_points(pts)
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        rect_current = cv2.cornerSubPix(gray, rect, (5, 5), (-1, -1), criteria)

        if smoothed_rect is None: smoothed_rect = rect_current
        else: smoothed_rect = (alpha * rect_current) + ((1 - alpha) * smoothed_rect)
        
        real_width_mm, real_height_mm = 450, 450
        scale = 2 
        width_px = real_width_mm * scale
        height_px = real_height_mm * scale

        dst_pts = np.array([
            [width_px - 1, height_px - 1], [0, height_px - 1],           
            [0, 0], [width_px - 1, 0]             
        ], dtype="float32")

        M = cv2.getPerspectiveTransform(smoothed_rect, dst_pts)
        warped_eq = cv2.warpPerspective(gray_eq, M, (int(width_px), int(height_px)))
        warped_color = cv2.warpPerspective(frame, M, (int(width_px), int(height_px)))
        warped_blur = cv2.GaussianBlur(warped_eq, (9, 9), 2)

        # Lê Trackbars
        hough_sens = cv2.getTrackbarPos("Sensibilidade (Hough)", "Ajustes")
        min_rad = cv2.getTrackbarPos("Raio Min Peca", "Ajustes")
        max_rad = cv2.getTrackbarPos("Raio Max Peca", "Ajustes")
        bw_thresh = cv2.getTrackbarPos("Divisor Preto/Branco", "Ajustes")
        persis_val = cv2.getTrackbarPos("Persistencia (Frames)", "Ajustes")
        
        # Atualiza persistencia em tempo real
        stabilizer.persistence = max(1, persis_val)

        if hough_sens < 1: hough_sens = 1
        if min_rad >= max_rad: max_rad = min_rad + 5

        circles = cv2.HoughCircles(
            warped_blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=80,
            param1=50, param2=hough_sens, minRadius=min_rad, maxRadius=max_rad
        )

        # Grid "cru" (Raw) deste frame
        raw_grid = np.zeros((8, 8), dtype=int)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cx, cy, r = i[0], i[1], i[2]
                
                offset_px, step_px = 100, 100
                col = int(round((int(cx) - offset_px) / step_px))
                row = int(round((int(cy) - offset_px) / step_px))

                if 0 <= col < 8 and 0 <= row < 8:
                    sample = warped_eq[cy-10:cy+10, cx-10:cx+10]
                    if sample.size > 0:
                        if np.mean(sample) > bw_thresh:
                            raw_grid[row][col] = -1 # Branco
                        else:
                            raw_grid[row][col] = 1 # Preto
                    
                    cv2.circle(warped_color, (cx, cy), r, (0, 255, 0), 2)

        # --- APLICA A ESTABILIZAÇÃO ---
        stable_grid = stabilizer.update(raw_grid)

        # Envia o GRID ESTÁVEL para a rede (A thread lê essa variável)
        global_board_state = stable_grid.tolist()

        # Desenha o Dashboard
        dashboard_img = draw_dashboard(stable_grid)
        
        if np.any(stabilizer.counters > 0):
            cv2.putText(dashboard_img, "Estabilizando...", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        cv2.imshow("Status Estavel (Rede)", dashboard_img)

        # --- DEBUG VISUAL (IGUAL AO HOUGH) ---
        for r in range(8):
            for c in range(8):
                px = int(100 + c * 100)
                py = int(100 + r * 100)
                x1, y1 = px - 40, py - 40
                x2, y2 = px + 40, py + 40
                
                # Desenha o número de brilho (ESSENCIAL PARA SUA CALIBRAÇÃO)
                stat_roi = warped_eq[py-15:py+15, px-15:px+15]
                if stat_roi.size > 0:
                    lum_val = int(np.mean(stat_roi))
                    cv2.putText(warped_color, str(lum_val), (px-20, py+5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                status = stable_grid[r][c]
                color = (0,0,255) # Vazio = Vermelho
                if status == 1: color = (0,0,0)
                elif status == -1: color = (255,255,255)
                
                cv2.rectangle(warped_color, (x1, y1), (x2, y2), color, 2)

        cv2.imshow("Visao Normalizada", warped_eq)
        cv2.imshow("Resultado Final", warped_color)
    else:
        smoothed_rect = None

    cv2.imshow("Webcam Original", debug_view)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

keep_running = False
cap.release()
cv2.destroyAllWindows()