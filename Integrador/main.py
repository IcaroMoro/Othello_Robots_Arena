import multiprocessing
import uvicorn
import time
import sys
import os

def start_api_server():
    """Roda o Uvicorn (FastAPI) em um processo separado"""
    from bridge_server import app 
    print("[SYSTEM] Iniciando Servidor Bridge na porta 8000...")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="error")

def start_vision_system():
    """Roda a Visão Computacional"""
    from opencv_module import run_vision_loop
    run_vision_loop(camera_id=2)

if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', force=True)

    print("=== SISTEMA INTEGRADO OTHELLO ROBÔ (MULTIPROCESSING) ===")
    print("1. Iniciando Bridge (Processo Isolado)...")
    
    server_process = multiprocessing.Process(target=start_api_server, daemon=True)
    server_process.start()
    
    time.sleep(3)
    
    print("2. Iniciando Visão Computacional (Processo Principal)...")
    print("   (Pressione 'q' na janela da câmera para sair)")
    
    try:
        start_vision_system()
        
    except KeyboardInterrupt:
        print("\n[SYSTEM] Interrupção detectada...")
    except Exception as e:
        print(f"\n[SYSTEM] Erro fatal: {e}")
    finally:
        print("[SYSTEM] Derrubando servidor...")
        server_process.terminate()
        server_process.join()
        print("[SYSTEM] Sistema finalizado.")
        sys.exit(0)