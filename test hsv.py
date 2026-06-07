import cv2
import numpy as np
import pandas as pd
from scipy.signal import find_peaks, savgol_filter
import matplotlib.pyplot as plt

class CintaVelocidadEstimator:
    def __init__(self, video_path, fps=120):
        self.video_path = video_path
        self.fps = fps
        self.cap = cv2.VideoCapture(video_path)

        if not self.cap.isOpened():
            raise ValueError(f"No se pudo abrir el video: {video_path}")

        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.ancho = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.alto  = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Video: {self.ancho}x{self.alto}, {self.total_frames} frames, {fps} FPS")

    # ------------------------------------------------------------------
    # Detección de marca amarilla
    # ------------------------------------------------------------------
    def detectar_marca_amarilla(self, frame, h_centro=28, tolerancia_h=15,
                             s_min=80, v_min=80):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([max(0, h_centro - tolerancia_h), s_min, v_min])
        upper = np.array([min(179, h_centro + tolerancia_h), 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        
        kernel_close = np.ones((16, 16), np.uint8)
        kernel_open  = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0, mask
        
        candidatos = [(cv2.contourArea(c), c) for c in contours if cv2.contourArea(c) >= 200]
        if not candidatos:
            return None, 0, mask
        
        candidatos.sort(key=lambda x: x[0], reverse=True)
        area_max, contorno_max = candidatos[0]
        
        M = cv2.moments(contorno_max)
        if M["m00"] == 0:
            return None, 0, mask
        
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy), area_max, mask
    
    # ------------------------------------------------------------------
    # Extraer trayectoria
    # ------------------------------------------------------------------
    def extraer_trayectoria(self, mostrar_video=False,
                             h_centro=28, tolerancia_h=15,
                             s_min=80, v_min=80,
                             roi_x=296, roi_y=600, roi_w=208, roi_h=169):
        
        trayectorias = []
        roi = (roi_x, roi_y, roi_w, roi_h)
        print(f"Usando ROI fijo: {roi}")

        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        frame_idx = 0
        ultimo_frame_detectado = -10
        MIN_FRAMES_ENTRE_DETECCIONES = 8

        if mostrar_video:
            cv2.namedWindow('Seguimiento', cv2.WINDOW_NORMAL)

        VELOCIDAD_REPRODUCCION = 0.2
        delay_ms = max(1, int((1000 / self.fps) / VELOCIDAD_REPRODUCCION))

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            centro, area, mask = self.detectar_marca_amarilla(
                frame, h_centro, tolerancia_h, s_min, v_min)

            if centro:
                rx, ry, rw, rh = roi
                dentro_x = rx <= centro[0] <= rx + rw
                dentro_y = ry <= centro[1] <= ry + rh

                if dentro_x and dentro_y:
                    if (frame_idx - ultimo_frame_detectado) >= MIN_FRAMES_ENTRE_DETECCIONES:
                        trayectorias.append((frame_idx, centro[0], centro[1], area))
                        ultimo_frame_detectado = frame_idx
                        
                        if mostrar_video:
                            cv2.circle(frame, centro, 12, (0, 255, 0), 3)
                            cv2.putText(frame, f"#{len(trayectorias)}", 
                                      (centro[0]-20, centro[1]-15), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if mostrar_video:
                rx, ry, rw, rh = roi
                cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
                cv2.putText(frame, f"Frame {frame_idx} | Detecciones: {len(trayectorias)}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                combined = np.hstack([
                    cv2.resize(frame, (self.ancho // 2, self.alto // 2)),
                    cv2.resize(mask_bgr, (self.ancho // 2, self.alto // 2)),
                ])
                cv2.imshow('Seguimiento', combined)

                if cv2.waitKey(delay_ms) & 0xFF == ord('q'):
                    break

            frame_idx += 1

        if trayectorias:
            return np.array(trayectorias)
        else:
            print("\n⚠️  No se detectó la marca dentro del ROI")
            return None

    # ------------------------------------------------------------------
    # Calcular velocidad DIRECTAMENTE de las detecciones
    # ------------------------------------------------------------------
    def calcular_velocidad(self, trayectoria, distancia_real=None):
        if trayectoria is None or len(trayectoria) < 2:
            print(f"No hay suficientes datos. Detectadas: {len(trayectoria) if trayectoria is not None else 0}")
            return None

        # Extraer frames y tiempos
        frames = trayectoria[:, 0].astype(float)
        tiempos = frames / self.fps
        posiciones_x = trayectoria[:, 1].astype(float)
        
        print(f"\n📊 Detecciones: {len(trayectoria)}")
        print(f"   Frames: {frames.astype(int)}")
        print(f"   Tiempos: {tiempos}")
        print(f"   Posiciones X: {posiciones_x.astype(int)}")
        
        # Calcular períodos entre detecciones consecutivas
        periodos = []
        for i in range(len(tiempos) - 1):
            periodo = tiempos[i+1] - tiempos[i]
            periodos.append(periodo)
            print(f"   Período {i+1}: {periodo:.3f} s (frames: {int(frames[i+1]-frames[i])})")
        
        # Filtrar periodos atípicos (opcional)
        if len(periodos) > 2:
            mediana = np.median(periodos)
            periodos_filtrados = [p for p in periodos if p < 2 * mediana and p > mediana / 2]
            if periodos_filtrados:
                periodos = periodos_filtrados
        
        periodo_medio = np.mean(periodos)
        
        # Calcular velocidad
        if distancia_real:
            # Cada detección podría ser medio ciclo o ciclo completo?
            # Pregunta: ¿Cada detección corresponde a un paso por el ROI?
            # Si la marca pasa por el ROI UNA VEZ por vuelta completa:
            distancia_por_deteccion = distancia_real
            
            # Si la marca pasa DOS VECES por vuelta (ida y vuelta):
            # distancia_por_deteccion = distancia_real / 2
            
            velocidad = distancia_por_deteccion / periodo_medio
            #velocidad_kmh = velocidad * 3.6
            
            print(f"\n📈 RESULTADOS:")
            print(f"   Período promedio entre detecciones: {periodo_medio:.3f} s")
            print(f"   Distancia por detección: {distancia_por_deteccion:.2f} m")
            print(f"   Velocidad: {velocidad:.2f} m/s ")
            
            return {
                'velocidad_mps': velocidad,
                'periodo_promedio': periodo_medio,
                'periodos': periodos,
                'frames': tiempos,
                'posiciones': posiciones_x,
                'detecciones': len(trayectoria)
            }
        else:
            print("\n⚠️  Se necesita la distancia real de la cinta")
            return None

    def visualizar_resultados(self, resultados):
        if not resultados:
            return

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle(f'Velocidad Cinta: {resultados["velocidad_mps"]:.2f} m/s', 
                    fontsize=14, fontweight='bold')

        # Gráfico 1: Posición vs Tiempo
        axes[0].plot(resultados['frames'], resultados['posiciones'], 
                    'bo-', linewidth=2, markersize=8, label='Detecciones')
        axes[0].set_xlabel('Tiempo (s)')
        axes[0].set_ylabel('Posición X (píxeles)')
        axes[0].set_title('Detecciones de la Marca')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        # Gráfico 2: Períodos entre detecciones
        axes[1].bar(range(1, len(resultados['periodos'])+1), resultados['periodos'], 
                   color='steelblue', alpha=0.7, edgecolor='black')
        axes[1].axhline(resultados['periodo_promedio'], color='red', 
                       linestyle='--', linewidth=2, 
                       label=f'Promedio: {resultados["periodo_promedio"]:.3f}s')
        axes[1].set_xlabel('Intervalo #')
        axes[1].set_ylabel('Período (s)')
        axes[1].set_title('Períodos Entre Detecciones')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Mostrar estadísticas
        stats_text = f"""
        📊 Estadísticas:
        
        Detecciones: {resultados['detecciones']}
        Período promedio: {resultados['periodo_promedio']:.3f} s
        Desv. estándar: {np.std(resultados['periodos']):.3f} s
        CV: {np.std(resultados['periodos'])/resultados['periodo_promedio']*100:.1f}%
        """
        fig.text(0.02, 0.02, stats_text, fontsize=10, fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        plt.tight_layout()
        plt.show()


# -----------------------------------------------------------------------
def main():
    video_path = r"C:\Users\Rafa\OneDrive - mail.pucv.cl\Escritorio\2.0.mp4"
    fps_video = 120
    distancia_real_cinta = 1.15  # metros por vuelta completa
    
    # IMPORTANTE: ¿Cuántas veces pasa la marca por el ROI por vuelta?
    # Si pasa 1 vez por vuelta: factor = 1
    # Si pasa 2 veces (ida y vuelta): factor = 2
    VECES_POR_VUELTA = 1  # AJUSTA ESTO según tu video
    
    distancia_por_deteccion = distancia_real_cinta / VECES_POR_VUELTA
    
    # ROI fijo (ajusta según tu video)
    ROI_X = 277
    ROI_Y = 656
    ROI_W = 409
    ROI_H = 145
    
    # Parámetros de color
    H_CENTRO = 29
    TOLERANCIA_H = 15
    S_MIN = 100
    V_MIN = 140

    estimador = CintaVelocidadEstimator(video_path, fps_video)
    
    trayectoria = estimador.extraer_trayectoria(
        mostrar_video=True,
        h_centro=H_CENTRO,
        tolerancia_h=TOLERANCIA_H,
        s_min=S_MIN,
        v_min=V_MIN,
        roi_x=ROI_X,
        roi_y=ROI_Y,
        roi_w=ROI_W,
        roi_h=ROI_H
    )

    if trayectoria is not None and len(trayectoria) >= 2:
        print(f"\n✅ Detecciones obtenidas: {len(trayectoria)}")
        print(f"   Frames: {trayectoria[:, 0].astype(int)}")
        print(f"   Posiciones X: {trayectoria[:, 1].astype(int)}")
        
        # Preguntar al usuario
        print(f"\n📝 ¿Cada detección corresponde a cuánto de la vuelta?")
        print(f"   1 vuelta completa = {distancia_real_cinta} m")
        print(f"   Opciones:")
        print(f"   1) Cada detección = 1 vuelta completa (la marca pasa 1 vez por vuelta)")
        print(f"   2) Cada detección = 1/2 vuelta (la marca pasa 2 veces por vuelta)")
        print(f"   3) Cada detección = 1/{VECES_POR_VUELTA} vuelta (configurado: {VECES_POR_VUELTA} veces/vuelta)")
        
        resultados = estimador.calcular_velocidad(trayectoria, distancia_por_deteccion)
        if resultados:
            print(f"\n{'='*50}")
            print(f"🎯 VELOCIDAD: {resultados['velocidad_mps']:.2f} m/s")
            print(f"{'='*50}")
            estimador.visualizar_resultados(resultados)
    else:
        print("\n❌ No se detectaron suficientes puntos")

if __name__ == "__main__":
    main()