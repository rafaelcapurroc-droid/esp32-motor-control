import cv2
import numpy as np

VIDEO_PATH = r"C:\Users\Rafa\OneDrive - mail.pucv.cl\Escritorio\1.8.mp4"

cap   = cv2.VideoCapture(VIDEO_PATH)
TOTAL = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
FPS   = cap.get(cv2.CAP_PROP_FPS)

roi_pts    = []
dibujando  = False
roi_final  = None
frame_idx  = 0
frame_base = None
frame_display = None

def cargar_frame(idx):
    global frame_base, frame_display
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
    ret, f = cap.read()
    if ret:
        frame_base    = f.copy()
        frame_display = f.copy()
        dibujar_info()

def dibujar_info():
    global frame_display
    frame_display = frame_base.copy()
    if roi_final:
        rx, ry, rw, rh = roi_final
        cv2.rectangle(frame_display, (rx, ry), (rx+rw, ry+rh), (0,255,0), 2)
    seg = frame_idx / FPS
    txt = (f"Frame {frame_idx}/{TOTAL-1}  t={seg:.1f}s  "
           f"| A/D=+-10  Q/W=+-100  | ENTER=ok  R=reset  ESC=salir")
    cv2.putText(frame_display, txt, (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)
    cv2.putText(frame_display, txt, (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 1)
    cv2.imshow("Selector ROI", frame_display)

def on_mouse(event, x, y, flags, param):
    global roi_pts, dibujando, roi_final
    if event == cv2.EVENT_LBUTTONDOWN:
        roi_pts  = [(x, y)]
        dibujando = True
    elif event == cv2.EVENT_MOUSEMOVE and dibujando:
        tmp = frame_base.copy()
        cv2.rectangle(tmp, roi_pts[0], (x, y), (0,255,0), 2)
        cv2.imshow("Selector ROI", tmp)
    elif event == cv2.EVENT_LBUTTONUP and dibujando:
        dibujando = False
        x0, y0 = roi_pts[0]
        rx, ry = min(x0,x), min(y0,y)
        rw, rh = abs(x-x0), abs(y-y0)
        if rw > 5 and rh > 5:
            roi_final = (rx, ry, rw, rh)
            print(f"\nROI → roi = ({rx}, {ry}, {rw}, {rh})")
        dibujar_info()

cv2.namedWindow("Selector ROI", cv2.WINDOW_NORMAL)
# Ajustar ventana a un tamaño manejable para video vertical
cv2.resizeWindow("Selector ROI", 400, 700)
cv2.setMouseCallback("Selector ROI", on_mouse)
cargar_frame(0)

while True:
    key = cv2.waitKey(30) & 0xFF
    salto = 0
    if   key == ord('a'): salto = -10
    elif key == ord('d'): salto =  10
    elif key == ord('q'): salto = -100
    elif key == ord('w'): salto =  100

    if salto:
        frame_idx = max(0, min(TOTAL-1, frame_idx + salto))
        cargar_frame(frame_idx)
    elif key == 13:  # ENTER
        if roi_final:
            print(f"✅ Confirmado: roi = {roi_final}")
            break
        else:
            print("⚠️  Dibuja el recuadro primero.")
    elif key == ord('r'):
        roi_final = None
        print("ROI borrado.")
        dibujar_info()
    elif key == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()