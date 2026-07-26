import serial

with serial.Serial('COM3', 115200, timeout=1) as s, open(r'C:\Users\Rafa\OneDrive - mail.pucv.cl\Escritorio\graficas python\log.csv', 'w') as f:
    s.write(b'LOG VEL START\n')
    s.write(b'SET SP 2.0\n')
    while True:
        line = s.readline().decode(errors='replace')
        f.write(line)
        print(line, end='')
