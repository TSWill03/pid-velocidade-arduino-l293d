#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Grava linhas CSV vindas da Serial (Arduino) em um arquivo .csv.
Espera (opcionalmente) pelo cabeçalho: t_ms,set_rpm,rpm,pwm,dist_cm

Uso:
  python scripts/log_serial.py --port COM3 --baud 115200 --outfile logs/teste.csv --echo
  python scripts/log_serial.py --port /dev/ttyACM0 --outfile logs/run.csv
"""
import argparse
import csv
import os
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    print("Erro: pyserial não encontrado. Instale com: pip install -r requirements.txt", file=sys.stderr)
    raise

HEADER = "t_ms,set_rpm,rpm,pwm,dist_cm"

def list_available_ports():
    return list(list_ports.comports())

def pick_port(port):
    if port:
        return port
    ports = list_available_ports()
    if not ports:
        print("Nenhuma porta serial encontrada. Conecte o Arduino e tente novamente.", file=sys.stderr)
        sys.exit(2)
    print("Portas encontradas:")
    for p in ports:
        print(f" - {p.device} ({p.description})")
    print("\nInforme a porta com --port (ex.: COM3, /dev/ttyACM0).")
    sys.exit(2)

def open_serial(dev, baud, timeout):
    ser = serial.Serial(dev, baudrate=baud, timeout=timeout)
    # Arduino UNO costuma resetar ao abrir a porta — aguarde estabilizar
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(2.0)
    return ser

def wait_header(ser, expected, wait_s):
    t0 = time.time()
    expected_l = expected.lower()
    while time.time() - t0 < wait_s:
        raw = ser.readline()
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", "ignore").strip()
        except Exception:
            continue
        if not line:
            continue
        if line.lower().startswith(expected_l):
            return line
    return None

def main():
    ap = argparse.ArgumentParser(description="Grava CSV da porta serial (Arduino).")
    ap.add_argument("-p", "--port", help="Porta serial (ex.: COM3, /dev/ttyACM0).")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="Baudrate (padrão: 115200).")
    ap.add_argument("-o", "--outfile", default="logs/log.csv", help="Arquivo CSV de saída (padrão: logs/log.csv).")
    ap.add_argument("--no-header", action="store_true", help="Não esperar cabeçalho; grava linhas diretamente.")
    ap.add_argument("--echo", action="store_true", help="Imprime as linhas no console enquanto grava.")
    ap.add_argument("--timeout", type=float, default=1.0, help="Timeout de leitura serial em segundos (padrão: 1.0).")
    ap.add_argument("--wait-header", type=float, default=8.0, help="Tempo máximo aguardando cabeçalho (s).")
    args = ap.parse_args()

    port = pick_port(args.port)
    ser = open_serial(port, args.baud, args.timeout)

    # Garante diretório de saída
    outdir = os.path.dirname(args.outfile) or "."
    os.makedirs(outdir, exist_ok=True)

    # Decide se precisa escrever cabeçalho no arquivo
    write_header = not (os.path.exists(args.outfile) and os.path.getsize(args.outfile) > 0)

    if not args.no_header:
        got = wait_header(ser, HEADER, args.wait_header)
        if got is None:
            print(f"Aviso: cabeçalho '{HEADER}' não recebido em {args.wait_header}s. Prosseguindo mesmo assim.",
                  file=sys.stderr)
        else:
            print(f"Recebido cabeçalho: {got}")

    print(f"Gravando em {args.outfile} (porta={port}, {args.baud} bps). Ctrl+C para parar.")
    lines = 0

    with open(args.outfile, "a", newline="") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(HEADER.split(","))

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    s = raw.decode("utf-8", "ignore").strip()
                except Exception:
                    continue
                if not s:
                    continue

                # Ignora cabeçalhos repetidos eventualmente enviados pelo Arduino
                if s.lower().startswith(HEADER.lower()):
                    continue

                parts = [col.strip() for col in s.split(",")]
                if len(parts) != 5:
                    # Linha inesperada (mensagens de debug etc.) — pode logar ou ignorar
                    continue

                writer.writerow(parts)
                lines += 1
                if args.echo:
                    print(s)
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
            print(f"\nFinalizado. Linhas gravadas: {lines}")

if __name__ == "__main__":
    main()
