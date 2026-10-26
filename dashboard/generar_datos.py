import csv
import json
import re
from datetime import datetime, timezone
from pathlib import Path

RAIZ = Path(__file__).resolve().parents[1]
AVISOS = []


def leer(ruta, comando):
    archivo = RAIZ / ruta
    if not archivo.exists():
        aviso = f'Falta {ruta}. Para generarlo: {comando}'
        AVISOS.append(aviso)
        print(aviso)
        return ''
    return archivo.read_text(encoding='utf-8-sig')


def carreras(carpeta, opciones=''):
    contenido = leer(f'resultados_simulacion/{carpeta}/resumen.csv',
                     f'./build/simulacion/atlas_sim --firmware {opciones}--out resultados_simulacion/{carpeta}')
    filas = []
    for fila in csv.DictReader(contenido.splitlines()):
        for clave, valor in list(fila.items()):
            if clave in ('obligatorio', 'aprobado'):
                fila[clave] = valor.lower() in ('si', 'sí', 'true', '1')
            elif clave not in ('escenario', 'pista', 'linea', 'falla'):
                try:
                    fila[clave] = float(valor)
                except (ValueError, TypeError):
                    fila[clave] = None
        intento = re.search(r'Intento\s+(\d+)', fila['escenario'])
        fila['intento'] = int(intento.group(1)) if intento else None
        filas.append(fila)
    return filas


def generar():
    con_red = carreras('firmware')
    sin_red = carreras('firmware_sin_red', '--sin-red ')
    log = leer('entrenamiento/resultados/entrenamiento_posicion.log',
               './build/simulacion/atlas_train_nn posicion > entrenamiento/resultados/entrenamiento_posicion.log')
    curva = [{'epoca': int(n), 'entrenamiento': float(x), 'validacion': float(y)}
             for n, x, y in re.findall(r'época\s+(\d+)\s+RMS entrenamiento\s+([\d.]+)\s+validación\s+([\d.]+)\s+sensores', log)]
    mejor = re.search(r'Mejor época\s+(\d+):\s+RMS de validación\s+([\d.]+)', log)
    tabla = [{'grupo': g, 'clasico': int(c), 'doble': int(d), 'enteros': int(e), 'maximo': int(m)}
             for g, c, d, e, m in re.findall(r'^\s*(entrenamiento|validación)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)', log, re.M)]
    evaluacion = leer('entrenamiento/resultados/evaluacion_red_posicion.json',
                      'consulta entrenamiento/README.md para regenerar la evaluación comparativa y guardar evaluacion_red_posicion.json')
    try:
        evaluacion = json.loads(evaluacion) if evaluacion else {}
    except json.JSONDecodeError as error:
        AVISOS.append(f'Evaluación JSON no válida: {error}')
        evaluacion = {}
    pesos = leer('firmware/bibliotecas/AtlasCore/src/red_neuronal/NeuralPositionWeights.h',
                 './build/simulacion/atlas_train_nn posicion')
    red = {}
    for nombre in ('Weights1', 'Bias1', 'Weights2', 'Bias2'):
        bloque = re.search(r'kPosition' + nombre + r'\s*\[[^\]]+\]\s*=\s*\{([^}]+)\}', pesos, re.S)
        red[nombre] = [int(v) for v in re.findall(r'-?\d+', bloque.group(1))] if bloque else []
    config = leer('firmware/bibliotecas/AtlasCore/src/configuracion/FirmwareConfig.h',
                  'restaura FirmwareConfig.h desde las fuentes del proyecto')
    ajustes = {clave: float(valor) for clave, valor in re.findall(r'config\.(\w+\.\w+)\s*=\s*(-?\d+(?:\.\d+)?)\s*;', config)}
    datos = {'generado': datetime.now(timezone.utc).isoformat(), 'con_red': con_red,
             'sin_red': sin_red, 'entrenamiento': {'curva': curva, 'tabla': tabla,
             'mejor_epoca': int(mejor.group(1)) if mejor else None,
             'mejor_rms': float(mejor.group(2)) if mejor else None},
             'evaluacion': evaluacion, 'red': red, 'configuracion': ajustes,
             'visor': (RAIZ / 'resultados_simulacion/firmware/index.html').exists(), 'avisos': AVISOS}
    destino = Path(__file__).resolve().with_name('datos.js')
    destino.write_text('window.ATLAS_DATOS = ' + json.dumps(datos, ensure_ascii=False, allow_nan=False, indent=2) + ';\n', encoding='utf-8')
    print(f'Generado {destino.name}: {len(con_red)} intentos con red, {len(sin_red)} sin red, {len(curva)} épocas registradas. Avisos: {len(AVISOS)}.')


if __name__ == '__main__':
    generar()
