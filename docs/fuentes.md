# Investigación: PID, turbina y lógica para seguidores rápidos (2026-10-17)

Fuentes consultadas por Claude y por Codex (Codex leyó cada URL marcada; los videos no se pudieron ver).

## PID / PD
- Orden de ajuste P → D → subir velocidad; Ki≈0 salvo sesgo persistente: thinkrobotics (guía), foro Pololu https://forum.pololu.com/t/tuning-line-follower-pid-constants/4372, Reddit https://www.reddit.com/r/arduino/comments/vi58ul/ , https://www.reddit.com/r/robotics/comments/1l16uzs/line_follower_pid_values/
- Comparar contribuciones de P y D (D por muestra vs por tiempo): Oguten y Kabas https://arxiv.org/html/2111.04149v1 — ATLAS usa D por muestra (Δe por ciclo de 750 µs).
- Filtrado de D con Savitzky–Golay reduce picos: https://rcmags.github.io/pdfs/line_follower_report.pdf
- Motoko Uprising: lazo 200 Hz, PD de dirección + red int8 que clasifica 5 tipos de trayectoria: https://hackaday.io/project/163799-motoko-uprising-deep-neural-network-line-following , https://github.com/michalnand/motoko_uprising

## Turbina
- Suckbot (Pololu): fan EDF ~30 000 rpm, levanta ~140 g de 300 g: https://www.pololu.com/blog/491
- Turbina hasta 0.7 N y 5.2 A de consumo: https://github.com/HaroldMurcia/Line_Follower
- Modo de prueba de turbina y telemetría: https://github.com/l1h2/line_follower_stm ; seguidor AVR con QRE1113 y ventilador: https://github.com/rmingon/line-follower
- Arranque previo de la EDF y refuerzo según maniobra (foro, sin ensayo): https://www.forosdeelectronica.com/threads/turbina-edf27.163506/
- NTF (Japón) permite adhesión desde 2023; X-303 con succión, 16 sensores, giroscopio y encoders: https://www.ntf.or.jp/?page_id=985 , https://www.ntf.or.jp/mouse/micromouse2025/Robot/AllJapan/TechRT62.html
- Ajustar el PID después de fijar la succión (cambia la dinámica).

## Lógica
- Reglamento CENIDET (líneas 19 mm, discontinuidades, esquinas, zigzag, cruces): https://robotcitca.cenidet.tecnm.mx/docs/reglamento_seguidor_linea.pdf
- RobotChallenge Enhanced (hueco de 10 cm en recta): https://robotchallenge.org.cn/doc/en/RC-LineFollowerEnhanced.pdf
- Estados de giro con el último sensor extremo para esquinas: https://arxiv.org/html/2111.04149v1
- Anticipación con mapa requiere localización fiable (encoders/IMU en line_follower_stm).

## IA / aprendizaje
- Q-learning en ATmega64 físico: +0.51 % sobre control P: https://arxiv.org/pdf/2001.08841
- Mezcla de expertos: Mixture of Controllers (NIPS 1993) https://proceedings.nips.cc/paper_files/paper/1993/file/b137fdd1f79d56c7edf3365fea7520f2-Paper.pdf ; MPC-Net (sigmoide + normalización mejor que softmax en la compuerta) https://arxiv.org/pdf/1909.05197
- Aleatorización de dominio para pasar de simulación a robot: https://arxiv.org/abs/2003.02471v4 , https://paperswithcode.com/paper/auto-tuned-sim-to-real-transfer
- CMA-ES: sin evidencia publicada en velocistas físicos de esta clase (Codex).

## Causas habituales del zigzag
- Distancia de la barra (18 cm: 1° de giro ≈ 3.1 mm en el sensor), retraso total sensor→PWM→rueda, saturación de la corrección, ruido en D, agarre/centro de masa/caída de batería al encender la EDF.
