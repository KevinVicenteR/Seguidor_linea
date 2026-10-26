'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const D = window.ATLAS_DATOS || {con_red: [], sin_red: [], configuracion: {}, evaluacion: {}, entrenamiento: {}, red: {}, avisos: ['No se encontró datos.js. Ejecuta python3 dashboard/generar_datos.py.']};
  const E = D.evaluacion;
  const C = D.configuracion;
  const numero = (v, dec = 1) => Number.isFinite(v) ? v.toLocaleString('es-ES', {minimumFractionDigits: dec, maximumFractionDigits: dec}) : 'Sin datos';
  const esc = texto => String(texto ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;'}[c]));
  const media = filas => filas.length ? filas.reduce((s, r) => s + r.tiempo_s, 0) / filas.length : null;
  const mapa = filas => filas.filter(r => r.intento > 1 && r.aprobado && Number.isFinite(r.tiempo_s));
  const ayuda = texto => `<button type="button" class="ayuda" aria-label="${esc(texto)}" data-ayuda="${esc(texto)}">?</button>`;
  const vacio = () => '<p class="vacio">No hay datos disponibles para esta comparación.</p>';
  const tabla = (cabeceras, filas) => `<div class="tabla-contenedor"><table><thead><tr>${cabeceras.map(c => `<th scope="col">${c}</th>`).join('')}</tr></thead><tbody>${filas.map(f => `<tr>${f.map(v => `<td>${v}</td>`).join('')}</tr>`).join('')}</tbody></table></div>`;
  const barras = (filas, maximo) => {
    if (!filas.length) return vacio();
    const max = maximo || Math.max(...filas.map(f => f.valor), 1);
    return filas.map(f => `<div class="barra-fila"><div class="barra-cab"><span>${esc(f.nombre)}</span><strong>${esc(f.etiqueta)}</strong></div><div class="carril"><div class="relleno" style="width:${Math.max(0, Math.min(100, f.valor / max * 100))}%;background:var(--${f.color || 'azul'})"></div></div></div>`).join('');
  };
  const preferencia = window.matchMedia('(prefers-color-scheme: dark)');
  let temaGuardado;
  try { temaGuardado = localStorage.getItem('atlas-tema'); } catch {}
  const aplicarTema = tema => {
    document.documentElement.dataset.tema = tema;
    $('tema').textContent = tema === 'claro' ? 'Usar modo oscuro' : 'Usar modo claro';
  };
  aplicarTema(temaGuardado || (preferencia.matches ? 'oscuro' : 'claro'));
  $('tema').addEventListener('click', () => {
    temaGuardado = document.documentElement.dataset.tema === 'claro' ? 'oscuro' : 'claro';
    aplicarTema(temaGuardado);
    try { localStorage.setItem('atlas-tema', temaGuardado); } catch {}
  });
  preferencia.addEventListener('change', e => { if (!temaGuardado) aplicarTema(e.matches ? 'oscuro' : 'claro'); });
  $('avisos').innerHTML = (D.avisos || []).map(a => `<p class="advertencia">${esc(a)}</p>`).join('');
  const aprobadas = D.con_red.filter(r => r.aprobado).length;
  const obligatorias = D.con_red.filter(r => r.obligatorio);
  const fallasObligatorias = obligatorias.filter(r => !r.aprobado).length;
  const nombres = [...new Set(D.con_red.map(r => r.pista))];
  const completo = nombres.length === 13 && nombres.every(n => [1, 2, 3].every(i => D.con_red.some(r => r.pista === n && r.intento === i)));
  const estado = fallasObligatorias ? 'rojo' : completo ? '' : 'amarillo';
  $('estado').innerHTML = `<div class="estado ${estado}"><span class="luz" aria-hidden="true"></span><div><b>${fallasObligatorias ? 'Rojo · Revisar antes de competir' : completo ? 'Verde · Listo para competir en simulación' : 'Amarillo · Faltan pruebas para decidir'}</b><p>${numero(fallasObligatorias, 0)} intentos obligatorios fallidos de ${numero(obligatorias.length, 0)} registrados. ${completo ? 'Confirma el ajuste en pista real.' : 'Regenera el banco completo de 13 pistas.'}</p></div></div>`;
  const errores = E.error_validacion || [];
  const clasico = errores[0]?.rms;
  const nueva = errores[2]?.rms;
  const mejora = Number.isFinite(clasico) && Number.isFinite(nueva) && clasico > 0 ? (1 - nueva / clasico) * 100 : null;
  const metricas = [
    ['Intentos aprobados', `${aprobadas}/${D.con_red.length}`, `${nombres.length} pistas · 3 intentos por pista`, 'Un intento aprobado completó la prueba del simulador. Cada pista tiene tres intentos.'],
    ['Vuelta con mapa', `${numero(media(mapa(D.con_red)))} s`, 'Media de intentos 2 y 3 aprobados', 'Tiempo medio solo de vueltas completadas con mapa, incluidas las pistas extremas.'],
    ['Menos error de posición', `${numero(mejora, 0)} %`, `${numero(clasico, 0)} → ${numero(nueva, 0)} milésimas de sensor`, 'RMS: error típico al estimar la posición de la línea. Se mide con lecturas de validación, no con el error lateral de una vuelta.'],
    ['Turbina', C['impeller.pwm'] === 0 ? 'Apagada' : C['impeller.pwm'] === undefined ? 'Sin datos' : 'Encendida', `PWM ${numero(C['impeller.pwm'], 0)}`, 'La turbina produce succión para aumentar el agarre. En esta rama se corre sin ella.'],
    ['Aceleración máxima', `${numero(C['speed.maxAccelPwmPerS'], 0)}`, 'PWM/s · subida de potencia limitada', 'PWM: potencia del motor de 0 a 255. El límite suaviza cuánto puede subir en un segundo para evitar caballitos.']
  ];
  $('metricas').innerHTML = metricas.map(([nombre, valor, nota, tip]) => `<article class="metrica"><div class="nombre">${esc(nombre)}${ayuda(tip)}</div><div class="valor">${esc(valor)}</div><p>${esc(nota)}</p></article>`).join('');
  const registros = (nombre, modo = 'con_red') => D[modo].filter(r => r.pista === nombre).sort((a,b) => a.intento - b.intento);
  function mostrarPistas() {
    let pistas = nombres.filter(n => {
      const filas = registros(n);
      return $('filtro').value === 'falladas' ? filas.some(r => !r.aprobado) : $('filtro').value === 'obligatorias' ? filas.some(r => r.obligatorio) : true;
    });
    if ($('orden').value === 'tiempo') pistas.sort((a,b) => (media(mapa(registros(a))) ?? Infinity) - (media(mapa(registros(b))) ?? Infinity));
    if ($('orden').value === 'nombre') pistas.sort((a,b) => a.localeCompare(b, 'es'));
    $('lista-pistas').innerHTML = pistas.length ? pistas.map(n => {
      const filas = registros(n);
      const reco = filas.find(r => r.intento === 1);
      const tiempoMapa = media(mapa(filas));
      const tiempoSinRed = media(mapa(registros(n, 'sin_red')));
      const comparables = [];
      if (reco?.aprobado) comparables.push({nombre:'Reconocimiento', valor:reco.tiempo_s, etiqueta:`${numero(reco.tiempo_s)} s`, color:'naranja'});
      if (tiempoMapa !== null) comparables.push({nombre:'Con mapa', valor:tiempoMapa, etiqueta:`${numero(tiempoMapa)} s`});
      return `<button class="pista" data-pista="${esc(n)}"><span class="badge ${filas.every(r => r.aprobado) ? 'bien' : 'mal'}">${filas.filter(r => r.aprobado).length}/3 aprobados</span><h2>${esc(n)}</h2><span class="nota">${filas.some(r => r.obligatorio) ? 'Obligatoria' : 'Extrema · no obligatoria'}</span><div class="intentos">${filas.map(r => `<span class="intento ${r.aprobado ? 'bien' : 'mal'}">${r.intento} · ${r.aprobado ? numero(r.tiempo_s) + ' s' : 'Falló'}</span>`).join('')}</div>${barras(comparables)}<p class="nota">Sin red, con mapa: ${numero(tiempoSinRed)} s · Ver detalle →</p></button>`;
    }).join('') : '<p class="vacio">No hay pistas que coincidan con este filtro.</p>';
    document.querySelectorAll('[data-pista]').forEach(b => b.addEventListener('click', () => mostrarDetalle(b.dataset.pista)));
  }
  function mostrarDetalle(nombre) {
    $('detalle-titulo').textContent = nombre;
    const campos = [
      ['tiempo_s','Tiempo registrado','s','Si el intento falló, este tiempo no es una vuelta completada.'],
      ['vel_media_mps','Velocidad media','m/s','Metros que avanza por segundo, de media.'],
      ['vel_max_mps','Velocidad máxima','m/s','Mayor velocidad registrada durante el intento.'],
      ['error_rms_mm','Error RMS','mm','Error típico entre el robot y la línea durante la vuelta. Menor es mejor.'],
      ['error_max_mm','Error máximo','mm','Mayor distancia registrada respecto a la línea.'],
      ['linea_perdida_pct','Línea perdida','%','Porcentaje del tiempo en que los sensores no encontraron la línea.'],
      ['derrape_pct','Derrape','%','Porcentaje del tiempo con pérdida de agarre en el modelo del simulador.']
    ];
    $('detalle-cuerpo').innerHTML = ['con_red', 'sin_red'].map(modo => `<div class="detalle-modos"><h3>${modo === 'con_red' ? 'Con red neuronal' : 'Sin red neuronal'}</h3><div class="detalle-tarjetas">${registros(nombre, modo).map(r => `<article class="detalle-intento"><h3>Intento ${r.intento}</h3><p>${r.intento === 1 ? 'Reconocimiento' : 'Con mapa'}</p><strong class="${r.aprobado ? 'bien' : 'mal'}">${r.aprobado ? 'Aprobado' : 'Falló'}</strong><dl>${campos.map(([clave, etiqueta, unidad, tip]) => `<dt>${etiqueta}${ayuda(tip)}</dt><dd>${numero(r[clave])} ${unidad}</dd>`).join('')}<dt>Motivo de falla</dt><dd>${esc(r.falla || (r.aprobado ? 'Sin falla' : 'No indicado'))}</dd></dl></article>`).join('') || vacio()}</div></div>`).join('');
    $('detalle').showModal();
  }
  $('cerrar').addEventListener('click', () => $('detalle').close());
  $('detalle').addEventListener('click', e => { if (e.target === $('detalle')) { const r = e.target.getBoundingClientRect(); if(e.clientX < r.left || e.clientX > r.right || e.clientY < r.top || e.clientY > r.bottom) e.target.close(); } });
  $('filtro').addEventListener('change', mostrarPistas);
  $('orden').addEventListener('change', mostrarPistas);
  mostrarPistas();
  function dibujarRed() {
    const R = D.red;
    if (R.Weights1?.length !== 56 || R.Weights2?.length !== 8) { $('diagrama').innerHTML = vacio(); return; }
    const entrada = i => 46 + i * 43;
    const oculta = i => 32 + i * 40;
    let lineas = '';
    const enlace = (x1,y1,x2,y2,peso,max,texto) => `<line class="conexion" tabindex="0" role="button" aria-label="${esc(texto)}" data-info="${esc(texto)}" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="var(--${peso >= 0 ? 'azul' : 'naranja'})" stroke-width="${1.2 + Math.abs(peso)/max*4}"><title>${esc(texto)}</title></line>`;
    for(let h = 0; h < 8; h++) for(let i = 0; i < 7; i++) lineas += enlace(90,entrada(i),350,oculta(h),R.Weights1[h*7+i],Math.max(...R.Weights1.map(Math.abs)),`Sensor ${i+1} → neurona ${h+1}: peso ${R.Weights1[h*7+i]}`);
    for(let h = 0; h < 8; h++) lineas += enlace(350,oculta(h),610,175,R.Weights2[h],Math.max(...R.Weights2.map(Math.abs)),`Neurona ${h+1} → posición: peso ${R.Weights2[h]}`);
    const neurona = (x,y,etiqueta,info) => `<circle class="neurona" tabindex="0" role="button" aria-label="${esc(info)}" data-info="${esc(info)}" cx="${x}" cy="${y}" r="14"><title>${esc(info)}</title></circle><text x="${x}" y="${y+4}" text-anchor="middle" pointer-events="none" style="font-size:9px;fill:var(--texto)">${etiqueta}</text>`;
    let nodos = '';
    for(let i = 0; i < 7; i++) nodos += neurona(90,entrada(i),i+1,`Entrada ${i+1}: lectura de uno de los 7 sensores cercanos a la línea.`);
    for(let h = 0; h < 8; h++) nodos += neurona(350,oculta(h),h+1,`Neurona ${h+1} · ReLU · sesgo ${R.Bias1[h]}. Pesos de entrada: ${R.Weights1.slice(h*7,h*7+7).join(', ')}. Peso de salida: ${R.Weights2[h]}.`);
    nodos += neurona(610,175,'P',`Posición afinada · sesgo de salida ${R.Bias2[0]}`);
    $('diagrama').innerHTML = `<svg viewBox="0 0 700 370" role="group" aria-label="Red neuronal interactiva de siete entradas, ocho neuronas y una salida">${lineas}${nodos}<text x="90" y="355" text-anchor="middle">7 sensores</text><text x="350" y="355" text-anchor="middle">8 neuronas ReLU</text><text x="610" y="355" text-anchor="middle">Posición</text></svg>`;
    document.querySelectorAll('[data-info]').forEach(n => ['pointerenter','focus','click'].forEach(evento => n.addEventListener(evento, () => { $('info-red').textContent = n.dataset.info; })));
  }
  dibujarRed();
  function curva() {
    const T = D.entrenamiento;
    const puntos = T.curva || [];
    if (!puntos.length) { $('curva').innerHTML = vacio(); return; }
    const w = 540, h = 280, izq = 53, der = 20, arriba = 20, abajo = 40;
    const maxX = Math.max(...puntos.map(p => p.epoca), T.mejor_epoca || 0, 1);
    const maxY = Math.max(...puntos.flatMap(p => [p.entrenamiento, p.validacion])) * 1.08;
    const x = n => izq + n / maxX * (w - izq - der);
    const y = n => h - abajo - n / maxY * (h - arriba - abajo);
    let contenido = '';
    for(let i = 0; i <= 4; i++) {
      const v = maxY*i/4;
      contenido += `<line class="eje" x1="${izq}" x2="${w-der}" y1="${y(v)}" y2="${y(v)}"/><text x="${izq-8}" y="${y(v)+4}" text-anchor="end">${numero(v,2)}</text>`;
      const ep = Math.round(maxX*i/4);
      contenido += `<text x="${x(ep)}" y="${h-12}" text-anchor="middle">${ep}</text>`;
    }
    ['entrenamiento','validacion'].forEach((clave,i) => {
      contenido += `<polyline fill="none" stroke="var(--${i ? 'naranja' : 'azul'})" stroke-width="2.5" points="${puntos.map(p => `${x(p.epoca)},${y(p[clave])}`).join(' ')}"/>`;
      contenido += puntos.map(p => `<circle cx="${x(p.epoca)}" cy="${y(p[clave])}" r="4" fill="var(--${i ? 'naranja' : 'azul'})"><title>Época ${p.epoca}, ${clave}: ${numero(p[clave],4)} sensores</title></circle>`).join('');
    });
    if (T.mejor_epoca !== null && Number.isFinite(T.mejor_rms)) contenido += `<line x1="${x(T.mejor_epoca)}" x2="${x(T.mejor_epoca)}" y1="${arriba}" y2="${h-abajo}" stroke="var(--verde)" stroke-dasharray="5 5"/><circle cx="${x(T.mejor_epoca)}" cy="${y(T.mejor_rms)}" r="6" fill="var(--verde)"/><text x="${x(T.mejor_epoca)+9}" y="32" style="fill:var(--verde)">Mejor: ${T.mejor_epoca}</text>`;
    $('curva').innerHTML = `<svg viewBox="0 0 ${w} ${h}" role="img" aria-label="Error de entrenamiento y validación por época; menor es mejor">${contenido}</svg>`;
    $('mejor').textContent = `Mejor época: ${T.mejor_epoca ?? 'sin datos'} · validación ${numero(T.mejor_rms,4)} sensores. El registro guarda puntos cada 50 épocas; la mejor época se marca por separado.`;
  }
  curva();
  $('errores').innerHTML = barras(errores.map((r,i) => ({nombre:r.metodo, valor:r.rms, etiqueta:numero(r.rms,0), color:i === 2 ? 'verde' : i === 1 ? 'azul' : 'naranja'})));
  $('tabla-rms').innerHTML = D.entrenamiento.tabla?.length ? tabla(['Grupo','Clásico RMS','Red decimal','Red entera','Máx. entera'], D.entrenamiento.tabla.map(r => [esc(r.grupo),r.clasico,r.doble,r.enteros,r.maximo])) : '';
  $('agarre').innerHTML = E.lazo_cerrado?.filas?.length ? tabla(['Agarre de llantas','Fallas con red','Fallas sin red','Mapa con / sin red'], E.lazo_cerrado.filas.map(r => [`${r.agarre_pct} %`,`${r.fallas_con_red} / ${E.lazo_cerrado.carreras_por_fila}`,`${r.fallas_sin_red} / ${E.lazo_cerrado.carreras_por_fila}`,`${numero(r.vuelta_mapa_con_red_s,2)} / ${numero(r.vuelta_mapa_sin_red_s,2)} s`])) : vacio();
  const cambios = [
    ['Reconocimiento','170','speed.basePwm','PWM','Aprender con más margen de agarre.'],
    ['Búsqueda de línea','175 / 201','lostLine.outerMotorPwm','PWM','Recuperar la línea sin un giro tan brusco.'],
    ['Velocidad en esquinas','74','corner.speedPercent','%','Entrar más despacio a las esquinas.'],
    ['Rectas con mapa, perfil rápido','255','planner.maxPwm','PWM','Pedir menos potencia sin succión.'],
    ['Giro cerrado, perfil rápido','255','planner.fullTurnPwm','PWM','Mantener el agarre en curvas cerradas.'],
    ['Frenado por tramo del mapa','8 / 5','planner.brakePwmPerBin','PWM','Adaptar la frenada al nuevo perfil.'],
    ['Aceleración máxima','Sin límite','speed.maxAccelPwmPerS','PWM/s','Subir la potencia suavemente para evitar caballitos.'],
    ['Impulso de arranque','Activo','speed.startKickMs','ms','Eliminar el golpe inicial de potencia.'],
    ['Turbina','Activa','impeller.pwm','PWM','Correr sin succión.']
  ];
  $('cambios').innerHTML = tabla(['Ajuste','Con turbina','Actual sin turbina','Para qué'], cambios.map(([n,a,k,u,p]) => [esc(n),esc(a),`<span class="cambio">${numero(C[k],0)} ${u}</span>`,esc(p)]));
  const ajuste = E.ajuste_sin_turbina;
  $('fallas').innerHTML = ajuste ? barras(['antes','despues'].map((k,i) => ({nombre:i ? 'Después del ajuste' : 'Antes del ajuste', valor:ajuste[k].fallas / ajuste[k].carreras * 100, etiqueta:`${ajuste[k].fallas} de ${ajuste[k].carreras} · ${numero(ajuste[k].fallas / ajuste[k].carreras * 100)} %`, color:i ? 'verde' : 'naranja'})), 100) + '<p class="nota">Escala de 0 a 100 % de fallas. Cero fallas se representa sin barra.</p>' : vacio();
  const aceleracion = E.limitador_aceleracion || [];
  const base = aceleracion.find(r => r.pwm_por_s === 0)?.vuelta_mapa_s;
  $('aceleracion').innerHTML = barras(aceleracion.map(r => ({nombre:r.pwm_por_s === 0 ? 'Sin límite' : `${r.pwm_por_s} PWM/s${r.pwm_por_s === C['speed.maxAccelPwmPerS'] ? ' · actual' : ''}`, valor:r.vuelta_mapa_s, etiqueta:`${numero(r.vuelta_mapa_s,2)} s${Number.isFinite(base) ? ' (+' + numero(r.vuelta_mapa_s-base,2) + ' s)' : ''}`, color:r.pwm_por_s === C['speed.maxAccelPwmPerS'] ? 'verde' : 'azul'})));
  $('configuracion').innerHTML = Object.keys(C).length ? tabla(['Parámetro del firmware','Valor'],Object.entries(C).map(([k,v]) => [esc(k),numero(v,0)])) : vacio();
  $('visor').innerHTML = D.visor ? '<a class="enlace" href="../resultados_simulacion/firmware/index.html">Abrir el visor animado del simulador →</a>' : '<p class="nota">El visor estará disponible al generar la simulación con red y actualizar este panel.</p>';
  $('fecha').textContent = D.generado ? `Datos generados: ${new Date(D.generado).toLocaleString('es-ES')}` : 'Datos pendientes';
  const enlaces = [...document.querySelectorAll('nav a')];
  const actualizarNavegacion = () => {
    const hash = location.hash || '#resumen';
    enlaces.forEach(a => { a.classList.toggle('activo', a.hash === hash); if (a.hash === hash) a.setAttribute('aria-current','location'); else a.removeAttribute('aria-current'); });
  };
  window.addEventListener('hashchange', actualizarNavegacion);
  actualizarNavegacion();
})();
