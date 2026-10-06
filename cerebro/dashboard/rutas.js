// Mapa de rutas, compartido por index.html y rutas.html
const COLORS = { red:'#dc2626', green:'#16a34a', blue:'#2563eb' };
const TASK = { r:'rojo', g:'verde', b:'azul' };
const ROVER_COLORS = { 10:'#d97706', 11:'#7c3aed' };
// Huella del robot en celdas, medida desde el marcador (ver rover_*/include/config.h)
const AXLE = 1.8, REAR = 1.9, FRONT = 4.9, HALF = 3.25, TIP = 7.8, PRONG = 2.9;

// Disposición de práctica generada en el cerebro (null = ninguna). La fija index.html.
let shadowLayout = null;
const PLACED_CUBE = 1.0, PLACED_ROVER = 2.5;       // Celdas: qué tan cerca cuenta como "en su lugar"

// Dibuja las sombras: dónde poner cada cubo y cada rover. Devuelve cuántos hay en su lugar.
function drawShadows(ctx, w, s) {
  if (!shadowLayout) return null;
  const X = c => c * s, Y = r => r * s, half = w.cube_side / 2;
  const state = { cubes: {}, rovers: {}, ready: true };
  ctx.save();
  ctx.lineWidth = 2.5;
  for (const [color, p] of Object.entries(shadowLayout.cubos)) {
    const real = w.cubes.find(c => c.color === color);
    const ok = !!real && Math.hypot(real.col - p[0], real.row - p[1]) <= PLACED_CUBE;
    state.cubes[color] = ok; state.ready = state.ready && ok;
    ctx.setLineDash(ok ? [] : [6, 5]);
    ctx.strokeStyle = COLORS[color]; ctx.fillStyle = COLORS[color] + (ok ? '00' : '2a');
    ctx.fillRect(X(p[0] - half), Y(p[1] - half), w.cube_side * s, w.cube_side * s);
    ctx.strokeRect(X(p[0] - half) - 3, Y(p[1] - half) - 3, w.cube_side * s + 6, w.cube_side * s + 6);
    if (ok) { ctx.setLineDash([]); ctx.fillStyle = '#15803d'; ctx.font = 'bold 15px system-ui'; ctx.fillText('✓', X(p[0] + half) + 5, Y(p[1] - half) + 4); }
  }
  for (const [id, p] of Object.entries(shadowLayout.rovers)) {
    const real = w.rovers.find(r => String(r.id) === String(id));
    const ok = !!real && Math.hypot(real.col - p[0], real.row - p[1]) <= PLACED_ROVER;
    state.rovers[id] = ok; state.ready = state.ready && ok;
    ctx.setLineDash(ok ? [] : [6, 5]);
    ctx.strokeStyle = ROVER_COLORS[id] || '#111';
    ctx.strokeRect(X(p[0] - AXLE - REAR), Y(p[1] - HALF), (REAR + FRONT) * s, 2 * HALF * s);
    ctx.setLineDash([]); ctx.fillStyle = ctx.strokeStyle; ctx.font = '12px system-ui';
    ctx.fillText(ok ? id + ' ✓' : id, X(p[0] - AXLE - REAR), Y(p[1] - HALF) - 4);
  }
  ctx.restore();
  return state;
}

// Plan de la fase CEREBRO (null = ninguno): lo que la PC calculó y cargó antes de arrancar.
// Lo fija la página con el evento 'cerebro'.
let cerebroPlan = null;

// Dibuja las rutas planificadas: línea llena para el primer cubo de cada rover y
// punteada para el comodín (el que toma quien quede libre primero).
function drawPlanned(ctx, w, s, resting) {
  if (!cerebroPlan || !cerebroPlan.dibujo) return;
  const X = c => c * s, Y = r => r * s;
  ctx.save();
  for (const [id, routes] of Object.entries(cerebroPlan.dibujo)) {
    if (!resting(Number(id))) continue;             // Ya en marcha: vale la ruta que reporta el rover
    routes.forEach((route, i) => {
      ctx.strokeStyle = ROVER_COLORS[id] || '#111';
      ctx.globalAlpha = i === 0 ? 0.9 : 0.5;
      ctx.lineWidth = i === 0 ? 3 : 2;
      ctx.setLineDash(route.comodin ? [3, 6] : []);
      ctx.beginPath();
      route.puntos.forEach((p, k) => k ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])));
      ctx.stroke();
      // Dónde toma el cubo: un aro del color del cubo
      const cube = w.cubes.find(c => c.color === route.color);
      if (cube) {
        ctx.setLineDash([]); ctx.lineWidth = 2; ctx.strokeStyle = ROVER_COLORS[id] || '#111';
        ctx.beginPath(); ctx.arc(X(cube.col), Y(cube.row), (2.6 + 0.5 * i) * s, 0, 7); ctx.stroke();
      }
    });
  }
  ctx.restore();
}

// Lugares donde cada rover avisó de un giro trabado (se acumulan mientras la página esté abierta)
const obstacleMarks = {};

// Dibuja en 'cv' la cancha, los cubos, los rovers con su forma y la ruta que cada
// uno reporta. 'online(id)' devuelve el último estado del rover, o null si no reporta.
function drawRoutesMap(cv, w, online) {
  const ctx = cv.getContext('2d');
  const s = cv.width / Math.max(w.grid.cols, w.grid.rows);
  const X = c => c * s, Y = r => r * s;
  ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.setLineDash([]); ctx.lineWidth = 1; ctx.globalAlpha = 1;

  ctx.strokeStyle = '#e5e7eb';
  for (let i = 0; i <= w.grid.cols; i += 5) { ctx.beginPath(); ctx.moveTo(X(i), 0); ctx.lineTo(X(i), Y(w.grid.rows)); ctx.stroke(); }
  for (let i = 0; i <= w.grid.rows; i += 5) { ctx.beginPath(); ctx.moveTo(0, Y(i)); ctx.lineTo(X(w.grid.cols), Y(i)); ctx.stroke(); }

  // Zonas: el lado largo apoya en el borde más cercano a su centro
  const L = w.depot_size.length, D = w.depot_size.depth;
  for (const d of w.depots) {
    const toV = Math.min(d.col, w.grid.cols - d.col), toH = Math.min(d.row, w.grid.rows - d.row);
    const [ww, hh] = toH < toV ? [L, D] : [D, L];
    ctx.fillStyle = COLORS[d.color] + '22'; ctx.strokeStyle = COLORS[d.color];
    ctx.fillRect(X(d.col - ww/2), Y(d.row - hh/2), ww*s, hh*s);
    ctx.strokeRect(X(d.col - ww/2), Y(d.row - hh/2), ww*s, hh*s);
  }

  const half = w.cube_side / 2;
  for (const c of w.cubes) {
    ctx.globalAlpha = c.age_ms > 500 ? 0.4 : 1;
    ctx.fillStyle = COLORS[c.color] || '#000';
    ctx.fillRect(X(c.col - half), Y(c.row - half), w.cube_side*s, w.cube_side*s);
  }
  ctx.globalAlpha = 1;

  drawShadows(ctx, w, s);
  drawPlanned(ctx, w, s, id => { const st = online(id); return st && (st.state === 'IDLE' || st.state === 'FINISHED'); });

  // Rutas, debajo de los rovers
  for (const r of w.rovers) {
    const st = online(r.id);
    if (!st || !st.route || !st.route.length) continue;
    const color = ROVER_COLORS[r.id] || '#111';
    const pts = [[r.col, r.row], ...st.route];
    ctx.strokeStyle = ctx.fillStyle = color;
    ctx.lineWidth = 3; ctx.setLineDash([10, 7]);
    ctx.beginPath(); ctx.moveTo(X(pts[0][0]), Y(pts[0][1]));
    for (const p of pts.slice(1)) ctx.lineTo(X(p[0]), Y(p[1]));
    ctx.stroke(); ctx.setLineDash([]);
    // Puntos intermedios numerados
    ctx.font = 'bold 13px system-ui';
    st.route.forEach((p, i) => {
      ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 5, 0, 7); ctx.fill();
      ctx.fillText(i + 1, X(p[0]) + 8, Y(p[1]) - 8);
    });
    // Cubo que va a tomar (círculo) y destino final (cruz)
    const cube = w.cubes.find(c => c.color[0] === st.task);
    if (cube) { ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(X(cube.col), Y(cube.row), 3.2*s, 0, 7); ctx.stroke(); }
    const end = pts[pts.length - 1], k = 1.4 * s;
    ctx.lineWidth = 4; ctx.beginPath();
    ctx.moveTo(X(end[0]) - k, Y(end[1]) - k); ctx.lineTo(X(end[0]) + k, Y(end[1]) + k);
    ctx.moveTo(X(end[0]) + k, Y(end[1]) - k); ctx.lineTo(X(end[0]) - k, Y(end[1]) + k); ctx.stroke();
  }

  // Obstáculos: dónde se trabó un giro (lo reporta el rover con su giroscopio)
  for (const id of [10, 11]) {
    const st = online(id);
    if (!st || !st.ob) continue;
    const seen = obstacleMarks[id] || (obstacleMarks[id] = { count: 0, pts: [] });
    if (st.ob > seen.count) { seen.count = st.ob; seen.pts.push(st.obp); }
    if (st.ob < seen.count) { seen.count = st.ob; seen.pts = []; }      // El rover se reinició
    for (const p of seen.pts) {
      const k = 1.3 * s;
      ctx.fillStyle = '#facc15'; ctx.strokeStyle = '#b91c1c'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(X(p[0]), Y(p[1]) - k); ctx.lineTo(X(p[0]) + k, Y(p[1]) + k);
      ctx.lineTo(X(p[0]) - k, Y(p[1]) + k); ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#b91c1c'; ctx.font = 'bold 14px system-ui'; ctx.fillText('!', X(p[0]) - 2, Y(p[1]) + k - 3);
    }
  }

  // Rovers con su forma (cuerpo y pinzas)
  for (const r of w.rovers) {
    const a = -r.theta * Math.PI / 180;       // theta antihorario, row hacia abajo
    const color = ROVER_COLORS[r.id] || '#111';
    ctx.save();
    ctx.translate(X(r.col), Y(r.row)); ctx.rotate(a); ctx.translate(-AXLE * s, 0);   // Origen en el eje
    ctx.globalAlpha = r.age_ms > 300 ? 0.35 : 1;
    ctx.fillStyle = color + '33'; ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.fillRect(-REAR*s, -HALF*s, (REAR + FRONT)*s, 2*HALF*s);
    ctx.strokeRect(-REAR*s, -HALF*s, (REAR + FRONT)*s, 2*HALF*s);
    ctx.lineWidth = 4; ctx.beginPath();
    ctx.moveTo(FRONT*s, -PRONG*s); ctx.lineTo(TIP*s, -PRONG*s);
    ctx.moveTo(FRONT*s, PRONG*s); ctx.lineTo(TIP*s, PRONG*s); ctx.stroke();
    ctx.restore();
    ctx.globalAlpha = 1; ctx.fillStyle = color; ctx.font = 'bold 16px system-ui';
    ctx.fillText(r.id, X(r.col) - 9, Y(r.row) + 6);
  }
}

