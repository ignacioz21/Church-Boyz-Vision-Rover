// Mapa de rutas, compartido por index.html y rutas.html
const COLORS = { red:'#dc2626', green:'#16a34a', blue:'#2563eb' };
const TASK = { r:'rojo', g:'verde', b:'azul' };
const ROVER_COLORS = { 10:'#d97706', 11:'#7c3aed' };
// Huella del robot en celdas, medida desde el marcador (ver rover_*/include/config.h)
const AXLE = 1.8, REAR = 1.9, FRONT = 4.9, HALF = 3.25, TIP = 7.8, PRONG = 2.9;

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

