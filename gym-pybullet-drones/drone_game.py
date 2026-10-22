#!/usr/bin/env python3
"""
SKY RUNNER - 3D drone game in ONE file.
No pybullet, no pygame, no pip installs: only Python's standard library.
The 3D projection, drawing, flight physics, collisions and game rules are all
written in this file.

RUN:   python drone_game.py

KEYS:  W/S fly forward/back   A/D turn   Q/E slide left/right
       SPACE up   SHIFT down   B boost   V camera (chase / drone-eye)
       P pause    ENTER start / next level / retry    ESC quit
GOAL:  fly through every glowing gate before the timer ends. Green cells give
       battery, red orbs and buildings hurt. Every level is harder and the
       scenery changes: Day -> Sunset -> Night.
"""
import tkinter as tk, math, random, time

W, H, F = 960, 600, 560
C, N = 34, 7                      # city cell size, cells per side
HALF = N * C / 2
TAU = math.tau
THEMES = [  # name, sky top, sky horizon, ground, building colours, accent
    ("DAY",    (60, 130, 230), (190, 225, 255), (80, 86, 96),  [(150, 160, 175), (125, 145, 170), (175, 168, 155)], (0, 200, 255)),
    ("SUNSET", (45, 40, 110),  (255, 150, 95),  (66, 56, 66),  [(160, 112, 108), (128, 102, 138), (168, 124, 92)], (255, 130, 60)),
    ("NIGHT",  (4, 6, 28),     (32, 38, 98),    (26, 28, 42),  [(52, 62, 96), (64, 52, 100), (46, 74, 90)],       (255, 70, 220)),
]


def mix(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def hx(c):
    return "#%02x%02x%02x" % tuple(int(max(0, min(255, v))) for v in c)


def clamp(x, a, b):
    return max(a, min(b, x))


class Game:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("SKY RUNNER - Drone Game")
        self.cv = tk.Canvas(self.root, width=W, height=H, highlightthickness=0, bg="black")
        self.cv.pack()
        self.keys, self.pend = set(), {}
        self.root.bind("<KeyPress>", self.kd)
        self.root.bind("<KeyRelease>", self.ku)
        self.state, self.level, self.score, self.best = "menu", 1, 0, 0
        self.clock, self.fpv, self.msg, self.msgt, self.why = 0.0, False, "", 0, ""
        self.cam = dict(x=0, y=20, z=0, yaw=0.0, pitch=0.0)
        self.D = None
        self.gen(1)
        self.state = "menu"
        self.last = time.time()
        self.loop()

    # ---------------- input ----------------
    def kd(self, e):
        k = e.keysym.lower()
        if k in self.pend:
            self.root.after_cancel(self.pend.pop(k))
        if k not in self.keys:
            self.once(k)
        self.keys.add(k)

    def ku(self, e):  # small delay hides the key-repeat flicker on Linux
        k = e.keysym.lower()
        self.pend[k] = self.root.after(45, lambda: (self.keys.discard(k), self.pend.pop(k, None)))

    def held(self, *names):
        return any(n in self.keys for n in names)

    def once(self, k):
        if k == "escape":
            self.root.destroy()
        elif k == "return":
            if self.state in ("menu", "over"):
                self.score = 0
                self.D = None
                self.gen(1)
            elif self.state == "clear":
                self.gen(self.level + 1)
        elif k == "p" and self.state in ("play", "pause"):
            self.state = "pause" if self.state == "play" else "play"
        elif k == "v":
            self.fpv = not self.fpv

    # ---------------- level generation ----------------
    def gen(self, n):
        self.level, self.th = n, THEMES[(n - 1) % 3]
        random.seed(n * 977 + 5)
        hmax = min(24 + n * 6, 65)
        self.blds = []
        for a in range(N):
            for b in range(N):
                park = random.random() < 0.12
                self.blds.append(dict(x=(a - N // 2) * C, z=(b - N // 2) * C,
                                      w=0 if park else 6 + random.random() * 4, d=6 + random.random() * 4,
                                      h=0 if park else 8 + random.random() ** 1.4 * (hmax - 8),
                                      c=self.th[4][(a + 2 * b) % 3], park=park))
        street = lambda: (random.randint(0, N - 2) - N // 2) * C + C / 2
        px, pz, total, self.gates = C / 2, C / 2, 0, []
        for _ in range(min(4 + n * 2, 14)):
            for _t in range(40):
                ax, s, o = random.random() < 0.5, street(), random.uniform(-HALF + 15, HALF - 15)
                g = dict(ax=ax, x=o if ax else s, z=s if ax else o, y=5 + random.random() * min(12 + n * 2, 24))
                d = math.hypot(g["x"] - px, g["z"] - pz)
                if 38 < d < 75 + n * 6:
                    break
            total += d
            px, pz = g["x"], g["z"]
            self.gates.append(g)
        self.picks = []
        for _ in range(len(self.gates) // 2 + 1):
            ax, s, o = random.random() < 0.5, street(), random.uniform(-HALF + 15, HALF - 15)
            self.picks.append(dict(x=o if ax else s, z=s if ax else o, y=4 + random.random() * 12, got=False))
        self.obs = []
        for _ in range(min(n, 6)):
            self.obs.append(dict(ax=random.random() < 0.5, s=street(), o=random.uniform(-30, 30), L=30 + random.random() * 40,
                                 y=6 + random.random() * 14, sp=0.25 + n * 0.04, ph=random.random() * TAU, x=0, z=0))
        keep = self.D["hull"] if self.D else 75
        self.D = dict(x=C / 2, y=1.2, z=C / 2, vx=0, vy=0, vz=0, yaw=0.0, yr=0.0, pitch=0.0, roll=0.0,
                      spin=0.0, hull=min(100, keep + 25), bat=100.0)
        self.gi, self.tl, self.cd, self.combo, self.combot, self.shake = 0, 22 + total / 8.5, 3.4, 0, 0, 0
        self.sparks = []
        self.cam["yaw"] = 0.0
        self.say("LEVEL %d - %s" % (n, self.th[0]), 2.5)
        self.state = "count"

    def say(self, t, s):
        self.msg, self.msgt = t, s

    # ---------------- 3D maths (all hand-written) ----------------
    def basis(self):
        c = self.cam
        cy, sy, cp, sp = math.cos(c["yaw"]), math.sin(c["yaw"]), math.cos(c["pitch"]), math.sin(c["pitch"])
        f, r = (sy * cp, sp, cy * cp), (cy, 0, -sy)
        u = (f[1] * r[2] - f[2] * r[1], f[2] * r[0] - f[0] * r[2], f[0] * r[1] - f[1] * r[0])
        self.B = (r, u, f)

    def tocam(self, p):
        c, (r, u, f) = self.cam, self.B
        d = (p[0] - c["x"], p[1] - c["y"], p[2] - c["z"])
        return (d[0] * r[0] + d[2] * r[2], d[0] * u[0] + d[1] * u[1] + d[2] * u[2],
                d[0] * f[0] + d[1] * f[1] + d[2] * f[2])

    def scr(self, p):
        q = self.tocam(p)
        return None if q[2] < 0.3 else (W / 2 + q[0] / q[2] * F, H / 2 - q[1] / q[2] * F, q[2])

    def poly(self, pts, fill, outline=""):
        q = [self.tocam(p) for p in pts]
        out = []
        for i in range(len(q)):
            a, b = q[i], q[(i + 1) % len(q)]
            if a[2] > 0.2:
                out.append(a)
            if (a[2] > 0.2) != (b[2] > 0.2):
                t = (0.2 - a[2]) / (b[2] - a[2])
                out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, 0.2))
        if len(out) < 3:
            return
        flat = []
        for x, y, z in out:
            flat += [W / 2 + x / z * F, H / 2 - y / z * F]
        self.cv.create_polygon(flat, fill=fill, outline=outline)

    def fog(self, c, d):
        return mix(c, self.th[2], clamp((d - 25) / 170, 0, 1) ** 1.2)

    def dist(self, x, z):
        return math.hypot(x - self.cam["x"], z - self.cam["z"])

    def box(self, cx, cy, cz, hxs, hys, hzs, col, tf=None, fog=True, edge=""):
        V = []
        for i in range(8):
            l = ((hxs if i & 4 else -hxs), (hys if i & 2 else -hys), (hzs if i & 1 else -hzs))
            V.append(tf(l) if tf else (cx + l[0], cy + l[1], cz + l[2]))
        faces = ((2, 3, 7, 6, 1), (0, 4, 5, 1, .35), (4, 5, 7, 6, .75), (0, 1, 3, 2, .55), (1, 5, 7, 3, .85), (0, 2, 6, 4, .5))
        cam, L = self.cam, []
        for f in faces:
            P = [V[i] for i in f[:4]]
            m = [sum(p[k] for p in P) / 4 for k in range(3)]
            L.append((math.dist(m, (cam["x"], cam["y"], cam["z"])), P, f[4]))
        for d, P, s in sorted(L, key=lambda t: -t[0]):
            c = tuple(v * (0.3 + 0.7 * s) for v in col)
            self.poly(P, hx(self.fog(c, d) if fog else c), edge if s == 1 else "")

    # ---------------- game update ----------------
    def hurt(self, d, why="HULL DESTROYED"):
        D = self.D
        D["hull"] -= d
        self.shake = min(1, self.shake + d / 25)
        for _ in range(12):
            self.sparks.append([D["x"], D["y"], D["z"], random.uniform(-5, 5), random.uniform(0, 6), random.uniform(-5, 5), 0.6])
        if D["hull"] <= 0:
            D["hull"] = 0
            self.finish(why)

    def finish(self, why):
        self.state, self.why = "over", why
        self.best = max(self.best, self.score)

    def update(self, dt):
        self.clock += dt
        D = self.D
        D["spin"] += dt * 60
        self.msgt -= dt
        self.shake *= 0.02 ** dt
        for o in self.obs:
            v = -o["L"] / 2 + o["L"] * (0.5 + 0.5 * math.sin(self.clock * o["sp"] * 6 + o["ph"]))
            o["x"], o["z"] = (o["o"] + v, o["s"]) if o["ax"] else (o["s"], o["o"] + v)
        for s in self.sparks:
            s[0] += s[3] * dt; s[1] += s[4] * dt; s[2] += s[5] * dt; s[4] -= 14 * dt; s[6] -= dt
        self.sparks = [s for s in self.sparks if s[6] > 0]
        if self.state == "count":
            self.cd -= dt
            D["y"] = 1.2 + math.sin(self.clock * 3) * 0.05
            if self.cd <= 0:
                self.state = "play"
                self.say("GO!", 1)
            return
        if self.state != "play":
            return
        fa = self.held("w", "up") - self.held("s", "down")
        ta = self.held("d", "right") - self.held("a", "left")
        sa = self.held("e") - self.held("q")
        va = self.held("space") - self.held("shift_l", "shift_r")
        boost = 1.6 if self.held("b") else 1.0
        D["yr"] += (ta * 1.9 - D["yr"]) * min(1, 6 * dt)
        D["yaw"] += D["yr"] * dt
        sy, cy = math.sin(D["yaw"]), math.cos(D["yaw"])
        k = min(1, 2.4 * dt)
        D["vx"] += (16 * boost * fa * sy + 10 * sa * cy - D["vx"]) * k
        D["vz"] += (16 * boost * fa * cy - 10 * sa * sy - D["vz"]) * k
        D["vy"] += (va * 8 - D["vy"]) * min(1, 3.5 * dt)
        ox, oy, oz = D["x"], D["y"], D["z"]
        D["x"] += D["vx"] * dt; D["y"] += D["vy"] * dt; D["z"] += D["vz"] * dt
        lf, lr = D["vx"] * sy + D["vz"] * cy, D["vx"] * cy - D["vz"] * sy
        D["pitch"] += (lf / 16 * 0.35 - D["pitch"]) * min(1, 6 * dt)
        D["roll"] += ((-lr / 10 * 0.35 - D["yr"] * 0.25) - D["roll"]) * min(1, 6 * dt)
        speed = math.hypot(D["vx"], D["vy"], D["vz"])
        D["bat"] -= dt * (0.9 + 0.7 * speed / 16 + (2.2 if boost > 1 else 0))
        self.tl -= dt
        self.combot -= dt
        for b in self.blds:  # sphere vs box collision
            if b["park"]:
                continue
            qx = clamp(D["x"], b["x"] - b["w"], b["x"] + b["w"]); qy = clamp(D["y"], 0, b["h"]); qz = clamp(D["z"], b["z"] - b["d"], b["z"] + b["d"])
            dx, dy, dz = D["x"] - qx, D["y"] - qy, D["z"] - qz
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d < 0.9:
                if d < 1e-4:
                    dx, d = 1, 1
                nx, ny, nz = dx / d, dy / d, dz / d
                D["x"], D["y"], D["z"] = qx + nx * .9, qy + ny * .9, qz + nz * .9
                vn = D["vx"] * nx + D["vy"] * ny + D["vz"] * nz
                if vn < 0:
                    D["vx"] -= 1.4 * vn * nx; D["vy"] -= 1.4 * vn * ny; D["vz"] -= 1.4 * vn * nz
                    if -vn > 2.5:
                        self.hurt(-vn * 2.2)
        for o in self.obs:
            if math.dist((D["x"], D["y"], D["z"]), (o["x"], o["y"], o["z"])) < 2.2:
                dx, dz = D["x"] - o["x"], D["z"] - o["z"]
                l = math.hypot(dx, dz) or 1
                D["vx"], D["vz"], D["vy"] = dx / l * 9, dz / l * 9, 3
                self.combo = 0
                self.hurt(18, "SHOT DOWN")
        if D["y"] < 0.7:
            if D["vy"] < -5:
                self.hurt(-D["vy"] * 3)
            D["y"], D["vy"] = 0.7, max(0, D["vy"])
        D["y"] = min(D["y"], 80)
        lim = HALF - 8
        if abs(D["x"]) > lim or abs(D["z"]) > lim:
            D["vx"] -= math.copysign(30 * dt, D["x"]) * (abs(D["x"]) > lim)
            D["vz"] -= math.copysign(30 * dt, D["z"]) * (abs(D["z"]) > lim)
            self.say("RETURN TO THE CITY", 0.3)
        for p in self.picks:
            if not p["got"] and math.dist((D["x"], D["y"], D["z"]), (p["x"], p["y"], p["z"])) < 2.6:
                p["got"] = True
                D["bat"] = min(100, D["bat"] + 35)
                self.score += 50
                self.say("+35% BATTERY", 1.2)
        if self.gi < len(self.gates):  # gate = crossing its plane inside the ring
            g = self.gates[self.gi]
            sd = (lambda x, z: x - g["x"]) if g["ax"] else (lambda x, z: z - g["z"])
            if sd(ox, oz) * sd(D["x"], D["z"]) <= 0:
                lat = (D["z"] - g["z"]) if g["ax"] else (D["x"] - g["x"])
                if math.hypot(lat, D["y"] - g["y"]) < 3.5:
                    self.combo = self.combo + 1 if self.combot > 0 else 1
                    self.combot = 14
                    pts = 100 + 50 * (self.combo - 1)
                    self.score += pts
                    self.tl += 3
                    self.gi += 1
                    self.say("GATE %d/%d  +%d%s" % (self.gi, len(self.gates), pts, "  COMBO x%d" % self.combo if self.combo > 1 else ""), 1.6)
                    if self.gi >= len(self.gates):
                        self.b1, self.b2 = int(max(0, self.tl) * 10), int(D["hull"] * 5)
                        self.score += self.b1 + self.b2
                        self.best = max(self.best, self.score)
                        self.state = "clear"
        if self.tl <= 0:
            self.tl = 0
            self.finish("TIME IS UP")
        elif D["bat"] <= 0:
            D["bat"] = 0
            self.finish("BATTERY EMPTY")

    def update_camera(self, dt):
        c, D = self.cam, self.D
        if self.state == "menu":
            a = self.clock * 0.25
            D["x"], D["z"], D["y"], D["yaw"] = C / 2, C / 2, 14 + math.sin(self.clock), a + 1
            D["spin"] += dt * 60
            c["x"], c["y"], c["z"] = C / 2 + math.sin(a) * 30, 19, C / 2 + math.cos(a) * 30
            self.aim(C / 2, 14, C / 2)
            return
        sy, cy = math.sin(D["yaw"]), math.cos(D["yaw"])
        if self.fpv:
            c["x"], c["y"], c["z"], c["yaw"], c["pitch"] = D["x"] + sy * .6, D["y"] + .15, D["z"] + cy * .6, D["yaw"], .12 - D["pitch"] * .3
            return
        da = (D["yaw"] - c["yaw"] + math.pi * 3) % TAU - math.pi
        c["yaw"] += da * (1 - math.exp(-5 * dt))
        sp = math.hypot(D["vx"], D["vz"])
        dist = 7.5 + sp * 0.14
        tx, ty, tz = D["x"] - math.sin(c["yaw"]) * dist, D["y"] + 2.8, D["z"] - math.cos(c["yaw"]) * dist
        k = 1 - math.exp(-10 * dt)
        c["x"] += (tx - c["x"]) * k; c["y"] += (ty - c["y"]) * k; c["z"] += (tz - c["z"]) * k
        c["y"] = max(c["y"], 0.8)
        for b in self.blds:  # keep the camera outside buildings
            if not b["park"] and abs(c["x"] - b["x"]) < b["w"] + .5 and abs(c["z"] - b["z"]) < b["d"] + .5 and c["y"] < b["h"] + .5:
                c["y"] = b["h"] + .6
        self.aim(D["x"] + math.sin(c["yaw"]) * 6, D["y"] + .8, D["z"] + math.cos(c["yaw"]) * 6)
        c["x"] += (random.random() - .5) * self.shake * .6
        c["y"] += (random.random() - .5) * self.shake * .6

    def aim(self, x, y, z):
        c = self.cam
        dx, dy, dz = x - c["x"], y - c["y"], z - c["z"]
        c["yaw"], c["pitch"] = math.atan2(dx, dz), math.asin(dy / math.sqrt(dx * dx + dy * dy + dz * dz))

    # ---------------- drawing ----------------
    def draw_world(self):
        cv, th, cam = self.cv, self.th, self.cam
        hy = H / 2 + math.tan(cam["pitch"]) * F
        for i in range(12):  # sky gradient bands
            y0, y1 = hy - 700 + i * 700 / 12, hy - 700 + (i + 1) * 700 / 12
            cv.create_rectangle(0, y0, W, y1 + 1, fill=hx(mix(th[1], th[2], i / 11)), width=0)
        cv.create_rectangle(0, hy, W, H + 50, fill=hx(mix(th[2], th[3], .35)), width=0)
        if th[0] == "NIGHT":
            rnd = random.Random(3)
            for _ in range(60):
                a, e = rnd.random() * TAU, rnd.random() * 1.1
                p = self.scr((cam["x"] + math.sin(a) * 900, cam["y"] + e * 900, cam["z"] + math.cos(a) * 900))
                if p:
                    cv.create_rectangle(p[0], p[1], p[0] + 2, p[1] + 2, fill="white", width=0)
        for b in self.blds:  # street pads
            d = self.dist(b["x"], b["z"])
            if d < 200:
                x, z, h = b["x"], b["z"], 11
                col = (40, 120, 60) if b["park"] else mix(th[3], (255, 255, 255), .22)
                self.poly([(x - h, 0, z - h), (x + h, 0, z - h), (x + h, 0, z + h), (x - h, 0, z + h)], hx(self.fog(col, d)))
        D = self.D
        r = 1.2 + D["y"] * .03
        self.poly([(D["x"] + math.cos(i / 12 * TAU) * r, .05, D["z"] + math.sin(i / 12 * TAU) * r) for i in range(12)], "#101010")
        items = []
        for b in self.blds:
            d = self.dist(b["x"], b["z"])
            if not b["park"] and d < 220 and self.tocam((b["x"], b["h"] / 2, b["z"]))[2] > -25:
                items.append((d - 3, lambda b=b: self.box(b["x"], b["h"] / 2, b["z"], b["w"], b["h"] / 2, b["d"], b["c"], None, True,
                                                          hx(th[5]) if th[0] == "NIGHT" else "")))
        for i, g in enumerate(self.gates):
            if self.gi - 1 <= i <= self.gi + 2:
                items.append((self.dist(g["x"], g["z"]), lambda g=g, i=i: self.draw_gate(g, i)))
        for p in self.picks:
            if not p["got"]:
                items.append((self.dist(p["x"], p["z"]), lambda p=p: self.draw_pick(p)))
        for o in self.obs:
            items.append((self.dist(o["x"], o["z"]), lambda o=o: self.draw_obs(o)))
        if not (self.fpv and self.state != "menu"):
            items.append((self.dist(D["x"], D["z"]) - 1, self.draw_drone))
        for d, f in sorted(items, key=lambda t: -t[0]):
            f()
        for s in self.sparks:
            p = self.scr(s[:3])
            if p:
                cv.create_oval(p[0] - 2, p[1] - 2, p[0] + 2, p[1] + 2, fill="#ffc850", width=0)

    def draw_gate(self, g, i):
        act, passed = i == self.gi, i < self.gi
        col = (60, 255, 120) if passed else mix(self.th[5], (255, 255, 255), .4 + .3 * math.sin(self.clock * 6)) if act else (255, 175, 45)
        n, s, M, R = ((1, 0, 0), (0, 0, 1), 18, 4.2) if g["ax"] else ((0, 0, 1), (1, 0, 0), 18, 4.2)
        P = lambda r, a, z: (g["x"] + s[0] * math.cos(a) * r + n[0] * z, g["y"] + math.sin(a) * r, g["z"] + s[2] * math.cos(a) * r + n[2] * z)
        for k in range(M):
            a0, a1 = k / M * TAU, (k + 1) / M * TAU
            self.poly([P(R, a0, -.35), P(R, a1, -.35), P(R, a1, .35), P(R, a0, .35)], hx(tuple(v * .55 for v in col)))
            self.poly([P(R, a0, .35), P(R, a1, .35), P(R - .8, a1, .35), P(R - .8, a0, .35)], hx(col))
            self.poly([P(R, a0, -.35), P(R, a1, -.35), P(R - .8, a1, -.35), P(R - .8, a0, -.35)], hx(col))
        if act:
            a, b = self.scr((g["x"], g["y"] + R, g["z"])), self.scr((g["x"], g["y"] + 130, g["z"]))
            if a and b:
                self.cv.create_line(a[0], a[1], b[0], b[1], fill=hx(col), width=3)

    def draw_pick(self, p):
        y = p["y"] + math.sin(self.clock * 3 + p["x"]) * .3
        for k in range(2):
            a = self.clock * 2 + k * math.pi / 2
            sx, sz = math.cos(a) * .8, math.sin(a) * .8
            self.poly([(p["x"] - sx, y, p["z"] - sz), (p["x"], y + 1.3, p["z"]), (p["x"] + sx, y, p["z"] + sz), (p["x"], y - 1.3, p["z"])], "#50ff8c", "white")

    def draw_obs(self, o):
        s = self.scr((o["x"], o["y"], o["z"]))
        if s:
            r = 1.3 / s[2] * F
            self.cv.create_oval(s[0] - r * 1.8, s[1] - r * 1.8, s[0] + r * 1.8, s[1] + r * 1.8, fill="#5a1010", outline="")
            self.cv.create_oval(s[0] - r, s[1] - r, s[0] + r, s[1] + r, fill="#3a0a0a", outline="#ff6060", width=2)
            q = r * .45 * (1 + .2 * math.sin(self.clock * 8))
            self.cv.create_oval(s[0] - q, s[1] - q, s[0] + q, s[1] + q, fill="#ff4d4d", outline="")

    def draw_drone(self):
        D = self.D

        def dw(l):
            x, y, z = l
            cb, sb, ca, sa = math.cos(D["roll"]), math.sin(D["roll"]), math.cos(D["pitch"]), math.sin(D["pitch"])
            cy, sy = math.cos(D["yaw"]), math.sin(D["yaw"])
            x1, y1 = x * cb - y * sb, x * sb + y * cb
            y2, z2 = y1 * ca - z * sa, y1 * sa + z * ca
            return (D["x"] + x1 * cy + z2 * sy, D["y"] + y2, D["z"] - x1 * sy + z2 * cy)
        arms = [(a * .95, .12, b * .95) for a, b in ((1, 1), (-1, 1), (1, -1), (-1, -1))]
        for a in arms:
            p, q = self.scr(dw((0, .05, 0))), self.scr(dw(a))
            if p and q:
                self.cv.create_line(p[0], p[1], q[0], q[1], fill="#1c1f26", width=max(2, .22 / q[2] * F))
        self.box(0, 0, 0, .32, .1, .5, (52, 56, 68), dw, False, hx(self.th[5]))
        self.box(0, .12, .18, .2, .06, .22, (80, 86, 100), dw, False)
        for l, col in (((-.95, .12, -.95), "#ff2828"), ((.95, .12, -.95), "#ff2828"), ((-.95, .12, .95), "#3cff78"), ((.95, .12, .95), "#3cff78")):
            s = self.scr(dw(l))
            if s:
                r = max(2, .12 / s[2] * F)
                self.cv.create_oval(s[0] - r, s[1] - r, s[0] + r, s[1] + r, fill=col, outline="")
        for a in sorted(arms, key=lambda a: -self.tocam(dw(a))[2]):
            self.poly([dw((a[0] + math.cos(i / 14 * TAU) * .6, .2, a[2] + math.sin(i / 14 * TAU) * .6)) for i in range(14)], "#6fa8c8", "#cfe8f8")
            for k in range(2):
                an = math.radians(D["spin"]) * 6 + k * math.pi
                self.poly([dw((a[0] + math.cos(an) * .6, .2, a[2] + math.sin(an) * .6)), dw((a[0] + math.cos(an + .35) * .6, .2, a[2] + math.sin(an + .35) * .6)), dw((a[0], .2, a[2]))], "#ffffff")

    # ---------------- HUD ----------------
    def text(self, s, x, y, size=14, col="white", anchor="w", bold=True):
        self.cv.create_text(x, y, text=s, fill=col, anchor=anchor, font=("Arial", size, "bold" if bold else "normal"))

    def bar(self, x, y, w, v, col, label):
        self.cv.create_rectangle(x, y, x + w, y + 16, fill="#101820", outline="#556")
        self.cv.create_rectangle(x + 2, y + 2, x + 2 + (w - 4) * clamp(v / 100, 0, 1), y + 14, fill=col, width=0)
        self.text(label, x, y - 9, 9, "#cfe")

    def hud(self):
        D, ac = self.D, hx(self.th[5])
        self.text("SCORE %d" % self.score, 20, 28, 20)
        self.text("BEST %d" % self.best, 20, 52, 10, "#9bd")
        self.text("%.1fs" % max(0, self.tl), W / 2, 30, 26, "#ff5050" if self.tl < 10 else "white", "center")
        self.text("GATE %d/%d    LEVEL %d - %s" % (min(self.gi + 1, len(self.gates)), len(self.gates), self.level, self.th[0]), W / 2, 56, 10, ac, "center")
        if self.combo > 1 and self.combot > 0:
            self.text("COMBO x%d" % self.combo, W / 2, 78, 14, "#ffd34d", "center")
        self.bar(20, H - 80, 200, D["bat"], "#ff4040" if D["bat"] < 25 else "#4dff88", "BATTERY %d%%" % D["bat"])
        self.bar(20, H - 40, 200, D["hull"], "#ff4040" if D["hull"] < 30 else ac, "HULL %d%%" % D["hull"])
        self.text("%d" % (math.sqrt(D["vx"] ** 2 + D["vy"] ** 2 + D["vz"] ** 2) * 3.6), W - 30, H - 52, 30, "white", "e")
        self.text("km/h   ALT %d m" % D["y"], W - 30, H - 24, 10, ac, "e")
        if self.gi < len(self.gates) and self.state != "clear":
            g = self.gates[self.gi]
            q = self.tocam((g["x"], g["y"], g["z"]))
            dist = math.dist((D["x"], D["y"], D["z"]), (g["x"], g["y"], g["z"]))
            if q[2] > .3 and abs(q[0] / q[2] * F) < W / 2 - 40 and abs(q[1] / q[2] * F) < H / 2 - 40:
                sx, sy = W / 2 + q[0] / q[2] * F, H / 2 - q[1] / q[2] * F
                self.cv.create_polygon(sx, sy - 22, sx + 22, sy, sx, sy + 22, sx - 22, sy, outline=ac, fill="", width=3)
                self.text("%dm" % dist, sx, sy + 38, 10, ac, "center")
            else:  # off-screen: arrow at the screen edge
                ang = math.atan2(-q[1], q[0] if q[2] > 0 else -q[0] or -1)
                ex = min((W / 2 - 45) / max(.01, abs(math.cos(ang))), (H / 2 - 45) / max(.01, abs(math.sin(ang))))
                sx, sy = W / 2 + math.cos(ang) * ex, H / 2 + math.sin(ang) * ex
                pts = []
                for dx, dy in ((18, 0), (-12, -14), (-12, 14)):
                    pts += [sx + dx * math.cos(ang) - dy * math.sin(ang), sy + dx * math.sin(ang) + dy * math.cos(ang)]
                self.cv.create_polygon(pts, fill=ac)
                self.text("%dm" % dist, sx, sy + 26, 10, ac, "center")
        if self.msgt > 0:
            self.text(self.msg, W / 2, H * .3, 22, "white", "center")
        if self.state == "count":
            self.text(str(math.ceil(self.cd - .4)) if self.cd > .4 else "GO!", W / 2, H / 2, 80, ac, "center")
        self.text("W/S fly  A/D turn  Q/E slide  SPACE up  SHIFT down  B boost  V camera  P pause", W / 2, H - 12, 9, "#aab", "center", False)

    def panel(self, title, lines, sub):
        self.cv.create_rectangle(0, 0, W, H, fill="#000", stipple="gray50", width=0)
        self.text(title, W / 2, H * .28, 48, hx(self.th[5]), "center")
        for i, l in enumerate(lines):
            self.text(l, W / 2, H * .28 + 55 + i * 28, 13, "white", "center", False)
        self.text(sub, W / 2, H * .85, 16, "#ffd34d", "center")

    # ---------------- main loop ----------------
    def loop(self):
        now = time.time()
        dt = min(.05, now - self.last)
        self.last = now
        if self.state == "menu":
            self.clock += dt
        if self.state != "pause":
            self.update(dt) if self.state != "menu" else None
            self.update_camera(dt)
        self.basis()
        self.cv.delete("all")
        self.draw_world()
        if self.state == "menu":
            self.panel("SKY RUNNER", ["Fly the drone through every glowing gate before time runs out.",
                                      "Grab green battery cells, dodge red guard drones, avoid buildings.",
                                      "Every level is harder - Day, Sunset, Neon Night.", "",
                                      "W/S fly   A/D turn   Q/E slide   SPACE up   SHIFT down",
                                      "B boost   V camera   P pause   ESC quit"], "Press ENTER to take off")
        else:
            self.hud()
            if self.state == "pause":
                self.panel("PAUSED", [], "Press P to resume")
            elif self.state == "over":
                self.panel("GAME OVER", [self.why, "Score %d   -   Best %d" % (self.score, self.best), "You reached level %d" % self.level], "Press ENTER to try again")
            elif self.state == "clear":
                self.panel("LEVEL CLEAR!", ["Time bonus +%d   Hull bonus +%d" % (self.b1, self.b2), "Score %d   -   Best %d" % (self.score, self.best)],
                           "Press ENTER for level %d" % (self.level + 1))
        self.root.after(16, self.loop)


if __name__ == "__main__":
    Game().root.mainloop()
