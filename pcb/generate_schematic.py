#!/usr/bin/env python3
"""Генерує pcb/ugv-column.kicad_sch зі схемою контрольної плати UGV
(перенесення Fig. 5 зі звіту "Column of Ground Robots" у KiCad).

Всі модулі - власні символи-прямокутники з пінами; з'єднання - глобальними
мітками, що стоять точно на кінцях пінів (у KiCad збіг точок = з'єднання).
"""
import uuid

G = 1.27  # базова сітка KiCad, мм


def u():
    return str(uuid.uuid4())


def mm(n):
    """координата в кроках сітки -> мм"""
    v = round(n * G, 2)
    return f"{v:g}"


FONT = "(effects (font (size 1.27 1.27)))"
FONT_H = "(effects (font (size 1.27 1.27)) hide)"

# ── описи символів ────────────────────────────────────────────────────────
# пін: (номер, ім'я, сторона l/r/t/b, зсув у кроках сітки від центру)
SYMBOLS = {}


def defsym(name, half_w, half_h, pins):
    SYMBOLS[name] = {"w": half_w, "h": half_h, "pins": pins}


# ESP32-WROVER (плата Freenove). IO для SW/LED ще не відомі - позначені "?".
defsym("ESP32_Board", 14, 16, [
    ("1", "5V", "l", 10), ("2", "GND", "l", 6),
    ("3", "IO12", "r", 10), ("4", "IO13", "r", 8),
    ("5", "IO14", "r", 6), ("6", "IO15", "r", 4),
    ("7", "IO26_SDA", "r", 2), ("8", "IO27_SCL", "r", 0),
    ("9", "IO?_SW1", "r", -4), ("10", "IO?_SW2", "r", -6),
    ("11", "IO?_SW3", "r", -8), ("12", "IO?_LED1", "r", -10),
    ("13", "IO?_LED2", "r", -12),
])

# Модуль L298N (червона платка): ENA/ENB на модулі заджамперені на 5V.
defsym("L298N_Module", 12, 14, [
    ("1", "VMS", "l", 10), ("2", "5V", "l", 8), ("3", "GND", "l", 6),
    ("4", "ENA", "l", 3), ("5", "IN1", "l", 1), ("6", "IN2", "l", -1),
    ("7", "IN3", "l", -3), ("8", "IN4", "l", -5), ("9", "ENB", "l", -8),
    ("10", "OUT1", "r", 8), ("11", "OUT2", "r", 6),
    ("12", "OUT3", "r", -4), ("13", "OUT4", "r", -6),
])

# Boost/charge модуль (Type-C, 3.7V -> 5V)
defsym("Boost_Module", 9, 5, [
    ("1", "IN+", "l", 2), ("2", "IN-", "l", -2),
    ("3", "OUT+", "r", 2), ("4", "OUT-", "r", -2),
])

# Акумуляторний блок 2x18650 (паралельно)
defsym("Battery_2x18650", 8, 4, [
    ("1", "+", "r", 2), ("2", "-", "r", -2),
])

# Перемикач/кнопка, 2 контакти
defsym("SW_2P", 5, 2, [("1", "A", "l", 0), ("2", "B", "r", 0)])

# LED (A зверху, K знизу)
defsym("LED_2P", 3, 4, [("1", "A", "t", 0), ("2", "K", "b", 0)])

# Резистор (вертикальний)
defsym("R_2P", 2, 4, [("1", "1", "t", 0), ("2", "2", "b", 0)])

# Роз'єм LCD1602 з I2C-адаптером
defsym("LCD1602_I2C", 8, 6, [
    ("1", "GND", "l", 3), ("2", "VCC", "l", 1),
    ("3", "SDA", "l", -1), ("4", "SCL", "l", -3),
])

# Мотор постійного струму
defsym("Motor_DC", 6, 4, [("1", "+", "l", 2), ("2", "-", "l", -2)])

PIN_LEN = 2  # довжина піна в кроках сітки (2.54 мм)


def sym_lib(name):
    """Текст визначення символа для lib_symbols."""
    s = SYMBOLS[name]
    w, h = s["w"], s["h"]
    out = [f'(symbol "UGV:{name}" (pin_names (offset 1.016)) '
           f'(exclude_from_sim no) (in_bom yes) (on_board yes)']
    out.append(f'  (property "Reference" "U" (at 0 {mm(h + 2)} 0) {FONT})')
    out.append(f'  (property "Value" "{name}" (at 0 {mm(-h - 2)} 0) {FONT})')
    out.append(f'  (property "Footprint" "" (at 0 0 0) {FONT_H})')
    out.append(f'  (property "Datasheet" "" (at 0 0 0) {FONT_H})')
    out.append(f'  (symbol "{name}_0_1" (rectangle (start {mm(-w)} {mm(h)}) '
               f'(end {mm(w)} {mm(-h)}) '
               f'(stroke (width 0.254) (type default)) (fill (type background))))')
    pins = []
    for num, pname, side, off in s["pins"]:
        if side == "l":
            at = f"(at {mm(-w - PIN_LEN)} {mm(off)} 0)"
        elif side == "r":
            at = f"(at {mm(w + PIN_LEN)} {mm(off)} 180)"
        elif side == "t":
            at = f"(at {mm(off)} {mm(h + PIN_LEN)} 270)"
        else:  # b
            at = f"(at {mm(off)} {mm(-h - PIN_LEN)} 90)"
        pins.append(f'    (pin passive line {at} (length {mm(PIN_LEN)}) '
                    f'(name "{pname}" {FONT}) (number "{num}" {FONT}))')
    out.append(f'  (symbol "{name}_1_1"\n' + "\n".join(pins) + ")")
    out.append(")")
    return "\n".join(out)


ROOT = u()
placed = []   # текст розміщених символів
labels = []   # глобальні мітки
texts = []    # текстові нотатки


def pin_conn(sym, gx, gy, num):
    """Точка підключення піна розміщеного символа (у кроках сітки).
    Вісь Y у схемі напрямлена вниз, у символі - вгору."""
    s = SYMBOLS[sym]
    for n, _, side, off in s["pins"]:
        if n == num:
            if side == "l":
                return gx - s["w"] - PIN_LEN, gy - off, "l"
            if side == "r":
                return gx + s["w"] + PIN_LEN, gy - off, "r"
            if side == "t":
                return gx + off, gy - s["h"] - PIN_LEN, "t"
            return gx + off, gy + s["h"] + PIN_LEN, "b"
    raise KeyError(num)


def place(sym, ref, value, gx, gy, nets):
    """Ставить символ і мітки на піни. nets: {номер_піна: ім'я_мережі|None}"""
    s = SYMBOLS[sym]
    x, y = mm(gx), mm(gy)
    if sym in ("R_2P", "LED_2P"):
        ref_at = f'(at {mm(gx + 4)} {mm(gy - 1.5)} 0) (effects (font (size 1.27 1.27)) (justify left))'
        val_at = f'(at {mm(gx + 4)} {mm(gy + 1.5)} 0) (effects (font (size 1.27 1.27)) (justify left))'
    else:
        ref_at = f'(at {x} {mm(gy - s["h"] - 2)} 0) {FONT}'
        val_at = f'(at {x} {mm(gy + s["h"] + 2)} 0) {FONT}'
    lines = [f'(symbol (lib_id "UGV:{sym}") (at {x} {y} 0) (unit 1) '
             f'(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)',
             f'  (uuid "{u()}")',
             f'  (property "Reference" "{ref}" {ref_at})',
             f'  (property "Value" "{value}" {val_at})',
             f'  (property "Footprint" "" (at {x} {y} 0) {FONT_H})',
             f'  (property "Datasheet" "" (at {x} {y} 0) {FONT_H})']
    for n, _, _, _ in s["pins"]:
        lines.append(f'  (pin "{n}" (uuid "{u()}"))')
    lines.append(f'  (instances (project "ugv-column" '
                 f'(path "/{ROOT}" (reference "{ref}") (unit 1)))))')
    placed.append("\n".join(lines))

    for num, net in nets.items():
        if net is None:
            continue
        px, py, side = pin_conn(sym, gx, gy, num)
        ang, just = {"l": (180, "right"), "r": (0, "left"),
                     "t": (90, "left"), "b": (270, "right")}[side]
        labels.append(
            f'(global_label "{net}" (shape input) (at {mm(px)} {mm(py)} {ang}) '
            f'(fields_autoplaced yes) (effects (font (size 1.27 1.27)) '
            f'(justify {just})) (uuid "{u()}")\n'
            f'  (property "Intersheetrefs" "${{INTERSHEET_REFS}}" '
            f'(at {mm(px)} {mm(py)} 0) {FONT_H}))')


def note(gx, gy, text):
    texts.append(f'(text "{text}" (exclude_from_sim no) (at {mm(gx)} {mm(gy)} 0) '
                 f'(effects (font (size 1.7 1.7)) (justify left bottom)) '
                 f'(uuid "{u()}"))')


# ── ЖИВЛЕННЯ (лівий стовпець) ────────────────────────────────────────────
place("Battery_2x18650", "BT1", "2x18650 4400mAh", 28, 56,
      {"1": "BAT+", "2": "GND"})
place("SW_2P", "SW4", "POWER", 60, 54, {"1": "BAT+", "2": "BAT_SW"})
place("Boost_Module", "U3", "Boost 3.7V->5V 15W Type-C", 108, 56,
      {"1": "BAT_SW", "2": "GND", "3": "5V", "4": "GND"})

# ── МОЗОК (центр) ────────────────────────────────────────────────────────
place("ESP32_Board", "U2", "Freenove ESP32-WROVER", 130, 84, {
    "1": "5V", "2": "GND",
    "3": "M_IN1", "4": "M_IN2", "5": "M_IN3", "6": "M_IN4",
    "7": "SDA", "8": "SCL",
    "9": "MODE_SW1", "10": "BTN_SW2", "11": "BTN_SW3",
    "12": "LED1_CTL", "13": "LED2_CTL",
})

# ── ДРАЙВЕР МОТОРІВ (правий стовпець) ────────────────────────────────────
place("L298N_Module", "U1", "L298N module", 196, 84, {
    "1": "5V", "2": "5V", "3": "GND", "4": "5V",
    "5": "M_IN1", "6": "M_IN2", "7": "M_IN3", "8": "M_IN4", "9": "5V",
    "10": "OUT1", "11": "OUT2", "12": "OUT3", "13": "OUT4",
})
place("Motor_DC", "M1", "TT gear motor L", 244, 76,
      {"1": "OUT1", "2": "OUT2"})
place("Motor_DC", "M2", "TT gear motor R", 244, 96,
      {"1": "OUT3", "2": "OUT4"})

# ── LCD (низ, центр) ─────────────────────────────────────────────────────
place("LCD1602_I2C", "DS1", "LCD1602 + I2C backpack", 130, 130,
      {"1": "GND", "2": "5V", "3": "SDA", "4": "SCL"})

# ── КНОПКИ (низ, ліворуч): замкнено -> GND, читається з pull-up ─────────
place("SW_2P", "SW1", "MODE leader/follower", 52, 118,
      {"1": "MODE_SW1", "2": "GND"})
place("SW_2P", "SW2", "BTN (reserved)", 52, 128, {"1": "BTN_SW2", "2": "GND"})
place("SW_2P", "SW3", "BTN (reserved)", 52, 138, {"1": "BTN_SW3", "2": "GND"})

# ── ІНДИКАТОРИ LED1-2 (GPIO -> R -> LED -> GND) ──────────────────────────
# R знизу і LED зверху стикуються кінцями пінів (збіг точки = з'єднання)
for i, (ref_r, ref_d, net, gx) in enumerate(
        [("R1", "LED1", "LED1_CTL", 156), ("R2", "LED2", "LED2_CTL", 172)]):
    ry = 122
    place("R_2P", ref_r, "220R", gx, ry, {"1": net, "2": None})
    dy = ry + (4 + PIN_LEN) + (4 + PIN_LEN)  # низ R = верх LED
    place("LED_2P", ref_d, "indicator", gx, dy, {"1": None, "2": "GND"})

# ── МАРКЕРИ LED3-5: три червоні ліхтарики (5V -> R -> LED -> GND) ───────
for i, (ref_r, ref_d, gx) in enumerate(
        [("R3", "LED3", 200), ("R4", "LED4", 216), ("R5", "LED5", 232)]):
    ry = 122
    place("R_2P", ref_r, "100R", gx, ry, {"1": "5V", "2": None})
    dy = ry + (4 + PIN_LEN) + (4 + PIN_LEN)
    place("LED_2P", ref_d, "red marker", gx, dy, {"1": None, "2": "GND"})

# ── Нотатки ──────────────────────────────────────────────────────────────
note(24, 18, "Живлення: 2x18650 -> SW4 -> boost 5V. Шина 5V живить ESP32 та L298N.")
note(24, 22, "ENA/ENB на модулі L298N заджамперені (тут показані на 5V).")
note(24, 26, "IO? = GPIO ще не визначені на платі (SW1-SW3, LED1-LED2) - уточнити і вписати.")
note(24, 30, "LED3-LED5 - задня планка-маркер: три червоні ліхтарики для камери фолловера.")
note(24, 160, "Камера OV3660 встановлюється в роз'єм плати Freenove (не розводиться тут).")
note(24, 164, "I2C шина SDA=IO26 / SCL=IO27 спільна: камера (SCCB) + LCD. Джерело: звіт, Fig. 5.")

# ── збірка файлу ─────────────────────────────────────────────────────────
libs = "\n".join(sym_lib(n) for n in SYMBOLS)
body = "\n".join(placed + labels + texts)

doc = f"""(kicad_sch (version 20231120) (generator "eeschema") (generator_version "8.0")
  (uuid "{ROOT}")
  (paper "A3")
  (title_block
    (title "UGV Column - Control Board")
    (date "2026-07-11")
    (rev "1.0")
    (company "UCU / UGV-swarm")
    (comment 1 "Перенесено зі звіту Column of Ground Robots, Fig. 5 (оригінал у EasyEDA)")
  )
  (lib_symbols
{libs}
  )
{body}
  (sheet_instances (path "/" (page "1")))
)
"""

out = "/Users/luzefik/Documents/UGV-swarm/pcb/ugv-column.kicad_sch"
with open(out, "w") as f:
    f.write(doc)
print(f"OK: {out}, {len(doc)} bytes, {len(placed)} symbols, {len(labels)} labels")
