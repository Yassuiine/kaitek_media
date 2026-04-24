"""
Shell Interface Reference — PDF generator.
Run:  python docs/gen_docs.py
Output: docs/Shell_Interface_Reference.pdf
"""

from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    HRFlowable, PageBreak, KeepTogether
)
from reportlab.platypus.tableofcontents import TableOfContents
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
import os, sys

# ── Output path ──────────────────────────────────────────────────────────────
OUT_DIR  = os.path.dirname(os.path.abspath(__file__))
OUT_FILE = os.path.join(OUT_DIR, "Shell_Interface_Reference.pdf")

# ── Palette ───────────────────────────────────────────────────────────────────
C_NAVY    = colors.HexColor("#0D1B2A")
C_BLUE    = colors.HexColor("#1B4F72")
C_ACCENT  = colors.HexColor("#2E86C1")
C_GREEN   = colors.HexColor("#1E8449")
C_AMBER   = colors.HexColor("#D68910")
C_BG_CODE = colors.HexColor("#F2F3F4")
C_BG_HEAD = colors.HexColor("#D6EAF8")
C_BG_SEC  = colors.HexColor("#EAF2FF")
C_GREY    = colors.HexColor("#566573")
C_WHITE   = colors.white
C_BLACK   = colors.HexColor("#1A1A1A")
C_DIVIDER = colors.HexColor("#AEB6BF")

PAGE_W, PAGE_H = A4
MARGIN = 18 * mm

# ── Styles ────────────────────────────────────────────────────────────────────
base  = getSampleStyleSheet()

def S(name, **kw):
    return ParagraphStyle(name, **kw)

sTitle = S("sTitle",
    fontSize=26, leading=32, textColor=C_NAVY, spaceAfter=4,
    fontName="Helvetica-Bold", alignment=TA_LEFT)

sSubtitle = S("sSubtitle",
    fontSize=11, leading=16, textColor=C_GREY, spaceAfter=2,
    fontName="Helvetica", alignment=TA_LEFT)

sVersion = S("sVersion",
    fontSize=9, leading=12, textColor=C_GREY, spaceAfter=0,
    fontName="Helvetica", alignment=TA_LEFT)

sChapter = S("sChapter",
    fontSize=15, leading=20, textColor=C_WHITE, spaceAfter=0,
    spaceBefore=14, fontName="Helvetica-Bold", alignment=TA_LEFT)

sSection = S("sSection",
    fontSize=12, leading=16, textColor=C_BLUE, spaceAfter=4,
    spaceBefore=10, fontName="Helvetica-Bold", alignment=TA_LEFT)

sCmdName = S("sCmdName",
    fontSize=11, leading=14, textColor=C_NAVY, spaceAfter=2,
    spaceBefore=8, fontName="Helvetica-Bold", alignment=TA_LEFT)

sBody = S("sBody",
    fontSize=9.5, leading=14, textColor=C_BLACK, spaceAfter=3,
    fontName="Helvetica", alignment=TA_JUSTIFY)

sBodySmall = S("sBodySmall",
    fontSize=8.5, leading=13, textColor=C_BLACK, spaceAfter=2,
    fontName="Helvetica", alignment=TA_LEFT)

sCode = S("sCode",
    fontSize=8.5, leading=13, textColor=colors.HexColor("#1A1A1A"),
    spaceAfter=2, fontName="Courier", alignment=TA_LEFT,
    backColor=C_BG_CODE, borderPadding=(3, 6, 3, 6))

sNote = S("sNote",
    fontSize=8.5, leading=13, textColor=colors.HexColor("#7D6608"),
    spaceAfter=3, fontName="Helvetica-Oblique", alignment=TA_LEFT)

sWarn = S("sWarn",
    fontSize=8.5, leading=13, textColor=colors.HexColor("#922B21"),
    spaceAfter=3, fontName="Helvetica-BoldOblique", alignment=TA_LEFT)

sTOC = S("sTOC",
    fontSize=9.5, leading=15, textColor=C_ACCENT, spaceAfter=1,
    fontName="Helvetica", alignment=TA_LEFT)

sTOCHead = S("sTOCHead",
    fontSize=10.5, leading=15, textColor=C_NAVY, spaceAfter=2,
    spaceBefore=6, fontName="Helvetica-Bold", alignment=TA_LEFT)

# ── Helper builders ───────────────────────────────────────────────────────────
def chapter_block(title, toc_text=None, toc_key=None):
    tbl = Table([[Paragraph(title, sChapter)]], colWidths=[PAGE_W - 2*MARGIN])
    tbl.setStyle(TableStyle([
        ("BACKGROUND", (0,0), (-1,-1), C_NAVY),
        ("TOPPADDING",    (0,0), (-1,-1), 6),
        ("BOTTOMPADDING", (0,0), (-1,-1), 6),
        ("LEFTPADDING",   (0,0), (-1,-1), 10),
        ("RIGHTPADDING",  (0,0), (-1,-1), 10),
        ("ROUNDEDCORNERS", [4]),
    ]))
    if toc_text and toc_key:
        tbl._toc_level = 0
        tbl._toc_text = toc_text
        tbl._toc_key = toc_key
    return tbl

def section_bar(title, toc_text=None, toc_key=None):
    tbl = Table([[Paragraph(title, sSection)]], colWidths=[PAGE_W - 2*MARGIN])
    tbl.setStyle(TableStyle([
        ("BACKGROUND",    (0,0), (-1,-1), C_BG_SEC),
        ("TOPPADDING",    (0,0), (-1,-1), 4),
        ("BOTTOMPADDING", (0,0), (-1,-1), 4),
        ("LEFTPADDING",   (0,0), (-1,-1), 8),
        ("LINEBELOW",     (0,0), (-1,-1), 1.2, C_ACCENT),
    ]))
    if toc_text and toc_key:
        tbl._toc_level = 1
        tbl._toc_text = toc_text
        tbl._toc_key = toc_key
    return tbl

def cmd_header(syntax, aliases=None):
    txt = f'<font name="Courier" size="10" color="#0D1B2A"><b>{syntax}</b></font>'
    if aliases:
        txt += f'  <font name="Helvetica" size="8" color="#566573">  alias: {aliases}</font>'
    return Paragraph(txt, sCmdName)

def body(text):
    return Paragraph(text, sBody)

def code(text):
    return Paragraph(f'<font name="Courier">{text}</font>', sCode)

def note(text):
    return Paragraph(f"<i>Note: {text}</i>", sNote)

def warn(text):
    return Paragraph(f"⚠  {text}", sWarn)

def label_value_table(rows):
    """rows = [('Label', 'value text'), ...]"""
    data = [[Paragraph(f'<b>{r[0]}</b>', sBodySmall), Paragraph(r[1], sBodySmall)] for r in rows]
    tbl  = Table(data, colWidths=[28*mm, PAGE_W - 2*MARGIN - 32*mm])
    tbl.setStyle(TableStyle([
        ("VALIGN",        (0,0), (-1,-1), "TOP"),
        ("TOPPADDING",    (0,0), (-1,-1), 2),
        ("BOTTOMPADDING", (0,0), (-1,-1), 2),
        ("LEFTPADDING",   (0,0), (-1,-1), 0),
        ("RIGHTPADDING",  (0,0), (-1,-1), 4),
    ]))
    return tbl

def divider():
    return HRFlowable(width="100%", thickness=0.5, color=C_DIVIDER, spaceAfter=4, spaceBefore=4)

def spacer(h=4):
    return Spacer(1, h * mm)

def example_box(lines):
    """lines = list of strings"""
    joined = "<br/>".join(f'<font name="Courier" size="8.5">{l}</font>' for l in lines)
    p = Paragraph(joined, sCode)
    tbl = Table([[p]], colWidths=[PAGE_W - 2*MARGIN])
    tbl.setStyle(TableStyle([
        ("BACKGROUND",   (0,0), (-1,-1), C_BG_CODE),
        ("LINEABOVE",    (0,0), (-1,-1), 1, C_ACCENT),
        ("LINEBELOW",    (0,0), (-1,-1), 1, C_ACCENT),
        ("TOPPADDING",   (0,0), (-1,-1), 5),
        ("BOTTOMPADDING",(0,0), (-1,-1), 5),
        ("LEFTPADDING",  (0,0), (-1,-1), 8),
    ]))
    return tbl

# ── Page template (header / footer) ──────────────────────────────────────────
def on_page(canvas, doc):
    canvas.saveState()
    # header rule
    canvas.setStrokeColor(C_ACCENT)
    canvas.setLineWidth(1.2)
    canvas.line(MARGIN, PAGE_H - 14*mm, PAGE_W - MARGIN, PAGE_H - 14*mm)
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(C_GREY)
    canvas.drawString(MARGIN, PAGE_H - 11*mm, "RP2350 / OV5640 / nRF54Lx  —  Shell Interface Reference")
    canvas.drawRightString(PAGE_W - MARGIN, PAGE_H - 11*mm, "Kaimasys · 2025")
    # footer rule
    canvas.line(MARGIN, 12*mm, PAGE_W - MARGIN, 12*mm)
    canvas.drawString(MARGIN, 8.5*mm, "CONFIDENTIAL — Internal use only")
    canvas.drawRightString(PAGE_W - MARGIN, 8.5*mm, f"Page {doc.page}")
    canvas.restoreState()


class ShellDocTemplate(SimpleDocTemplate):
    def afterFlowable(self, flowable):
        level = getattr(flowable, "_toc_level", None)
        if level is None:
            return
        text = getattr(flowable, "_toc_text", None)
        key = getattr(flowable, "_toc_key", None)
        if key:
            self.canv.bookmarkPage(key)
        if text:
            self.notify("TOCEntry", (level, text, self.page, key))

# ── Document assembly ─────────────────────────────────────────────────────────
doc = ShellDocTemplate(
    OUT_FILE,
    pagesize=A4,
    leftMargin=MARGIN, rightMargin=MARGIN,
    topMargin=20*mm, bottomMargin=18*mm,
    title="Shell Interface Reference",
    author="Kaimasys",
    subject="RP2350 / OV5640 / nRF54Lx firmware shell documentation",
)

story = []

# ═══════════════════════════════════════════════════════════════════════════════
# COVER PAGE
# ═══════════════════════════════════════════════════════════════════════════════
story += [spacer(30)]
story += [Paragraph("Shell Interface", sTitle)]
story += [Paragraph("Reference Manual", sTitle)]
story += [spacer(2)]
story += [HRFlowable(width="60%", thickness=3, color=C_ACCENT, spaceAfter=6)]
story += [Paragraph(
    "RP2350 · OV5640 Camera · ILI9488 LCD · nRF54Lx SPI Peripheral",
    sSubtitle)]
story += [spacer(2)]
story += [Paragraph("Revision 1.0  ·  April 2025  ·  Kaimasys", sVersion)]
story += [spacer(20)]

cover_data = [[
    Paragraph("<b>Platform</b>", sBodySmall),
    Paragraph("Raspberry Pi RP2350 (Pico 2)", sBodySmall),
],[
    Paragraph("<b>Camera</b>", sBodySmall),
    Paragraph("OmniVision OV5640 — 240×320 RGB565 via PIO/DMA", sBodySmall),
],[
    Paragraph("<b>Display</b>", sBodySmall),
    Paragraph("ILI9488 3.5\" 320×240 SPI LCD with CST216D touch", sBodySmall),
],[
    Paragraph("<b>NRF Peripheral</b>", sBodySmall),
    Paragraph("Nordic nRF54Lx (BM20_C) — SPI slave at up to 15 MHz", sBodySmall),
],[
    Paragraph("<b>Storage</b>", sBodySmall),
    Paragraph("FatFS on microSD (SPI), FAT32/exFAT", sBodySmall),
],[
    Paragraph("<b>Shell access</b>", sBodySmall),
    Paragraph("USB CDC or UART, 115 200 baud, CR/LF line ending accepted, editable command line", sBodySmall),
]]
cover_tbl = Table(cover_data, colWidths=[32*mm, PAGE_W - 2*MARGIN - 36*mm])
cover_tbl.setStyle(TableStyle([
    ("BACKGROUND",    (0,0), (-1,-1), C_BG_SEC),
    ("ROWBACKGROUNDS",(0,0), (-1,-1), [C_BG_SEC, C_WHITE]),
    ("LINEBELOW",     (0,0), (-1,-1), 0.5, C_DIVIDER),
    ("TOPPADDING",    (0,0), (-1,-1), 5),
    ("BOTTOMPADDING", (0,0), (-1,-1), 5),
    ("LEFTPADDING",   (0,0), (-1,-1), 8),
    ("VALIGN",        (0,0), (-1,-1), "TOP"),
]))
story += [cover_tbl]
story += [spacer(10)]
story += [Paragraph(
    "This document describes the camera, LCD, and nRF-SPI commands available in the interactive shell embedded in the "
    "RP2350 firmware. It covers camera control, LCD display management, live camera-to-LCD "
    "streaming, and SPI communication with the nRF54Lx co-processor. "
    "No prior knowledge of the firmware is assumed — read from front to back and you will be "
    "able to exercise all major features within minutes.",
    sBody)]
story += [PageBreak()]

# ═══════════════════════════════════════════════════════════════════════════════
# TABLE OF CONTENTS
# ═══════════════════════════════════════════════════════════════════════════════
story += [Paragraph("Contents", sChapter)]
story += [spacer(4)]
toc = TableOfContents()
toc.dotsMinLevel = 0
toc.levelStyles = [
    ParagraphStyle(
        "toc_l0",
        parent=sTOC,
        leftIndent=0,
        firstLineIndent=0,
        spaceBefore=1,
        leading=14,
    ),
    ParagraphStyle(
        "toc_l1",
        parent=sTOC,
        leftIndent=12,
        firstLineIndent=0,
        spaceBefore=0.5,
        leading=13,
    ),
]
story += [toc]
story += [PageBreak()]

# ═══════════════════════════════════════════════════════════════════════════════
# 1. QUICK-START GUIDE
# ═══════════════════════════════════════════════════════════════════════════════
story += [chapter_block("1  Quick-Start Guide", toc_text="1  Quick-Start Guide", toc_key="toc_1")]
story += [spacer(3)]
story += [body(
    "Connect to the board via USB CDC or UART at 115 200 baud. You will see a prompt "
    "<b>Kaitek&gt;</b> on the terminal. Type a command and press <b>Enter</b>. "
    "Type <b>help</b> at any time to list every registered command.")]
story += [spacer(2)]
story += [section_bar("Minimal sequence — camera still capture")]
story += [spacer(2)]

qs_steps = [
    ("Step 1", "Mount the SD card",               "mount 0:"),
    ("Step 2", "Initialise sensor clock + defaults", "cam xclk\ncam defaults"),
    ("Step 3", "Capture a photo",                  "cam snap 0:/photo.bin"),
    ("Step 4", "Verify the file exists",           "ls 0:/"),
]
for step, desc, cmd_text in qs_steps:
    row = [
        Paragraph(f'<b>{step}</b><br/><font color="#566573" size="8">{desc}</font>', sBodySmall),
        example_box(cmd_text.split("\n")),
    ]
    t = Table([row], colWidths=[38*mm, PAGE_W - 2*MARGIN - 42*mm])
    t.setStyle(TableStyle([
        ("VALIGN",        (0,0), (-1,-1), "TOP"),
        ("TOPPADDING",    (0,0), (-1,-1), 4),
        ("BOTTOMPADDING", (0,0), (-1,-1), 4),
    ]))
    story += [t, spacer(1)]

story += [spacer(3)]
story += [section_bar("Minimal sequence — LCD live preview")]
story += [spacer(2)]
lcd_qs = [
    ("Step 1", "Mount SD (if using images)",       "mount 0:"),
    ("Step 2", "Initialise LCD",                   "lcd init vertical\nlcd bl 100"),
    ("Step 3", "Initialise camera",                "cam xclk\ncam i2c\ncam defaults\ncam alloc\ncam dma\ncam start"),
    ("Step 4", "Start live stream",                "lcd stream"),
    ("Step 5", "Stop stream",                      "lcd stream stop"),
]
for step, desc, cmd_text in lcd_qs:
    row = [
        Paragraph(f'<b>{step}</b><br/><font color="#566573" size="8">{desc}</font>', sBodySmall),
        example_box(cmd_text.split("\n")),
    ]
    t = Table([row], colWidths=[38*mm, PAGE_W - 2*MARGIN - 42*mm])
    t.setStyle(TableStyle([
        ("VALIGN",        (0,0), (-1,-1), "TOP"),
        ("TOPPADDING",    (0,0), (-1,-1), 4),
        ("BOTTOMPADDING", (0,0), (-1,-1), 4),
    ]))
    story += [t, spacer(1)]

story += [PageBreak()]

# ═══════════════════════════════════════════════════════════════════════════════
# 2. SHELL CONVENTIONS
# ═══════════════════════════════════════════════════════════════════════════════
story += [chapter_block("2  Shell Conventions", toc_text="2  Shell Conventions", toc_key="toc_2")]
story += [spacer(3)]
conv_rows = [
    ("<b>Prompt</b>",         "Kaitek&gt;  (appears after each command completes or on power-up)"),
    ("<b>Line ending</b>",    "LF (\\n). Most terminal emulators default to CR+LF — both are accepted."),
    ("<b>Echo</b>",           "Characters are echoed and editable in-line (history, arrows, backspace, tab completion)."),
    ("<b>Case</b>",           "All command names are <b>lower-case</b>. Arguments are case-sensitive where noted."),
    ("<b>Separators</b>",     "Arguments are separated by one or more spaces. Quoted strings are not supported."),
    ("<b>Hex values</b>",     "Hex arguments are bare hex digits — no <i>0x</i> prefix (e.g. <font name='Courier'>FF</font> not <font name='Courier'>0xFF</font>)."),
    ("<b>Drive paths</b>",    "<font name='Courier'>0:</font> refers to the first logical drive (SD card). Paths are FatFS-style: <font name='Courier'>0:/dir/file.bin</font>"),
    ("<b>Background cmds</b>","Commands marked <i>non-blocking</i> return the prompt immediately. The operation "
                              "continues and prints a completion message when done."),
    ("<b>Groups</b>",         "Commands are grouped: <font name='Courier'>cam</font>, <font name='Courier'>lcd</font>, <font name='Courier'>nrf</font>. "
                              "Type <font name='Courier'>cam help</font> / <font name='Courier'>lcd help</font> / <font name='Courier'>nrf help</font> for a quick list."),
    ("<b>Legacy aliases</b>", "Every group command also exists as a legacy top-level name "
                              "(e.g. <font name='Courier'>cam_snap</font> ≡ <font name='Courier'>cam snap</font>). Both forms work identically."),
]
story += [label_value_table(conv_rows)]
story += [PageBreak()]

# ═══════════════════════════════════════════════════════════════════════════════
# 3. CAMERA COMMANDS
# ═══════════════════════════════════════════════════════════════════════════════
story += [chapter_block("3  Camera Commands  (cam …)", toc_text="3  Camera Commands  (cam …)", toc_key="toc_3")]
story += [spacer(2)]
story += [body(
    "All camera commands are accessed via the <b>cam</b> group. The OV5640 sensor communicates "
    "over I²C (SCCB) for register configuration and over a parallel PIO interface for pixel data. "
    "DMA channels are used to stream pixel data directly to a heap-allocated buffer. "
    "For continuous capture/streaming, initialise in the order shown in <b>3.1</b>. "
    "For one-shot <font name='Courier'>cam snap</font>, a shorter path is available (see below).")]
story += [spacer(3)]

# 3.1 Init sequence
story += [section_bar("3.1  Initialisation Sequence", toc_text="3.1  Initialisation Sequence", toc_key="toc_3_1")]
story += [spacer(2)]
story += [body("Recommended full initialisation for continuous capture/streaming on each power-cycle:")]
story += [spacer(1)]

init_seq = [
    ("1", "cam xclk [freq_khz]",  "Start the 24 MHz XCLK PWM output to the sensor."),
    ("2", "cam i2c",              "Initialise the I²C1 bus used for SCCB register access."),
    ("3", "cam id",               "Optional — read and verify the OV5640 chip ID (0x5640)."),
    ("4", "cam defaults",         "Apply the full OV5640 register init sequence."),
    ("5", "cam alloc",            "Allocate the frame buffer (width × height × 2 bytes)."),
    ("6", "cam dma",              "Claim a DMA channel and arm IRQ-driven capture."),
    ("7", "cam start",            "Enable the PIO state machine and start continuous capture."),
]
tbl_data = [
    [Paragraph(f'<b>{n}</b>', sBodySmall),
     Paragraph(f'<font name="Courier" size="8.5">{cmd}</font>', sBodySmall),
     Paragraph(desc, sBodySmall)]
    for n, cmd, desc in init_seq
]
init_tbl = Table(tbl_data, colWidths=[7*mm, 52*mm, PAGE_W - 2*MARGIN - 63*mm])
init_tbl.setStyle(TableStyle([
    ("BACKGROUND",    (0,0), (-1,0), C_BG_HEAD),
    ("ROWBACKGROUNDS",(0,0), (-1,-1), [C_BG_HEAD, C_WHITE]),
    ("LINEBELOW",     (0,0), (-1,-1), 0.4, C_DIVIDER),
    ("TOPPADDING",    (0,0), (-1,-1), 3),
    ("BOTTOMPADDING", (0,0), (-1,-1), 3),
    ("LEFTPADDING",   (0,0), (-1,-1), 4),
    ("VALIGN",        (0,0), (-1,-1), "TOP"),
]))
story += [init_tbl, spacer(3)]
story += [note(
    "Fast path for one-shot file capture: "
    "<font name='Courier'>cam xclk</font> + <font name='Courier'>cam defaults</font> then "
    "<font name='Courier'>cam snap ...</font>. "
    "<font name='Courier'>cam snap</font> arms buffer/DMA/start internally.")]

# ── cam xclk ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam xclk  [<freq_khz>]"),
    label_value_table([
        ("Synopsis", "Configure the PWM output on GP11 that drives the camera XCLK pin."),
        ("Arguments",
         "<i>freq_khz</i> — Clock frequency in kHz. Range: 6 000 – 27 000. Default: 24 000."),
        ("Output",   "Prints the requested and actual achieved frequency."),
    ]),
    example_box(["cam xclk", "cam xclk 24000"]),
    divider(),
])]

# ── cam i2c ──────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam i2c"),
    label_value_table([
        ("Synopsis",  "Initialise the I²C1 bus on SDA=GP22, SCL=GP23 for OV5640 SCCB access."),
        ("Arguments", "None."),
        ("Output",    "Confirmation message."),
    ]),
    example_box(["cam i2c"]),
    divider(),
])]

# ── cam id ───────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam id"),
    label_value_table([
        ("Synopsis",    "Read OV5640 chip ID registers 0x300A / 0x300B and print the result."),
        ("Arguments",   "None."),
        ("Precondition","<font name='Courier'>cam i2c</font> must have been called first."),
        ("Output",      "Chip ID bytes. Expected value: <b>0x5640</b>."),
        ("Use",         "Confirm that the I²C wiring and address are correct before attempting sensor configuration."),
    ]),
    example_box(["cam i2c", "cam id", "> ID: 22080  # decimal for 0x5640"]),
    divider(),
])]

# ── cam defaults ─────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam defaults"),
    label_value_table([
        ("Synopsis",    "Write the full OV5640 default register table to the sensor (PLL, timing, colour)."),
        ("Arguments",   "None."),
        ("Precondition","<font name='Courier'>cam xclk</font> must be running. "
                        "<font name='Courier'>cam defaults</font> initialises SCCB/I²C internally."),
        ("Output",      "Confirmation message."),
        ("Note",        "Resets the sensor to 240×320, RGB565, 24 MHz PCLK. Run after power-on or if the "
                        "sensor appears unresponsive."),
    ]),
    example_box(["cam defaults"]),
    divider(),
])]

# ── cam size ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam size  <w>  <h>"),
    label_value_table([
        ("Synopsis",    "Change the capture resolution and reprogram the OV5640 output window."),
        ("Arguments",   "<i>w</i> — Width in pixels.  <i>h</i> — Height in pixels."),
        ("Precondition","Camera initialised. Any active pipeline must be stopped first "
                        "(<font name='Courier'>cam stop</font>)."),
        ("Output",      "Confirmation and reminder to re-run <font name='Courier'>cam alloc</font>, "
                        "<font name='Courier'>cam dma</font>, <font name='Courier'>cam start</font>."),
        ("Note",        "Default is 240×320 (portrait). Changing resolution requires a full "
                        "reinitialisation of alloc → dma → start."),
    ]),
    example_box(["cam stop", "cam size 240 320", "cam alloc", "cam dma", "cam start"]),
    divider(),
])]

# ── cam alloc ────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam alloc"),
    label_value_table([
        ("Synopsis",    "Allocate (or reallocate) the frame buffer on the heap: <i>width × height × 2</i> bytes."),
        ("Arguments",   "None."),
        ("Precondition","<font name='Courier'>cam defaults</font> (or <font name='Courier'>cam size</font>) must "
                        "have been called to set dimensions."),
        ("Output",      "Silent on success; error message if allocation fails."),
    ]),
    example_box(["cam alloc"]),
    divider(),
])]

# ── cam dma ──────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam dma"),
    label_value_table([
        ("Synopsis",    "Claim a DMA channel, install the IRQ handler, and arm the first transfer."),
        ("Arguments",   "None."),
        ("Precondition","<font name='Courier'>cam alloc</font> must have been called."),
        ("Output",      "Reports the claimed DMA channel number."),
        ("Note",        "Calling <font name='Courier'>cam dma</font> while the DMA is already running is safe — "
                        "it silently returns without reconfiguring."),
    ]),
    example_box(["cam dma", "> DMA_CH= 2"]),
    divider(),
])]

# ── cam start ────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam start"),
    label_value_table([
        ("Synopsis",    "Load the PIO program, start the state machine, and begin continuous frame capture."),
        ("Arguments",   "None."),
        ("Precondition","<font name='Courier'>cam dma</font> must have been called."),
        ("Output",      "Silent. Camera begins capturing immediately."),
        ("Note",        "If the PIO state machine is already running, this command is a no-op "
                        "(DMA re-arming is handled automatically by the IRQ handler)."),
    ]),
    example_box(["cam start"]),
    divider(),
])]

# ── cam stop ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam stop"),
    label_value_table([
        ("Synopsis",    "Abort DMA, disable the PIO state machine, and release resources."),
        ("Arguments",   "None."),
        ("Output",      "Silent."),
        ("Note",        "Must be called before changing resolution or reinitialising the sensor."),
    ]),
    example_box(["cam stop"]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("3.2  Sensor Register Access", toc_text="3.2  Sensor Register Access", toc_key="toc_3_2")]
story += [spacer(2)]

# ── cam rreg ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam rreg  <reg>"),
    label_value_table([
        ("Synopsis",    "Read one OV5640 register over I²C and print its value."),
        ("Arguments",   "<i>reg</i> — 16-bit register address in hex (no 0x prefix)."),
        ("Precondition","<font name='Courier'>cam i2c</font>."),
        ("Output",      "Register value in hex and decimal."),
    ]),
    example_box(["cam rreg 300a", "> reg[0x300A] = 0x56 (86)"]),
    divider(),
])]

# ── cam wreg ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam wreg  <reg>  <val>"),
    label_value_table([
        ("Synopsis",    "Write one OV5640 register over I²C and read it back to confirm."),
        ("Arguments",   "<i>reg</i> — Register address (hex).  <i>val</i> — Byte value (hex)."),
        ("Precondition","<font name='Courier'>cam i2c</font>."),
        ("Output",      "Confirmation with written value."),
    ]),
    example_box(["cam wreg 501f 01", "> wrote reg[0x501F] = 0x01"]),
    divider(),
])]

# ── cam flip / cam mirror ────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam flip  <val>"),
    label_value_table([
        ("Synopsis",    "Set vertical flip.  <i>val</i>: <b>0</b> = normal, <b>6</b> = flipped."),
        ("Precondition","<font name='Courier'>cam i2c</font>."),
        ("Output",      "Confirmation."),
    ]),
    example_box(["cam flip 0", "cam flip 6"]),
    divider(),
])]

story += [KeepTogether([
    cmd_header("cam mirror  <val>"),
    label_value_table([
        ("Synopsis",    "Set horizontal mirror.  <i>val</i>: <b>0</b> = normal, <b>6</b> = mirrored."),
        ("Precondition","<font name='Courier'>cam i2c</font>."),
        ("Output",      "Confirmation."),
        ("Note",        "Writing this register during an active stream can stall the VSYNC signal. "
                        "Prefer stopping the pipeline first, or use the LCD touch-mirror feature which "
                        "handles recovery automatically."),
    ]),
    example_box(["cam mirror 6"]),
    divider(),
])]

# ── cam pll ──────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam pll  <multiplier>"),
    label_value_table([
        ("Synopsis",    "Adjust OV5640 PLL multiplier to change PCLK speed and therefore frame rate."),
        ("Arguments",   "<i>multiplier</i> — Integer 4–252. Even values only above 127."),
        ("Precondition","<font name='Courier'>cam i2c</font>."),
        ("Output",      "Confirmation."),
        ("Note",        "Default is 11 (≈ 25 fps at 240×320). Higher values increase FPS but may cause "
                        "colour artefacts if PCLK exceeds the PIO sampling rate."),
    ]),
    example_box(["cam pll 11"]),
    divider(),
])]

# ── cam format ───────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam format  <rgb565 | yuv422>"),
    label_value_table([
        ("Synopsis",    "Select OV5640 output colour encoding."),
        ("Precondition","<font name='Courier'>cam i2c</font>."),
        ("Output",      "Confirmation."),
        ("Note",        "The display and SD-card pipeline expect <b>rgb565</b>. YUV422 is available for "
                        "raw data analysis only."),
    ]),
    example_box(["cam format rgb565"]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("3.3  Frame Capture to SD Card", toc_text="3.3  Frame Capture to SD Card", toc_key="toc_3_3")]
story += [spacer(2)]

# ── cam snap ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam snap  <filepath>",
               aliases="cam_snap · cam_capture"),
    label_value_table([
        ("Synopsis",
         "Capture one complete frame and write it to <i>filepath</i> on the SD card. "
         "The command is <b>non-blocking</b>: it returns the prompt immediately and prints a "
         "completion message when the file is closed."),
        ("Arguments",
         "<i>filepath</i> — Destination path on the SD card, e.g. "
         "<font name='Courier'>0:/cam/photo.bin</font>. "
         "The parent directory is created automatically if it does not exist."),
        ("Precondition",
         "<font name='Courier'>cam xclk</font> and <font name='Courier'>cam defaults</font> completed, and "
         "<font name='Courier'>mount 0:</font>."),
        ("Output",
         "VSYNC check result; sensor recovery message if VSYNC was stalled; "
         "completion line with file size when done."),
        ("File format",
         "Raw binary RGB565, row-major, width × height × 2 bytes. "
         "No header. To view: load into any image viewer that accepts raw RGB565 "
         "(e.g. <i>ffmpeg -s 240x320 -pix_fmt rgb565be -i photo.bin photo.png</i>)."),
        ("Recovery",
         "Before arming DMA the command checks VSYNC on GP8 for 100 ms. If no transitions "
         "are detected (sensor stalled after a mirror/flip write during streaming), the SCCB "
         "bus is re-initialised automatically and a second VSYNC check is performed. "
         "If VSYNC is still absent the capture is aborted with an error message."),
    ]),
    example_box([
        "mount 0:",
        "cam xclk",
        "cam defaults",
        "cam snap 0:/cam/test0.bin",
        "> VSYNC check (GP8): 48 transitions in 100 ms",
        "> Wrote 153600 bytes to 0:/cam/test0.bin",
    ]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("3.4  Status & Diagnostics", toc_text="3.4  Status & Diagnostics", toc_key="toc_3_4")]
story += [spacer(2)]

# ── cam status ───────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("cam status"),
    label_value_table([
        ("Synopsis",  "Print a full summary of camera state: resolution, buffer pointers, FPS, "
                      "DMA/IRQ flags, mirror and flip state."),
        ("Arguments", "None."),
        ("Output",    "Multi-line status block."),
    ]),
    example_box(["cam status",
                 "> res=240x320  buf=0x20012000  dma=busy  fps=24  mirror=0  flip=0"]),
    divider(),
])]

story += [PageBreak()]

# ═══════════════════════════════════════════════════════════════════════════════
# 4. LCD COMMANDS
# ═══════════════════════════════════════════════════════════════════════════════
story += [chapter_block("4  LCD Commands  (lcd …)", toc_text="4  LCD Commands  (lcd …)", toc_key="toc_4")]
story += [spacer(2)]
story += [body(
    "The LCD is 2\" ILI9488 panel (320×240) connected via the high-speed SPI peripheral at "
    "max frequency of 62.5 MHz. A CST216D touch controller is used for touch events. "
    "All LCD commands are accessed via the <b>lcd</b> group.")]
story += [spacer(3)]

story += [section_bar("4.1  Initialisation", toc_text="4.1  Initialisation", toc_key="toc_4_1")]
story += [spacer(2)]

# ── lcd init ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd init  [vertical | horizontal]",
               aliases="lcd_init"),
    label_value_table([
        ("Synopsis",
         "Full LCD panel initialisation: configures the SPI clock, runs the ILI9488 init "
         "sequence, allocates the display framebuffer, clears the screen to white, initialises "
         "the CST216D touch controller, and installs the NRF poll timer."),
        ("Arguments",
         "<i>vertical</i> (default) — Portrait orientation, 240 wide × 320 tall. "
         "<i>horizontal</i> — Landscape, 320 wide × 240 tall."),
        ("Output",   "Init progress messages; final confirmation."),
        ("Note",     "Run once on power-up. Running <font name='Courier'>lcd init</font> a second time "
                     "reinitialises cleanly. "
                     "<b>The backlight is off (0%) after init</b> — always follow with "
                     "<font name='Courier'>lcd bl 100</font> to turn the screen on."),
    ]),
    example_box(["lcd init vertical", "lcd bl 100", "> LCD ready — 240x320 vertical"]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("4.2  Drawing Primitives", toc_text="4.2  Drawing Primitives", toc_key="toc_4_2")]
story += [spacer(2)]
story += [note("For visible output on screen, run <font name='Courier'>lcd bl &lt;value&gt;</font> after <font name='Courier'>lcd init</font> (for example, <font name='Courier'>lcd bl 100</font>).")]
story += [spacer(1)]

# ── lcd bl ───────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd bl  <0-100>",
               aliases="lcd_bl"),
    label_value_table([
        ("Synopsis",    "Set backlight brightness as a percentage."),
        ("Arguments",   "Integer 0 (off) to 100 (full brightness)."),
        ("Precondition","<font name='Courier'>lcd init</font>."),
        ("Output",      "Confirmation."),
    ]),
    example_box(["lcd bl 80"]),
    divider(),
])]

# ── lcd clear ────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd clear  <color>",
               aliases="lcd_clear"),
    label_value_table([
        ("Synopsis",    "Fill the entire LCD panel with a solid colour."),
        ("Arguments",   "<i>color</i> — RGB565 value in hex (no 0x prefix). "
                        "Common values: <font name='Courier'>ffff</font>=white, "
                        "<font name='Courier'>0000</font>=black, "
                        "<font name='Courier'>f800</font>=red, "
                        "<font name='Courier'>07e0</font>=green, "
                        "<font name='Courier'>001f</font>=blue."),
        ("Precondition","<font name='Courier'>lcd init</font> and <font name='Courier'>lcd bl</font> must have been called."),
        ("Output",      "Confirmation."),
    ]),
    example_box(["lcd clear 0000", "lcd clear f800"]),
    divider(),
])]

# ── lcd pixel ────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd pixel  <x>  <y>  <color>",
               aliases="lcd_pixel"),
    label_value_table([
        ("Synopsis",    "Set a single pixel."),
        ("Arguments",   "<i>x, y</i> — Pixel coordinates (0-based, decimal). "
                        "<i>color</i> — RGB565 hex."),
        ("Precondition","<font name='Courier'>lcd init</font> and <font name='Courier'>lcd bl</font> must have been called."),
        ("Output",      "Confirmation, or error if coordinates are out of bounds."),
    ]),
    example_box(["lcd pixel 120 160 f800"]),
    divider(),
])]

# ── lcd fillrect ─────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd fillrect  <x>  <y>  <w>  <h>  <color>",
               aliases="lcd_fillrect"),
    label_value_table([
        ("Synopsis",    "Fill a rectangle with a solid colour."),
        ("Arguments",   "<i>x, y</i> — Top-left corner (decimal). "
                        "<i>w, h</i> — Width and height (decimal). "
                        "<i>color</i> — RGB565 hex."),
        ("Precondition","<font name='Courier'>lcd init</font> and <font name='Courier'>lcd bl</font> must have been called."),
        ("Output",      "Confirmation, or error if rectangle is out of bounds."),
    ]),
    example_box(["lcd fillrect 0 0 240 40 001f   # blue banner at top"]),
    divider(),
])]

# ── lcd orient ───────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd orient  <vertical | horizontal>",
               aliases="lcd_set_orientation · lcd orientation"),
    label_value_table([
        ("Synopsis",    "Switch scan direction between portrait and landscape without a full reinit."),
        ("Arguments",   "<i>vertical</i> or <i>horizontal</i>."),
        ("Precondition","<font name='Courier'>lcd init</font> and <font name='Courier'>lcd bl</font> should be set for visible output."),
        ("Output",      "Confirmation."),
        ("Note",        "<font name='Courier'>lcd load</font> requires <b>vertical</b> orientation "
                        "because raw camera files are 240×320."),
    ]),
    example_box(["lcd orient horizontal"]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("4.3  Image Loading from SD", toc_text="4.3  Image Loading from SD", toc_key="toc_4_3")]
story += [spacer(2)]

# ── lcd snap ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd snap",
               aliases="lcd_cam_snap"),
    label_value_table([
        ("Synopsis",
         "Capture one camera frame and display it on the LCD. <b>Non-blocking</b> — returns "
         "the prompt while capture and display happen in the background."),
        ("Precondition","<font name='Courier'>lcd init</font> + <font name='Courier'>lcd bl</font>, and camera ready "
                        "(short path: <font name='Courier'>cam xclk → cam defaults</font>; full path: "
                        "<font name='Courier'>cam xclk → cam i2c → cam defaults → cam alloc → cam dma → cam start</font>)."),
        ("Output",      "Startup message; background completion."),
    ]),
    example_box(["lcd snap"]),
    divider(),
])]

# ── lcd load ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd load  <file>",
               aliases="lcd_load_image"),
    label_value_table([
        ("Synopsis",
         "Load a raw 240×320 RGB565 binary file from SD and display it on the LCD. "
         "<b>Non-blocking.</b>"),
        ("Arguments",   "<i>file</i> — Path on SD card."),
        ("Precondition","<font name='Courier'>lcd init vertical</font> + <font name='Courier'>lcd bl</font> + <font name='Courier'>mount 0:</font>."),
        ("Output",      "Startup message; completion or error in background."),
        ("File format", "Raw binary RGB565, row-major, exactly 240×320×2 = 153 600 bytes. "
                        "Files saved by <font name='Courier'>cam snap</font> are directly compatible."),
    ]),
    example_box(["lcd load 0:/cam/test0.bin"]),
    divider(),
])]

# ── lcd unload ───────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd unload",
               aliases="lcd_unload_image"),
    label_value_table([
        ("Synopsis",    "Clear the display image buffer and fill the screen with black."),
        ("Arguments",   "None."),
        ("Precondition","<font name='Courier'>lcd init</font> and <font name='Courier'>lcd bl</font> should be set for visible output."),
        ("Output",      "Startup message; completion in background."),
    ]),
    example_box(["lcd unload"]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("4.4  Live Camera Stream", toc_text="4.4  Live Camera Stream", toc_key="toc_4_4")]
story += [spacer(2)]

# ── lcd stream ───────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd stream  [stop | -s | --stop]",
               aliases="lcd_cam_stream"),
    label_value_table([
        ("Synopsis",
         "Start or stop continuous camera-to-LCD preview. Each captured frame is transferred to "
         "the LCD as soon as it is ready. The command is <b>non-blocking</b> — the stream runs "
         "entirely in the background and the shell remains responsive."),
        ("Arguments",
         "No arguments — start the stream. "
         "<i>stop</i>, <i>-s</i>, or <i>--stop</i> — stop the stream."),
        ("Precondition",
         "<font name='Courier'>lcd init</font> + <font name='Courier'>lcd bl</font>, and camera ready. "
         "Two valid camera paths: short (<font name='Courier'>cam xclk → cam defaults</font>) "
         "or full (<font name='Courier'>cam xclk → cam i2c → cam defaults → cam alloc → cam dma → cam start</font>)."),
        ("Output",      "Status message on start/stop."),
        ("Performance", "Typical throughput on RP2350 at 150 MHz system clock: "
                        "≥ 15 fps at 240×320 RGB565. Check with <font name='Courier'>lcd fps</font>."),
        ("Note",        "While the stream is running the shell can still accept commands. "
                        "Issuing <font name='Courier'>cam snap</font> while streaming will temporarily "
                        "pause the stream for one frame."),
    ]),
    example_box([
        "lcd stream          # start",
        "lcd fps             # check frame rate",
        "lcd stream stop     # stop",
    ]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("4.5  Status & FPS", toc_text="4.5  Status & FPS", toc_key="toc_4_5")]
story += [spacer(2)]

# ── lcd status ───────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd status",
               aliases="lcd_status"),
    label_value_table([
        ("Synopsis",  "Print LCD state: initialisation status, orientation, backlight %, dimensions."),
        ("Arguments", "None."),
        ("Output",    "Multi-line status block."),
    ]),
    example_box(["lcd status",
                 "> LCD: init=yes  orient=vertical  bl=80%  240x320"]),
    divider(),
])]

# ── lcd fps ──────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("lcd fps",
               aliases="lcd_fps"),
    label_value_table([
        ("Synopsis",  "Print rolling camera-capture FPS and LCD-display FPS with cumulative frame counts."),
        ("Arguments", "None."),
        ("Output",    "FPS line. Update the reading by calling the command again."),
    ]),
    example_box(["lcd fps",
                 "> cam=24 fps (total=1248)  lcd=23 fps (total=1235)"]),
    divider(),
])]

story += [PageBreak()]

# ═══════════════════════════════════════════════════════════════════════════════
# 5. NRF SPI COMMANDS
# ═══════════════════════════════════════════════════════════════════════════════
story += [chapter_block("5  NRF SPI Commands  (nrf …)", toc_text="5  NRF SPI Commands  (nrf …)", toc_key="toc_5")]
story += [spacer(2)]
story += [body(
    "These commands control the high-speed SPI master interface that talks to the nRF54Lx "
    "co-processor (BM20_C board). The nRF device runs as an SPI slave and "
    "echoes received data back to the master for validation. "
    "All commands are accessed via the <b>nrf</b> group.")]
story += [spacer(2)]
story += [body(
    "Pin assignment (RP2350 side): "
    "<font name='Courier'>SCK=GP18  MOSI=GP19  MISO=GP20  CS=GP22</font>.")]
story += [spacer(2)]
story += [warn(
    "Maximum validated SPI speed for this board/wiring combination: <b>15 MHz</b>. "
    "At 18.75 MHz a systematic single-bit error at byte offset 2 of every transfer has "
    "been observed (signal integrity). Always initialise at 15 MHz unless testing on a "
    "PCB-routed assembly.")]
story += [spacer(3)]

story += [section_bar("5.1  Bus Initialisation", toc_text="5.1  Bus Initialisation", toc_key="toc_5_1")]
story += [spacer(2)]

# ── nrf init ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("nrf init  [<baud_hz>]",
               aliases="nrf_spi_init"),
    label_value_table([
        ("Synopsis",
         "Initialise the SPI master and configure the four dedicated HSSPI pins."),
        ("Arguments",
         "<i>baud_hz</i> — Baud rate in Hz. Default: <b>15 000 000</b>. "
         "Must be run before any other <font name='Courier'>nrf</font> command."),
        ("Output",
         "Pin mapping and achieved baud rate."),
    ]),
    example_box([
        "nrf init              # 15 MHz default",
        "nrf init 10000000     # explicit 10 MHz",
        "> NRF SPI init: SCK=GP18 MOSI=GP19 MISO=GP20 CS=GP22 baud=15000000",
    ]),
    divider(),
])]

# ── nrf status ───────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("nrf status",
               aliases="nrf_spi_status"),
    label_value_table([
        ("Synopsis",    "Show current SPI init state, pin mapping, and configured baud rate."),
        ("Arguments",   "None."),
        ("Output",      "Status block."),
    ]),
    example_box(["nrf status"]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("5.2  Manual Transfers", toc_text="5.2  Manual Transfers", toc_key="toc_5_2")]
story += [spacer(2)]

# ── nrf xfer ─────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("nrf xfer  <byte_hex> …",
               aliases="nrf_spi_xfer"),
    label_value_table([
        ("Synopsis",
         "Perform a single full-duplex SPI transfer with the supplied bytes, then print the "
         "received (MISO) bytes."),
        ("Arguments",
         "1–256 hex bytes, space-separated (no 0x prefix). "
         "Bytes are transmitted simultaneously in both directions."),
        ("Precondition","<font name='Courier'>nrf init</font>."),
        ("Output",      "TX bytes and RX bytes in hex."),
        ("Use",
         "Quick connectivity check. After one transfer the nRF slave will echo these bytes "
         "back on the <i>next</i> transfer (one-behind echo model)."),
    ]),
    example_box([
        "nrf xfer 01 02 03 04",
        "> TX: 01 02 03 04",
        "> RX: A5 5A A5 5A   # first transfer — slave had 0xFF in tx buffer",
        "nrf xfer 01 02 03 04",
        "> TX: 01 02 03 04",
        "> RX: 01 02 03 04   # slave echoed previous TX",
    ]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("5.3  Sweep", toc_text="5.3  Sweep", toc_key="toc_5_3")]
story += [spacer(2)]

# ── nrf sweep ────────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("nrf sweep  <start_hz>  <stop_hz>  <step_hz>  <loops>  <tx_hex>  <expect_hex>",
               aliases="nrf_spi_sweep"),
    label_value_table([
        ("Synopsis",
         "Iterate SPI clock rates from <i>start_hz</i> to <i>stop_hz</i> in increments of "
         "<i>step_hz</i>. At each rate, transmit <i>tx_hex</i> for <i>loops</i> repetitions "
         "and compare the echo against <i>expect_hex</i>. Reports pass/fail counts and the "
         "last failing RX bytes."),
        ("Arguments",
         "<i>start_hz</i> — Lowest rate to test (Hz).<br/>"
         "<i>stop_hz</i>  — Highest rate to test (Hz).<br/>"
         "<i>step_hz</i>  — Step between rates (Hz).<br/>"
         "<i>loops</i>    — Repetitions per rate (1–10 000).<br/>"
         "<i>tx_hex</i>   — Payload hex string (no spaces, even number of chars, max 256 B).<br/>"
         "<i>expect_hex</i> — Expected echo hex string (same length as tx_hex)."),
        ("Precondition","<font name='Courier'>nrf init</font>. nRF slave must be running echo firmware."),
        ("Output",      "One line per frequency: rate, pass count, fail count, first failure detail."),
        ("Note",
         "A warm-up transfer is sent at each rate before timed measurements begin. "
         "Reported pass/fail counts reflect the full <i>loops</i> timed transfers."),
    ]),
    example_box([
        "nrf sweep 1000000 20000000 1000000 100 A5A5A5A5 A5A5A5A5",
        ">  1 MHz: pass=100  fail=0",
        "> ...",
        "> 15 MHz: pass=100  fail=0",
        "> 18 MHz: pass=88  fail=12  rx[2]=A3 (expected A5)",
    ]),
    divider(),
])]

story += [spacer(2)]
story += [section_bar("5.4  Timing Tuning", toc_text="5.4  Timing Tuning", toc_key="toc_5_4")]
story += [spacer(2)]

# ── nrf timing ───────────────────────────────────────────────────────────────
story += [KeepTogether([
    cmd_header("nrf timing  [gap <µs> | setup <µs> | hold <µs> | chunk <rows>]",
               aliases="nrf_timing"),
    label_value_table([
        ("Synopsis",
         "Show or adjust Mode-B transfer timing parameters used by the streaming pipeline."),
        ("Arguments",
         "No arguments — display current values and FPS estimate.<br/>"
         "<i>gap &lt;µs&gt;</i>    — Inter-frame gap (µs). Default 200.<br/>"
         "<i>setup &lt;µs&gt;</i>  — CS setup time before first SCK edge (µs). Default 15.<br/>"
         "<i>hold &lt;µs&gt;</i>   — CS hold time after last SCK edge (µs). Default 4.<br/>"
         "<i>chunk &lt;rows&gt;</i> — LCD rows per SPI transfer (1–8). Default 4."),
        ("Precondition","<font name='Courier'>nrf init</font> (for display); no precondition for set."),
        ("Output",      "Current values + estimated streaming FPS; or confirmation of new setting."),
        ("Note",
         "Gap is automatically floored by the transfer engine to "
         "<font name='Courier'>400 + 2×frame_bytes</font> µs to satisfy nRF54Lx SPIS "
         "transaction restart requirements. Setting a smaller gap value here is safe — the "
         "floor will apply."),
    ]),
    example_box([
        "nrf timing",
        "> gap=200µs  setup=15µs  hold=4µs  chunk=4rows  est=~18fps",
        "",
        "nrf timing gap 50",
        "nrf timing chunk 8",
    ]),
    divider(),
])]

story += [PageBreak()]

# ═══════════════════════════════════════════════════════════════════════════════
# 6. USE CASES & WORKED EXAMPLES
# ═══════════════════════════════════════════════════════════════════════════════
story += [chapter_block("6  Use Cases & Worked Examples", toc_text="6  Use Cases & Worked Examples", toc_key="toc_6")]
story += [spacer(3)]

# 6.1 Verify camera sensor
story += [section_bar("6.1  Verify the Camera Sensor", toc_text="6.1  Verify the Camera Sensor", toc_key="toc_6_1")]
story += [spacer(2)]
story += [body(
    "Use this sequence to confirm that the OV5640 is wired correctly and responding to I²C "
    "before attempting any capture. A chip ID of 0x5640 confirms a healthy bus and correct "
    "device address. Both camera-init paths are shown.")]
story += [spacer(2)]
story += [body("<b>Option A — explicit I²C path</b>:")]
story += [spacer(1)]
story += [example_box([
    "cam xclk             # Start 24 MHz XCLK — sensor needs it to respond on I²C",
    "cam i2c              # Initialise I²C1 bus",
    "cam id               # Read chip ID",
    "> ID: 22080          # expected (0x5640)",
    "",
    "# If chip_id is 0x0000 or 0xFFFF:",
    "#   1. Check XCLK is present on GP11 with a scope or LED",
    "#   2. Verify SDA/SCL wiring (GP22/GP23)",
    "#   3. Confirm 3.3 V supply to sensor module",
    "cam rreg 300a        # Read high byte of chip ID directly",
    "> reg[0x300A] = 0x56",
])]
story += [spacer(2)]
story += [body("<b>Option B — short path via defaults</b>:")]
story += [spacer(1)]
story += [example_box([
    "cam xclk",
    "cam defaults         # includes SCCB/I²C initialisation",
    "cam id",
    "> ID: 22080          # expected (0x5640)",
])]
story += [spacer(3)]

# 6.2 Capture still photo
story += [section_bar("6.2  Capture a Still Photo to SD", toc_text="6.2  Capture a Still Photo to SD", toc_key="toc_6_2")]
story += [spacer(2)]
story += [body(
    "Capture a single frame from the camera and save it as a raw binary file. "
    "Two camera-init paths are shown below: short path and full path. "
    "The file can be transferred off the SD card and converted to PNG with ffmpeg.")]
story += [spacer(2)]
story += [body("<b>Option A — short path</b> (recommended for simple one-shot capture):")]
story += [spacer(1)]
story += [example_box([
    "mount 0:             # Mount the SD card",
    "cam xclk",
    "cam defaults         # Full OV5640 register init",
    "",
    "cam snap 0:/cam/photo.bin",
    "> VSYNC check (GP8): 48 transitions in 100 ms",
    "> Wrote 153600 bytes to 0:/cam/photo.bin",
    "",
    "ls 0:/cam            # Verify file",
    "> photo.bin  153600  2025-04-23 14:32",
])]
story += [spacer(2)]
story += [body("<b>Option B — full path</b> (explicit full camera pipeline setup):")]
story += [spacer(1)]
story += [example_box([
    "mount 0:",
    "cam xclk",
    "cam i2c",
    "cam defaults",
    "cam alloc",
    "cam dma",
    "cam start",
    "cam snap 0:/cam/photo_full.bin",
    "> Wrote 153600 bytes to 0:/cam/photo_full.bin",
])]
story += [spacer(2)]
story += [note(
    "Convert to PNG on a PC: "
    "<font name='Courier'>ffmpeg -s 240x320 -pix_fmt rgb565be -i photo.bin photo.png</font>")]
story += [spacer(2)]
story += [note(
    "Capture multiple frames in sequence by repeating "
    "<font name='Courier'>cam snap 0:/cam/photo_N.bin</font>. "
    "Wait for the completion message before issuing the next snap.")]
story += [spacer(3)]

# 6.3 Live camera preview
story += [section_bar("6.3  Live Camera Preview on LCD", toc_text="6.3  Live Camera Preview on LCD", toc_key="toc_6_3")]
story += [spacer(2)]
story += [body(
    "Display a continuous live video feed from the OV5640 camera on the ILI9488 LCD. "
    "Both camera-init paths are shown below.")]
story += [spacer(2)]
story += [body("<b>Option A — short camera path</b> (with <font name='Courier'>cam defaults</font>):")]
story += [spacer(1)]
story += [example_box([
    "# Initialise LCD first",
    "lcd init vertical",
    "lcd bl 100",
    "",
    "# Short camera init path",
    "cam xclk",
    "cam defaults",
    "",
    "# Start stream",
    "lcd stream",
    "lcd fps",
    "lcd stream stop",
])]
story += [spacer(2)]
story += [body("<b>Option B — full camera path</b> (explicit i2c/alloc/dma/start):")]
story += [spacer(1)]
story += [example_box([
    "# Initialise LCD first",
    "lcd init vertical",
    "lcd bl 100",
    "> LCD ready — 240x320 vertical",
    "",
    "# Initialise camera",
    "cam xclk",
    "cam i2c",
    "cam defaults",
    "cam alloc",
    "cam dma",
    "cam start",
    "",
    "# Start stream",
    "lcd stream",
    "> LCD stream started",
    "",
    "lcd fps              # Check frame rate — expect 15-24 fps",
    "> cam=24 fps (total=2048)  lcd=23 fps (total=2031)",
    "",
    "# Freeze a frame on LCD without stopping stream",
    "cam snap 0:/cam/freeze.bin",
    "",
    "# Stop stream",
    "lcd stream stop",
    "> LCD stream stopped",
])]
story += [spacer(3)]

# 6.4 SPI connectivity test
story += [section_bar("6.4  SPI Loopback / Connectivity Test", toc_text="6.4  SPI Loopback / Connectivity Test", toc_key="toc_6_4")]
story += [spacer(2)]
story += [body(
    "Verify SPI wiring between the RP2350 and the nRF54Lx with production-documented commands. "
    "Use repeated <font name='Courier'>nrf xfer</font> transactions to confirm stable echo behavior.")]
story += [spacer(2)]
story += [example_box([
    "nrf init             # Initialise at 15 MHz",
    "",
    "# Manual echo test",
    "nrf xfer AA BB CC DD  # prime slave",
    "nrf xfer AA BB CC DD  # should echo AA BB CC DD",
    "> RX: AA BB CC DD",
])]
story += [spacer(3)]

# 6.5 SPI reliability sweep
story += [section_bar("6.5  SPI Reliability Boundary Sweep", toc_text="6.5  SPI Reliability Boundary Sweep", toc_key="toc_6_5")]
story += [spacer(2)]
story += [body(
    "Use <font name='Courier'>nrf sweep</font> to characterize pass/fail behavior across SPI rates "
    "with production-documented tooling. This provides a reproducible boundary check for wiring quality.")]
story += [spacer(2)]
story += [example_box([
    "nrf init 15000000    # Confirmed max reliable rate for dev-kit wiring",
    "",
    "# Sweep to find the reliability boundary of your specific wiring",
    "nrf sweep 5000000 18750000 1000000 200 A5A5A5A5 A5A5A5A5",
    ">  5 MHz: pass=200  fail=0",
    "> 15 MHz: pass=200  fail=0",
    "> 16 MHz: pass=195  fail=4    # marginal — wiring-dependent",
    "> 18 MHz: pass=170  fail=30   # unreliable on jumper wires",
])]
story += [spacer(2)]
story += [note(
    "On a PCB-routed assembly the reliability boundary typically moves to "
    "≥ 18.75 MHz. Always re-validate with <font name='Courier'>nrf sweep</font> "
    "on the target hardware.")]

story += [PageBreak()]

# ═══════════════════════════════════════════════════════════════════════════════
# 7. QUICK REFERENCE TABLE
# ═══════════════════════════════════════════════════════════════════════════════
story += [chapter_block("7  Command Quick Reference", toc_text="7  Command Quick Reference", toc_key="toc_7")]
story += [spacer(3)]

def qr_section(title, rows):
    out = [section_bar(title), spacer(1)]
    hdr = [
        Paragraph("<b>Command</b>", sBodySmall),
        Paragraph("<b>Syntax summary</b>", sBodySmall),
        Paragraph("<b>Description</b>", sBodySmall),
    ]
    data = [hdr]
    for cmd, syn, desc in rows:
        data.append([
            Paragraph(f'<font name="Courier" size="8">{cmd}</font>', sBodySmall),
            Paragraph(f'<font name="Courier" size="7.5">{syn}</font>', sBodySmall),
            Paragraph(desc, sBodySmall),
        ])
    tbl = Table(data, colWidths=[28*mm, 62*mm, PAGE_W - 2*MARGIN - 94*mm])
    tbl.setStyle(TableStyle([
        ("BACKGROUND",    (0,0), (-1,0), C_BG_HEAD),
        ("ROWBACKGROUNDS",(0,1), (-1,-1), [C_WHITE, C_BG_SEC]),
        ("LINEBELOW",     (0,0), (-1,-1), 0.4, C_DIVIDER),
        ("TOPPADDING",    (0,0), (-1,-1), 3),
        ("BOTTOMPADDING", (0,0), (-1,-1), 3),
        ("LEFTPADDING",   (0,0), (-1,-1), 4),
        ("VALIGN",        (0,0), (-1,-1), "TOP"),
        ("FONTNAME",      (0,0), (-1,0), "Helvetica-Bold"),
    ]))
    out += [tbl, spacer(3)]
    return out

story += qr_section("Camera  (cam …)", [
    ("cam xclk",     "cam xclk [freq_khz]",           "Set OV5640 XCLK (PWM on GP11). Default 24 000 kHz."),
    ("cam i2c",      "cam i2c",                        "Init I²C1 bus for SCCB register access."),
    ("cam id",       "cam id",                         "Read and print OV5640 chip ID (expect 0x5640)."),
    ("cam defaults", "cam defaults",                   "Apply full OV5640 default register table."),
    ("cam size",     "cam size <w> <h>",               "Change capture resolution and reprogram sensor."),
    ("cam alloc",    "cam alloc",                      "Allocate frame buffer (w×h×2 bytes)."),
    ("cam dma",      "cam dma",                        "Claim DMA channel, install IRQ, arm transfer."),
    ("cam start",    "cam start",                      "Start PIO state machine and begin capture."),
    ("cam stop",     "cam stop",                       "Abort DMA and stop PIO state machine."),
    ("cam snap",     "cam snap <filepath>",            "Capture one frame to SD card (non-blocking)."),
    ("cam status",   "cam status",                     "Print camera state summary."),
    ("cam rreg",     "cam rreg <reg>",                 "Read one OV5640 register (hex addr)."),
    ("cam wreg",     "cam wreg <reg> <val>",           "Write one OV5640 register (hex addr, hex val)."),
    ("cam flip",     "cam flip <0|6>",                 "Vertical flip (0=normal, 6=flipped)."),
    ("cam mirror",   "cam mirror <0|6>",               "Horizontal mirror (0=normal, 6=mirrored)."),
    ("cam pll",      "cam pll <mult>",                 "Set PLL multiplier (affects FPS)."),
    ("cam format",   "cam format <rgb565|yuv422>",     "Set colour output format."),
])

story += qr_section("LCD  (lcd …)", [
    ("lcd init",      "lcd init [vertical|horizontal]", "Full LCD init (SPI, panel, touch, framebuffer)."),
    ("lcd bl",        "lcd bl <0-100>",                 "Set backlight brightness (%)."),
    ("lcd clear",     "lcd clear <rgb565_hex>",         "Fill screen with solid colour."),
    ("lcd pixel",     "lcd pixel <x> <y> <color>",      "Set one pixel."),
    ("lcd fillrect",  "lcd fillrect <x> <y> <w> <h> <c>","Fill a rectangle."),
    ("lcd orient",    "lcd orient <vertical|horizontal>","Switch portrait/landscape."),
    ("lcd snap",      "lcd snap",                       "Capture one frame and display (non-blocking)."),
    ("lcd load",      "lcd load <file>",                "Load 240×320 RGB565 file from SD and display."),
    ("lcd unload",    "lcd unload",                     "Clear display to black."),
    ("lcd stream",    "lcd stream [stop]",              "Start/stop continuous camera preview."),
    ("lcd status",    "lcd status",                     "Print LCD state summary."),
    ("lcd fps",       "lcd fps",                        "Print camera and display FPS counters."),
])

story += qr_section("NRF SPI  (nrf …)", [
    ("nrf init",    "nrf init [baud_hz]",                                  "Init SPI master (default 15 MHz)."),
    ("nrf status",  "nrf status",                                           "Show SPI init state and baud rate."),
    ("nrf xfer",    "nrf xfer <hex_bytes…>",                               "Single manual SPI transfer (hex bytes)."),
    ("nrf sweep",   "nrf sweep <start> <stop> <step> <n> <tx> <expect>",  "Sweep SPI rates, report pass/fail."),
    ("nrf timing",  "nrf timing [gap|setup|hold|chunk <val>]",             "Show/set Mode-B stream timing."),
])

# ── Build ─────────────────────────────────────────────────────────────────────
doc.multiBuild(story, onFirstPage=on_page, onLaterPages=on_page)
print(f"PDF written: {OUT_FILE}")
