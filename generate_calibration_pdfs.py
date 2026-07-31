#!/usr/bin/env python3
"""Build print-accurate PDF sheets from the generated fiducial PNG assets."""

from pathlib import Path

from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "data" / "fiducial_assets"
OUTPUT = ROOT / "output" / "pdf"
WEB = ROOT / "static" / "calibration-assets"


def header(pdf, title, subtitle):
    pdf.setFont("Helvetica-Bold", 18)
    pdf.drawString(16 * mm, 194 * mm, title)
    pdf.setFont("Helvetica", 9)
    pdf.drawString(16 * mm, 188 * mm, subtitle)
    pdf.setLineWidth(0.25)
    pdf.line(16 * mm, 184 * mm, 281 * mm, 184 * mm)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    WEB.mkdir(parents=True, exist_ok=True)

    charuco = OUTPUT / "mycobot_charuco_board.pdf"
    pdf = canvas.Canvas(str(charuco), pagesize=landscape(A4))
    header(pdf, "myCobot Camera Lens Calibration", "Print at Actual Size / 100%. Do not use Fit or Scale to Page.")
    pdf.drawImage(str(ASSETS / "charuco_apriltag_36h11_7x5.png"), 43.5 * mm, 25 * mm, width=210 * mm, height=150 * mm, mask="auto")
    pdf.setFont("Helvetica", 8)
    pdf.drawCentredString(148.5 * mm, 17 * mm, "Board must measure exactly 210 x 150 mm across the outside edges.")
    pdf.save()

    tags = OUTPUT / "mycobot_workspace_tags.pdf"
    pdf = canvas.Canvas(str(tags), pagesize=landscape(A4))
    header(pdf, "myCobot Workspace Reference Tags", "Cut out tags 0-3. The black square of every tag must measure exactly 50 x 50 mm.")
    for marker_id, (x, y) in enumerate(((35, 105), (105, 105), (175, 105), (245, 105))):
        image = ASSETS / f"workspace_tag_{marker_id}_50mm.png"
        pdf.drawImage(str(image), (x - 25) * mm, (y - 25) * mm, width=50 * mm, height=50 * mm, mask="auto")
        pdf.setFont("Helvetica-Bold", 12)
        pdf.drawCentredString(x * mm, (y - 32) * mm, f"ID {marker_id}")
    pdf.setLineWidth(1)
    pdf.line(25 * mm, 35 * mm, 75 * mm, 35 * mm)
    for tick in range(0, 51, 10):
        pdf.line((25 + tick) * mm, 33 * mm, (25 + tick) * mm, 37 * mm)
    pdf.setFont("Helvetica", 9)
    pdf.drawString(25 * mm, 26 * mm, "Verification ruler: this line must measure exactly 50 mm")
    pdf.setFont("Helvetica-Bold", 9)
    pdf.drawString(120 * mm, 35 * mm, "Keep all tags flat, rigid, visible, and outside the robot's handling area.")
    pdf.save()

    object_tags = OUTPUT / "mycobot_object_tags_10_25.pdf"
    pdf = canvas.Canvas(str(object_tags), pagesize=landscape(A4))
    header(pdf, "myCobot Object Tags 10–25", "Print at Actual Size / 100%. Every black square must measure exactly 30 x 30 mm.")
    positions = [(28 + column * 34, 155 - row * 43) for row in range(4) for column in range(4)]
    for marker_id, (x, y) in zip(range(10, 26), positions):
        image = ASSETS / f"object_tag_{marker_id}_30mm.png"
        pdf.drawImage(str(image), (x - 15) * mm, (y - 15) * mm, width=30 * mm, height=30 * mm, mask="auto")
        pdf.setFont("Helvetica-Bold", 8)
        pdf.drawCentredString(x * mm, (y - 19) * mm, f"ID {marker_id}")
        pdf.setLineWidth(0.2)
        pdf.rect((x - 17) * mm, (y - 17) * mm, 34 * mm, 36 * mm)
    pdf.setLineWidth(1)
    pdf.line(230 * mm, 28 * mm, 260 * mm, 28 * mm)
    pdf.setFont("Helvetica", 8)
    pdf.drawString(225 * mm, 21 * mm, "Verification line: 30 mm")
    pdf.save()

    for source in (charuco, tags, object_tags):
        (WEB / source.name).write_bytes(source.read_bytes())


if __name__ == "__main__":
    main()
