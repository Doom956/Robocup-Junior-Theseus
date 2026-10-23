# Competition camera files (tested 2026-10-22 night)

- `FINAL_2026-10-22/left/main.py`, `.../right/main.py`: exactly what is on each camera (debug output off).
- `FINAL_2026-10-22_debug/left/main.py`, `.../right/main.py`: the same with debug printing and drawing on,
  for checks in the OpenMV IDE (these were the files run in the home tests).
- Put each one on its own camera as `main.py`, next to that camera's own `labels.txt` and `trained.tflite`
  (same model as `Camera_models/trained.tflite`). Don't copy `labels.txt` from a Windows git checkout
  (CRLF line endings make the letters disappear).
- What changed and why: `Camera_models/VISION_NOTES.md`, "Competition version".
