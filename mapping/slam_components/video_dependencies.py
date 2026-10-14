"""Chargement optionnel de Pillow pour l’interface vidéo."""

try:
    from PIL import Image, ImageTk
    HAS_PILLOW = True
except Exception:
    Image = ImageTk = None
    HAS_PILLOW = False
