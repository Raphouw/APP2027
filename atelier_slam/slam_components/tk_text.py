"""Avoid non-BMP pictograms which crash Tk 8.6/font fallback on this host."""


def tk_safe_text(value):
    return ''.join(char if ord(char) <= 0xffff else '[symbole]' for char in str(value))
