import ctypes
from ctypes import wintypes
from PIL import Image
user32 = ctypes.windll.user32; gdi32 = ctypes.windll.gdi32
def capture(widget, path, crop=None):
	widget.update_idletasks(); widget.update()
	hwnd = int(widget.winfo_toplevel().wm_frame(), 16)
	r = wintypes.RECT(); user32.GetWindowRect(hwnd, ctypes.byref(r))
	w, h = r.right - r.left, r.bottom - r.top
	hdc = user32.GetWindowDC(hwnd); mdc = gdi32.CreateCompatibleDC(hdc)
	bmp = gdi32.CreateCompatibleBitmap(hdc, w, h); gdi32.SelectObject(mdc, bmp)
	user32.PrintWindow(hwnd, mdc, 2)
	class BMI(ctypes.Structure):
		_fields_ = [("biSize", ctypes.c_uint32), ("biWidth", ctypes.c_int32), ("biHeight", ctypes.c_int32),
		            ("biPlanes", ctypes.c_uint16), ("biBitCount", ctypes.c_uint16), ("biCompression", ctypes.c_uint32),
		            ("biSizeImage", ctypes.c_uint32), ("a", ctypes.c_int32), ("b", ctypes.c_int32),
		            ("c", ctypes.c_uint32), ("d", ctypes.c_uint32)]
	bmi = BMI(); bmi.biSize = ctypes.sizeof(BMI); bmi.biWidth = w; bmi.biHeight = -h
	bmi.biPlanes = 1; bmi.biBitCount = 32
	buf = ctypes.create_string_buffer(w * h * 4)
	gdi32.GetDIBits(mdc, bmp, 0, h, buf, ctypes.byref(bmi), 0)
	img = Image.frombuffer("RGBA", (w, h), buf, "raw", "BGRA", 0, 1).convert("RGB")
	gdi32.DeleteObject(bmp); gdi32.DeleteDC(mdc); user32.ReleaseDC(hwnd, hdc)
	if crop:
		img = img.crop(crop)
	img.save(path)
	return r

