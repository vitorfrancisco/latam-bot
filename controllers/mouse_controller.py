import ctypes
import serial
import win32gui

ARDUINO_PORT = "COM5"

def move_mouse_and_click(dx, dy, window_title, port=ARDUINO_PORT):
    if not is_window_in_foreground(window_title):
        print("⛔ Ação cancelada. Janela não está em primeiro plano ou está minimizada.")
        return

    try:
        with serial.Serial(port, 9600, timeout=1) as arduino:
            move_command = f"move {dx} {dy}\n"
            arduino.write(move_command.encode())
            print(f"🖱️ Move: {move_command.strip()}")

            arduino.write(b"click\n")
            print("🖱️ Click enviado")
    except Exception as e:
        print(f"Erro na comunicação com Arduino: {e}")

def only_click(dx, dy, window_title, port=ARDUINO_PORT):
    if not is_window_in_foreground(window_title):
        print("⛔ Ação cancelada. Janela não está em primeiro plano ou está minimizada.")
        return

    try:
        with serial.Serial(port, 9600, timeout=1) as arduino:
            arduino.write(b"click\n")
            print("🖱️ Click enviado")
    except Exception as e:
        print(f"Erro na comunicação com Arduino: {e}")

def get_mouse_position():
    pt = ctypes.wintypes.POINT()
    ctypes.windll.user32.GetCursorPos(ctypes.byref(pt))
    return pt.x, pt.y


def is_window_in_foreground(window_title):
    """Verifica se a janela está em primeiro plano e visível."""
    hwnd = win32gui.FindWindow(None, window_title)
    if hwnd == 0:
        print("❌ Janela não encontrada.")
        return False

    foreground = win32gui.GetForegroundWindow()
    if hwnd != foreground:
        print("⚠️ A janela não está em primeiro plano.")
        return False

    if win32gui.IsIconic(hwnd):
        print("⚠️ A janela está minimizada.")
        return False

    return True

def press_key(key, window_title, port=ARDUINO_PORT):
    if not is_window_in_foreground(window_title):
        print("⛔ Ação cancelada. Janela não está em primeiro plano ou está minimizada.")
        return

    try:
        with serial.Serial(port, 9600, timeout=1) as arduino:
            command = f"key {key}\n"
            arduino.write(command.encode())
            print(f"⌨️ Tecla enviada: {key}")
    except Exception as e:
        print(f"Erro na comunicação com Arduino: {e}")