import tkinter as tk
import rospy
from std_msgs.msg import String
from PIL import Image, ImageTk, ImageSequence

class ROSInterface:
    def __init__(self, app):
        rospy.init_node('ros_interface', anonymous=True)
        self.pub_continue_move = rospy.Publisher('/continue_move', String, queue_size=10)
        self.app = app
        rospy.Subscriber('/boton', String, self.callback)

    def send_continue_move(self):
        self.pub_continue_move.publish("continue_move")
        rospy.loginfo("Published 'continue_move' to /continue_move")
        self.app.hide_gracias_button()

    def callback(self, msg):
        if msg.data == "GRACIAS":
            self.app.show_gracias_button()

class App:
    def __init__(self, root):
        self.root = root
        self.root.attributes('-fullscreen', True)
        self.root.configure(bg='#fafafa')  # Fondo gris muy claro

        # Crear un canvas para mostrar el GIF
        self.canvas = tk.Canvas(self.root, bg='#fafafa', width=root.winfo_screenwidth(), height=root.winfo_screenheight())
        self.canvas.pack(fill='both', expand=True)

        # Cargar el GIF animado usando PIL
        self.gif = Image.open('/home/ubuntu/catkin_ws/src/my_python_scripts/scripts/mee6_wii_chicken_face.gif')
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        new_width = int(screen_width * 0.8)
        new_height = int(screen_height * 0.8)
        self.frames = self.create_resized_frames(self.gif, new_width, new_height)
        self.current_frame = 0

        # Calcular nueva posición para mover 25% arriba
        new_y_position = screen_height // 2 - int(screen_height * 0.25)

        # Mostrar el primer frame del GIF en el canvas
        self.gif_id = self.canvas.create_image(screen_width // 2, new_y_position, image=self.frames[self.current_frame])

        # Iniciar la animación del GIF
        self.animate_gif()

        self.interface = ROSInterface(self)

        # Crear botón en la parte inferior derecha, movido más a la izquierda y un poco más arriba
        self.gracias_button = tk.Button(self.root, text='Gracias!', command=self.interface.send_continue_move, height=3, width=12, font=('Helvetica', 48), bg='black', fg='white')
        self.gracias_button_window = self.canvas.create_window(screen_width - 300, screen_height - 150, anchor='center', window=self.gracias_button)
        
        # Inicialmente ocultar el botón
        self.canvas.itemconfigure(self.gracias_button_window, state='hidden')

    def create_resized_frames(self, gif, width, height):
        frames = []
        for frame in ImageSequence.Iterator(gif):
            resized_frame = frame.copy().resize((width, height), Image.LANCZOS)
            frames.append(ImageTk.PhotoImage(resized_frame.convert('RGBA')))
        return frames

    def animate_gif(self):
        self.current_frame = (self.current_frame + 1) % len(self.frames)
        self.canvas.itemconfig(self.gif_id, image=self.frames[self.current_frame])
        self.root.after(1, self.animate_gif)  # Cambiar el frame cada 100 ms

    def show_gracias_button(self):
        self.canvas.itemconfigure(self.gracias_button_window, state='normal')

    def hide_gracias_button(self):
        self.canvas.itemconfigure(self.gracias_button_window, state='hidden')

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
