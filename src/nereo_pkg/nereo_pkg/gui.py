import rclpy
from rclpy.node import Node
import sensor_msgs.msg

import cv2
from cv_bridge import CvBridge

import tkinter as tk
import customtkinter as ctk
from PIL import Image

import threading

def main(args = None) -> None:
    rclpy.init(args=args)

    app = Control_RPI_GUI()
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(app.camera_frame.camera_1_sub_node,)
    )
    spin_thread.start()

    app.mainloop()
    rclpy.shutdown()

class Control_RPI_GUI(ctk.CTk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Nereo control station")
        self.geometry(f'{self.winfo_screenwidth()}x{self.winfo_screenheight()}')
        self.minsize(800,600)
        self.camera_frame = CameraFrame(self)
        self.control_frame = ControlFrame(self)
        control_frame_width = 0.2
        self.camera_frame.place(rely = 0, relx = 0, relwidth = 1-control_frame_width, relheight = 1)
        self.control_frame.place(rely = 0, relx = 1-control_frame_width, relwidth = control_frame_width, relheight = 1)

class CameraFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)

        self.label_1 = ctk.CTkLabel(self, text='')
        self.label_1.pack(fill=tk.BOTH, expand = True)

        self.camera_1_sub_node = CameraSubNode(1)
        self.camera_1_sub_node.create_subscription(
                sensor_msgs.msg.Image,
                'camera1',
                self.update_frame_callback,
                5
            )
        self.camera_1_sub_node.declare_parameters(
            namespace='',
            parameters=[
                ('fps', 30),
                ('size', (720,480))
            ]
        )
        self.photo_image_1 = None
        self.after(1, self.update_image)

    def update_frame_callback(self, msg: sensor_msgs.msg.Image):
        frame = self.camera_1_sub_node.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        photo = ctk.CTkImage(dark_image=image, size=self.camera_1_sub_node.get_parameter('size'))
        self.photo_image_1 = photo

    def update_image(self):
        if self.photo_image_1:
            self.label_1.configure(image = self.photo_image_1)
            self.label_1._image = self.photo_image_1
        self.after(int(1000/self.camera_1_sub_node.get_parameter('fps')), self.update_image)

class ControlFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        self.is_rov_running = False
        super().__init__(master, **kwargs, bg_color='black')
        self.label = ctk.CTkLabel(self, text='Control Frame: furure implementation\n of the controller regulator', fg_color='steelblue', bg_color='steelblue')
        self.label.pack(fill = tk.BOTH, expand = True, side = tk.BOTTOM)
        # peripheral status panel
        self.peripheral_status_frame = PeripheralStatusFrame(self)
        self.peripheral_status_frame.pack(fill = tk.BOTH, expand = True, side = tk.TOP)
        self.central_frame = ctk.CTkFrame(self)
        # pitch & roll values panel
        self.pitch_and_roll_frame = PitchAndRollFrame(self, fg_color='royalblue')
        self.pitch_and_roll_frame.pack(fill = tk.BOTH, expand = True, side = tk.LEFT)
        # start & stop button
        self.start_stop_frame = StartStopFrame(self.central_frame)
        self.start_stop_frame.pack(fill = tk.BOTH, expand = True, side = tk.RIGHT)
        self.central_frame.pack(fill = tk.BOTH, expand = True)
        # controller regulator?

class PeripheralStatusFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self._peripheral_status_dictionary = {}

    @property
    def peripheral_status_dictionary(self):
        return self._peripheral_status_dictionary
    
    @peripheral_status_dictionary.setter
    def peripheral_status_dictionary(self, dictionary):
        self._peripheral_status_dictionary = dictionary
        for widget in self.winfo_children():
            widget.destroy()
        i = 0
        for key, value in dictionary.items():
            bgcolor, fgcolor = ('steelblue', 'steelblue') if i%2==0 else ('royalblue', 'royalblue')
            lbl = ctk.CTkLabel(self, text = f'{key}: {value}', height = 7, bg_color=bgcolor, fg_color=fgcolor)
            lbl.pack(fill = tk.BOTH, expand = True)
            i+=1

class PitchAndRollFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.pitch_label = ctk.CTkLabel(self, text='Pitch')
        self.pitch_label.pack(fill = tk.BOTH, expand = True)
        self.roll_label = ctk.CTkLabel(self, text='Roll')
        self.roll_label.pack(fill = tk.BOTH, expand = True)
        self._pitch, self._roll = 0, 0
    @property
    def pitch(self):
        return self._pitch
    @property
    def roll(self):
        return self._roll
    @pitch.setter
    def pitch(self, value):
        self._pitch = value
        self.pitch_label.configure(text = f'Pitch: {value}')
    @roll.setter
    def roll(self, value):
        self._roll = value
        self.roll_label.configure(text = f'Roll: {value}')

class  StartStopFrame(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        self.is_rov_running = False
        super().__init__(master, **kwargs)
        self.start_button = ctk.CTkButton(self, text = 'START', command = self.start_rov, bg_color='green', fg_color='green')
        self.start_button.pack(fill = tk.BOTH, expand = True, side = tk.RIGHT)
        self.stop_button = ctk.CTkButton(self, text = 'STOP', command = self.stop_rov, bg_color='red', fg_color='red')
    
    def start_rov(self):
        if self.is_rov_running: return
        self.is_rov_running = True
        self.start_button.pack_forget()
        self.stop_button.pack(fill = tk.BOTH, expand = True, side = tk.LEFT)

    def stop_rov(self):
        if not(self.is_rov_running): return
        self.is_rov_running = False
        self.stop_button.pack_forget()
        self.start_button.pack(fill = tk.BOTH, expand = True, side = tk.RIGHT)

class CameraSubNode(Node):
    def __init__(self, camera_number: int):
        super().__init__(f'camera_{str(camera_number)}_subscription')
        self.bridge = CvBridge()

class SensorDataSubNode(Node):
    def __init__(self):
        super().__init__('pitch_and_roll_subscriber')

class PeripheralStatusSubNode(Node):
    def __init__(self):
        super().__init__('peripheral_status_subscriber')