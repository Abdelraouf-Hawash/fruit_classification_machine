# BY: Abdelraouf Hawash
# DATE: 9/4/2023
# GMAIL: abdelraouf.hawash@gmail.com

#### fruit classification machine ####

import cv2
import tkinter as tk
import PIL.Image, PIL.ImageTk
import tensorflow as tf
import numpy as np
import os
import serial
import time


class App:

    def __init__(self,window,window_title):

        # declare some parameters
        self.startCase = False
        self.fruit_is_normal = True
        self.models = os.listdir('./models/') # get list with model names 
        
        # Graphic user interface
        self.window = window
        self.window.title(window_title)
        window.configure(bg='gray65')
        self.window.resizable(width=False, height=False)
            # arduino port selection
        self.port_label = tk.Label(window, bg='gray65', text="Arduino port")
        self.port_entry = tk.Entry(window, width=15, borderwidth=1)
        self.port_entry.insert(0, "COM1")
            # camera source selection
        self.cam_label = tk.Label(window, bg='gray65', text="Cam source")
        self.cam_ID = tk.IntVar(window)
        self.cam_ID.set(1) # default value
        self.cam_menu= tk.OptionMenu(window, self.cam_ID, 0,1, 2, 3,4)
            # model selection
        self.model_label = tk.Label(window, bg='gray65', text="Model")
        self.model_ID = tk.StringVar(window)
        self.model_ID.set("lemon_quality") # default value
        self.model_menu= tk.OptionMenu(window, self.model_ID, *self.models)
            # speed and PID detection
        self.speed_label = tk.Label(window, bg='gray65', text="Speed (cm)")
        self.speed_entry = tk.Entry(window, width=10,  borderwidth=1)
        self.speed_entry.insert(0, "6.0")
        self.current_speed_label = tk.Label(window, bg='gray65')
        self.Kp_label = tk.Label(window, bg='gray65', text="KP")
        self.Kp_entry = tk.Entry(window, width=10, borderwidth=1)
        self.Kp_entry.insert(0, "0.3")
        self.Ki_label = tk.Label(window, bg='gray65', text="Ki")
        self.Ki_entry = tk.Entry(window, width=10, borderwidth=1)
        self.Ki_entry.insert(0, "0.001")
        self.kd_label = tk.Label(window, bg='gray65', text="Kd")
        self.kd_entry = tk.Entry(window, width=10, borderwidth=1)
        self.kd_entry.insert(0, "0.002")
            # white color sensitivity
        self.sensitivity_label = tk.Label(window, bg='gray65', text="White sensitivity")
        self.sensitivity_entry = tk.Entry(window, width=10, borderwidth=1)
        self.sensitivity_entry.insert(0, "100")
            # classification score threshold
        self.score_threshold_label = tk.Label(window, bg='gray65', text="Score threshold")
        self.score_threshold_entry = tk.Entry(window, width=10, borderwidth=1)
        self.score_threshold_entry.insert(0, "0.2")
            # canvas for showing image
        self.canvas = tk.Canvas(window, bg="white",width=640,height = 480)
            # start button
        self.start_button = tk.Button(window, text="start", width=15,bd=4,font="arial 10 bold",command = self.start)
            # label give user info
        self.note_label = tk.Label(window, bg='gray65', width=50,font="arial 10 bold")
            # positions
        self.port_label.grid(row=0, column=0)
        self.port_entry.grid(row=0,column=1)
        self.cam_label.grid(row=1, column=0)
        self.cam_menu.grid(row=1, column=1)
        self.model_label.grid(row=2, column=0)
        self.model_menu.grid(row=2, column=1)
        self.speed_label.grid(row=3,column=0)
        self.speed_entry.grid(row=3,column=1)
        self.Kp_label.grid(row=4, column=0)
        self.Kp_entry.grid(row=4,column=1)
        self.Ki_label.grid(row=5,column=0)
        self.Ki_entry.grid(row=5,column=1)
        self.kd_label.grid(row=6, column=0)
        self.kd_entry.grid(row=6,column=1)
        self.sensitivity_label.grid(row=7, column=0)
        self.sensitivity_entry.grid(row=7,column=1)
        self.score_threshold_label.grid(row=8, column=0)
        self.score_threshold_entry.grid(row=8,column=1)
        self.start_button.grid(row=9, column=0,columnspan=2)
        self.current_speed_label.grid(row=10, column=0, columnspan=2, pady=10)
        self.note_label.grid(row=10, column=2)
        self.canvas.grid(row=0, column=2, rowspan=10, padx=30,pady=30)

        # mainloop
        self.window.mainloop()

    def start(self):
        if not self.startCase :

            # start arduino communication
            try:
                Port = str(self.port_entry.get()).strip()
                self.arduino = serial.Serial(port= Port , baudrate=115200, timeout=0.1) # object for communication with arduino
            except:
                self.note_label.config(text="Cannot start, check arduino port",fg = "red")
                return
            
            # model loading
            try:
                self.model = tf.keras.models.load_model(f'./models/{self.model_ID.get()}')
            except:
                self.note_label.config(text="Cannot start, check model path",fg = "red")
                return
            
            # start video capture
            self.cap = cv2.VideoCapture(self.cam_ID.get(),cv2.CAP_DSHOW) # good for windows
            # self.cap = cv2.VideoCapture(self.cam_ID.get())
            if self.cap.isOpened():
                self.cap.set(3, 640)
                self.cap.set(4, 480)
            else:
                self.note_label.config(text="Cannot start, check camera ID",fg = "red")
                return
            
            # start
            self.startCase = True    
            self.start_button.config(text="stop")
            self.note_label.config(text="")
            self.update()
        else :
            self.startCase = False
            self.start_button.config(text="start")
            self.current_speed_label.config(text = '')
            self.note_label.config(text="")
            self.arduino.close()
            if self.canvas_image is not None :
                self.canvas.delete(self.canvas_image)
            if self.cap.isOpened():
                self.cap.release()

    def update(self):
        '''
        classify and update frame in GUI and communicate with arduino
        '''
        if self.startCase:
            ret, frame = self.cap.read()
            if ret:
                frame = self.classify(frame)
                # communicate with hardware
                self.HW_update()
                # update GUI frame
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(frame))
                self.canvas_image = self.canvas.create_image(0,0,image = self.photo, anchor= tk.NW)
                # call it self
                self.window.after(30, self.update)                     
            else:
                self.note_label.config(text="connection error",fg = "red")

    def classify(self, frame):
        '''
        take frame as BGR and return BGR frame
        detect fruit and classify upon selected model
        '''
        white_sensitivity = int(self.sensitivity_entry.get())
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        # hsv boundaries
        l_b = np.array([0,white_sensitivity, 0])
        u_b = np.array([255, 255, 255])
        mask = cv2.inRange(hsv, l_b, u_b)
        # detect fruits and classify
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < 2000: continue
            x, y, w, h = cv2.boundingRect(contour)
            fruit = frame[y:y+h, x:x+w]
            fruit = cv2.resize(fruit,(180,180))
            fruit = cv2.cvtColor(fruit, cv2.COLOR_BGR2RGB)
            fruit = tf.expand_dims(fruit, 0)
            predictions = self.model.predict(fruit)
            score = round(float(predictions[0][0]), 2)
            if score > float(self.score_threshold_entry.get()) :
                self.fruit_is_normal = True
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2) # green rectangle for normal
            else:
                self.fruit_is_normal = False
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2) # red rectangle for abnormal
                
            frame = cv2.putText(frame, f'score={score}', (x, y), 0, 0.5, (255, 0, 0))
            
        return frame

    def HW_update(self):

        # write Data
        target_speed = self.speed_entry.get()
        if self.fruit_is_normal : target_theta_pos = 42
        else : target_theta_pos = 148
        Kp = self.Kp_entry.get()
        Ki = self.Ki_entry.get()
        Kd = self.kd_entry.get()
        # max_pwm = 255
        
        data = f'{target_speed},{target_theta_pos},{Kp},{Ki},{Kd}'
        self.arduino.write(data.encode('utf-8'))
        time.sleep(0.0005)

        #read Data
        Data = self.arduino.readline()
        Data = str(Data)[2:-1]
        
        # update current state
        self.current_speed_label.config(text = f'current speed = {Data} cm')


App(tk.Tk(), "fruit classification machine")
