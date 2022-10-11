'''
    Yildiz Rocket Team
    Ground Station Interface

    Dogukan Yalcin
'''
import sys
import threading
import time
from pathlib import Path
from tkinter import Tk, Canvas, Button, PhotoImage, Label, filedialog, messagebox
import cv2 as cv2
import matplotlib.animation as animation
import numpy as np
import pandas as pd
import csv
import serial as sr
from PIL import ImageTk, Image, ImageDraw
from matplotlib import pyplot as plt
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from OpenGL.GL import *
from OpenGL.GLU import *
import pywavefront
import os
if not sys.platform.startswith('darwin'):
    from pyopengltk import OpenGLFrame

SERIAL_DATA_SIZE = 22


# hexadecimal constants
CMD_CONNECT_SATELLITE = '1'
CMD_STANDBY_MODE = '2'
CMD_POWER_UP_ENGINES = '6'
CMD_POWER_OFF_ENGINES = '5'
CMD_DETACH_PAYLOAD = '3'
CMD_REVERT_SERVO = '4'

GPS_CSV_FILE_NAME = "payload_coor.csv"
GPS_CSV_ICO = "par_icon.png"

GPS2_CSV_FILE_NAME = "container_coor.csv"
GPS2_CSV_ICO = "sat_icon.png"

GPS_MAP_IMAGE_FILE_NAME = "ytu_vadi.jpg"

THREE_D_MODEL_FILE_NAME = "roket.obj"

SERIAL_PORT_NAME = "/dev/tty.usbserial-A50285BI"
SERIAL_BAUD_RATE = 921600
# SERIAL_BAUD_RATE = 115200

# SERIAL2_PORT_NAME = "/dev/tty.usbserial-A50285BI"
# SERIAL2_BAUD_RATE = 115200

GRAPH_DATA_SIZE = 100
GRAPH_FIGURE_SIZE = (4.5, 2.5)
GRAPH_FIGURE_DPI = 40

VIDEO_RESOURCE = 0


  
if not sys.platform.startswith('darwin'):
    class IMUModel(OpenGLFrame):

        curr_roll = 0
        curr_pitch = 0
        curr_yaw = 0
        new_vals = np.array([270, 0, 0])
        objPath = ""
        # serialData = np.array((0, 270), dtype=np.float64)
        serialData = np.zeros(SERIAL_DATA_SIZE, dtype=np.float32)
        
        readyToDraw = True

        def __init__(self, obj_path, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.scene_trans = None
            self.scene_scale = None
            self.scene = None
            self.objPath = obj_path
            self.start_stop = True

        def initgl(self):
            self.scene = pywavefront.Wavefront(
                self.objPath, collect_faces=True)

            scene_box = (self.scene.vertices[0], self.scene.vertices[0])
            for vertex in self.scene.vertices:
                min_v = [min(scene_box[0][i], vertex[i]) for i in range(3)]
                max_v = [max(scene_box[1][i], vertex[i]) for i in range(3)]
                scene_box = (min_v, max_v)

            scene_size = [scene_box[1][i] -
                          scene_box[0][i] for i in range(3)]
            max_scene_size = max(scene_size)
            scaled_size = 5
            self.scene_scale = [scaled_size /
                                max_scene_size for i in range(3)]
            self.scene_trans = [-(scene_box[1][i] +
                                  scene_box[0][i]) / 2 for i in range(3)]

            gluPerspective(45, (self.width / self.height), 0.1, 50.0)

            glTranslatef(0.0, 0.0, -10)

        def redraw(self):
            if not self.start_stop:
                time.sleep(.5)
                return

            if not self.readyToDraw:
                return
            else:
                 self.readyToDraw = False

            try:
                self.new_vals = [float(self.serialData[14]),
                             float(self.serialData[15]),
                             float(self.serialData[16])]
            except Exception as e:
                print(e)
                print("Error: IMU angle data not valid")
                return

            glRotatef(self.new_vals[0] - self.curr_roll, 1, 0, 0)
            self.curr_roll = self.new_vals[0]

            glRotatef(self.new_vals[1] - self.curr_pitch, 0, 1, 0)
            self.curr_pitch = self.new_vals[1]

            glRotatef(self.new_vals[2] - self.curr_yaw, 0, 0, 1)
            self.curr_yaw = self.new_vals[2]

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            self.model()

        def model(self):
            glPushMatrix()
            glScalef(*self.scene_scale)
            glTranslatef(*self.scene_trans)

            for mesh in self.scene.mesh_list:
                glBegin(GL_TRIANGLES)
                for face in mesh.faces:
                    for vertex_i in face:
                        glVertex3f(*self.scene.vertices[vertex_i])
                glEnd()

            glPopMatrix()


class Map(Label):
    points = (0, 0, 0, 0)
    data_path = GPS_CSV_FILE_NAME
    data2_path = GPS2_CSV_FILE_NAME
    map_path = GPS_MAP_IMAGE_FILE_NAME
    map = Image.open(map_path, 'r')
    color = (0, 0, 255)
    color2 = (255, 0, 0)
    width = 8
    img_points = []
    image_height = 0
    image_width = 0
    currentImage = None
    loop_speed = 0
    start_stop = True

    def __init__(self, window, image_height, image_width, place_x1, place_y1, loop_speed):
        self.image_height = image_height
        self.image_width = image_width

        # Yenibosna
        # TLlat = 41.00387
        # TLlong = 28.82462
        # BRlat = 40.99244
        # BRlong = 28.84479
        # self.points = (TLlat, TLlong, BRlat, BRlong)
        
        # YTU 
        # TLlat = 41.035398
        # TLlong = 28.867805
        # BRlat = 41.016772
        # BRlong = 28.909124
        # self.points = (TLlat, TLlong, BRlat, BRlong)

        # VADI 41.028808, 28.882688    --    41.024533, 28.892162
        # AKSARAY 38.390860, 33.734419    --    38.383412, 33.751419
        # YURT 38.351642, 33.989127  --   38.347701, 33.980888
        
        
        TLlat = 41.028808
        TLlong = 28.882688
        BRlat = 41.024533
        BRlong = 28.892162
        self.points = (TLlat, TLlong, BRlat, BRlong)
        

        img = ImageTk.PhotoImage(self.map.resize(
            (self.image_width, self.image_height), Image.ANTIALIAS))
        super().__init__(window, image=img, borderwidth=0)
        self.image = img
        self.place(x=place_x1, y=place_y1)
        self.loop_speed = loop_speed

        print("Map initiliazed")

    def scale_to_img(self, lat_lon, h_w):
        old = (self.points[2], self.points[0])
        new = (0, h_w[1])
        y = ((lat_lon[0] - old[0]) * (new[1] - new[0]) /
             (old[1] - old[0])) + new[0]
        old = (self.points[1], self.points[3])
        new = (0, h_w[0])
        x = ((lat_lon[1] - old[0]) * (new[1] - new[0]) /
             (old[1] - old[0])) + new[0]
        return int(x), h_w[1] - int(y)

    def draw(self):
        x1, y1 = (0, 0)
        if not self.start_stop:
            try:
                self.after(500, self.draw)
            except:
                pass
            return None
        data = pd.read_csv(self.data_path, names=[
                           'LATITUDE', 'LONGITUDE'], sep=',')
        gps_data = zip(data['LATITUDE'].values, data['LONGITUDE'].values)
        for d in gps_data:
            try:
                x1, y1 = self.scale_to_img(d, (self.map.size[0], self.map.size[1]))
                self.img_points.append((x1, y1))
            except:
                print("Error: GPS data not valid")
                
            

        img_amogus = Image.open(GPS_CSV_ICO, 'r').resize(
            (100, 100)).convert('RGBA')
        combine = self.map.copy()
        draw = ImageDraw.Draw(combine)
        draw.line(self.img_points, fill=self.color, width=self.width)
        combine.paste(img_amogus, (x1 - 50, y1 - 50), img_amogus)



        
        # draw second gps data
        self.img_points = []
        x1, y1 = (0, 0)
        data2 = pd.read_csv(self.data2_path, names=[
                           'LATITUDE', 'LONGITUDE'], sep=',')
        gps_data2 = zip(data2['LATITUDE'].values, data2['LONGITUDE'].values)
        for d in gps_data2:
            try:
                x1, y1 = self.scale_to_img(d, (self.map.size[0], self.map.size[1]))
                self.img_points.append((x1, y1))
            except:
                print("Error: GPS data not valid")
                
            

        img_amogus = Image.open(GPS2_CSV_ICO, 'r').resize(
            (100, 100)).convert('RGBA')
        draw = ImageDraw.Draw(combine)
        draw.line(self.img_points, fill=self.color2, width=self.width)
        combine.paste(img_amogus, (x1 - 50, y1 - 50), img_amogus)
   



        img = ImageTk.PhotoImage(combine.resize(
            (self.image_width, self.image_height)))
        self.configure(image=img)
        self.image = img

        self.img_points.clear()
        try:
            self.after(self.loop_speed, self.draw)
        except:
            pass
        # print("Map drawn")


class VideoLabel(Label):
    loop_speed = 0
    app_condition = True
    

    def __init__(self, window, video_width, video_height, place_x1, place_y1, video_resource):
        super().__init__(window, image=ImageTk.PhotoImage(
            Image.new('RGB', (video_width, video_height))), borderwidth=0)
        self.image = ImageTk.PhotoImage(
            Image.new('RGB', (video_width, video_height)))
        self.video_width = video_width
        self.video_height = video_height
        self.video_resource = video_resource
        self.cap = cv2.VideoCapture(video_resource)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.size = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
			int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

        fps = self.cap.get(cv2.CAP_PROP_FPS)

        datetime_name = time.strftime("%Y-%m-%d_%H-%M-%S_saved_video.avi")

        self.videoWriter = cv2.VideoWriter(datetime_name, 
            cv2.VideoWriter_fourcc('X','V','I','D'), 15, self.size)

        self.place(x=place_x1, y=place_y1)
        self.start_stop = True
        print("VideoLabel initialized")
        threading.Thread(target= self.loop).start()

    def loop(self):
        if self.start_stop:
            self.cap = cv2.VideoCapture(self.video_resource)

        while self.start_stop:
            try:
                frame = self.cap.read()[1]
                cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
                self.videoWriter.write(frame)
                # crop image
                # cv2image = cv2image[0:self.video_height, 0:self.video_width]
                (im_h, im_w) = cv2image.shape[:2]
                h = self.video_height / float(im_h)    
                w = self.video_width / float(im_w)
                dim = (int(im_w * w), int(im_h * h))
                cv2image = cv2.resize(cv2image, dim, interpolation=cv2.INTER_AREA)
                img = Image.fromarray(cv2image)
                imgtk = ImageTk.PhotoImage(image=img)
                self.configure(image=imgtk)
                self.image = imgtk
                # print("VideoLabel looped")
            except Exception as e:
                print("VideoLabel loop error")
                print(e)

        self.cap.release()
        time.sleep(.5)
        self.loop()


class GraphForOne(FigureCanvasTkAgg):
    data = np.arange(0, GRAPH_DATA_SIZE, 1)
    x = np.arange(0, GRAPH_DATA_SIZE, 1)
    #fig, ax = plt.subplots(figsize=(3.8, 2.1), dpi=80)
    #ln = ax.plot(0, 0)[0]
    serialData = np.zeros(SERIAL_DATA_SIZE, dtype=np.float32)
    prevValue = np.zeros(1 , dtype=np.float32)
    readyToDraw = True

    def __init__(self, window, place_x1, place_y1, data_position):
        style.use('ggplot')
        #4 , 2.3 ,80
        self.data = np.arange(0, GRAPH_DATA_SIZE, 1)
        self.fig, self.ax = plt.subplots(figsize=GRAPH_FIGURE_SIZE, dpi=GRAPH_FIGURE_DPI)
        self.ax.set_ylim(-10, 1000)
        (self.ln,) = self.ax.plot(self.x, self.data, animated=True)
        self.ln.set_xdata(np.arange(0, len(self.data)))
        self.data_position = data_position
        super().__init__(self.fig, window)
        self.get_tk_widget().place(x=place_x1, y=place_y1)
        self.ani = animation.FuncAnimation(
            self.fig, self.animate, interval=50, blit=True)
        self.cond = True

    def animate(self, i):
        if not self.readyToDraw:
            return self.ln,

        try:
            data_1 = float(self.serialData[self.data_position])

        except Exception as e:
            print("GraphAnimate error / Value not numeric")
            print(e)
            return self.ln
      
        self.data[0:GRAPH_DATA_SIZE - 1] = self.data[1:GRAPH_DATA_SIZE]
        self.data[GRAPH_DATA_SIZE - 1] = data_1
        self.ln.set_ydata(self.data)
        self.prevValue = data_1
        self.readyToDraw = False
        return self.ln,

    def toggle_pause(self):
        self.cond = not self.cond
        if self.cond:
            self.ani.resume()
        else:
            self.ani.pause()


class GraphForTree(FigureCanvasTkAgg):
    data1 = np.arange(0, GRAPH_DATA_SIZE, 1)
    data2 = np.arange(0, GRAPH_DATA_SIZE, 1)
    data3 = np.arange(0, GRAPH_DATA_SIZE, 1)
    x = np.arange(0, GRAPH_DATA_SIZE, 1)
    #fig, ax = plt.subplots(figsize=(2.8, 1.8), dpi=80)
    #ln1 = ax.plot(0, 0)[0]
    #ln2 = ax.plot(0, 0)[0]
    #ln3 = ax.plot(0, 0)[0]
    serialData = np.zeros(SERIAL_DATA_SIZE, dtype=np.float32)
    prevValues = np.zeros(3, dtype=np.float32)
    readyToDraw = True


    def __init__(self, window, place_x1, place_y1, data_position):
        self.fig, self.ax = plt.subplots(figsize=GRAPH_FIGURE_SIZE, dpi=GRAPH_FIGURE_DPI)
        self.ax.set_ylim(-370, 370)
        (self.ln1,) = self.ax.plot(self.x, self.data1, animated=True)
        (self.ln2,) = self.ax.plot(self.x, self.data2, animated=True)
        (self.ln3,) = self.ax.plot(self.x, self.data3, animated=True)
        self.ln1.set_xdata(np.arange(0, len(self.data1)))
        self.ln2.set_xdata(np.arange(0, len(self.data2)))
        self.ln3.set_xdata(np.arange(0, len(self.data3)))
        super().__init__(self.fig, window)
        self.data_position = data_position
        self.get_tk_widget().place(x=place_x1, y=place_y1)
        self.ani = animation.FuncAnimation(
            self.fig, self.animate, interval=50, blit=True)
        print("GraphAnimateThree init")
        self.cond = True

    def animate(self, i):
        if not self.readyToDraw:
            return self.ln1, self.ln2, self.ln3,

        try:
            data_1 = float(self.serialData[self.data_position])
            data_2 = float(self.serialData[self.data_position + 1])
            data_3 = float(self.serialData[self.data_position + 2])
        except Exception as e:
            print("GraphAnimateThree error / Values not numeric")
            print(e)
            return self.ln1, self.ln2, self.ln3,

        self.data1[0:GRAPH_DATA_SIZE - 1] = self.data1[1:GRAPH_DATA_SIZE]
        self.data2[0:GRAPH_DATA_SIZE - 1] = self.data2[1:GRAPH_DATA_SIZE]
        self.data3[0:GRAPH_DATA_SIZE - 1] = self.data3[1:GRAPH_DATA_SIZE]
        self.data1[GRAPH_DATA_SIZE - 1] = data_1
        self.data2[GRAPH_DATA_SIZE - 1] = data_2
        self.data3[GRAPH_DATA_SIZE - 1] = data_3
        self.ln1.set_ydata(self.data1)
        self.ln2.set_ydata(self.data2)
        self.ln3.set_ydata(self.data3)
        self.readyToDraw = False
        return self.ln1, self.ln2, self.ln3

    def toggle_pause(self):
        self.cond = not self.cond
        if self.cond:
            self.ani.resume()
        else:
            self.ani.pause()


class csvOperations:


    def __init__(self, file_name, columns):
        self.file_name = file_name
        self.df = pd.DataFrame(columns=columns)
        self.create_file_name_date()
        self.df.to_csv(self.file_name, index=False)

    def create_file_name(self):
        i = 0
        while os.path.exists( f'{self.file_name}{i}.csv'):
            i += 1
        self.file_name = f'{self.file_name}{i}.csv'

    def create_file_name_date(self):
        self.file_name =time.strftime("%Y-%m-%d_%H-%M-%S_serial_log.csv")

    def append_array(self, array):
        with open(self.file_name, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(array)

class App(Tk):

    OUTPUT_PATH = Path(__file__).parent
    ASSETS_PATH = OUTPUT_PATH / Path("./assets")
    serialData = np.zeros(SERIAL_DATA_SIZE, dtype=np.float32)
    logging_start_stop = True
    app_condition = True



    def __init__(self):
        super().__init__()
  
        self.payload_velocity_graph = None
        self.container_altitude_graph = None
        self.roll_pitch_yaw_graph = None
        self.map = None
        self.payload_altitude_graph = None
        self.video = None
        self.geometry("1288x694")
        self.configure(bg="#FFFFFF")
        self.resizable(False, False)
        threading.Thread(target=self.read_serial).start()
        self.resizable = (False, False)
        self.canvas = Canvas(self, bg="#FFFFFF", height=694,
                             width=1288, bd=0, highlightthickness=0, relief="ridge")
        self.canvas.place(x=0, y=0)
        # self.ser2 = sr.Serial(SERIAL2_PORT_NAME, SERIAL2_BAUD_RATE)
        # self.ser2.reset_input_buffer()
        self.create_widgets()
        self.create_serial_texts()
        self.create_models()
        self.update_serial_texts()

    def read_serial(self):
        while 1:
            try:
                self.ser = sr.Serial(SERIAL_PORT_NAME, SERIAL_BAUD_RATE)
                
                self.ser.reset_input_buffer()
                
                break
            except:
                print("Serial port acilamadi") 
                time.sleep(3.5)

        print("Seri okuma başladı")

# sprintf(stringWithAllData, 
#  [0]       [1]                [2]                [3]             [4]               [5]            [6]           [7]            [8]          [9]              [10]            [11]             [12]                     [13]               [14]        [15]          [16]     [17]          [18]                   [19]                    [20]
#  "%d,      %d,                 %d,                %d,            %.2f,            %.2f,          %.2f,         %.2f,          %.2f,          %.2f,           %.2f,           %.2f,            %.2f,                    %.2f,              %.2f,       %.2f,         %.2f,   %.2f,           %d                     %f                     %f\n",
#  teamNo, packetNo, (int)gps.utc_time + 30000.f, flightState, altitude.pressure, jei.pressure, gps.latitude, gps.longitude, gps.altitude, jei.latitude, jei.longtitude, jei.altitude, velocity.verticalVelocity, altitude.temperature , angle.roll, angle.pitch, angle.yaw, voltage, lenna.tranmissionPercentage), altitude.altitude, jei.altitude, spin);

        serialWriteCSV = csvOperations("su_anlik_tarihe_gore_yapiyor", ["team_no", "packet_no","utc_time", "flight_state", "payload_pressure", "container_pressure", "payload_latitude", "payload_longtitude", "payload_gps_altitude", "container_latitude", "container_longtitude", "container_altitude", "vertical_velocity", "temperature", "roll", "pitch", "yaw", "voltage", "transmission_percentage"])
        while self.app_condition:
            try:
                self.serialData = self.ser.readline()[:-1].decode('latin1').split(",")
                # if self.serialData[0] is not numeric :
                if self.serialData[0].isnumeric():
                    print(self.serialData)
                    serialWriteCSV.append_array(self.serialData)
                    self.update_models()
                    self.update_serial_texts()
            except:
                print("Seri port okunurken hata")             
                time.sleep(1.5)

    def update_serial_texts(self):
        if not self.logging_start_stop:
            self.after(500, self.update_serial_texts)
            return
        
        try:
            # self.canvas.itemconfig(self.video_status_text,
            #                    text="18237")
            self.canvas.itemconfig(self.team_no_text, text = f'{self.serialData[0]}')
            self.canvas.itemconfig(self.packet_no_text, text = f'{self.serialData[1]}')

            # float to string
            timetext = str(int(float(self.serialData[2])))

            datetext = timetext[0:2] + ':' + timetext[2:4] + ':' + timetext[4:6]
            self.canvas.itemconfig(self.date_text, text = datetext)
            
            self.canvas.itemconfig(self.status_text, text = f'{self.serialData[3]}')
            self.canvas.itemconfig(self.pressure_text, text = f'{self.serialData[5]} Pa')
            self.canvas.itemconfig(self.pressure2_text, text = f'{self.serialData[4]} Pa')
            self.canvas.itemconfig(self.container_latitude_text, text = f'{self.serialData[6]}')
            self.canvas.itemconfig(self.container_longitude_text, text = f'{self.serialData[7]}')
            self.canvas.itemconfig(self.container_gps_altitude_text, text = f'{self.serialData[8]} m')
            self.canvas.itemconfig(self.payload_latitude_text, text = f'{self.serialData[9]}')
            self.canvas.itemconfig(self.payload_longitude_text, text = f'{self.serialData[10]}')
            self.canvas.itemconfig(self.payload_gps_altitude_text, text = f'{self.serialData[11]}')
            self.canvas.itemconfig(self.velocity_text, text = f'{self.serialData[12]} m/s')
            self.canvas.itemconfig(self.temparature_text, text = f'{self.serialData[13]} C')
            self.canvas.itemconfig(self.pitch_text, text = f'{self.serialData[14]}')
            self.canvas.itemconfig(self.yaw_text, text = f'{self.serialData[15]}')
            self.canvas.itemconfig(self.roll_text, text = f'{self.serialData[16]}')
            self.canvas.itemconfig(self.voltage_text, text = f'{self.serialData[17]} V')
            self.canvas.itemconfig(self.video_process_text, text = f'-')
            self.canvas.itemconfig(self.payload_altitude_text, text = f'{self.serialData[19]} m')
            self.canvas.itemconfig(self.container_altitude_text, text = f'{self.serialData[20]} m')
            self.canvas.itemconfig(self.roll_count_text, text = f'{self.serialData[21]}')
            # altitude diff
            self.canvas.itemconfig(self.altitude_difference, text = f'{float(self.serialData[19]) - float(self.serialData[20])} m')
            if 0 < float(self.serialData[18]) < 200:
                self.canvas.itemconfig(self.video_status_text, text = "SENDING")
            elif self.serialData[18] == 200:
                self.canvas.itemconfig(self.video_status_text, text = "COMPLETED")
            else:
                self.canvas.itemconfig(self.video_status_text, text = "-")
            

        except Exception as e:
            print(e)
            
        
        # self.after(30, self.update_serial_texts)
        

    def update_models(self):
        if not sys.platform.startswith('darwin'):
            self.imu_model.serialData = self.serialData
            self.imu_model.readyToDraw = True

        # add coordinates to the coordinates.csv
        try:
            coor = [float(self.serialData[6]), float(self.serialData[7])]
            if coor[0] != 0 and coor[1] != 0:
                with open(GPS_CSV_FILE_NAME, "a") as f:
                    writer = csv.writer(f)
                    writer.writerow(coor)
        except Exception as e:
            print("Coordinates values invalid")
            print(e)

        try:
            coor = [float(self.serialData[9]), float(self.serialData[10])]
            if coor[0] != 0 and coor[1] != 0:
                with open(GPS2_CSV_FILE_NAME, "a") as f:
                    writer = csv.writer(f)
                    writer.writerow(coor)
        except Exception as e:
            print("Coordinates values invalid")
            print(e)

        self.payload_velocity_graph.serialData = self.serialData
        self.payload_altitude_graph.serialData = self.serialData
        self.container_altitude_graph.serialData = self.serialData
        self.roll_pitch_yaw_graph.serialData = self.serialData

        self.payload_velocity_graph.readyToDraw = True
        self.payload_altitude_graph.readyToDraw = True
        self.container_altitude_graph.readyToDraw = True
        self.roll_pitch_yaw_graph.readyToDraw = True


    def create_models(self):
        if not sys.platform.startswith('darwin'):
            self.imu_model = IMUModel(THREE_D_MODEL_FILE_NAME, self, width=354, height=310)
            self.imu_model.place(x=912, y=365)
            self.imu_model.animate = 1

        # create empty coordinates.csv
        with open(GPS_CSV_FILE_NAME, "w") as f:
            writer = csv.writer(f)

        with open(GPS2_CSV_FILE_NAME, "w") as f:
            writer = csv.writer(f)

        self.video = VideoLabel(
            # self, video_width=349, video_height=300, place_x1=915, place_y1=20, video_resource="http://192.168.1.108:8080/video")
            self, video_width=349, video_height=300, place_x1=915, place_y1=20, video_resource= VIDEO_RESOURCE)

        self.map = Map(self, image_width=539, image_height=318,
                       place_x1=328, place_y1=18, loop_speed=200)
        self.map.draw()

        self.payload_velocity_graph = GraphForOne(self, place_x1=610, place_y1=365,
                                                  data_position=12)
        self.payload_altitude_graph = GraphForOne(self, place_x1=340, place_y1=530,
                                                  data_position=19)
        self.container_altitude_graph = GraphForOne(self, place_x1=610, place_y1=530,
                                                    data_position=20)
        self.roll_pitch_yaw_graph = GraphForTree(self, place_x1=340, place_y1=365,
                                                 data_position=14)

    def start_logging(self):
        if self.logging_start_stop:
            return
        self.logging_start_stop = True
        if not sys.platform.startswith('darwin'):
            self.imu_model.start_stop = True

        self.map.start_stop = True

        self.video.start_stop = True

        self.payload_velocity_graph.toggle_pause()
        self.payload_altitude_graph.toggle_pause()
        self.container_altitude_graph.toggle_pause()
        self.roll_pitch_yaw_graph.toggle_pause()

        print("Logging started")

    def stop_logging(self):
        if not self.logging_start_stop:
            return
        self.logging_start_stop = False
        if not sys.platform.startswith('darwin'):
            self.imu_model.start_stop = False

        self.map.start_stop = False

        self.video.start_stop = False

        self.payload_velocity_graph.toggle_pause()
        self.payload_altitude_graph.toggle_pause()
        self.container_altitude_graph.toggle_pause()
        self.roll_pitch_yaw_graph.toggle_pause()

        print("Logging stopped")

    def send_command_connect_satellite(self):
        try:
            self.ser.write(CMD_CONNECT_SATELLITE.encode())
            # self.ser.write(CMD_CONNECT_SATELLITE)
            print("Satellite conncected")
        except Exception as e:
            print("Komut goderilemedi")
            print(e)
            messagebox.showwarning("Error", "Komut gonderilemedi")

    def send_command_standby_mode(self):
        try:
            self.ser.write(CMD_STANDBY_MODE.encode())
            print("Standby mode aktif")
        except Exception as e:
            print("Komut goderilemedi")
            print(e)
            messagebox.showwarning("Error", "Komut gonderilemedi")
    
    def send_command_power_up_engines(self):
        try:
            self.ser.write(CMD_POWER_UP_ENGINES.encode())
            print("Engines powered up")
        except Exception as e:
            print("Komut goderilemedi")
            print(e)
            messagebox.showwarning("Error", "Komut gonderilemedi")

    def send_command_power_off_engines(self):
        try:
            # self.ser.write(CMD_POWER_OFF_ENGINES)
            self.ser.write(CMD_POWER_OFF_ENGINES.encode())
            print("Engines powered down")
        except Exception as e:
            print("Komut goderilemedi")
            print(e)
            messagebox.showwarning("Error", "Komut gonderilemedi")

    def send_command_detach_payload(self):
        try:
            self.ser.write(CMD_DETACH_PAYLOAD.encode())

            print("detach komutu uart uzerinden gonderildi")      
        except Exception as e:
            print("Komut goderilemedi")
            print(e)
            messagebox.showwarning("Error", "Komut gonderilemedi")

    def send_command_revert_servo(self):
        try:
            self.ser.write(CMD_REVERT_SERVO.encode())
            # self.ser.write(CMD_REVERT_SERVO)
            print("revert komutu uart uzerinden gonderildi")      
        except Exception as e:
            print("Komut goderilemedi")
            print(e)
            messagebox.showwarning("Error", "Komut gonderilemedi")


    def send_file(self):

        thread = threading.Thread(target=self.send_file_thread)
        thread.start()


    def send_file_thread(self):
        file_name = filedialog.askopenfilename(initialdir="~", title="Select file to Send", filetypes=(("all files", "*.*"), ("all files", "*.*")))
        print(file_name)
        try:
            # open serial port
            # ser2 = sr.Serial(SERIAL2_PORT_NAME, SERIAL2_BAUD_RATE)

            # self.ser2.write(b'D')
            # self.ser2.write(open(file_name, 'rb').read())

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.ser.write(b'D')
            time.sleep(0.1)
            self.ser.write(open(file_name, 'rb').read())

            # ser2.write(b'D')
            # ser2.write(open(file_name, 'rb').read())
            # ser2.close()
        except Exception as e:
            print(e)
            messagebox.showwarning("Error", "Seri porttan veri gonderilemedi")
        print(file_name)



    def on_close(self):
        self.app_condition = False
        self.video.start_stop = False
        self.ser.close()
        self.destroy()

    def create_serial_texts(self):
        self.team_no_text = self.canvas.create_text(
            79.0,
            365.0,
            anchor="nw",
            text="419253",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.packet_no_text = self.canvas.create_text(
            234.0,
            366.0,
            anchor="nw",
            text="123",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.date_text = self.canvas.create_text(
            79.0,
            393.0,
            anchor="nw",
            text="01/01/2001",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.status_text = self.canvas.create_text(
            234.0,
            393.0,
            anchor="nw",
            text="01",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.pressure_text = self.canvas.create_text(
            234.0,
            419.0,
            anchor="nw",
            text="1 Pa",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.pressure2_text = self.canvas.create_text(
            79.0,
            419.0,
            anchor="nw",
            text="1 Pa",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.payload_latitude_text = self.canvas.create_text(
            79.0,
            447.0,
            anchor="nw",
            text="41.027327",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.payload_longitude_text = self.canvas.create_text(
            79.0,
            475.0,
            anchor="nw",
            text="28.884870",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.payload_altitude_text = self.canvas.create_text(
            79.0,
            501.0,
            anchor="nw",
            text="500 m",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.payload_gps_altitude_text = self.canvas.create_text(
            78.0,
            528.0,
            anchor="nw",
            text="499 m",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.altitude_difference = self.canvas.create_text(
            78.0,
            554.0,
            anchor="nw",
            text="200 m",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.temparature_text = self.canvas.create_text(
            79.0,
            584.0,
            anchor="nw",
            text="30 C",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.roll_text = self.canvas.create_text(
            82.0,
            610.0,
            anchor="nw",
            text="17",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.pitch_text = self.canvas.create_text(
            82.0,
            638.0,
            anchor="nw",
            text="42",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.video_status_text = self.canvas.create_text(
            79.0,
            664.0,
            anchor="nw",
            text="Sending",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.video_process_text = self.canvas.create_text(
            234.0,
            665.0,
            anchor="nw",
            text="%69",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.roll_count_text = self.canvas.create_text(
            234.0,
            637.0,
            anchor="nw",
            text="23",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.yaw_text = self.canvas.create_text(
            234.0,
            610.0,
            anchor="nw",
            text="34",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.voltage_text = self.canvas.create_text(
            234.0,
            585.0,
            anchor="nw",
            text="10 V",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.velocity_text = self.canvas.create_text(
            234.0,
            555.0,
            anchor="nw",
            text="20 m/s",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.container_latitude_text = self.canvas.create_text(
            234.0,
            448.0,
            anchor="nw",
            text="41.026830",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.container_longitude_text = self.canvas.create_text(
            234.0,
            474.0,
            anchor="nw",
            text="28.889181",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.container_altitude_text = self.canvas.create_text(
            234.0,
            500.0,
            anchor="nw",
            text="400 m",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.container_gps_altitude_text = self.canvas.create_text(
            234.0,
            528.0,
            anchor="nw",
            text="400 m",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

    def relative_to_assets(self, path: str) -> Path:
        return self.ASSETS_PATH / Path(path)

    def create_widgets(self):
        self.canvas.create_rectangle(
            .0,
            .0,
            1288.0,
            696.0,
            fill="#13152D",
            outline="")

        self.image_image_1 = PhotoImage(
            file=self.relative_to_assets("image_1.png"))
        image_1 = self.canvas.create_image(
            115.0,
            349.0,
            image=self.image_image_1
        )

        self.canvas.create_text(
            77.0,
            154.0,
            anchor="nw",
            text="GROUND STATION v1.0",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 12 * -1)
        )

        self.image_image_2 = PhotoImage(
            file=self.relative_to_assets("image_2.png"))
        image_2 = self.canvas.create_image(
            151.0,
            54.0,
            image=self.image_image_2
        )

        self.image_image_3 = PhotoImage(
            file=self.relative_to_assets("image_3.png"))
        image_3 = self.canvas.create_image(
            599.0,
            174.0,
            image=self.image_image_3
        )

        self.image_image_4 = PhotoImage(
            file=self.relative_to_assets("image_4.png"))
        image_4 = self.canvas.create_image(
            599.0,
            515.0,
            image=self.image_image_4
        )

        self.image_image_5 = PhotoImage(
            file=self.relative_to_assets("image_5.png"))
        image_5 = self.canvas.create_image(
            1090.0,
            173.0,
            image=self.image_image_5
        )

        self.image_image_6 = PhotoImage(
            file=self.relative_to_assets("image_6.png"))
        image_6 = self.canvas.create_image(
            1090.0,
            515.0,
            image=self.image_image_6
        )

        self.canvas.create_text(
            40.0,
            126.0,
            anchor="nw",
            text="YILDIZ ROCKET TEAM\n",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 20 * -1)
        )

        self.image_image_7 = PhotoImage(
            file=self.relative_to_assets("image_7.png"))
        image_7 = self.canvas.create_image(
            1090.0,
            519.0,
            image=self.image_image_7
        )

        self.image_image_8 = PhotoImage(
            file=self.relative_to_assets("image_8.png"))
        image_8 = self.canvas.create_image(
            1090.0,
            174.0,
            image=self.image_image_8
        )

        self.image_image_9 = PhotoImage(
            file=self.relative_to_assets("image_9.png"))
        image_9 = self.canvas.create_image(
            596.0,
            174.0,
            image=self.image_image_9
        )

        #self.image_image_10 = PhotoImage(file=self.relative_to_assets("image_10.png"))
        #image_10 = self.canvas.create_image(731.0,438.0,image=self.image_image_10)

        self.image_image_11 = PhotoImage(
            file=self.relative_to_assets("image_11.png"))
        image_11 = self.canvas.create_image(
            151.0,
            59.0,
            image=self.image_image_11
        )

        self.button_image_1 = PhotoImage(
            file=self.relative_to_assets("button_1.png"))
        button_1 = Button(
            image=self.button_image_1,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: self.send_command_standby_mode(),
            relief="flat"
        )
        button_1.place(
            x=151.0,
            y=176.0,
            width=121.98065185546875,
            height=32.9384765625
        )

        self.button_image_2 = PhotoImage(
            file=self.relative_to_assets("button_2.png"))
        button_2 = Button(
            image=self.button_image_2,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: self.send_command_connect_satellite(),
            relief="flat"
        )
        button_2.place(
            x=23.0,
            y=176.0,
            width=121.98001098632812,
            height=32.9384765625
        )

        self.button_image_3 = PhotoImage(
            file=self.relative_to_assets("button_3.png"))
        button_3 = Button(
            image=self.button_image_3,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: self.send_command_power_off_engines(),
            relief="flat"
        )
        button_3.place(
            x=151.0,
            y=213.0,
            width=121.98065185546875,
            height=32.9384765625
        )

        self.button_image_4 = PhotoImage(
            file=self.relative_to_assets("button_4.png"))
        button_4 = Button(
            image=self.button_image_4,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: self.send_command_power_up_engines(),
            relief="flat"
        )
        button_4.place(
            x=23.0,
            y=213.0,
            width=121.98001098632812,
            height=32.9384765625
        )

        self.button_image_5 = PhotoImage(
            file=self.relative_to_assets("button_5.png"))
        button_5 = Button(
            image=self.button_image_5,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: self.send_command_revert_servo(),
            relief="flat"
        )
        button_5.place(
            x=151.0,
            y=249.0,
            width=121.98065185546875,
            height=32.9384765625
        )

        self.button_image_6 = PhotoImage(
            file=self.relative_to_assets("button_6.png"))
        button_6 = Button(
            image=self.button_image_6,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: self.send_command_detach_payload(),
            relief="flat"
        )
        button_6.place(
            x=23.0,
            y=249.0,
            width=121.98001098632812,
            height=32.9384765625
        )

        self.button_image_7 = PhotoImage(
            file=self.relative_to_assets("button_7.png"))
        button_7 = Button(
            image=self.button_image_7,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: self.send_file(),
            relief="flat"
        )
        button_7.place(
            x=23.0,
            y=323.0,
            width=261.0,
            height=33.76177978515625
        )

        self.button_image_8 = PhotoImage(
            file=self.relative_to_assets("button_8.png"))
        button_8 = Button(
            image=self.button_image_8,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: self.stop_logging(),
            relief="flat"
        )
        button_8.place(
            x=151.0,
            y=286.0,
            width=121.98065185546875,
            height=32.9384765625
        )

        self.button_image_9 = PhotoImage(
            file=self.relative_to_assets("button_9.png"))
        button_9 = Button(
            image=self.button_image_9,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: self.start_logging(),
            relief="flat"
        )
        button_9.place(
            x=23.0,
            y=286.0,
            width=121.98001098632812,
            height=32.9384765625
        )


        self.image_image_12 = PhotoImage(
            file=self.relative_to_assets("image_12.png"))
        image_12 = self.canvas.create_image(
            74.0,
            372.0,
            image=self.image_image_12
        )

        image_13 = self.canvas.create_image(
            224.0,
            372.0,
            image=self.image_image_12
        )

        image_14 = self.canvas.create_image(
            74.0,
            399.0,
            image=self.image_image_12
        )

        image_15 = self.canvas.create_image(
            224.0,
            399.0,
            image=self.image_image_12
        )

        image_16 = self.canvas.create_image(
            74.0,
            426.0,
            image=self.image_image_12
        )

        image_17 = self.canvas.create_image(
            224.0,
            426.0,
            image=self.image_image_12
        )

        image_18 = self.canvas.create_image(
            74.0,
            454.0,
            image=self.image_image_12
        )

        image_19 = self.canvas.create_image(
            224.0,
            454.0,
            image=self.image_image_12
        )

        image_20 = self.canvas.create_image(
            74.0,
            480.0,
            image=self.image_image_12
        )

        image_21 = self.canvas.create_image(
            224.0,
            480.0,
            image=self.image_image_12
        )

        image_22 = self.canvas.create_image(
            74.0,
            507.0,
            image=self.image_image_12
        )

        image_23 = self.canvas.create_image(
            224.0,
            507.0,
            image=self.image_image_12
        )

        image_24 = self.canvas.create_image(
            74.0,
            535.0,
            image=self.image_image_12
        )

        image_25 = self.canvas.create_image(
            224.0,
            535.0,
            image=self.image_image_12
        )

        image_26 = self.canvas.create_image(
            74.0,
            562.0,
            image=self.image_image_12
        )

        self.canvas.create_text(
            11.0,
            560.0,
            anchor="nw",
            text="Difference:",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        image_27 = self.canvas.create_image(
            224.0,
            562.0,
            image=self.image_image_12
        )

        image_28 = self.canvas.create_image(
            74.0,
            590.0,
            image=self.image_image_12
        )

        image_29 = self.canvas.create_image(
            224.0,
            590.0,
            image=self.image_image_12
        )

        image_30 = self.canvas.create_image(
            74.0,
            617.0,
            image=self.image_image_12
        )

        image_31 = self.canvas.create_image(
            224.0,
            617.0,
            image=self.image_image_12
        )

        image_32 = self.canvas.create_image(
            74.0,
            644.0,
            image=self.image_image_12
        )

        image_33 = self.canvas.create_image(
            224.0,
            671.0,
            image=self.image_image_12
        )

        image_34 = self.canvas.create_image(
            224.0,
            644.0,
            image=self.image_image_12
        )

        image_35 = self.canvas.create_image(
            74.0,
            671.0,
            image=self.image_image_12
        )

        # self.button_image_6 = PhotoImage(
        #     file=self.relative_to_assets("button_6.png"))
        # button_6 = Button(
        #     image=self.button_image_6,
        #     borderwidth=0,
        #     highlightthickness=0,
        #     command=lambda: self.connect_satellite(),
        #     relief="flat"
        # )
        # button_6.place(
        #     x=41.948524475097656,
        #     y=170.92539978027344,
        #     width=213.11355590820312,
        #     height=39.0
        # )

        self.canvas.create_text(
            9.0,
            369.0,
            anchor="nw",
            text="Team No:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            367.0,
            anchor="nw",
            text="Packet No:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            366.0,
            353.0,
            anchor="nw",
            text="PAYLOAD YAW & PITCH & ROLL GRAPH",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 10 * -1)
        )

        self.canvas.create_text(
            491.0,
            10.0,
            anchor="nw",
            text="PAYLOAD AND CONTAINER LOCATION",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 10 * -1)
        )

        self.canvas.create_text(
            1032.0,
            354.0,
            anchor="nw",
            text="PAYLOAD SIMULATION",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 10 * -1)
        )

        self.canvas.create_text(
            651.0,
            354.0,
            anchor="nw",
            text="PAYLOAD VELOCITY GRAPH",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 10 * -1)
        )

        self.canvas.create_text(
            390.0,
            516.0,
            anchor="nw",
            text="PAYLOAD ALTITUDE GRAPH",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 10 * -1)
        )

        self.canvas.create_text(
            648.0,
            515.0,
            anchor="nw",
            text="CONTAINER ALTITUDE GRAPH",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 10 * -1)
        )

        self.canvas.create_text(
            9.0,
            421.0,
            anchor="nw",
            text="Pressure:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            421.0,
            anchor="nw",
            text="Pressure:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            553.0,
            anchor="nw",
            text=" Altitude",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            557.0,
            anchor="nw",
            text="Velocity:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            585.0,
            anchor="nw",
            text="Temperature:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            586.0,
            anchor="nw",
            text="Voltage:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            11.0,
            612.0,
            anchor="nw",
            text="Roll:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            612.0,
            anchor="nw",
            text="Yaw:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            11.0,
            642.0,
            anchor="nw",
            text="Pitch:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            639.0,
            anchor="nw",
            text="Yaw Count:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            11.0,
            667.0,
            anchor="nw",
            text="Video Status:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            666.0,
            anchor="nw",
            text="Video Process:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            394.0,
            anchor="nw",
            text="UTC Time:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            395.0,
            anchor="nw",
            text="Status:",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 9 * -1)
        )

        #self.image_image_36 = PhotoImage(file=self.relative_to_assets("image_36.png"))
        #image_36 = self.canvas.create_image( 460.0,438.0, image=self.image_image_36 )

        #self.image_image_37 = PhotoImage( file=self.relative_to_assets("image_37.png"))
        #image_37 = self.canvas.create_image( 731.0, 601.0,  image=self.image_image_37)

        #self.image_image_38 = PhotoImage(file=self.relative_to_assets("image_38.png"))
        #image_38 = self.canvas.create_image(460.0, 602.0,image=self.image_image_38 )

        self.canvas.create_text(
            1033.0,
            8.0,
            anchor="nw",
            text="PAYLOAD LIVE VIDEO",
            fill="#FFFFFF",
            font=("Inter ExtraBold", 10 * -1)
        )

        self.canvas.create_text(
            9.0,
            444.0,
            anchor="nw",
            text="Payload",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            452.0,
            anchor="nw",
            text="Latitude",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            472.0,
            anchor="nw",
            text="Payload",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            479.0,
            anchor="nw",
            text="Longtitude:",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            498.0,
            anchor="nw",
            text="Payload",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            506.0,
            anchor="nw",
            text="Altitude:",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            527.0,
            anchor="nw",
            text="Payload GPS",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            9.0,
            535.0,
            anchor="nw",
            text="Altitude:",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            444.0,
            anchor="nw",
            text="Container",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            158.0,
            452.0,
            anchor="nw",
            text="Latitude:",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            157.0,
            471.0,
            anchor="nw",
            text="Container",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            157.0,
            478.0,
            anchor="nw",
            text="Longtitude:",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            157.0,
            497.0,
            anchor="nw",
            text="Container",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            157.0,
            505.0,
            anchor="nw",
            text="Altitude:",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            157.0,
            525.0,
            anchor="nw",
            text="Container GPS",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )

        self.canvas.create_text(
            157.0,
            533.0,
            anchor="nw",
            text="Altitude:",
            fill="#FFFFFF",
            font=("Inter", 9 * -1)
        )


myApp = App()
myApp.protocol("WM_DELETE_WINDOW", myApp.on_close)
myApp.mainloop()
