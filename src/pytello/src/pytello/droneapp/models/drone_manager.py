import logging
import contextlib
import os
import socket
import subprocess
import threading
import time
import rospy

import cv2 as cv
import numpy as np

from pytello.droneapp.models.base import Singleton
from ultralytics import YOLO

logger = logging.getLogger(__name__)

DEFAULT_DISTANCE = 0.3
DEFAULT_SPEED = 50
DEFAULT_DEGREE = 90
DEFAULT_DIRECTION = 'l'

FRAME_X = int(960 / 3)
FRAME_Y = int(720 / 3)
FRAME_AREA = FRAME_X * FRAME_Y

FRAME_SIZE = FRAME_AREA * 3
FRAME_CENTER_X = FRAME_X / 2
FRAME_CENTER_Y = FRAME_Y / 2

CMD_FFMPEG = (f'ffmpeg -hwaccel auto -hwaccel_device opencl -i pipe:0 -loglevel quiet '
              f'-pix_fmt bgr24 -s {FRAME_X}x{FRAME_Y} -f rawvideo pipe:1')

FACE_DETECT_XML_FILE = './src/pytello/src/pytello/droneapp/models/haarcascade_frontalface_default.xml'

SNAPSHOTS_IMAGE_FOLDER = './src/pytello/src/pytello/droneapp/static/img/snapshots'
VIDEOS_IMAGE_FOLDER = './src/pytello/src/pytello/droneapp/static/videos/'
FRAMES_IMAGE_FOLDER = './src/pytello/src/pytello/droneapp/static/img/frames/'
MODEL_PATH = './src/pytello/src/pytello/yoloModel/yolov8n.pt'


class ErrorNoFaceDectectXMLFile(Exception):
    """Error no face detect xml file"""


class ErrorNoImageDir(Exception):
    """Error no image dir"""


class DroneManager(metaclass=Singleton):
    def __init__(self, host_ip='', host_port=8889,
                 drone_ip='192.168.10.1', drone_port=8889,
                 is_imperial=False, speed=DEFAULT_SPEED):
        self.host_ip = host_ip
        self.host_port = host_port
        self.drone_ip = drone_ip
        self.drone_port = drone_port
        self.drone_address = (drone_ip, drone_port)
        self.is_imperial = is_imperial
        self.speed = speed
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host_ip, self.host_port))

        self.response = None
        self.stop_event = threading.Event()
        self._response_thread = threading.Thread(target=self.receive_response,
                                                 args=(self.stop_event,))
        self._response_thread.start()

        self.patrol_event = None
        self.is_patrol = False
        self._patrol_semaphore = threading.Semaphore(1)
        self._thread_patrol = None

        self.proc = subprocess.Popen(CMD_FFMPEG.split(' '),
                                     stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE)

        self.proc_stdin = self.proc.stdin
        self.proc_stdout = self.proc.stdout

        self.video_port = 11111

        self._receive_video_thread = threading.Thread(
            target=self.receive_video,
            args=(self.stop_event, self.proc_stdin,
                  self.host_ip, self.video_port,)
        )
        self._receive_video_thread.start()

        if not os.path.exists(FACE_DETECT_XML_FILE):
            raise ErrorNoFaceDectectXMLFile(f'No {FACE_DETECT_XML_FILE}')
        self.face_cascade = cv.CascadeClassifier(FACE_DETECT_XML_FILE)
        self._is_enable_face_detect = False

        if not os.path.exists(SNAPSHOTS_IMAGE_FOLDER):
            raise ErrorNoImageDir(f'No {SNAPSHOTS_IMAGE_FOLDER} does not exists')
        self.is_snapshot = False
        self.is_video = False
        self.take_frames = False
        self.stop_frames = False

        self.numFrame = 0
        self.dir_frames = None
        self.frame_time = 0
        self.frame_total_time = 0
        self.folder = None
        self.file_frames = None

        self.video_out = None
        self.ini_video = None
        self.fin_video = None

        self.camera = 1

        self._command_semaphore = threading.Semaphore(1)
        self._command_thread = None

        self.detection = False
        self.model = YOLO(MODEL_PATH)
        self.num_frames_total = 0

        self.is_track = False
        self.centrado = 0

        self.lejosXY = 0
        self.lejosX = 0
        self.lejosY = 0

        self.send_command('command')
        self.send_command('downvision 0')
        self.send_command('downvision 1')
        self.send_command('streamon')
        self.send_command('battery?', blocking=False)
        self.set_speed(self.speed)

    def receive_response(self, stop_event):
        while not stop_event.is_set():
            try:
                self.response, ip = self.socket.recvfrom(3000)
                logger.info({'action': 'receive_response',
                             'response': self.response})
                rospy.loginfo({'action': 'receive_response', 'response': self.response})

            except socket.error as ex:
                logger.error({'action': 'receive_response',
                              'ex': ex})
                rospy.logger({'action': 'receive_response', 'ex': ex})
                break

    def __dell__(self):
        self.stop()

    def stop(self):
        self.stop_event.set()
        retry = 0
        while self._response_thread.is_alive():
            time.sleep(0.3)
            if retry > 30:
                break
            retry += 1
        self.socket.close()
        os.kill(self.proc.pid, 9)
        import signal
        os.kill(self.proc.pid, signal.CTRL_C_EVENT)

    def send_command(self, command, blocking=True):
        self._command_thread = threading.Thread(
            target=self._send_command,
            args=(command, blocking,)
        )
        self._command_thread.start()

    def _send_command(self, command, blocking=True):
        is_acquire = self._command_semaphore.acquire(blocking=blocking)
        if is_acquire:
            with contextlib.ExitStack() as stack:
                stack.callback(self._command_semaphore.release)
                logger.info({'action': 'send_command', 'command': command})
                rospy.loginfo({'action': 'send_command', 'command': command})
                self.socket.sendto(command.encode('utf-8'), self.drone_address)

                retry = 0
                while self.response is None:
                    time.sleep(0.3)
                    if retry > 3:
                        break
                    retry += 1

                if self.response is None:
                    response = None
                else:
                    response = self.response.decode('utf-8')

                self.response = None
                return response

        else:
            logger.warning({'action': 'send_command', 'command': command, 'status': 'not_acquire'})

    def takeoff(self):
        return self.send_command('takeoff')

    def emergency(self):
        return self.send_command('emergency')

    def land(self):
        return self.send_command('land')

    def move(self, direction, distance):
        distance = float(distance)
        if self.is_imperial:
            distance = int(round(distance * 30.48))
        else:
            distance = int(round(distance * 100))
        return self.send_command(f'{direction} {distance}')

    def up(self, distance=DEFAULT_DISTANCE):
        return self.move('up', distance)

    def down(self, distance=DEFAULT_DISTANCE):
        return self.move('down', distance)

    def left(self, distance=DEFAULT_DISTANCE):
        return self.move('left', distance)

    def right(self, distance=DEFAULT_DISTANCE):
        return self.move('right', distance)

    def forward(self, distance=DEFAULT_DISTANCE):
        return self.move('forward', distance)

    def backward(self, distance=DEFAULT_DISTANCE):
        return self.move('back', distance)

    def set_speed(self, speed):
        return self.send_command(f'speed {speed}')

    def clockwise(self, degree=DEFAULT_DEGREE):
        return self.send_command(f'cw {degree}')

    def counter_clockwise(self, degree=DEFAULT_DEGREE):
        return self.send_command(f'ccw {degree}')

    def flip(self, direction=DEFAULT_DIRECTION):
        return self.send_command(f'flip {direction}')

    def patrol(self):
        if not self.is_patrol:
            self.patrol_event = threading.Event()
            self._thread_patrol = threading.Thread(
                target=self._patrol,
                args=(self._patrol_semaphore, self.patrol_event,)
            )
            self._thread_patrol.start()
            self.is_patrol = True

    def stop_patrol(self):
        if self.is_patrol:
            self.patrol_event.set()
            retry = 0
            while self._thread_patrol.is_alive():
                time.sleep(0.3)
                if retry > 300:
                    break
                retry += 1
            self.is_patrol = False

    def _patrol(self, semaphore, stop_event):
        is_acquire = semaphore.acquire(blocking=False)
        if is_acquire:
            logger.info({'action': '_patrol', 'status': 'acquire'})
            with contextlib.ExitStack() as stack:
                stack.callback(semaphore.release)
                status = 0
                while not stop_event.is_set():
                    status += 1
                    if status == 1:
                        self.up()
                    if status == 2:
                        self.clockwise(90)
                    if status == 3:
                        self.down()
                    if status == 4:
                        status = 0
                    time.sleep(5)
        else:
            logger.warning({'action': '_patrol', 'status': 'not_acquire'})

    def receive_video(self, stop_event, pipe_in, host_ip, video_port):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock_video:
            sock_video.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock_video.settimeout(.5)
            sock_video.bind((host_ip, video_port))
            data = bytearray(2048)
            while not stop_event.is_set():
                try:
                    size, addr = sock_video.recvfrom_into(data)
                    # logger.info({'action': 'receive_video', 'data': data})
                except socket.timeout as ex:
                    logger.warning({'action': 'receive_video', 'ex': ex})
                    rospy.loginfo({'action': 'receive_video_wr', 'ex': ex})
                    time.sleep(0.5)
                    continue
                except socket.error as ex:
                    logger.error({'action': 'receive_video', 'ex': ex})
                    rospy.loginfo({'action': 'receive_video_err', 'ex': ex})
                    break

                try:
                    pipe_in.write(data[:size])
                    pipe_in.flush()
                except Exception as ex:
                    logger.error({'action': 'receive_video', 'ex': ex})
                    rospy.loginfo({'action': 'receive_video_err2', 'ex': ex})
                    break

    def video_binary_generator(self):
        while True:
            try:
                frame = self.proc_stdout.read(FRAME_SIZE)
            except Exception as ex:
                logger.error({'action': 'video_binary_generator', 'ex': ex})
                continue

            if not frame:
                continue

            frame = np.fromstring(frame, np.uint8).reshape(FRAME_Y, FRAME_X, 3)
            yield frame

    def enable_face_detect(self):
        self._is_enable_face_detect = True

    def disable_face_detect(self):
        self._is_enable_face_detect = False

    def video_jpeg_generator(self):
        for frame in self.video_binary_generator():
            
            # if self.camera == 1:
            #     frame = frame[0:240, 0:320]

            self.num_frames_total += 1
            _, jpeg = cv.imencode('.jpg', frame)
            jpeg_binary = jpeg.tobytes()

            if self.is_track and self.num_frames_total % 5 == 0:
                t3 = time.time()
                #Run YOLOv8 inference on the frame
                t1 = time.time()
                results = self.model(frame, verbose=False)
                t2 = time.time()
                print("timpo yolo", t2-t1)

                # self.frame_queue.put(frame)

                # results = self.results_queue.get()
                if not results:
                    print("no results")
                    self.send_command('stop')
                # View results
                for r in results:
                    drone_x, drone_y, drone_z, speed = 0, 0, 0, self.speed
                    if len(r.boxes.cls) > 0:
                        i = int(r.boxes.cls[0])
                        print(r.names[i])
                        
                        x, y, x2, y2 = r.boxes.xyxy[0]
                        # Convierte las coordenadas de punto flotante a enteros
                        x, y, x2, y2 = int(x), int(y), int(x2), int(y2)

                        x_center, y_center, w, h = r.boxes.xywh[0]

                        x_center, y_center, w, h = int(x_center), int(y_center), int(w), int(h)

                        diff_x = FRAME_CENTER_X - x_center
                        diff_y = FRAME_CENTER_Y - y_center

                        area = w * h
                        percent = area / FRAME_AREA
                        print("percent", percent, "area", area)
                        if (i == 74) and 0.35 > percent > 0.03: # es un gato

                            cv.rectangle(frame, (x, y), (x2, y2), (255, 0, 0), 2)
                            # if perc
                            v_speed_x = 0
                            v_speed_y = 0
                            w_speed = 0
                            print("frame center x", FRAME_CENTER_X)
                            print("x center", x_center)
                            print("y center", y_center)
                            print("frame center y", FRAME_CENTER_Y)
                            print("diff x", diff_x)
                            print("diff y", diff_y)
                            if abs(diff_x) >= 40:
                                v_speed_x = diff_x / 8
                                if abs(diff_x) >= 100:
                                    v_speed_x = diff_x / 12
                                    

                            # if abs(diff_x) >= 50:
                            #     w_speed = -diff_x / 6

                            if abs(diff_y) >= 40: 
                                v_speed_y = diff_y / 8
                                if abs(diff_y) >= 90:
                                    v_speed_y = diff_y / 12

                            if abs(diff_y) >= 75 and abs(diff_x) >= 90:
                                self.lejosXY += 1
                            elif abs(diff_x) >= 90:
                                self.lejosX += 1
                            elif abs(diff_y) >= 75:
                                self.lejosY += 1


                            if abs(diff_x) < 40 and (abs(diff_y) < 40):
                                print("centrado")
                                self.send_command('stop')
                                time.sleep(1)
                                self.send_command(f'rc 0 0 0 0', blocking=False)
                                self.centrado += 1
                                # time.sleep(2)
                                # print("land")
                                # self.send_command('land')
                                # self.is_track = False
                            else:
                                self.send_command(f'rc {v_speed_y} {v_speed_x} 0 {w_speed}', blocking=False)
                                self.centrado = 0

                            if self.centrado >= 5:
                                print("land")
                                self.send_command('land')
                                self.is_track = False

                            
                            break
                        
                        else:
                            self.send_command('stop')
                        #     time.sleep(2)
                        #     self.send_command(f'rc 0 0 0 0', blocking=False)
                        #     time.sleep(2)
                                         
                        
                # Display the annotated frame
                _, jpeg = cv.imencode('.jpg', frame)
                jpeg_binary = jpeg.tobytes()
                t4 = time.time()
                print("tiempo frame yolo", t4-t3)

            if self._is_enable_face_detect:
                if self.is_patrol:
                    self.stop_patrol()

                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in faces:
                    cv.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

                    face_center_x = x + (w/2)
                    face_center_y = y + (h/2)
                    diff_x = FRAME_CENTER_X - face_center_x
                    diff_y = FRAME_CENTER_Y - face_center_y
                    face_area = w * h
                    percent_face = face_area / FRAME_AREA

                    drone_x, drone_y, drone_z, speed = 0, 0, 0, self.speed
                    if diff_x < -30:  
                        drone_y = -30
                    if diff_x > 30:
                        drone_y = 30
                    if diff_y < -15:
                        drone_z = -30
                    if diff_y > 15:
                        drone_z = 30
                    if percent_face > 0.30:
                        drone_x = -30
                    if percent_face < 0.02:
                        drone_x = 30
                    self.send_command(f'go {drone_x} {drone_y} {drone_z} {speed}', 
                                      blocking=False)
                    break

                _, jpeg = cv.imencode('.jpg', frame)
                jpeg_binary = jpeg.tobytes()
            
            elif self.detection and self.num_frames_total % 5 == 0:
                #Run YOLOv8 inference on the frame
                results = self.model(frame, verbose=False)

                # Visualize the results on the frame
                frame = results[0].plot()

                # Display the annotated frame
                _, jpeg = cv.imencode('.jpg', frame)
                jpeg_binary = jpeg.tobytes()

            if self.is_video and (not self.detection or self.num_frames_total % 5 == 0):
                # logger.info({'action': 'write_frame', 'frame': or_frame})
                self.video_out.write(frame)
                
            if self.take_frames:
                backup_file = "frame" + str(self.numFrame) + '.jpg'
                file_path = os.path.join(
                    self.folder, backup_file
                )
                current_time = time.time()
                dif_time = current_time - self.frame_time                
                self.frame_time = current_time
                with open(file_path, 'wb') as f:
                    f.write(jpeg_binary)
                self.frame_total_time += dif_time
                fichero = open(self.file_frames, 'a')
                fichero.write("Tiempo frame  "+str(self.numFrame)+": "+str(dif_time)+"\n")
                fichero.close()
                self.numFrame += 1 
                
            if self.is_snapshot:
                backup_file = time.strftime("%Y%m%d-%H%M%S") + '.jpg'
                snapshot_file = 'snapshot.jpg'
                for filename in (backup_file, snapshot_file):
                    file_path = os.path.join(
                        SNAPSHOTS_IMAGE_FOLDER, filename
                    )
                    with open(file_path, 'wb') as f:
                        f.write(jpeg_binary)
                self.is_snapshot = False

            if not (self.detection or self.is_track) or self.num_frames_total % 5 == 0:
                yield jpeg_binary


    def snapshot(self):
        self.is_snapshot = True
        retry = 0
        while retry < 3:
            if not self.is_snapshot:
                return True
            time.sleep(0.1)
            retry += 1
        return False

    def takeVideo(self):
        if not self.is_video:
            backup_file = time.strftime("%Y%m%d-%H%M%S") + '.avi'
            file_path = os.path.join(
                VIDEOS_IMAGE_FOLDER, backup_file
            )
            self.video_out = cv.VideoWriter(file_path,
                                            cv.VideoWriter_fourcc('M', 'J', 'P', 'G'),
                                            30.0, (FRAME_X, FRAME_Y))
            self.ini_video = time.time()
            self.is_video = True
            logger.info({'action': 'take_video'})

    def stopVideo(self):
        logger.info({'action': 'stop_video'})
        self.is_video = False
        self.video_out.release()
        self.fin_video = time.time()
        tiempo = self.fin_video - self.ini_video
        logger.info({'Tiempo tomar video': tiempo})

    def takeFrames(self):
        self.dir_frames = 'frames' + time.strftime("%Y%m%d-%H%M%S")
        self.numFrame = 0
        self.folder = FRAMES_IMAGE_FOLDER + self.dir_frames
        os.mkdir(self.folder)
        self.file_frames = FRAMES_IMAGE_FOLDER + self.dir_frames + "times.txt"
        fichero = open(self.file_frames, 'w')
        fichero.close()
        self.frame_time = time.time()
        self.frame_total_time = 0
        self.take_frames = True

    def stopFrames(self):
        self.stop_frames = False
        med_time = self.frame_total_time / float(self.numFrame)
        fichero = open(self.file_frames, 'a')
        fichero.write("Tiempo medio por frame: " + str(med_time) + '\n')
        fichero.close()

    def detect(self):
        self.detection = True
        
    def stopDetect(self):
        self.detection = False

    # def changeCamera(self):
    #     self.camera = self.camera + 1 % 2
    #     rospy.loginfo('downvision ' + str(self.camera))
    #     self.send_command('streamoff')
    #     self.send_command('downvision ' + str(self.camera))
    #     self.send_command('streamon')

    def track(self):
        self.centrado = 0
        # self.send_command(f'down {20}', blocking = True)
        # time.sleep(3)
        self.is_track = True

    def stopTrack(self):
        self.is_track = False
        fichero_tello = f"no_centrado_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        with open(fichero_tello, 'a') as fichero:
            # Escribe la información recibida
            fichero.write("Veces que se ha salido del area central:\n")
            fichero.write("X:" + str(self.lejosX) + "\n")
            fichero.write("Y:" + str(self.lejosY) + "\n")
            fichero.write("Ambos:" + str(self.lejosXY) + "\n")
        self.lejosX = 0
        self.lejosY = 0
        self.lejosXY = 0