from ultralytics import YOLO
import cv2
import math 
import torch
from queue import Queue


class Model:
    def __init__(self, device = "/dev/video0",
                yolo_model = "yolov3u.pt", queue_size = 100):
        self.cap = cv2.VideoCapture(device)

        self.cap.set(3, 224)
        self.cap.set(4, 224)
        #self.device = torch.device("cpu")
        self.model = YOLO("yolov3u.pt")#.to(self.device)
        self.queue = Queue(maxsize = queue_size)
        try:
            self.model = YOLO("yolov3u.tflite")
        except:
            self.ptmodel = YOLO(yolo_model)
            self.ptmodel.export(format="tflite", imgsz=224, optimize=True, half=True, device="cpu")
            self.model = YOLO("yolov3u.tflite")

        self.classNames = [
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
            "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
            "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
            "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
            "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
            "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
            "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
            "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
            "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
            "teddy bear", "hair drier", "toothbrush"
        ]

    # conf - prob class
    # cls - name class
    # xyxy - point of box

    def stream(self):
        while True:
            success, image = self.cap.read()
            if success:
                results = self.model(image, stream=True)
                self.queue.put(results)
                    
if __name__ == "__main__":
    model = Model()
    model.stream()
