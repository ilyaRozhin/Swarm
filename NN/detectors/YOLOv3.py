from ultralytics import YOLO
import cv2
import torch
import os 
from queue import Queue
from collections import namedtuple
from threading import Event
import shutil

# Function to get class colors
def getColours(cls_num):
    base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    color_index = cls_num % len(base_colors)
    increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
    color = [base_colors[color_index][i] + increments[color_index][i] * 
    (cls_num // len(base_colors)) % 256 for i in range(3)]
    return tuple(color)

DetectResults = namedtuple("DetectResults", ['acc', 'xy', 'image'])

class Model:
    def __init__(self, triggers, device = "/dev/video0", yolo_model = "NN/detectors/yolov3-tinyu.pt"):
        self.cap = cv2.VideoCapture(device)

        self.cap.set(3, 224)
        self.cap.set(4, 224)

        self.device = torch.device("cpu")
        self.model = YOLO(yolo_model).to(self.device)
        self.blocker = Event()
        self.container = {}

        self.triggers = triggers

    def stream(self):
        if os.path.exists("NN/detectors/results/"):
            shutil.rmtree("NN/detectors/results/")
        os.mkdir("NN/detectors/results")
        while True:
            success, image = self.cap.read()
            if success:
                results = self.model.track(image, stream=True, tracker="NN/detectors/tracker_config.yaml")
                for result in results:
                    class_names = result.names
                    for box in result.boxes:
                        cls = int(box.cls[0])
                        box_name = class_names[cls]
                        if not (box.id is None) and box_name in self.triggers:
                            if not f"{box_name}:{int(box.id.item())}" in self.container:
                                colour = getColours(cls)
                                [x1, y1, x2, y2] = box.xyxy[0]
                                x1, y1, x2, y2 = (int(x1), int(y1), int(x2), int(y2))
                                cv2.rectangle(image, (x1, y1), (x2, y2), colour, 1)
                                borders = (x1, y1, x2, y2)
                                cv2.putText(image, f'{box_name}:{int(box.id.item())} {box.conf[0]:.2f} ', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.25, colour, 1)
                                self.container[f"{box_name}:{int(box.id.item())}"] = DetectResults(box.conf[0].item(), borders, image)
                                cv2.imwrite(f"NN/detectors/results/{box_name}_{int(box.id.item())}.jpg", image)
                    
if __name__ == "__main__":
    model = Model(["bottle"])
    model.stream()
