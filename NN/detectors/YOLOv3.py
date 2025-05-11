from ultralytics import YOLO
import cv2
import math 
import torch
from queue import Queue

# Function to get class colors
def getColours(cls_num):
    base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    color_index = cls_num % len(base_colors)
    increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
    color = [base_colors[color_index][i] + increments[color_index][i] * 
    (cls_num // len(base_colors)) % 256 for i in range(3)]
    return tuple(color)

class Model:
    def __init__(self, device = "/dev/video0",
                yolo_model = "yolov3u.pt", queue_size = 100):
        self.cap = cv2.VideoCapture(device)

        self.cap.set(3, 512)
        self.cap.set(4, 512)
        self.device = torch.device("cuda:0")
        self.model = YOLO("yolov3u.pt")#.to(self.device)
        self.queue = Queue(maxsize = queue_size)
        try:
            self.model = YOLO("yolov3u.tflite")
        except:
            self.ptmodel = YOLO(yolo_model)
            self.ptmodel.export(format="tflite", imgsz=512, optimize=True, half=True, device="cpu")
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
                for result in results:
                    # get the classes names
                    classes_names = result.names

                    # iterate over each box
                    for box in result.boxes:
                        # check if confidence is greater than 40 percent
                        if box.conf[0] > 0.4:
                            # get coordinates
                            [x1, y1, x2, y2] = box.xyxy[0]
                            # convert to int
                            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                            # get the class
                            cls = int(box.cls[0])

                            # get the class name
                            class_name = classes_names[cls]

                            # get the respective colour
                            colour = getColours(cls)

                            # draw the rectangle
                            cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)

                            # put the class name and confidence on the image
                            cv2.putText(image, f'{classes_names[int(box.cls[0])]} {box.conf[0]:.2f}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
                            
                # show the image
                cv2.imshow('frame', image)

                # break the loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        # release the video capture and destroy all windows
        self.cap.release()
        cv2.destroyAllWindows()
                    
if __name__ == "__main__":
    model = Model()
    model.stream()
