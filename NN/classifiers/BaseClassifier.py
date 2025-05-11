from torchvision import transforms
import torch
import cv2

class Model:
    def __init__(self, cernel):
        self.cernel = cernel
        self.cernel.eval()
        self.preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])
        with open("NN/classifiers/imagenet_classes.txt", "r") as f:
            self.categories = [s.strip() for s in f.readlines()]

    def video_flow(self, device):
        try:
            cam = cv2.VideoCapture(device)
        except:
            print("Camera didn't define")
            return
        while True:
            result, image = cam.read()
            if result:
                print(self.request(image))

    def request(self, image):
        input_tensor = self.preprocess(image).unsqueeze(0)
        with torch.inference_mode():
            output_tensor = self.cernel(input_tensor)
        probabilities = torch.nn.functional.softmax(output_tensor[0], dim=0)
        object_index = probabilities.argmax(dim=0)
        object_name = self.categories[object_index]
        confidence = probabilities.max().item()
        return object_name, confidence