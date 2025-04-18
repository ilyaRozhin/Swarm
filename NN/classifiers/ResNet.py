from torchvision.models import resnet152
import torchvision
from base import Model
import cv2


if __name__ == "__main__":
    model = Model(resnet152(weights=torchvision.models.ResNet152_Weights.IMAGENET1K_V2))

    name, _ = model.request(
        cv2.imread("NN/classifiers/test_images/car.jpeg")
    )
    assert name == "pickup", "Bad Car Test"

    name, _ = model.request(
        cv2.imread("NN/classifiers/test_images/dog.jpg")
    )
    assert name == "Samoyed", "Bad Dog Test"

    name, _ = model.request(
        cv2.imread("NN/classifiers/test_images/building.jpg")
    )
    assert name == "beacon", "Bad Beacon Test"

    model.video_flow(0)

