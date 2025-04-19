from torchvision.models import alexnet
import torchvision
from NN.classifiers.BaseClassifier import Model
import cv2


class CurrentModel(Model):
    def __init__(self):
        super().__init__(
            alexnet(
                weights=torchvision.models.AlexNet_Weights.DEFAULT
            )
        )

if __name__ == "__main__":
    model = CurrentModel()

    name, _ = model.request(
        cv2.imread("NN/classifiers/test_images/car.jpeg")
    )
    assert name == "pickup", "Bad Car Test"

    name, _ = model.request(
        cv2.imread("NN/classifiers/test_images/dog.jpg")
    )
    assert name == "Samoyed", "Bad Dog Test"

    #model.video_flow(0)
