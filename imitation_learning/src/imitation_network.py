import matplotlib.pyplot as plt

import pandas as pd
import numpy as np

from torch.utils.data import Dataset

import torch
import torch.nn as nn
from torchsummary import summary
import torch.functional as F
import torchvision
import torch.optim as optim

from torchvision import transforms
from torchvision.transforms.transforms import RandomResizedCrop, ToPILImage, ToTensor


class Flatten(nn.Module):
    def forward(self, x):
        return x.view(x.shape[0], -1)


class ImitationNet(nn.Module):
    def __init__(self, googlenet):
        super(ImitationNet, self).__init__()

        # Image processing with googlenet
        self.img_net = nn.Sequential(
            nn.Sequential(*list(googlenet.children())[:-2]),
            Flatten(),
            nn.Dropout(p=0.5), nn.ReLU(),
            nn.Linear(1024, 128)
        )

        # State processing with a small MLP
        self.state_net = nn.Sequential(
            nn.Linear(2, 16), nn.Dropout(p=0.5), nn.ReLU(),
            nn.Linear(16, 128)
        )

        # Small MLP to process aggregate data into output
        self.agg = nn.Sequential(
            nn.Dropout(p=0.5), nn.ReLU(),
            nn.Linear(256, 32), nn.Dropout(p=0.5), nn.ReLU(),
            nn.Linear(32, 2)
        )

    def forward(self, img, state):
        s = self.state_net(state)
        i = self.img_net(img)
        return self.agg(torch.cat((i, s), 1))


# Example implementation
# googlenet = torchvision.models.googlenet(pretrained=True)
# test_net = ImitationNet(googlenet)
# test_img = torch.rand((1, 3, 224, 224))
# test_state = torch.rand((1, 2))
# print(test_net(test_img, test_state))
