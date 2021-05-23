#!/usr/bin/env python3

import torch

import glob
import PIL
import matplotlib.pyplot as plt

import pandas as pd
import numpy as np

from torch.utils.data import Dataset

import torch.nn as nn
import torchvision
import torch.optim as optim

from torchvision import transforms
from torchvision.transforms.transforms import RandomResizedCrop, ToPILImage, ToTensor

# see for more https://pytorch.org/tutorials/beginner/blitz/cifar10_tutorial.html



class ImitationDataset(Dataset):
    """Imitation Learning Dataset for MiniDrone"""

    def __init__(self, DATA_PATH="../data/"):

        self.DATA_PATH = DATA_PATH
        self.df = pd.read_csv(self.DATA_PATH + "data.csv")
        self.transform =  transforms.Compose([
            # transforms.ToPILImage(),
            transforms.RandomResizedCrop((224, 224)),
            transforms.ToTensor()
            ]) 

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        fname = self.df.iloc[idx]["fname"]
        img = PIL.Image.open(self.DATA_PATH + fname)

        steering = self.df.iloc[idx]["steer"]
        speed = self.df.iloc[idx]["speed"]

        if self.transform:
            img = self.transform(img)

        sample = {
                "image": img, 
                "steering": steering, 
                "speed": speed
                }

        return sample


if __name__ == "__main__":

    print("Imitation learning training script")
    # TODO: load config from cmd line args or config.yaml

    # training hyperparameters
    epochs = 2
    batch_size = 1

    # imitation dataset
    imitation_dataset = ImitationDataset()

    
    # dataloader
    train_dataloader = torch.utils.data.DataLoader(imitation_dataset, batch_size=batch_size,
                                          shuffle=True, num_workers=2)

    # device 
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # model 
    print("Model Loading...")
    model = torchvision.models.resnet18(pretrained=True)
    num_fts = model.fc.in_features
    model.fc = nn.Linear(num_fts, 1)
    model = model.to(device)

    print("Model Loaded.")

    # criterion (loss)
    criterion = nn.MSELoss() # mean square error loss
    # optimiser
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    for epoch in range(2):  # loop over the dataset multiple times

        running_loss = 0.0
        for i, sample in enumerate(train_dataloader, 0):
            
            # get the image, steering and speed from sample
            img = sample["image"].to(device)
            steering = sample["steering"].to(device).unsqueeze(0) #TODO: does unsqueeze work to fix broadcast issue?
            speed = sample["speed"].to(device)

            # TODO: fix broadcasting error


            # zero the parameter gradients
            optimizer.zero_grad()

            # forward + backward + optimize
            outputs = model(img)
            loss = criterion(outputs.float(), steering.float()) # needs float for mse
            loss.backward()
            optimizer.step()

            # print statistics
            running_loss += loss.item()
            if i % 9 == 0:    # print every 10 mini-batches
                print('[%d, %5d] loss: %.3f' %
                    (epoch + 1, i + 1, running_loss / 10))
                running_loss = 0.0

                # convert tensor back to np img
                npimg = np.transpose(img.cpu().squeeze(0).numpy(), (1, 2, 0))
            
                plt.title(f"Steering: {steering}, Speed: {speed}")
                plt.imshow(npimg)
                plt.show()

    print('Finished Training')

    model_fname = "../models/model2.pt"
    print(f"Saving model to: {model_fname}")
    torch.save(model.state_dict(), model_fname)

# NOTES:
# memory issues when training
# make sure you increase the swap size to train on jetson
# https://github.com/prlz77/ResNeXt.pytorch/issues/5
# longer term should use a more mobile friendly model, e.g. mobilenetv2