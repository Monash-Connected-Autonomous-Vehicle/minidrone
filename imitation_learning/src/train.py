#!/usr/bin/env python3

import torch

import glob
import PIL
import matplotlib.pyplot as plt

import pandas as pd
import numpy as np

from torch.utils.data import Dataset


# TODO: create Dataset Class
# TODO: create training loop


class ImitationDataset(Dataset):
    """Imitation Learning Dataset for MiniDrone"""

    def __init__(self, data_path="../data/"):

        self.DATA_DIR = data_path
        self.df = pd.read_csv(self.DATA_DIR + "data.csv")
        self.transform = None # TODO

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        fname = self.df.iloc[idx]["fname"]
        img = PIL.Image.open(self.DATA_DIR + fname)

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

    print("Training Script")

    df = pd.read_csv("../data/data.csv")

    print("Dataframe: ", df.head())
    print("Size: ", len(df))

    imitation_dataset = ImitationDataset()

    for sample in imitation_dataset:
        
        img = sample["image"]
        steering = sample["steering"]
        speed = sample["speed"]

        plt.title(f"Steering: {steering}, Speed: {speed}")
        plt.imshow(img)
        plt.show()
