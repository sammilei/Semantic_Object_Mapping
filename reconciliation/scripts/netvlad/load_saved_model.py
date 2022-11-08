# Adapted from main.py of https://github.com/Nanne/pytorch-NetVlad
# Check that repo for training different models

# From this same repo, one set of pretrained weights can be found at https://drive.google.com/open?id=17luTjZFCX639guSVy00OUtzfTQo4AMF2
# These weights correspond to default architecture and settings
from os.path import isfile

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models

import numpy as np
import netvlad


class Flatten(nn.Module):
    def forward(self, input):
        return input.view(input.size(0), -1)


class L2Norm(nn.Module):
    def __init__(self, dim=1):
        super().__init__()
        self.dim = dim

    def forward(self, input):
        return F.normalize(input, p=2, dim=self.dim)


def load_saved_model(checkpoint_path, base_arch='vgg16', pooling='netvlad', cuda=True, vladv2=False, num_clusters=64):
    """Builds and returns the model to extract NetVLAD descriptors from images

    Arguments:
        checkpoint_path {str} -- Path to the saved Pytorch model. Must correspond to architecture defined with other keyword arguments.

    Keyword Arguments:
        base_arch {str} -- Base feature extraction architecture, choices=['vgg16','alexnet'] (default: {'vgg16'})
        pooling {str} -- Type of pooling to use, choices=['netvlad', 'max', 'avg'] (default: {'netvlad'})
        cuda {bool} -- Whether a GPU should/can be used (default: {True})
        vladv2 {bool} -- Use VLAD v2 (default: {False})
        num_clusters {int} -- Number of NetVlad clusters. (default: {64})

    Returns:
        torch.nn.Module -- Pytorch model that takes in images (N,3,W,H) and outputs NetVLAD descriptors
    """

    device = torch.device("cuda" if cuda else "cpu")

    print('===> Building model')

    if base_arch == 'alexnet':
        encoder_dim = 256
        encoder = models.alexnet(pretrained=False)
        # capture only features and remove last relu and maxpool
        layers = list(encoder.features.children())[:-2]

        # if using pretrained only train conv5
        for l in layers[:-1]:
            for p in l.parameters():
                p.requires_grad = False

    elif base_arch == 'vgg16':
        encoder_dim = 512
        encoder = models.vgg16(pretrained=False)
        # capture only feature part and remove last relu and maxpool
        layers = list(encoder.features.children())[:-2]

        # if using pretrained then only train conv5_1, conv5_2, and conv5_3
        for l in layers[:-5]:
            for p in l.parameters():
                p.requires_grad = False

    encoder = nn.Sequential(*layers)
    model = nn.Module()
    model.add_module('encoder', encoder)

    if pooling.lower() == 'netvlad':
        net_vlad = netvlad.NetVLAD(num_clusters=num_clusters, dim=encoder_dim, vladv2=vladv2)
        model.add_module('pool', net_vlad)
    elif pooling.lower() == 'max':
        global_pool = nn.AdaptiveMaxPool2d((1, 1))
        model.add_module('pool', nn.Sequential(*[global_pool, Flatten(), L2Norm()]))
    elif pooling.lower() == 'avg':
        global_pool = nn.AdaptiveAvgPool2d((1, 1))
        model.add_module('pool', nn.Sequential(*[global_pool, Flatten(), L2Norm()]))
    else:
        raise ValueError('Unknown pooling type: ' + pooling)

    if isfile(checkpoint_path):
        print("=> loading checkpoint '{}'".format(checkpoint_path))
        checkpoint = torch.load(checkpoint_path, map_location=lambda storage, loc: storage)
        model.load_state_dict(checkpoint['state_dict'])
        model = model.to(device)
        print("=> loaded checkpoint '{}' (epoch {})"
            .format(checkpoint_path, checkpoint['epoch']))
    else:
        print("=> no checkpoint found at '{}'".format(checkpoint_path))
    model.eval()
    return model
