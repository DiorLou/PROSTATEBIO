from __future__ import annotations

import torch
from torch import nn
from torchvision import models


class SevenFrameCenterSelector(nn.Module):
    """Shared CNN encoder for seven yaw-sweep frames, followed by an 8-class selector."""

    def __init__(
        self,
        input_channels: int = 1,
        num_classes: int = 8,
        backbone: str = "resnet18",
        pretrained: bool = False,
        dropout: float = 0.2,
    ) -> None:
        super().__init__()
        if backbone != "resnet18":
            raise ValueError("Only resnet18 is implemented for the first Model 1 baseline")

        weights = models.ResNet18_Weights.DEFAULT if pretrained else None
        resnet = models.resnet18(weights=weights)
        if input_channels != 3:
            resnet.conv1 = nn.Conv2d(
                input_channels,
                resnet.conv1.out_channels,
                kernel_size=resnet.conv1.kernel_size,
                stride=resnet.conv1.stride,
                padding=resnet.conv1.padding,
                bias=False,
            )
        feature_dim = resnet.fc.in_features
        resnet.fc = nn.Identity()
        self.encoder = resnet
        self.classifier = nn.Sequential(
            nn.LayerNorm(feature_dim * 7),
            nn.Dropout(dropout),
            nn.Linear(feature_dim * 7, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(512, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x shape: [batch, 7, channels, height, width]
        batch_size, num_frames, channels, height, width = x.shape
        if num_frames != 7:
            raise ValueError(f"Expected 7 frames, got {num_frames}")

        features = self.encoder(x.view(batch_size * num_frames, channels, height, width))
        features = features.view(batch_size, num_frames, -1)
        return self.classifier(features.flatten(start_dim=1))

