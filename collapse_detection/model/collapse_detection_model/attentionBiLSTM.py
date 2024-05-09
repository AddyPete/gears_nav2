import torch
import torch.nn as nn
import torch.nn.functional as F
from collapse_detection.model.collapse_detection_model.LissaLayer import LiSSALayer
from collapse_detection.model.collapse_detection_model.attentionModule import (
    AttentionMechanism,
)
from collapse_detection.model.collapse_detection_model.feedForwardNN import (
    FeedForwardNN,
)

# Define the indices for each body part
# print(keypoints_sequences)
left_arm_indices = [5, 7, 9]
right_arm_indices = [6, 8, 10]
trunk_indices = [0, 5, 6]
left_leg_indices = [12, 14, 16]
right_leg_indices = [11, 13, 15]


class ActionRecognizationModel(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dim, num_layers):
        super(ActionRecognizationModel, self).__init__()
        self.attention = AttentionMechanism(input_dim, 5)
        self.lissa = LiSSALayer(input_dim, output_dim)
        # self.bilstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=False, bidirectional=True)
        self.feedForwardNN = FeedForwardNN(input_dim, hidden_dim)
        self.fc = nn.Linear(hidden_dim * 2, 1)
        self.num_layers = num_layers
        self.hidden_dims = hidden_dim
        self.to("cuda")

    def forward(self, keypoints_sequences):

        left_arm_keypoints = keypoints_sequences[:, :, left_arm_indices, :]
        right_arm_keypoints = keypoints_sequences[:, :, right_arm_indices, :]
        trunk_keypoints = keypoints_sequences[:, :, trunk_indices, :]
        left_leg_keypoints = keypoints_sequences[:, :, left_leg_indices, :]
        right_leg_keypoints = keypoints_sequences[:, :, right_leg_indices, :]

        processed_left_arm = self.lissa(left_arm_keypoints)
        processed_right_arm = self.lissa(right_arm_keypoints)
        processed_trunk = self.lissa(trunk_keypoints)
        processed_left_leg = self.lissa(left_leg_keypoints)
        processed_right_leg = self.lissa(right_leg_keypoints)

        output = self.attention(
            processed_left_arm,
            processed_right_arm,
            processed_trunk,
            processed_left_leg,
            processed_right_leg,
        )

        output = self.feedForwardNN(output)
        return output
