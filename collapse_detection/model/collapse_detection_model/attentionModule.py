import torch
import torch.nn as nn

class AttentionMechanism(nn.Module):
    def __init__(self, input_dim, num_parts):
        super(AttentionMechanism, self).__init__()
        self.fc = nn.Linear(input_dim * num_parts, 1)

    def forward(self, left_arm, right_arm, trunk, left_leg, right_leg):
        # Concatenate the processed features of each body part
        concatenated_parts = torch.cat([left_arm, right_arm, trunk, left_leg, right_leg], dim=1)
        

        # Calculate importance weights for each part
        importance_weights = self.fc(concatenated_parts)
        attention_weights = torch.softmax(importance_weights, dim=1)
        
        # Apply attention to the parts
        attended_left_arm = attention_weights[:, :left_arm.size(1)] * left_arm
        attended_right_arm = attention_weights[:, :right_arm.size(1)] * right_arm
        attended_trunk = attention_weights[:, :trunk.size(1)] * trunk
        attended_left_leg = attention_weights[:, :left_leg.size(1)] * left_leg
        attended_right_leg = attention_weights[:, :right_leg.size(1)] * right_leg

        # Sum up the attended features
        attended_features = attended_left_arm + attended_right_arm + attended_trunk + attended_left_leg + attended_right_leg
        
        return attended_features
