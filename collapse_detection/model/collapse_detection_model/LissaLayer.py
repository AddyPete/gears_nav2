import torch
import torch.nn as nn

class LiSSALayer(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(LiSSALayer, self).__init__()
        self.temporal_conv = nn.Conv1d(input_dim, output_dim, kernel_size=3, padding=1)
        self.spatial_conv = nn.Conv1d(output_dim, output_dim, kernel_size=3, padding=1)

    def forward(self, x):
        batch_size = x.size(0)
        sequence_length = x.size(1)
        x = x.view(batch_size, sequence_length , -1)
        batch_size, sequence_length, num_features = x.size()


        # Reshape the input tensor for Conv1d
        x = x.permute(0, 2, 1)  # Adjust the dimension for Conv1d
        
        # Apply temporal convolution to capture temporal information
        temporal_features = self.temporal_conv(x)
        temporal_features = torch.relu(temporal_features)

        # Apply spatial convolution to capture spatial information
        spatial_features = self.spatial_conv(temporal_features)
        spatial_features = torch.relu(spatial_features)
        
        # Summing across temporal dimension to aggregate temporal information
        aggregated_features, _ = torch.max(spatial_features, dim=2)
        
        return aggregated_features