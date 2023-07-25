import torch
import torch.nn as nn
import numpy as np


class DQN1(nn.Module):

    def __init__(self, input_shape, hidden_size, n_actions):
        super(DQN1, self).__init__()

        self.net = nn.Sequential(
            nn.Linear(input_shape[0], hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),

            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),

            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),

            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(0.3),

            nn.Linear(hidden_size, n_actions)
            # TODO (Deepak): Try using SoftMax here instead
        )

    def forward(self, x):
        return self.net(x.float())


class DQN1WithSoftmax(nn.Module):

    def __init__(self, input_shape, hidden_size, n_actions):
        super(DQN1WithSoftmax, self).__init__()

        self.net = nn.Sequential(
            nn.Linear(input_shape[0], hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),

            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),

            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),

            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Dropout(0.3),

            nn.Softmax(hidden_size, n_actions)
            # TODO (Deepak): Try using SoftMax here instead
        )

    def forward(self, x):
        return self.net(x.float())
