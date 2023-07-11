import torch
import torch.nn as nn
import torch.optim as optim
from torch.autograd import Variable
import os
import torch.nn.functional as F
import numpy as np
import random
from torch.utils.data import Dataset, DataLoader, SubsetRandomSampler


import torch

class MemoryDataset(Dataset):
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
    
    def __len__(self):
        return len(self.memory)
    
    def __getitem__(self, idx):
        last_state, new_state, action, reward, position = self.memory[idx]
        return last_state, new_state, action, reward, position
    
    def push(self, state, new_state, action, reward, position):
        self.memory.append((state, new_state, action, reward, position))
        if len(self.memory) > self.capacity:
            self.memory.pop(0)

class Network(nn.Module):
    
    def __init__(self, input_shape, nb_action):
        super(Network, self).__init__()
        self.input_shape = input_shape
        self.nb_action = nb_action
        self.conv1 = nn.Conv2d(input_shape[0], 32, kernel_size=4, stride=2)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=2, stride=1)
        self.conv3 = nn.Conv2d(64, 64, kernel_size=2, stride=1)

        self.adaptive_pool = nn.AdaptiveAvgPool2d((1, 1))  # to ensure the size before linear layer is fixed
        self.fc = nn.Linear(96, nb_action)  # we are flattening conv3 output, so it will have 64 feature maps
    
         # we are flattening conv3 output, so it will have 64 feature maps
        self.position_fc = nn.Linear(2, 32)
    
    def forward(self, state, position):# function for propagation
        x = F.relu(self.conv1(state))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = self.adaptive_pool(x)
        x = x.view(x.size(0), -1)
        position_encoded = F.relu(self.position_fc(position))

        if (torch.unsqueeze(position_encoded, 0)).dim() == 3:
            lab = position_encoded
            x = torch.cat((x, lab), dim=1)
        else:
            x = torch.cat((x, torch.unsqueeze(position_encoded,0)), dim=1)

        q_values = self.fc(x)
        return q_values



class Dqn():
    
    def __init__(self, input_shape, nb_action, gamma):
        self.gamma = gamma
        self.model = Network(input_shape, nb_action)
        self.optimizer = optim.Adam(self.model.parameters(), lr = 0.001)
        self.last_state = torch.Tensor(input_shape).unsqueeze(0)
        self.last_action = 0
        self.last_reward = 0
        self.sample_size = 25
        self.reward_window = []
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.dataset = MemoryDataset(25)
    
    def select_action(self, state):
        # This will return the index of the action corresponding to the neighboring cell with the highest expected future reward.
        with torch.no_grad():
            return self.model(state).max(1)[1].view(1, 1)
    

    #put 9 as dont move
    def update(self, reward, new_state, action, last_state, last_position, new_position):
        tensor = torch.tensor([[1, 2]])
        self.dataset.push(torch.tensor(last_state), torch.Tensor(new_state), torch.LongTensor([int(action)]), torch.Tensor([reward]), torch.Tensor([[last_position[0],last_position[1]]]).squeeze(0) )
        #action = self.select_action(new_state, new_position)
        if len(self.dataset.memory) == self.dataset.capacity or len(self.dataset.memory) > 5:
            #print(self.memory.memory[0], self.memory.memory_positions[0])
            num_samples = len(self.dataset.memory)
            indices = torch.randperm(num_samples)[:self.sample_size]

            batch_size = self.sample_size  
            sampler = SubsetRandomSampler(indices)

            data_loader = DataLoader(self.dataset, batch_size=batch_size, sampler=sampler)
    #        return last_state, new_state, action, reward, position

            for batch_last_state, batch_new_state, batch_action,batch_reward , batch_position in data_loader:
                batch_last_state = batch_last_state.squeeze(0).unsqueeze(1)
                batch_new_state = batch_new_state.squeeze(0).unsqueeze(1)

                self.learn(batch_last_state, batch_position, batch_new_state, batch_reward, batch_action)

        self.last_action = action
        self.last_state = new_state
        self.last_position = new_position
        self.last_reward = reward
        self.reward_window.append(reward)
        if len(self.reward_window) > 25:
            del self.reward_window[0]
        return action


    def learn(self, batch_state, batch_position, batch_next_state, batch_reward, batch_action):
        outputs = self.model(batch_state, batch_position).gather(1, batch_action).squeeze(1)
        next_outputs = self.model(batch_next_state, batch_position).detach().max(1)[0]
        target = self.gamma * next_outputs + batch_reward
        td_loss = F.smooth_l1_loss( (outputs.view(-1, 1)).expand(-1, target.size(1)), target)
        self.optimizer.zero_grad()
        td_loss.backward(retain_graph=True)
        self.optimizer.step()