import numpy as np
import random

poster_adjacent = [(0,1),(2,3),(1,0),(3,2)]
poster_count = 4
num_trials = 30  # Total length desired
repetitions = 5   # How many times each poster appears per block

# Initialize with first valid sequence block
reward_sequence = np.arange(poster_count)
reward_sequence = np.squeeze(np.transpose(
    np.matlib.repmat(reward_sequence, 1, repetitions)))

while True:
    np.random.shuffle(reward_sequence)
    check_consec_repeat = np.diff(reward_sequence)
    check_adjacent = False
    
    # Check for adjacent pairs
    for i in range(len(reward_sequence)-1):
        if (reward_sequence[i], reward_sequence[i+1]) in poster_adjacent:
            check_adjacent = True
            break
    
    # Break if no consecutive repeats AND no adjacent pairs
    if 0 not in check_consec_repeat and check_adjacent == False:
        break

full_targets = reward_sequence.copy()

# Generate additional blocks until we reach num_trials
while len(full_targets) < num_trials:
    # Create new block
    reward_sequence = np.arange(poster_count)
    reward_sequence = np.squeeze(np.transpose(
        np.matlib.repmat(reward_sequence, 1, repetitions)))
    
    while True:
        np.random.shuffle(reward_sequence)
        check_consec_repeat = np.diff(reward_sequence)
        check_adjacent = False
        
        # Check for adjacent pairs
        for i in range(len(reward_sequence)-1):
            if (reward_sequence[i], reward_sequence[i+1]) in poster_adjacent:
                check_adjacent = True
                break
        
        # Break if no consecutive repeats AND no adjacent pairs
        if 0 not in check_consec_repeat and check_adjacent == False:
            break
    
    # Handle boundary condition (prevent repeats between blocks)
    if full_targets[-1] == reward_sequence[0]:
        reward_sequence = reward_sequence + 1
        reward_sequence[reward_sequence == poster_count] = 0
        
        # Check if shifting created adjacent pair at boundary
        boundary_pair = (full_targets[-1], reward_sequence[0])
        if boundary_pair in poster_adjacent:
            # Shift again if boundary creates adjacent pair
            reward_sequence = reward_sequence + 1
            reward_sequence[reward_sequence >= poster_count] = reward_sequence[reward_sequence >= poster_count] - poster_count
    
    full_targets = np.hstack((full_targets, reward_sequence))

    print(full_targets)