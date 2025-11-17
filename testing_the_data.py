import numpy as np
data = np.load("small_dataset.npz", allow_pickle=True)
print(data.files)
print(data["height"][0].shape)
