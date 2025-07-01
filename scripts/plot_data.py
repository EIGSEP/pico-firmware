"""Script that plots data. Must have 'freqs' and 'antenna' keys in the npzs."""

import sys
import numpy as np
import matplotlib.pyplot as plt

plt.figure()
for filename in sys.argv[1:]:
    print(f"Reading {filename}")
    npz = np.load(filename)
    freqs, data = npz["freqs"], npz["antenna"]
    plt.plot(freqs / 1e6, 20 * np.log10(np.abs(data[0])), label=filename)
plt.legend()
plt.show()
