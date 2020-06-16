from matplotlib import pyplot as plt

def read_vec(filename):
    vec = []
    with open(filename, "r") as f:
        for line in f:
            vec.append(float(line))
    return vec

horizontal_hist = read_vec('/home/philipp/tmp/horizontal_hist.csv')
horizontal_hist_gradient = read_vec('/home/philipp/tmp/horizontal_hist_gradient.csv')
horizontal_peak_indicators = read_vec('/home/philipp/tmp/horizontal_peak_indicators.csv')

vertical_hist = read_vec('/home/philipp/tmp/vertical_hist.csv')
vertical_hist_gradient = read_vec('/home/philipp/tmp/vertical_hist_gradient.csv')
vertical_peak_indicators = read_vec('/home/philipp/tmp/vertical_peak_indicators.csv')

fig, axes = plt.subplots(6)

axes[0].bar(list(range(len(horizontal_hist))), horizontal_hist)
axes[1].plot(list(range(len(horizontal_hist_gradient))), horizontal_hist_gradient)
axes[2].plot(list(range(len(horizontal_peak_indicators))), horizontal_peak_indicators)

axes[3].bar(list(range(len(vertical_hist))), vertical_hist)
axes[4].plot(list(range(len(vertical_hist_gradient))), vertical_hist_gradient)
axes[5].plot(list(range(len(vertical_peak_indicators))), vertical_peak_indicators)

plt.show()