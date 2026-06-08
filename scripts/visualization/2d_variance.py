from cv2 import line
import matplotlib.pyplot as plt
import os

def read_file(filename):
    if not filename.endswith('.txt'):
        raise ValueError("File must be a .txt file")
    elif not os.path.exists(filename):
        raise FileNotFoundError(f"File {filename} does not exist")

    data = []
    with open(filename, 'r') as f:
        for line in f:
            line_data = line.strip().split()
            data.append(line_data)
    return data

def main(esitmated_pos_file, variance_file):
    # Load data from files
    estimated_positions = read_file(esitmated_pos_file)
    variances = read_file(variance_file)

    # Extract X positions and variances
    x_positions = [float(pos[0]) for pos in estimated_positions]
    x_variances = [float(var[0]) for var in variances]

    # computer factor used to scale variances for better visualization
    factor = max(x_positions) / max(x_variances) if max(x_variances) > 0 else 1

    x_variances = [var * factor for var in x_variances]

    plt.plot(x_positions, color='blue', label='X Position')
    plt.fill_between(range(len(x_positions)), 
                     [x - var for x, var in zip(x_positions, x_variances)], 
                     [x + var for x, var in zip(x_positions, x_variances)], 
                     color='blue', alpha=0.2, label='X Variance')
    plt.xlabel('Time step')
    plt.ylabel('X Position')
    plt.title(f'X Position and X Variance (scaled by {factor:.0f}) over time')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Plot X Position and X Variance over time.')
    parser.add_argument('estimated_positions_file', type=str, help='File containing estimated positions')
    parser.add_argument('variance_file', type=str, help='File containing variances')
    args = parser.parse_args()
    main(args.estimated_positions_file, args.variance_file)