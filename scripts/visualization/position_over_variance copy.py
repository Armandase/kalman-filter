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

    # Create a figure and axis
    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    # Plot X positions
    ax1.plot(x_positions, color='blue', label='X Position')
    ax1.set_xlabel('Time step')
    ax1.set_ylabel('X Position', color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')
    # Plot X variances
    ax2.plot(x_variances, color='red', label='X Variance')
    ax2.set_ylabel('X Variance', color='red')
    ax2.tick_params(axis='y', labelcolor='red')
    # Add title and legend
    plt.title('X Position and X Variance over time')
    fig.legend(loc='upper right')
    # Show the plot
    plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Plot X Position and X Variance over time.')
    parser.add_argument('estimated_positions_file', type=str, help='File containing estimated positions')
    parser.add_argument('variance_file', type=str, help='File containing variances')
    args = parser.parse_args()
    main(args.estimated_positions_file, args.variance_file)