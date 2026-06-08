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


def main(estimated_pos_file):
    '''
    Main function to read estimated positions and plot the trajectory in 2D.
    '''
    estimated_positions = read_file(estimated_pos_file)

    x_positions = [float(pos[0]) for pos in estimated_positions]
    y_positions = [float(pos[1]) for pos in estimated_positions]

    plt.figure(figsize=(8, 8))
    plt.plot(x_positions, y_positions, color='blue', label='Estimated Position')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Estimated Position Trajectory')
    plt.legend()
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Plot estimated position trajectory in 2D.')
    parser.add_argument('estimated_positions_file', type=str, help='File containing estimated positions')
    args = parser.parse_args()
    main(args.estimated_positions_file)
