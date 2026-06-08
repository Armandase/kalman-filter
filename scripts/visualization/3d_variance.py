from cv2 import line
import matplotlib.pyplot as plt
import os

def read_file(filename, max_lines=None):
    if not filename.endswith('.txt'):
        raise ValueError("File must be a .txt file")
    elif not os.path.exists(filename):
        raise FileNotFoundError(f"File {filename} does not exist")

    data = []
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            if max_lines is not None and i >= max_lines:
                break
            line_data = line.strip().split()
            data.append(line_data)
    return data

def main(estimated_pos_file, variance_file, max_lines=None):
    # Load data from files
    estimated_positions = read_file(estimated_pos_file, max_lines)
    variances = read_file(variance_file, max_lines)

    # 3d plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x_positions = [float(pos[0]) for pos in estimated_positions]
    y_positions = [float(pos[1]) for pos in estimated_positions]
    z_positions = [float(pos[2]) for pos in estimated_positions]
    
    x_variances = [float(var[0]) for var in variances]
    y_variances = [float(var[1]) for var in variances]
    z_variances = [float(var[2]) for var in variances]

    x_factor = max(x_positions) / max(x_variances) if max(x_variances) > 0 else 1
    y_factor = max(y_positions) / max(y_variances) if max(y_variances) > 0 else 1
    z_factor = max(z_positions) / max(z_variances) if max(z_variances) > 0 else 1

    x_variances = [var * x_factor for var in x_variances]
    y_variances = [var * y_factor for var in y_variances]
    z_variances = [var * z_factor for var in z_variances]

    ax.fill_between([x - var for x, var in zip(x_positions, x_variances)], 
                    [y - var for y, var in zip(y_positions, y_variances)], 
                    [z - var for z, var in zip(z_positions, z_variances)],
                    [x + var for x, var in zip(x_positions, x_variances)],
                    [y + var for y, var in zip(y_positions, y_variances)],
                    [z + var for z, var in zip(z_positions, z_variances)],
                     color='red', alpha=0.7, label='X Variance', lw=0)

    ax.plot(x_positions, y_positions, z_positions, color='blue', label='Estimated Trajectory')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('3D Trajectory of Estimated Positions')
    ax.legend()

    plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Plot X Position and X Variance over time.')
    parser.add_argument('estimated_positions_file', type=str, help='File containing estimated positions')
    parser.add_argument('variance_file', type=str, help='File containing variances')
    parser.add_argument('--max_lines', type=int, default=None, help='Maximum number of lines to read from files')
    args = parser.parse_args()
    main(args.estimated_positions_file, args.variance_file, args.max_lines)