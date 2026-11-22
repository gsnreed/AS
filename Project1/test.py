import numpy as np

def main(args=None):
    ranges = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0])
    phi_increment = 0.1
    phi_min = -0.2
    front_angle = 0.35

    angles = np.arange(len(ranges)) * phi_increment + phi_min    # Winkel der Messungen (arange macht ein Array von 0 bis len(ranges)-1)
    front_indices = np.abs(angles) < front_angle   # Indizes der Messungen im Frontbereich
    front_ranges = ranges[front_indices]   # Messungen im Frontbereich
    print(front_ranges)

if __name__ == '__main__':
    main()