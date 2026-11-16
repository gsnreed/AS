import pandas as pd
import matplotlib.pyplot as plt

def plot_data(df, columns, labels, title, ylabel):
    """Plot data from a DataFrame."""
    ax = df.plot(x='timestamp', y=columns, title=title, xlabel='t [s]', ylabel=ylabel)
    ax.legend(labels)


def main():
    imu = pd.read_csv('data/p1a_imu.csv')
    odom = pd.read_csv('data/p1a_odom.csv')
    # Timestamp normalisieren
    imu['timestamp'] = imu['timestamp'] - imu['timestamp'].iloc[0]
    odom['timestamp'] = odom['timestamp'] - odom['timestamp'].iloc[0]

    print(imu.head())
    print(odom.head())

    # https://stackoverflow.com/questions/47775220/how-to-plot-multiple-pandas-columns

    plot_data(imu, ['orientation.x', 'orientation.y', 'orientation.z', 'orientation.w'], ['X', 'Y', 'Z', 'W'], 'IMU Orientation', 'Orientation [rad]')
    plot_data(imu, ['angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z'], ['X', 'Y', 'Z'], 'IMU Angular Velocity', 'Angular Velocity [rad/s]')
    plot_data(imu, ['linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z'], ['X', 'Y', 'Z'], 'IMU Linear Acceleration', 'Linear Acceleration [m/s^2]')

    plot_data(odom, ['position.x', 'position.y', 'position.z'], ['X', 'Y', 'Z'], 'Odometry Position', 'Position [m]')
    plot_data(odom, ['orientation.x', 'orientation.y', 'orientation.z', 'orientation.w'], ['X', 'Y', 'Z', 'W'], 'Odometry Orientation', 'Orientation [rad]')
    plot_data(odom, ['linear.x', 'linear.y', 'linear.z'], ['X', 'Y', 'Z'], 'Odometry Linear Velocity', 'Linear Velocity [m/s]')
    plot_data(odom, ['angular.x', 'angular.y', 'angular.z'], ['X', 'Y', 'Z'], 'Odometry Angular Velocity', 'Angular Velocity [rad/s]')
    plt.show()

if __name__ == '__main__':
    main()