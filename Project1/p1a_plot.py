import pandas as pd
import matplotlib.pyplot as plt

def main():
    imu = pd.read_csv('data/p1a_imu.csv')
    odom = pd.read_csv('data/p1a_odom.csv')

    # Timestamp normalisieren
    imu['timestamp'] = imu['timestamp'] - imu['timestamp'].iloc[0]
    odom['timestamp'] = odom['timestamp'] - odom['timestamp'].iloc[0]

    print(imu.head())
    print(odom.head())

    # https://stackoverflow.com/questions/47775220/how-to-plot-multiple-pandas-columns

    imu.plot(x='timestamp', y=['orientation.x', 'orientation.y', 'orientation.z', 'orientation.w'], title='IMU Orientation', xlabel='Zeit in s', ylabel='Orientation')
    imu.plot(x='timestamp', y=['angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z'], title='IMU Angular Velocity', xlabel='Zeit in s', ylabel='Angular Velocity')
    imu.plot(x='timestamp', y=['linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z'], title='IMU Linear Acceleration', xlabel='Zeit in s', ylabel='Linear Acceleration')

    odom.plot(x='timestamp', y=['position.x', 'position.y', 'position.z'], title='Odometrie Position', xlabel='Zeit in s', ylabel='Position')
    odom.plot(x='timestamp', y=['orientation.x', 'orientation.y', 'orientation.z', 'orientation.w'], title='Odometrie Orientation', xlabel='Zeit in s', ylabel='Orientation')
    odom.plot(x='timestamp', y=['linear.x', 'linear.y', 'linear.z'], title='Odometrie Linear Velocity', xlabel='Zeit in s', ylabel='Linear Velocity')
    odom.plot(x='timestamp', y=['angular.x', 'angular.y', 'angular.z'], title='Odometrie Angular Velocity', xlabel='Zeit in s', ylabel='Angular Velocity')
    plt.show()

if __name__ == '__main__':
    main()