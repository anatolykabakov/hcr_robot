
from arduino import protocol
from test_odometry_lib import Utils, TestOdometry, Controller


if __name__ == '__main__':
    port = '/dev/ttyACM0'
    baudrate = 115200

    robot = protocol(port, baudrate)
    controller = Controller()
                            
    utils_for_tests = Utils()
    test_control_level = TestOdometry(robot, controller)

    ## Test move to goal point

    # controller.x_goal = 1
    # controller.y_goal = 2
    # controller.max_angular_speed = 1
    # controller.max_linear_speed = 0.3
    # trajectory_log = test_control_level.move_to_point_test()
    # utils_for_tests.plot_trajectory(trajectory_log['x'], trajectory_log['y'])

    ### Test speed

    set_linear = 0.3
    set_angular = 1.0
    full_time_test = 30
    log_vel = test_control_level.test_speed(set_linear,
                                            set_angular,
                                            full_time_test)
                   
    utils_for_tests.plot_velocity(log_vel['vr'], log_vel['time'], 'right vel[m/s]', '-b')
    utils_for_tests.plot_velocity(log_vel['vl'], log_vel['time'], 'left vel[m/s]', '-r')
    utils_for_tests.show_plot_and_legend()

    robot.stop()