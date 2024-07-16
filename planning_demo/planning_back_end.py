import numpy as np
from gtsam import *
from gpmp2 import *
import matplotlib.pyplot as plt

from gpmp2_python.utils.plot_utils import *

from PIL import Image
import map2sdf as m2s
import a_star
import math

def planning_back_end():


    file = "simple_rooms.png"

    im = Image.open(file)
    # im.show()

    data = np.array(im)
    data = data[:, :, 0]

    # data,_,_ = m2s.image_to_matrix(file)
    # print(data.shape)

    cell_size = 0.05
    origin_x = 0
    origin_y = 0
    origin_z = 0

    # Signed Distance fsignedDistanceField2Dield
    field = m2s.signedDistanceField2D(data, cell_size)

    # plot sdf
    # figure1 = plt.figure(0)
    # axis1 = figure1.gca()  # for 3-d, set gca(projection='3d')
    # plotSignedDistanceField2D(figure1, axis1, field, origin_x, origin_y, cell_size)
    # plt.show()

    origin_point2 = Point2(np.array([origin_x, origin_y]))

    sdf = PlanarSDF(origin_point2, cell_size, field)

    # settings
    total_time_sec = 25.0
    total_time_step = 70
    total_check_step = 2100
    delta_t = total_time_sec / total_time_step
    check_inter = int(total_check_step / total_time_step - 1)

    use_GP_inter = True


    # point robot model
    pR = PointRobot(3, 1)
    spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 0.2])
    nr_body = spheres_data.shape[0]
    sphere_vec = BodySphereVector()
    sphere_vec.push_back(
        BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4]))
    )
    pR_model = Pose2MobileBaseModel(Pose2MobileBase(), sphere_vec)

    # GP
    Qc = 0.01*np.identity(pR_model.dof())
    Qc_model = noiseModel_Gaussian.Covariance(Qc)

    # Obstacle avoid settings
    cost_sigma = 0.005
    epsilon_dist = 0.5

    # prior to start/goal
    pose_fix = noiseModel_Isotropic.Sigma(pR_model.dof(), 0.0001)
    vel_fix = noiseModel_Isotropic.Sigma(pR_model.dof(), 0.0001)


    # start and end conf
    start_conf_val = np.asarray([2, 2, 0])
    start_conf = Pose2(start_conf_val)
    start_vel = np.asarray([0, 0, 0])
    end_conf_val = np.asarray([16, 12, 0])
    end_conf = Pose2(end_conf_val)
    end_vel = np.asarray([0, 0, 0])
    avg_vel = (end_conf_val - start_conf_val) / total_time_sec


    # plot param
    pause_time = total_time_sec / total_time_step


    # init optimization
    graph = NonlinearFactorGraph()
    init_values = Values()

    start_point, goal_point, rx, ry = a_star.main()
    rx.reverse()
    ry.reverse()
    rx = np.array(rx)
    ry = np.array(ry)
    rx = rx * 0.2
    ry = ry * 0.2
    stride =  len(rx) // (total_time_step + 1)

    init_values.insert(symbol(ord("x"), 0), start_conf);
    init_values.insert(symbol(ord("v"), 0), avg_vel);
    init_values.insert(symbol(ord("x"), total_time_step), end_conf);
    init_values.insert(symbol(ord("v"), total_time_step), avg_vel);

    for i in range(1, total_time_step):
        x = rx[i * stride]
        y = ry[i * stride]
        theta = math.atan2(y -ry[(i - 1) * stride], x - rx[(i - 1) * stride])
        pose = Pose2(x, y, theta);
        init_values.insert(symbol(ord("x"), i), pose);
        init_values.insert(symbol(ord("v"), i), avg_vel);
      


    for i in range(0, total_time_step + 1):
        key_pos = symbol(ord("x"), i)
        key_vel = symbol(ord("v"), i)

        # # % initialize as straight line in conf space
        # pose = Pose2(
        #     start_conf_val * float(total_time_step - i) / float(total_time_step)
        #     + end_conf_val * i / float(total_time_step)
        # )
        # vel = avg_vel
        # # print(pose)
        # init_values.insert(key_pos, pose)
        # init_values.insert(key_vel, vel)

      

        # % start/end priors
        if i == 0:
            graph.push_back(PriorFactorPose2(key_pos, start_conf, pose_fix))
            # graph.push_back(PriorFactorVector(key_vel, start_vel, vel_fix))
        elif i == total_time_step:
            graph.push_back(PriorFactorPose2(key_pos, end_conf, pose_fix))
            graph.push_back(PriorFactorVector(key_vel, end_vel, vel_fix))

        graph.add(VehicleDynamicsFactorPose2(key_pos, key_vel, cost_sigma))

        # GP priors and cost factor
        if i > 0:
            key_pos1 = symbol(ord("x"), i - 1)
            key_pos2 = symbol(ord("x"), i)
            key_vel1 = symbol(ord("v"), i - 1)
            key_vel2 = symbol(ord("v"), i)

            temp = GaussianProcessPriorPose2(
                key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model
            )
            graph.push_back(temp)

            # % cost factor
            graph.push_back(
                ObstaclePlanarSDFFactorPose2MobileBase(
                    key_pos, pR_model, sdf, cost_sigma, epsilon_dist
                )
            )

            graph.push_back(
                VehicleDynamicsFactorPose2(key_pos, key_vel, cost_sigma)
            )

            # % GP cost factor
            if use_GP_inter and check_inter > 0:
                for j in range(1, check_inter + 1):
                    tau = j * (total_time_sec / total_check_step)
                    graph.add(
                        ObstaclePlanarSDFFactorGPPose2MobileBase(
                            key_pos1,
                            key_vel1,
                            key_pos2,
                            key_vel2,
                            pR_model,
                            sdf,
                            cost_sigma,
                            epsilon_dist,
                            Qc_model,
                            delta_t,
                            tau,
                        )
                    )

    use_trustregion_opt = True

    if use_trustregion_opt:
        parameters = DoglegParams()
        # parameters.setVerbosity('ERROR')
        optimizer = DoglegOptimizer(graph, init_values, parameters)
    else:
        parameters = GaussNewtonParams()
        # parameters.setRelativeErrorTol(1e-5)
        # parameters.setMaxIterations(100)
        # parameters.setVerbosity('ERROR')
        optimizer = GaussNewtonOptimizer(graph, init_values, parameters)

    print("Initial Error = %d\n", graph.error(init_values))


    optimizer.optimizeSafely()
    result = optimizer.values()

    print("Final Error = %d\n", graph.error(result))


    # plot final values
    figure = plt.figure(1)
    axis = figure.gca()

    # plot world  
    plotEvidenceMap2D(figure, axis, data, origin_x, origin_y, cell_size)

    for i in range(total_time_step + 1):
        axis.set_title("Optimized Values")
        conf = result.atPose2(symbol(ord("x"), i))
        # print(conf.x(), conf.y(), conf.theta(),
        #       result.atVector(symbol(ord("v"), i)))
        plotPointRobot2D_theta(figure, axis, pR_model, [
                               conf.x(), conf.y(), conf.theta()])
        # plt.pause(pause_time)

    plt.show()


    cx = []
    cy = []
    cyaw = []
    cv = []
    cw = []

    for i in range(total_time_step + 1):
        conf = result.atPose2(symbol(ord("x"), i))
        cx.append(conf.x())
        cy.append(conf.y())
        cyaw.append(conf.theta())
        vel = result.atVector(symbol(ord("v"), i))
        cv.append(vel[0])
        cw.append(vel[2])

    return start_point, goal_point, cx, cy, cyaw, cv, cw

if __name__ == '__main__':
    planning_back_end()
