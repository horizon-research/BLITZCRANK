import numpy as np
from gtsam import *
from gpmp2 import *
from scipy import ndimage
import PIL.Image as Image
import matplotlib.pyplot as plt
import sys


def image_to_matrix(file_name):
    image = Image.open(file_name)
    # image.show()
    width, height = image.size
    image_grey = image.convert("L")
    data = image_grey.getdata()
    data = np.matrix(data, dtype="float") / 255.0
    new_data = np.reshape(data, (height, width))
    return new_data, height, width


def signedDistanceField2D(ground_truth_map, cell_size):
    # SIGNEDDISTANCEFIELD2D 2D signed distance field
    #   Given a ground truth 2D map defined in Matrix in 0-1,
    #   calculate 2D signed distance field, which is defined as a matrix
    #   map matrix and signed distance field matrix have the same resolution.
    #
    #   Usage: field = SIGNEDDISTANCEFIELD2D(ground_truth_map, cell_siz)
    #   @map        evidence grid from dataset, map use 0 show open area, 1 show objects.
    #   @cell_size  cell sizeto given metric information
    #
    #   Output:
    #   @field      sdf, row is Y, col is X

    # regularize unknow area to open area
    cur_map = ground_truth_map < 0.5
    cur_map = cur_map.astype(int)

    if np.amin(cur_map) == 1:
        return np.ones(ground_truth_map.shape) * 1000

    # inverse map
    inv_map = 1 - cur_map

    # get signed distance from map and inverse map
    # since bwdist(foo) = ndimage.distance_transform_edt(1-foo)
    map_dist = ndimage.distance_transform_edt(inv_map)
    inv_map_dist = ndimage.distance_transform_edt(cur_map)

    field = map_dist - inv_map_dist

    # metric
    field = field * cell_size
    field = field.astype(float)

    return field


def plotSignedDistanceField2D(figure, axis, field, origin_x, origin_y, cell_size, epsilon_dist=0):
    # %PLOTSIGNEDDISTANCEFIELD2D plot 2D SignedDistanceField
    # %
    # %   Usage: PLOTSIGNEDDISTANCEFIELD2D(field, origin_x, origin_y, cell_size, epsilon_dist)
    # %   @field                  field matrix
    # %   @origin_x, origin_y     origin (down-left) corner of the map
    # %   @cell_size              cell size
    # %   @epsilon_dist           optional plot obstacle safety distance, default = 0

    # get X-Y coordinates
    grid_rows = field.shape[0]
    grid_cols = field.shape[1]
    grid_corner_x = origin_x + (grid_cols - 1) * cell_size
    grid_corner_y = origin_y + (grid_rows - 1) * cell_size

    grid_X = np.linspace(origin_x, grid_corner_x, num=grid_cols)
    grid_Y = np.linspace(origin_y, grid_corner_y, num=grid_rows)

    z_min = np.amin(field)
    z_max = np.amax(field)
    c = axis.pcolor(grid_X, grid_Y, field, cmap="Greens_r",
                    vmin=z_min, vmax=z_max)
    figure.colorbar(c, ax=axis)  # add colorbar

    # set(gca,'YDir','normal')
    axis.invert_yaxis()  # TODO: check this again! same as set(gca,'YDir','normal')

    axis.axis("equal")
    axis.axis(
        [
            origin_x - cell_size / 2,
            grid_corner_x + cell_size / 2,
            origin_y - cell_size / 2,
            grid_corner_y + cell_size / 2,
        ]
    )

    # colorbar
    axis.set_xlabel("X/m")
    axis.set_ylabel("Y/m")
    axis.set_title("Signed Distance Field")


def test():
    cell_size = 1
    origin_x = 0
    origin_y = 0
    origin_z = 0
    data = np.array([[[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]], [[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]], [[1, 1, 1, 1], [1, 0, 0, 1], [
                    1, 0, 0, 1], [1, 1, 1, 1]], [[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]], [[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]]])
    field = signedDistanceField2D(data, cell_size)
    figure1 = plt.figure(0)
    axis1 = figure1.gca()  # for 3-d, set gca(projection='3d')
    # plotSignedDistanceField2D(figure1, axis1, field,
    #                           origin_x, origin_y, cell_size)
    # plt.show()
    # field.resize(4, 4, 1)
    print(field.shape)
    print(field)
    origin_point3 = Point3(np.array([origin_x, origin_y, origin_z]))
    sdf = SignedDistanceField(origin_point3, cell_size,
                              field.shape[1], field.shape[2], field.shape[0])
    for i in range(field.shape[0]):
        sdf.initFieldData(i, field[i, :, :])
    sdf.saveSDF("../sdf/test_sdf.bin".encode('utf8'))


def main():

    if (len(sys.argv) != 2):
        print("please input the figure path!")
        sys.exit()

    cell_size = 0.05
    origin_x = 0
    origin_y = 0
    # origin_z = 0
    file_name = sys.argv[1]
    data, height, width = image_to_matrix(file_name)
    field = signedDistanceField2D(data, cell_size)

    # plot sdf
    figure1 = plt.figure(0)
    axis1 = figure1.gca()  # for 3-d, set gca(projection='3d')
    plotSignedDistanceField2D(figure1, axis1, field, origin_x, origin_y, cell_size)
    plt.show()
    # field = np.array([field, field])
    print(field.shape)

    # 3d sdf
    # origin_point3 = Point3(np.array([origin_x, origin_y, origin_z]))
    # sdf = SignedDistanceField(origin_point3, cell_size, field.shape[1], field.shape[2], field.shape[0])
    # for i in range(field.shape[0]):
    #     sdf.initFieldData(i, field[i, :, :].T)

    # 2d sdf
    origin_point2 = Point2(origin_x, origin_y)
    sdf = PlanarSDF(origin_point2, cell_size, field)
    sdf.saveSDF("../sdf/simple_rooms.bin".encode('utf8'))


if __name__ == "__main__":
    main()
