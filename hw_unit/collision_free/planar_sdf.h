#pragma once

#include <ap_int.h>
#include <hls_math.h>
#include "point.h"
#include "hls_half.h"

#define ROW 100
#define COL 100

const int FACTOR = 10;

struct Float_index
{
    float x0, x1;

    Float_index()
    {
    }

    Float_index(float x, float y) : x0(x), x1(y) {}
};

struct Index
{
    int x0, x1;

    Index()
    {
    }

    Index(int x, int y) : x0(x), x1(y) {}
};

class PlanarSDF
{
public:
    Point2 origin;
    // int field_rows, field_cols;
    float cell_size;

    ap_uint<1> map[ROW][COL];
    ap_uint<1> inv_map[ROW][COL];

    half sdf[ROW][COL];

    PlanarSDF()
    {
    }

    PlanarSDF(const ap_uint<1> cur_map[ROW][COL])
    {

        for (int i = 0; i < ROW; i++)
        {
            for (int j = 0; j < COL; j++)
            {
                map[i][j] = cur_map[i][j];
                inv_map[i][j] = 1 - cur_map[i][j];
            }
        }

        // std::cout << "*************" << std::endl;
        // std::cout << "inv_map" << std::endl;
        // for (int i = 0; i < ROW; i++)
        // {
        //     for (int j = 0; j < COL; j++)
        //     {
        //         std::cout << std::setw(6) << inv_map[i][j];
        //     }
        //     std::cout << std::endl;
        // }
        // std::cout << "*************" << std::endl;

        sdf_compute();
    }

    PlanarSDF(Point2 &o, float c_size, const ap_uint<1> cur_map[ROW][COL])
    {
        origin = o;
        // field_rows = f_row;
        // field_cols = f_col;
        cell_size = c_size;
        for (int i = 0; i < ROW; i++)
        {

            for (int j = 0; j < COL; j++)
            {

                map[i][j] = cur_map[i][j];
                inv_map[i][j] = 1 - cur_map[i][j];
            }
        }

        sdf_compute();
    }

    void sdf_compute()
    {

        bool all_open_area = true;

        for (int i = 0; i < ROW; i++)
        {

            for (int j = 0; j < COL; j++)
            {

                if (map[i][j] == 1)
                {
                    all_open_area = false;
                    break;
                }
            }
        }

        if (all_open_area)
        {
            for (int i = 0; i < ROW; i++)
            {

                for (int j = 0; j < COL; j++)
                {

                    sdf[i][j] = 1000.0;
                }
            }
        }
        else
        {
            ap_uint<17> G_map[ROW][COL];

            ap_uint<18> H_map[ROW][COL];

            ap_uint<17> G_map_inv[ROW][COL];

            ap_uint<18> H_map_inv[ROW][COL];

            for (int i = 0; i < ROW; i++)
            {
                for (int j = 0; j < COL; j++)
                {
                    ap_uint<9> forward_cnt[COL];
                    ap_uint<9> backward_cnt[COL];
                    ap_uint<9> inv_forward_cnt[COL];
                    ap_uint<9> inv_backward_cnt[COL];

                    forward_cnt[j] = 0;
                    inv_forward_cnt[j] = 0;
                    backward_cnt[COL - j - 1] = 0;
                    inv_backward_cnt[COL - j - 1] = 0;

                    // map forward pass
                    if (map[i][j] == 1)
                    {
                        for (int m = 0; m < COL; m++)
                        {
                            if (m < j)
                                continue;
                            if (map[i][m] == 0)
                            {
                                break;
                            }
                            forward_cnt[j]++;
                        }
                    }
                    // inv_map forward pass
                    if (inv_map[i][j] == 1)
                    {
                        for (int m = 0; m < COL; m++)
                        {

                            if (m < j)
                                continue;
                            if (inv_map[i][m] == 0)
                            {
                                break;
                            }
                            inv_forward_cnt[j]++;
                        }
                    }

                    // map backward pass
                    if (map[i][COL - j - 1] == 1)
                    {
                        for (int m = COL - 1; m >= 0; m--)
                        {

                            if (m > COL - j - 1)
                                continue;
                            if (map[i][m] == 0)
                            {
                                break;
                            }
                            backward_cnt[COL - j - 1]++;
                        }
                    }
                    // inv_map backward pass
                    if (inv_map[i][COL - j - 1] == 1)
                    {
                        for (int m = COL - 1; m >= 0; m--)
                        {

                            if (m > COL - j - 1)
                                continue;
                            if (inv_map[i][m] == 0)
                            {
                                break;
                            }
                            inv_backward_cnt[COL - j - 1]++;
                        }
                    }

                    if (j == COL - 1)
                    {
                        G_map[i][0] = forward_cnt[0] * forward_cnt[0];
                        G_map[i][COL - 1] = backward_cnt[COL - 1] * backward_cnt[COL - 1];
                        G_map_inv[i][0] = inv_forward_cnt[0] * inv_forward_cnt[0];
                        G_map_inv[i][COL - 1] = inv_backward_cnt[COL - 1] * inv_backward_cnt[COL - 1];

                        for (int k = 1; k < COL - 1; k++)
                        {

                            G_map[i][k] = forward_cnt[k] < backward_cnt[k] ? forward_cnt[k] * forward_cnt[k] : backward_cnt[k] * backward_cnt[k];
                            G_map_inv[i][k] = inv_forward_cnt[k] < inv_backward_cnt[k] ? inv_forward_cnt[k] * inv_forward_cnt[k] : inv_backward_cnt[k] * inv_backward_cnt[k];
                        }
                    }
                }
            }

            // for (int i = 0; i < ROW; i++)
            // {
            //     G_map[i][0] = forward_cnt[i][0] * forward_cnt[i][0];
            //     G_map[i][COL - 1] = backward_cnt[i][COL - 1] * backward_cnt[i][COL - 1];
            //     G_map_inv[i][0] = inv_forward_cnt[i][0] * inv_forward_cnt[i][0];
            //     G_map_inv[i][COL - 1] = inv_backward_cnt[i][COL - 1] * inv_backward_cnt[i][COL - 1];
            // }

            // for (int i = 0; i < ROW; i++)
            // {
            //     for (int j = 1; j < COL - 1; j++)
            //     {
            //         G_map[i][j] = forward_cnt[i][j] < backward_cnt[i][j] ? forward_cnt[i][j] * forward_cnt[i][j] : backward_cnt[i][j] * backward_cnt[i][j];
            //         G_map_inv[i][j] = inv_forward_cnt[i][j] < inv_backward_cnt[i][j] ? inv_forward_cnt[i][j] * inv_forward_cnt[i][j] : inv_backward_cnt[i][j] * inv_backward_cnt[i][j];
            //     }
            // }

            // std::cout << "*************" << std::endl;
            // std::cout << "G_map" << std::endl;
            // for (int i = 0; i < ROW; i++)
            // {
            //     for (int j = 0; j < COL; j++)
            //     {
            //         std::cout << std::setw(6) << G_map[i][j];
            //     }
            //     std::cout << std::endl;
            // }
            // std::cout << "*************" << std::endl;

            // std::cout << "*************" << std::endl;
            // std::cout << "G_map_inv" << std::endl;
            // for (int i = 0; i < ROW; i++)
            // {
            //     for (int j = 0; j < COL; j++)
            //     {
            //         std::cout << std::setw(6) << G_map_inv[i][j];
            //     }
            //     std::cout << std::endl;
            // }
            // std::cout << "*************" << std::endl;

            // compute H

            for (int j = 0; j < COL; j++)
            {
                for (int i = 0; i < ROW; i++)
                {

                    ap_uint<18> H_temp[ROW];
                    ap_uint<18> H_temp_inv[ROW];
                    ap_uint<18> min = 0;
                    ap_uint<18> min_inv = 0;

                    for (int x = 0; x < ROW; x++)
                    {

                        H_temp[x] = G_map[x][j] + (x - i) * (x - i);
                        H_temp_inv[x] = G_map_inv[x][j] + (x - i) * (x - i);

                        if (x == 0)
                        {
                            min = H_temp[0];
                            min_inv = H_temp_inv[0];
                        }
                        else
                        {
                            min = H_temp[x] < H_temp[x - 1] ? H_temp[x] : min;
                            min_inv = H_temp_inv[x] < H_temp_inv[x - 1] ? H_temp_inv[x] : min_inv;
                        }
                    }

                    H_map[i][j] = min;
                    H_map_inv[i][j] = min_inv;
                }
            }

            // std::cout << "*************" << std::endl;
            // std::cout << "H_map" << std::endl;
            // for (int i = 0; i < ROW; i++)
            // {
            //     for (int j = 0; j < COL; j++)
            //     {
            //         std::cout << std::setw(6) << H_map[i][j];
            //     }
            //     std::cout << std::endl;
            // }
            // std::cout << "*************" << std::endl;

            // std::cout << "*************" << std::endl;
            // std::cout << "H_map_inv" << std::endl;
            // for (int i = 0; i < ROW; i++)
            // {
            //     for (int j = 0; j < COL; j++)
            //     {
            //         std::cout << std::setw(6) << H_map_inv[i][j];
            //     }
            //     std::cout << std::endl;
            // }
            // std::cout << "*************" << std::endl;

            // compute sdf
            for (int i = 0; i < ROW; i++)
            {

                for (int j = 0; j < COL; j++)
                {
                    sdf[i][j] = half(hls::sqrt(float(H_map_inv[i][j])) - hls::sqrt(float(H_map[i][j])));
                }
            }

            // std::cout << "*************" << std::endl;
            // std::cout << "sdf" << std::endl;
            // for (int i = 0; i < ROW; i++)
            // {
            //     for (int j = 0; j < COL; j++)
            //     {
            //         std::cout << std::setw(10) << sdf[i][j];
            //     }
            //     std::cout << std::endl;
            // }
            // std::cout << "*************" << std::endl;
        }
    }

    float get_signed_distance(const Point2 &point, Point2 &g)
    {
        const Float_index pidx = convert_point2_to_cell(point);

        const Point2 g_idx = gradient(pidx);

        g.x = g_idx.x / cell_size;
        g.y = g_idx.y / cell_size;

        return signed_distance(pidx);
    }

    Float_index &convert_point2_to_cell(const Point2 &point)
    {
        const float col = (point.x - origin.x) / cell_size;
        const float row = (point.y - origin.y) / cell_size;

        Float_index float_index(row, col);

        return float_index;
    }

    float signed_distance(const Float_index &idx)
    {
        const int lr = int(idx.x0);
        const int lc = int(idx.x1);
        const int hr = lr + 1;
        const int hc = lc + 1;

        return (hr - idx.x0) * (hc - idx.x1) * sdf[lr][lc] +
               (idx.x0 - lr) * (hc - idx.x1) * sdf[hr][lc] +
               (hr - idx.x0) * (idx.x1 - lc) * sdf[lr][hc] +
               (idx.x0 - lr) * (idx.x1 - lc) * sdf[hr][hc];
    }

    Point2 &gradient(const Float_index &idx)
    {
        const int lr = int(idx.x0);
        const int lc = int(idx.x1);
        const int hr = lr + 1;
        const int hc = lc + 1;

        Point2 g;
        g.x = (hc - idx.x1) * (sdf[hr][lc] - sdf[lr][lc]) + (idx.x1 - lc) * (sdf[hr][hc] - sdf[lr][hc]);
        g.y = (hr - idx.x0) * (sdf[lr][hc] - sdf[lr][lc]) + (idx.x0 - lr) * (sdf[hr][hc] - sdf[hr][lc]);
        return g;
    }
};
