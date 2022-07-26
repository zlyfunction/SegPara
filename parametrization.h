#include <igl/arap.h>
#include <igl/boundary_loop.h>
#include <igl/harmonic.h>
#include <igl/map_vertices_to_circle.h>

void parametrization(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Eigen::MatrixXd &V_uv);