from acados_template import AcadosModel, AcadosOcp
import numpy as np
from casadi import SX, vertcat, sin, cos


def export_pendulum_ode_model() -> AcadosModel:
    model_name = 'pendulum_cart'

    # Constants
    gravity = 9.81  # gravity constant [m/s^2]
    length = 0.8  # length of the rod [m]

    # Parameters
    M = SX.sym('M')  # mass of the cart [kg] -> default = 1.
    m = SX.sym('m')  # mass of the ball [kg] -> default = 0.1

    # set up states & controls
    x1 = SX.sym('x1')
    v1 = SX.sym('v1')
    theta = SX.sym('theta')
    dtheta = SX.sym('dtheta')

    x = vertcat(x1, theta, v1, dtheta)
    p = vertcat(M, m)
    F = SX.sym('F')
    u = vertcat(F)

    # xdot
    x1_dot = SX.sym('x1_dot')
    theta_dot = SX.sym('theta_dot')
    v1_dot = SX.sym('v1_dot')
    dtheta_dot = SX.sym('dtheta_dot')

    xdot = vertcat(x1_dot, theta_dot, v1_dot, dtheta_dot)

    # dynamics
    cos_theta = cos(theta)
    sin_theta = sin(theta)
    denominator = M + m - m*cos_theta*cos_theta
    f_expl = vertcat(v1,
                     dtheta,
                     (-m*length*sin_theta*dtheta*dtheta + m *
                      gravity*cos_theta*sin_theta+F)/denominator,
                     (-m*length*cos_theta*sin_theta*dtheta*dtheta + F *
                      cos_theta+(M+m)*gravity*sin_theta)/(length*denominator)
                     )

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.p = p
    model.xdot = xdot
    model.u = u
    model.name = model_name

    return model


def export_pendulum_cart_acados_ocp():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
    # set model
    model = export_pendulum_ode_model()
    ocp.model = model
    Tf = 1.0
    N = 20
    # set dimensions
    ocp.dims.N = N
    # set parameters
    M_value = 1.0
    m_value = 0.1
    ocp.parameter_values = np.array([M_value, m_value])
    # set cost
    Q_mat = 2*np.diag([1e3, 1e3, 1e-2, 1e-2])
    R_mat = 2*np.diag([1e-2])
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = model.x.T @ Q_mat @ model.x + \
        model.u.T @ R_mat @ model.u
    ocp.model.cost_expr_ext_cost_e = model.x.T @ Q_mat @ model.x

    # set constraints
    x0 = np.array([0.0, np.pi, 0.0, 0.0])
    ocp.constraints.x0 = x0

    Fmax = 80
    ocp.constraints.lbu = np.array([-Fmax])
    ocp.constraints.ubu = np.array([+Fmax])
    ocp.constraints.idxbu = np.array([0])

    v_max = 10
    ocp.constraints.lbx = np.array([-v_max])  # p_dot max
    ocp.constraints.ubx = np.array([v_max])  # p_dot min
    ocp.constraints.idxbx = np.array([2])
    ocp.constraints.lbx_e = ocp.constraints.lbx
    ocp.constraints.ubx_e = ocp.constraints.ubx
    ocp.constraints.idxbx_e = ocp.constraints.idxbx

    # set options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'EXACT'
    ocp.solver_options.integrator_type = 'IRK'
    ocp.solver_options.nlp_solver_type = 'SQP'  # SQP_RTI, SQP
    ocp.solver_options.qp_solver_cond_N = N
    # set prediction horizon
    ocp.solver_options.tf = Tf

    return ocp
