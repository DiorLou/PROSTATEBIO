import numpy as np
import math
from sympy import *
from scipy.optimize import least_squares

class RobotKinematics:
    """
    一个用于机器人运动学计算的类。
    """
    def __init__(self, a, alpha, d_list, theta_list, variable_names):
        """
        初始化机器人DH参数，并创建符号变量。

        Args:
            a (list or np.array): DH参数中的a。
            alpha (list or np.array): DH参数中的alpha（度）。
            d_list (list or np.array): DH参数中的d。
            theta_list (list or np.array): DH参数中的theta（度或包含变量名的字符串）。
            variable_names (list): 字符串列表，定义所有需要作为符号变量的名称（例如 ['x0', 'x1', 'x2', 'x3']）。
        """
        # 在类的内部创建符号变量
        self.variables = symbols(' '.join(variable_names))
        # 如果只有一个变量，symbols会返回一个Symbol对象，我们统一转为元组
        if not isinstance(self.variables, tuple):
            self.variables = (self.variables,)
        self.variable_map = {name: var for name, var in zip(variable_names, self.variables)}

        # 将包含变量名的列表转换为SymPy表达式
        # 我们可以用一个辅助函数来处理
        def process_list(lst):
            return [sympify(str(val), locals=self.variable_map) for val in lst]

        self.a = process_list(a)
        self.alpha = process_list(alpha)
        self.d = process_list(d_list)
        self.theta = process_list(theta_list)

        # 检查参数数量是否一致
        if not (len(self.a) == len(self.alpha) == len(self.d) == len(self.theta)):
            raise ValueError("All DH parameters (a, alpha, d, theta) must have the same length.")
        
        T04 = self._forward([0,0,0,0], 4)
        z4 = np.array(T04[:3, 2].tolist(), dtype=float).flatten()
        T03 = self._forward([0,0,0,0], 3)
        z3 = np.array(T03[:3, 2].tolist(), dtype=float).flatten()
        T02 = self._forward([0,0,0,0], 2)
        z2 = np.array(T02[:3, 2].tolist(), dtype=float).flatten()
        self.OA = z2
        self.OB = z3
        self.angle_BOA = np.pi / 6
        self.angle_AOC = np.pi / 12

        n_ODB = np.cross(z3, z4)
        n_ODB = n_ODB / np.linalg.norm(n_ODB)
        n_OBA = np.cross(z3, z2)
        n_OBA = n_OBA / np.linalg.norm(n_OBA)
        self.OD0 = self._rotate_vector_around_axis(self.OB, n_ODB, -self.angle_AOC)
        self.OC0 = self._solve_OC_analytically(self.OA, self.OD0)
        
    def _angle_of_rotation(self, src, dst, axis):
        src_perp = src - np.dot(src, axis) * axis
        src_perp = src_perp / np.linalg.norm(src_perp)
        dst_perp = dst - np.dot(dst, axis) * axis
        dst_perp = dst_perp / np.linalg.norm(dst_perp)
        return np.arccos(np.dot(src_perp, dst_perp)) * 180 / np.pi

    def _rotate_vector_around_axis(self, v, axis, theta):
        """
        使用罗德里格斯旋转公式将一个向量绕指定轴旋转给定的角度。

        参数:
        v (np.ndarray): 要旋转的3D向量。
        axis (np.ndarray): 旋转轴的3D向量。
        theta (float): 旋转角度（以弧度为单位）。

        返回:
        np.ndarray: 旋转后的3D向量。
        """
        # 确保输入是NumPy数组
        v = np.asarray(v)
        axis = np.asarray(axis)

        # 1. 将旋转轴向量归一化
        axis_norm = np.linalg.norm(axis)
        if axis_norm == 0:
            raise ValueError("旋转轴向量不能是零向量。")
        axis_vector = axis / axis_norm

        # 2. 罗德里格斯旋转公式的组成部分
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        # 3. 计算叉积矩阵 (Skew-symmetric matrix)
        #    注意Python是0-based索引
        K = np.array([
            [0, -axis_vector[2], axis_vector[1]],
            [axis_vector[2], 0, -axis_vector[0]],
            [-axis_vector[1], axis_vector[0], 0]
        ])

        # 4. 计算旋转矩阵 R
        #    在NumPy中, @ 运算符用于矩阵乘法
        R = np.identity(3) + sin_theta * K + (1 - cos_theta) * (K @ K)

        # 5. 应用旋转
        #    使用 @ 运算符进行矩阵向量乘法
        rotated_vector = R @ v
        
        return rotated_vector

    def _solve_OC_analytically(self, OA, OD):
        
        # --- 步骤 1: 求两个平面的交线 ---
        d = np.cross(OA, OD)
        
        if np.linalg.norm(d) < 1e-9:
            # 此处可以进一步检查平面是重合还是平行不相交
            # 但为简化，我们假设它们不平行
            return []

        # 寻找交线上的一个点 v0
        # 我们需要解方程组 M*v = b，其中 M=[OA; OD], b=[a1; a2]
        # 这是一个欠定方程组，np.linalg.lstsq 可以给出一个特解
        M = np.array([OA, OD])
        b = np.array([np.cos(self.angle_AOC), np.cos(self.angle_BOA)])
        v0, _, _, _ = np.linalg.lstsq(M, b, rcond=None)
        
        # --- 步骤 2: 构建关于 t 的二次方程 At^2 + Bt + C = 0 ---
        A = np.dot(d, d)
        B = 2 * np.dot(v0, d)
        C = np.dot(v0, v0) - 1
        
        # --- 步骤 3: 解二次方程 ---
        
        # 计算判别式
        discriminant = B**2 - 4*A*C
        
        solutions = []
        if discriminant < -1e-9: # 允许一些浮点误差
            print("判别式为负，方程组无实数解。")
        elif abs(discriminant) < 1e-9: # 判别式约等于0
            t = -B / (2*A)
            v = v0 + t * d
            solutions.append(v)
        else:
            sqrt_discriminant = np.sqrt(discriminant)
            t1 = (-B + sqrt_discriminant) / (2*A)
            t2 = (-B - sqrt_discriminant) / (2*A)
            
            v1 = v0 + t1 * d
            v2 = v0 + t2 * d
            solutions.extend([v1, v2])
        if hasattr(self, 'OC0'):
            for sol in solutions:
                sign0 = np.sign(np.dot(OA, np.cross(OD, self.OC0)))
                sign = np.sign(np.dot(OA, np.cross(OD, sol)))
                if sign == sign0:
                    self.OC = sol
                    break
        else:
            for sol in solutions:
                sign = np.sign(np.dot(OA, np.cross(OD, sol)))
                if sign == 1:
                    self.OC = sol
                    break
        return self.OC

    def _joint2_compensate(self, theta2):
        theta2  = theta2 * np.pi / 180
        OD_cur = self._rotate_vector_around_axis(self.OD0, self.OB, -theta2)
        self._solve_OC_analytically(self.OA, OD_cur)

        
    def _T(self, alpha_i, a_i, d_i, theta_i):
        """
        计算单个关节的齐次变换矩阵。
        """
        alpha_rad = alpha_i * pi / 180
        theta_rad = theta_i * pi / 180
        
        return Matrix([
            [cos(theta_rad),               -sin(theta_rad),              0,            a_i],
            [sin(theta_rad) * cos(alpha_rad), cos(theta_rad) * cos(alpha_rad), -sin(alpha_rad), -d_i * sin(alpha_rad)],
            [sin(theta_rad) * sin(alpha_rad), cos(theta_rad) * sin(alpha_rad), cos(alpha_rad),  d_i * cos(alpha_rad)],
            [0, 0, 0, 1]
        ])
    
    def get_tip_of_needle(self, joint_values):
        return self._forward(joint_values)[:3, 3]

    
    def get_rcm_point(self, joint_values):
        return self._forward(joint_values, 2)[:3, 3]
    
    def get_needle_vector(self, joint_values):
        return self._forward(joint_values)[:3, 2]
    
    def _forward(self, joint_values, n = None):
        Trans = self.get_T0n(n)
        res = Trans.evalf(subs={self.variables[0]: joint_values[0],
                        self.variables[1]: joint_values[1],
                        self.variables[2]: joint_values[2],
                        self.variables[3]: joint_values[3]})
        return res
    
    def get_T0n(self, n=None):
        """
        计算末端执行器相对于基座的齐次变换矩阵T0n。
        """
        if n is None:
            n = len(self.a)
        T0n = eye(4)
        for i in range(n):
            T0n = T0n @ self._T(self.alpha[i], self.a[i], self.d[i], self.theta[i])
        return T0n

    def get_joint1_value(self, target_z):
        """
        计算要穿刺rcm点z轴电机运动量
        """
        T02 = self.get_T0n(2)
        p_rcm_z = T02[2,3]
        equ = p_rcm_z - target_z
        solution = solve(equ, self.variables[0])
        return solution


    def get_joint23_value(self, target_vector, initial_guess = None):
        """
        使用最小二乘法求解逆运动学。
        
        Args:
            target_vector (list or np.array): 目标向量（例如，z轴的方向向量）。
            joint_vars (list): 待求解的关节变量符号（例如，[x1, x2]）。
            initial_guess (list or np.array): 求解器的初始猜测值。
        
        Returns:
            scipy.optimize.OptimizeResult: 最小二乘求解的结果对象。
        """
        # 获取符号表达式的 z 向量
        T0n = self.get_T0n()
        z_symbolic = T0n[:3, 2]
        
        # 构建残差表达式
        residuals_expr = z_symbolic - Matrix(target_vector)
        
        def fsolve_fun(variables):
            x1_val, x2_val = variables
            # Substitute numerical values into the symbolic expression and convert to float
            residual_vec = residuals_expr.evalf(subs={self.variables[1]: x1_val, self.variables[2]: x2_val})
            return [float(res) for res in residual_vec]

        # Use fsolve to find the roots
        # The 'fsolve' function is a numerical solver, so it requires initial guesses
        # and a function that returns a list or array of the errors.
        if initial_guess is None:
            initial_guess = [0, 0]
        result = least_squares(fsolve_fun, initial_guess)
        [joint1_val, joint2_val] = result.x
        self._joint2_compensate(joint2_val)
        joint2_val_compensated = self._angle_of_rotation(self.OC0, self.OC, self.OA)
        joint2_val_compensated2 = joint2_val_compensated + joint1_val
        return [joint1_val, joint2_val_compensated2]

if __name__ == '__main__':
    # 定义需要作为符号的变量名
    variable_names = ['x0', 'x1', 'x2', 'x3']
    
    a_params = [247.204, 0, 0, 0]
    alpha_params = [0, -65, -30, 34]
    # d_params和theta_params中现在使用字符串来表示变量
    d_params = ['x0 - 184.845', 545.517, 0, 'x3 + 60.7']
    theta_params = [30, 'x1', 'x2 + 85.96', 0]
    
    # 初始化类实例
    robot = RobotKinematics(a_params, alpha_params, d_params, theta_params, variable_names)
    
    rcm00 = robot.get_rcm_point([0,0,0,0])
    print(rcm00)
    needle_vector = robot.get_needle_vector([0,0,85,0])
    print(needle_vector)

    # 定义needle的向量
    calculated_joint23_value = robot.get_joint23_value(needle_vector)
    
    print(calculated_joint23_value)
    