import numpy as np
import math
from sympy import *
from scipy.optimize import least_squares

class RobotKinematics:
    """
    一个用于机器人运动学计算的类。
    """
    def __init__(self, a, alpha, d_list, theta_list, variable_names, angle_AOC):
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

        self.joint_vars_to_solve = [self.variables[1], self.variables[2]]
        T0n = self.get_T0n()
        z_symbolic = T0n[:3, 2]
        
        self.target_vector_sym = Matrix(symbols('tx, ty, tz'))
        residuals_expr = z_symbolic - self.target_vector_sym

        self._residuals_func = lambdify(
            self.joint_vars_to_solve + list(self.target_vector_sym), 
            residuals_expr.tolist(), # ravel() 将矩阵展平
            'numpy'
        )

        # 检查参数数量是否一致
        if not (len(self.a) == len(self.alpha) == len(self.d) == len(self.theta)):
            raise ValueError("All DH parameters (a, alpha, d, theta) must have the same length.")
        
        T04 = self._forward([0,0,0,0], 4, True)
        z4 = -np.array(T04[:3, 2].tolist(), dtype=float).flatten()
        T03 = self._forward([0,0,0,0], 3, True)
        z3 = -np.array(T03[:3, 2].tolist(), dtype=float).flatten()
        T02 = self._forward([0,0,0,0], 2, True)
        z2 = -np.array(T02[:3, 2].tolist(), dtype=float).flatten()
        self.OA = z2
        self.OB = z3
        self.OB00 = z3
        self.angle_BOA = np.acos(np.dot(z2,z3)/np.linalg.norm(z2)/ np.linalg.norm(z2))
        self.angle_AOC = angle_AOC 

        n_ODB = np.cross(z3, z4)
        n_ODB = n_ODB / np.linalg.norm(n_ODB)
        n_OBA = np.cross(z3, z2)
        n_OBA = n_OBA / np.linalg.norm(n_OBA)
        self.OD00 = self._rotate_vector_around_axis(self.OB, n_ODB, -self.angle_AOC)
        self.OD0 = self.OD00
        self.OD = self.OD00
        solutions = self._solve_included_side_analytically(self.OA, self.OD00)
        for sol in solutions:
            if(np.sign(np.dot(np.cross(self.OD00 - self.OA, self.OB - self.OD00), np.cross(sol - self.OA, self.OD00 - sol))) == 1):
                self.OC = sol
        self.OC00 = self.OC
        self.OC0 = self.OC
        
    def _angle_of_rotation(self, src, dst, axis):
        """
        计算 src 向量围绕 axis 旋转到 dst 向量的有符号角度。

        Args:
            src (np.ndarray): 源向量。
            dst (np.ndarray): 目标向量。
            axis (np.ndarray): 旋转轴，必须是单位向量。

        Returns:
            float: 有符号的角度（度数），正值表示沿 axis 方向的逆时针旋转。
        """
        src_perp = src - np.dot(src, axis) * axis
        dst_perp = dst - np.dot(dst, axis) * axis

        src_perp = src_perp / np.linalg.norm(src_perp)
        dst_perp = dst_perp / np.linalg.norm(dst_perp)

        dot_product = np.clip(np.dot(src_perp, dst_perp), -1.0, 1.0)
        angle_rad = np.arccos(dot_product)

        cross_product = np.cross(src_perp, dst_perp)
        
        direction = np.dot(cross_product, axis)
        signed_angle_rad = angle_rad * np.sign(direction)

        return signed_angle_rad * 180 / np.pi

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

        axis_norm = np.linalg.norm(axis)
        if axis_norm == 0:
            raise ValueError("旋转轴向量不能是零向量。")
        axis_vector = axis / axis_norm

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        K = np.array([
            [0, -axis_vector[2], axis_vector[1]],
            [axis_vector[2], 0, -axis_vector[0]],
            [-axis_vector[1], axis_vector[0], 0]
        ])

        R = np.identity(3) + sin_theta * K + (1 - cos_theta) * (K @ K)

        rotated_vector = R @ v
        
        return rotated_vector

    def _solve_included_side_analytically(self, side1, side2):
        
        # --- 步骤 1: 求两个平面的交线 ---
        d = np.cross(side1, side2)
        
        if np.linalg.norm(d) < 1e-9:
            # 此处可以进一步检查平面是重合还是平行不相交
            # 但为简化，我们假设它们不平行
            return []

        # 寻找交线上的一个点 v0
        # 我们需要解方程组 M*v = b，其中 M=[side1; side2], b=[a1; a2]
        # 这是一个欠定方程组，np.linalg.lstsq 可以给出一个特解
        M = np.array([side1, side2])
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
        return solutions
        
    def _joint3_compensate(self, theta3):
        """
        有BD绕B转动角度，求AC绕A转动角度
        """
        theta3  = theta3 * np.pi / 180
        OD_cur = self._rotate_vector_around_axis(self.OD0, self.OB, theta3)
        solutions = self._solve_included_side_analytically(self.OA, OD_cur)
        angle = []
        index = []
        for sol in solutions:
            sign = np.sign(np.dot(np.cross(self.OB,self.OA), (sol - self.OA)))
            a = self._angle_of_rotation(self.OC0, sol, self.OA)
            if sign == -1:
                a = abs((a + 180) % 360 - 180)
            index.append(np.dot(np.cross(OD_cur - self.OA, self.OB - OD_cur), np.cross(sol - self.OA, OD_cur - sol)))
            angle.append(a)
        self.OC = solutions[np.argmax(index)]
        return angle[np.argmax(index)]

    def _joint3_compensate_conversely(self, theta3):
        """
        有AC绕A转动角度，求BD绕B转动角度
        """
        theta3  = theta3 * np.pi / 180
        OC_cur = self._rotate_vector_around_axis(self.OC0, self.OA, theta3)
        solutions = self._solve_included_side_analytically(self.OB, OC_cur)
        angle = []
        index = []
        
        for sol in solutions:
            sign = np.sign(np.dot(np.cross(self.OB,self.OA), (sol - self.OB)))
            a = self._angle_of_rotation(self.OD0, sol, self.OB)
            if sign == -1:
                a = abs((a + 180) % 360 - 180)
            index.append(np.dot(np.cross(self.OB - OC_cur, self.OA - self.OB), np.cross(sol - OC_cur, self.OB - sol)))
            angle.append(a)
        self.OD = solutions[np.argmax(index)]
        return angle[np.argmax(index)]
        
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
        return np.array(self._forward(joint_values)[:3, 3]).flatten()
    
    def get_needle_vector(self, joint_values):
        return np.array(self._forward(joint_values)[:3, 2]).flatten()
    
    def get_rcm_point(self, joint_values):
        return np.array(self._forward(joint_values, 2)[:3, 3]).flatten()
    
    def _forward(self, joint_values, n = None, initializing = False):
        # joint_values[2] = joint_values[2] - joint_values[1]
        if(not initializing):
            self.OB = self._rotate_vector_around_axis(self.OB00, self.OA, joint_values[1])
            self.OD0 = self._rotate_vector_around_axis(self.OD00, self.OA, joint_values[1])
            self.OC0 = self._rotate_vector_around_axis(self.OC00, self.OA, joint_values[1])
            val2_compensated = self._joint3_compensate_conversely(joint_values[2])
            joint_values[2] = val2_compensated
        trans = self.get_T0n(n)
        res = trans.evalf(subs={self.variables[0]: joint_values[0],
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

    def get_joint1_value(self, rcm_z):
        """
        计算要穿刺rcm点z轴电机运动量
        """
        T02 = self.get_T0n(2)
        p_rcm_z = T02[2,3]
        equ = p_rcm_z - rcm_z
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

        def fsolve_fun(variables):
            joint2_val, joint3_val = variables
            # 调用预编译的函数，传入关节值和目标向量
            return np.squeeze(self._residuals_func(joint2_val, joint3_val, target_vector[0], target_vector[1], target_vector[2])).astype(np.float64) 

        if initial_guess is None:
            initial_guess = [0, 0]
        result = least_squares(fsolve_fun, initial_guess)
        [joint2_val, joint3_val] = result.x
        self.OB = self._rotate_vector_around_axis(self.OB00, self.OA, joint2_val)
        self.OD0 = self._rotate_vector_around_axis(self.OD00, self.OA, joint2_val)
        self.OC0 = self._rotate_vector_around_axis(self.OC00, self.OA, joint2_val)
        joint3_val_compensated = self._joint3_compensate(joint3_val)
        #joint3_val_compensated2 = joint3_val_compensated + joint2_val
        return np.array([joint2_val, joint3_val_compensated])

    def get_joint4_value(self, offset_from_rcm):
        """
        计算要针尖到目标点穿刺电机运动量
        """
        T04 = self._forward([0,0,0,0], 4)
        T03 = self._forward([0,0,0,0], 3)
        T43 = np.linalg.inv(np.matrix(T04, dtype=float)) @ np.matrix(T03, dtype=float)
        z_initial = T43[2,3]
        return offset_from_rcm - z_initial
    
def valid(num, error=1e-3):
    return abs(num) < error
    
if __name__ == '__main__':
    # 定义需要作为符号的变量名
    variable_names = ['x0', 'x1', 'x2', 'x3']
    a_params = [245.472, 0, 0, 0]
    alpha_params = [0, -65, -30, 34]
    # d_params和theta_params中现在使用字符串来表示变量  
    d_params = ['x0 - 174.830', 541.695, 0, 'x3 - 26.0']
    theta_params = [30, 'x1', 'x2 + 85.96', 0]
    
    # 初始化类实例
    robot = RobotKinematics(a_params, alpha_params, d_params, theta_params, variable_names, angle_AOC=np.pi/12)
    
    # rcm00 = robot.get_rcm_point([0,0,0,0])
    # # print(rcm00)
    # needle_tip = robot.get_tip_of_needle([0,0,0,0])
    # # print(needle_tip)
    # val4 = robot.get_joint4_value(0)
    # # print(val4)

            
    # # 正逆正验证
    print("请检查输出第一行与第三行是否相同，第二行是否与给定数值相同")
    needle_vector = robot.get_needle_vector([0,10,18,0])
    print(needle_vector)
    calculated_joint23_value = robot.get_joint23_value(needle_vector)
    print(calculated_joint23_value)
    calculated_joint23_value[1]=calculated_joint23_value[1]+calculated_joint23_value[0]
    print(calculated_joint23_value)
    #------计算正运动学时需要由真实电机旋转toDH关节旋转
    calculated_joint23_value[1] = calculated_joint23_value[1] - calculated_joint23_value[0]
    print(calculated_joint23_value)
    needle_vector_test = robot.get_needle_vector([0,calculated_joint23_value[0],calculated_joint23_value[1],0])
    print(needle_vector_test)
    
    # 逆正逆验证
    #print("\n请检查输出第二行与第四行是否相同，第一行与第三行是否相同")
    #target = [0.2,1,0]
    #target = target/np.linalg.norm(target)
    #print(target)
    #calculated_joint23_value = robot.get_joint23_value(target)
    #print(calculated_joint23_value)
    #needle_vector_test = robot.get_needle_vector([0,calculated_joint23_value[0],calculated_joint23_value[2],0])
    #print(needle_vector_test)
    #calculated_joint23_value = robot.get_joint23_value(needle_vector_test)
    #print(calculated_joint23_value)

    #needle_vector_sim = robot.get_needle_vector([0, 0, -45, 0])
    #print(needle_vector_sim)
    #calculated_joint23_value = robot.get_joint23_value(needle_vector_sim)
    #print(calculated_joint23_value)

    #rcm00 = robot.get_rcm_point([0,0,0,0])
    #print(rcm00)

    #d1 = robot.get_joint1_value(rcm00[2])
    #print(d1)
    #needle_tip = rcm00+1000*target
    #print(needle_tip)