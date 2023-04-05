import numpy as np 
from scipy.spatial.transform import Rotation as R

def rot_x(theta: float) -> np.ndarray:
     cT = np.cos(theta)
     sT = np.sin(theta)

     return np.array([[1.0, 0.0, 0.0],
                      [0.0,  cT, -sT],
                      [0.0,  sT,  cT]])

def rot_y(theta: float) -> np.ndarray:
     cT = np.cos(theta)
     sT = np.sin(theta)

     return np.array([[cT,  0.0,  sT],
                      [0.0, 1.0, 0.0],
                      [-sT, 0.0,  cT]])

def rot_z(theta: float) -> np.ndarray:
     cT = np.cos(theta)
     sT = np.sin(theta)

     return np.array([[ cT, -sT, 0.0],
                      [ sT,  cT, 0.0],
                      [0.0, 0.0, 1.0]])

# Robot Parameters
l = np.array([0.025, 0.025, 0.0578, 0.058, 0.045])
alignments = [rot_x(0) @ rot_y(0) @ rot_z(0),           # 0
              rot_x(0) @ rot_y(0) @ rot_z(np.pi / 2),   # 1
              rot_x(0) @ rot_y(0) @ rot_z(np.pi),       # 2
              rot_x(0) @ rot_y(0) @ rot_z(np.pi),       # 3
              rot_x(0) @ rot_y(0) @ rot_z(- np.pi / 2)  # 4
             ]
base = rot_x(np.pi) @ rot_y(0) @ rot_z(np.pi / 2)
offset = np.array([0.0, 0.4, 0.0])

def fk(x: np.ndarray, q: np.ndarray, xi: np.ndarray) -> np.ndarray:
     r = R.from_quat(q).as_matrix()
     return (x -  r @ (offset + base @ relative_fk(xi)))

def relative_fk(xi: np.ndarray) -> np.ndarray:
     bar = np.array([0,0,1])

     return (l[0] * bar 
            + l[1] * alignments[0] @ rot_x(xi[0]) @ bar
            + l[2] * alignments[0] @ rot_x(xi[0]) @ alignments[1] @ rot_x(xi[1]) @ bar
            + l[3] * alignments[0] @ rot_x(xi[0]) @ alignments[1] @ rot_x(xi[1]) @ alignments[2] @  rot_x(xi[2]) @ bar
            + l[4] * alignments[0] @ rot_x(xi[0]) @ alignments[1] @ rot_x(xi[1]) @ alignments[2] @  rot_x(xi[2]) @ alignments[3] @ rot_x(xi[3]) @ bar)