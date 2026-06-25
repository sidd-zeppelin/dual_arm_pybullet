import pybullet as p
import numpy as np

def draw_frame(
    T,
    scale=0.1,
    line_width=3
):

    origin = T[:3, 3]
    R = T[:3, :3]

    x_id = p.addUserDebugLine(
        origin,
        origin + scale * R[:, 0],
        [1, 0, 0],
        line_width
    )

    y_id = p.addUserDebugLine(
        origin,
        origin + scale * R[:, 1],
        [0, 1, 0],
        line_width
    )

    z_id = p.addUserDebugLine(
        origin,
        origin + scale * R[:, 2],
        [0, 0, 1],
        line_width
    )

    return [
        x_id,
        y_id,
        z_id
    ]
