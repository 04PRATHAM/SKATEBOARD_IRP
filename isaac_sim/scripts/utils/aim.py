"""
Shared camera-aim helper (SKATEBOARD_IRP).

Background: hand-rolled euler math (e.g. RotateXYZOp (rot_y, 0, rot_z)) aims
USD cameras incorrectly — probes showed 85+ deg errors (sensors staring at
the floor, upside-down inspection views). USD cameras look along local -Z,
and RotateXYZOp composes as R = Rz*Ry*Rx, which makes intuitive eulers wrong.

The correct approach (same as NVIDIA's own ViewportCameraState.set_target_world):
    M_world = SetLookAt(eye, target, up).GetInverse()
then convert to the parent frame. This gives exact facing AND horizon-level
roll for any target, with no euler ambiguity.
"""


def compute_aim_matrix(eye, target, up=(0.0, 0.0, 1.0)):
    """Return world matrix aiming a -Z-forward camera at eye toward target."""
    from pxr import Gf
    eye_v = Gf.Vec3d(*eye)
    tgt_v = Gf.Vec3d(*target)
    fwd = (tgt_v - eye_v).GetNormalized()
    up_v = Gf.Vec3d(*up)
    if abs(fwd.GetDot(up_v)) > 0.999:
        # Looking straight up/down: fall back to Y as image-up reference.
        up_v = Gf.Vec3d(0.0, 1.0, 0.0)
    view = Gf.Matrix4d(1)
    view.SetLookAt(eye_v, tgt_v, up_v)
    return view.GetInverse()


def aim_prim(stage, prim_path: str, eye, target):
    """
    Point the prim at prim_path (Camera or plain Xform) from eye at target.

    Handles arbitrary parenting: converts the desired world matrix into the
    parent frame and writes it as a single Transform op.
    """
    from pxr import Usd, UsdGeom
    prim = stage.GetPrimAtPath(prim_path)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    parent = prim.GetParent()
    parent_world = None
    if parent and parent.IsValid():
        parent_world = UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
    m_world = compute_aim_matrix(eye, target)
    m_local = m_world if parent is None else m_world * parent_world.GetInverse()
    xform.AddTransformOp().Set(m_local)
    return m_local
