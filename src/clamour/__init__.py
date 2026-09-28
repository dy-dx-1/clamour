__all__ = ["Clamour", "Pose", "ContextManagedQueue"]


def Clamour(*args, **kwargs):
    from .clamour import Clamour as _Clamour
    return _Clamour(*args, **kwargs)


def Pose(*args, **kwargs):
    from .interfaces.containers import Pose as _Pose
    return _Pose(*args, **kwargs)


def ContextManagedQueue(*args, **kwargs):
    from .contextManagedQueue import ContextManagedQueue as _ContextManagedQueue
    return _ContextManagedQueue(*args, **kwargs)