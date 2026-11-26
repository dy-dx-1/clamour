"""
Serves the World to the tag processes over a multiprocessing manager, from a daemon thread of sim.py.

Protocol (the client lives in clamour's simulated interfaces, so clamour never imports from sim/):
- A BaseManager listening at the address returned by serve(), registering the typeid WORLD_TYPEID.
- Clients call EXPOSED methods only. They return plain types (numbers, tuples, sets, dicts, bytes, numpy arrays) or
  what a tag published itself, so clients don't need sim modules to unpickle results.
- Authentication uses the multiprocessing authkey, which tag processes spawned by sim.py (and their forked children)
  inherit automatically: clients only need the address.
- Each client connection gets its own server thread, which is why the World is thread-safe.
"""
import threading
from multiprocessing.managers import BaseManager

from world import World

WORLD_TYPEID = 'world'
EXPOSED = ('range', 'publish_pose', 'broadcast', 'listen', 'reachable_devices',
           'imu_spec', 'imu_samples', 'imu_attitude')

class _WorldManager(BaseManager):
    pass

def serve(world: World):
    """Start serving world from a daemon thread of this process. Returns the address clients connect to."""
    _WorldManager.register(WORLD_TYPEID, callable=lambda: world, exposed=EXPOSED)
    server = _WorldManager().get_server()
    threading.Thread(target=server.serve_forever, name='world-server', daemon=True).start()
    return server.address
