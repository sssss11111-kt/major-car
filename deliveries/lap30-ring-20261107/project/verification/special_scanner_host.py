"""Build the same scanner and route gate used by the RT1064 foreground loop."""
from pathlib import Path

verification = Path(__file__).resolve().parent
shared = verification.parents[3] / 'firmware/camera_car/demo/user'
vehicle = verification.parent / 'demo/user'


def scanner_args():
    modules = ('special_track', 'special_track_scan', 'special_route', 'cross_island', 'roundabout')
    return ['-I', str(shared / 'inc'), str(vehicle / 'src/roundabout_scan.c'), str(vehicle / 'src/cross_drive.c'),
            *(str(shared / 'src' / (name + '.c')) for name in modules)]
