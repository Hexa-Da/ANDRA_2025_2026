#!/usr/bin/env python3
"""Lance navigation autonome + sequence photo en parallele."""

import argparse
import os
import signal
import subprocess
import sys
import time


def _start(command):
    # Nouveau groupe de processus pour pouvoir arrêter proprement ros2 + enfant(s).
    return subprocess.Popen(command, start_new_session=True)


def _stop_all(processes):
    for process in processes:
        _stop_process(process)


def _stop_process(process):
    if process is None:
        return
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.time() + 5.0
    while process.poll() is None and time.time() < deadline:
        time.sleep(0.1)
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        deadline = time.time() + 2.0
        while process.poll() is None and time.time() < deadline:
            time.sleep(0.1)
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return


def _send_ptz_home():
    """Demande un retour PTZ Home via le topic preset."""
    try:
        subprocess.run(
            [
                'ros2',
                'topic',
                'pub',
                '--once',
                '/ptz/preset',
                'std_msgs/msg/Int32',
                '{data: -1}',
            ],
            check=False,
            timeout=3.0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def _stop_robot_motion():
    """Commande de sécurité pour arrêter le robot après interruption."""
    try:
        subprocess.run(
            [
                'ros2',
                'topic',
                'pub',
                '--once',
                '/cmd_vel',
                'geometry_msgs/msg/Twist',
                '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}',
            ],
            check=False,
            timeout=2.0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(
        description='Fusion navigation autonome + sequence photo'
    )
    parser.add_argument(
        '--waypoints-file',
        default='/home/techlab/Documents/ANDRA_2025-2026/ros2_ws/src/navigation_utils/trajectoire/ma_carte_traj.yaml',
        help='Fichier YAML de waypoints pour trajectoire_mission',
    )
    parser.add_argument(
        '--capture-interval',
        type=float,
        default=5.0,
        help='Intervalle d arret pour captures PTZ (secondes)',
    )
    parser.add_argument(
        '--loop',
        action='store_true',
        help='Active la boucle de mission pour trajectoire_mission',
    )
    args, unknown = parser.parse_known_args()

    mission_cmd = [
        'ros2', 'run', 'navigation_utils', 'trajectoire_mission',
    ]
    if args.waypoints_file:
        mission_cmd.extend(
            [
                '--ros-args',
                '-p', f'waypoints_file:={args.waypoints_file}',
                '-p', 'frame_id:=map',
                '-p', f'loop:={"true" if args.loop else "false"}',
            ]
        )

    photo_cmd = ['ros2', 'run', 'navigation_utils', 'sequence_photo']

    processes = []
    mission_process = None
    photo_process = None
    interrupted = False
    try:
        mission_process = _start(mission_cmd + unknown)
        photo_process = _start(photo_cmd)
        processes.append(mission_process)
        processes.append(photo_process)
        while True:
            mission_status = mission_process.poll() if mission_process else None
            photo_status = photo_process.poll() if photo_process else None

            # Dès que la mission se termine (succès ou erreur), stopper la séquence photo.
            if mission_status is not None and photo_process is not None and photo_status is None:
                _stop_process(photo_process)
                _send_ptz_home()
                break

            if mission_status is not None or photo_status is not None:
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        interrupted = True
        _stop_process(photo_process)
        _stop_process(mission_process)
    finally:
        _stop_all(processes)
        _send_ptz_home()
        _stop_robot_motion()

    if interrupted:
        return 130

    for status in [process.poll() for process in processes]:
        if status not in (0, None):
            return status
    return 0


if __name__ == '__main__':
    sys.exit(main())
