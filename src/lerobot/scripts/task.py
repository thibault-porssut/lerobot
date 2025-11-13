# task.py — Pilotage Reachy2 + récupération de poses MuJoCo via WebSocket
import asyncio
import json
import time
import logging
import numpy as np
from reachy2_sdk import ReachySDK
from lerobot.scripts.scripted_control import ScriptedEventListener
from reachy2_sdk.utils.utils import get_pose_matrix
import websockets

# ===============================
# --- WebSocket Client Helper ---
# ===============================
class MujocoWSClient:
    def __init__(self, uri="ws://localhost:8765"):
        self.uri = uri
        self.ws = None
        self.loop = None

    async def _connect(self):
        """Ouvre la connexion WebSocket si elle n'existe pas ou est fermée."""
        if self.ws is None or not getattr(self.ws, "open", False):
            self.ws = await websockets.connect(self.uri)

    async def _send_request(self, payload):
        """Envoie une requête et attend une réponse JSON."""
        await self._connect()
        await self.ws.send(json.dumps(payload))
        msg = await self.ws.recv()
        return json.loads(msg)

    def _execute_async(self, coro, timeout):
        """
        Exécute un coroutine en mode bloquant.
        Si aucune boucle n’existe (par ex. dans un thread secondaire),
        crée une nouvelle event loop.
        """
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        return loop.run_until_complete(asyncio.wait_for(coro, timeout))

    # ---- Interface publique ----
    def connect(self, timeout=3.0):
        """Connexion persistante (à appeler une seule fois)."""
        return self._execute_async(self._connect(), timeout)

    def close(self, timeout=2.0):
        """Fermeture propre de la connexion."""
        if self.ws and not self.ws.closed:
            return self._execute_async(self.ws.close(), timeout)

    def get_pose_in_frame(self, frame: str, target: str, timeout=3.0):
        """Renvoie la pose (pos, rot) du body `target` dans le repère `frame`."""
        payload = {"type": "get_pose_in_frame", "frame": frame, "target": target}
        return self._execute_async(self._send_request(payload), timeout)

    def get_pose(self, target: str, timeout=3.0):
        """Renvoie la pose absolue (dans le monde MuJoCo)."""
        payload = {"type": "get_pose", "target": target}
        return self._execute_async(self._send_request(payload), timeout)

    def reset_env(self, timeout=3.0):
        """Réinitialise entièrement la simulation MuJoCo."""
        payload = {"type": "reset"}
        return self._execute_async(self._send_request(payload), timeout)
    
# ===============================
# --- Fonctions utilitaires ---
# ===============================
def wait_for_gripper(arm):
    while arm.gripper.is_moving():
        time.sleep(0.1)
    time.sleep(0.5)

def move_head(reachy: ReachySDK, target_world_pose: np.ndarray):
    reachy.head.look_at(float(target_world_pose[0]), float(target_world_pose[1]), float(target_world_pose[2]))

# ===============================
# --- Séquences de mouvement ---
# ===============================
def get_to_waiting_pose(reachy: ReachySDK, duration: float = 4.0) -> None:
    logging.info("[Task] Pose d’attente")
    reachy.r_arm.goto([30, 10, -15, -115, 0, 0, -15], duration)
    reachy.l_arm.goto([30, -10, 15, -115, 0, 0, 15], duration)
    reachy.r_arm.gripper.open()
    time.sleep(10)
    reachy.l_arm.gripper.open()
    wait_for_gripper(reachy.r_arm)
    wait_for_gripper(reachy.l_arm)
    time.sleep(duration)
    logging.info("[Task] Reachy en position d’attente")



# ===============================
# --- Script principal ---
# ===============================
def run_scripted_task(reachy: ReachySDK, event_listener: ScriptedEventListener, num_episodes_to_run: int, reset_time_s: float):
    try:
        mujoco_ws = MujocoWSClient("ws://localhost:8765")
        mujoco_ws.connect()
        reachy.turn_on()
        time.sleep(1.0)
        reachy.mobile_base.turn_off()
        mujoco_ws.reset_env()
        time.sleep(2)
        reachy.mobile_base.reset_odometry()
        reachy.mobile_base.turn_on()
        reachy.goto_posture('default', wait = True)
        reachy.mobile_base.reset_odometry()


        for episode in range(num_episodes_to_run):
            event_listener.wait_for_ready()
            

            logging.info(f"[Task] Début de l’épisode {episode}")
            reachy.r_arm.turn_on()
            time.sleep(4)
            get_to_waiting_pose(reachy)
            reachy.r_arm.translate_by(0.15,0.17,0, frame='robot',wait=True)
            reachy.r_arm.translate_by(0.09,0,-0.05, frame='robot',wait=True)
            reachy.r_arm.gripper.close()
            time.sleep(10)
            # wait_for_gripper(reachy.r_arm)
            reachy.r_arm.translate_by(0,-0.18,0.08, frame='robot',wait=True)
            reachy.mobile_base.goto(x=0.2, y=0.0, theta=0.0,wait=True)
            reachy.r_arm.gripper.open()
            time.sleep(4)
            # wait_for_gripper(reachy.r_arm)
            reachy.mobile_base.goto(x=-0.2, y=0.0, theta=0.0,wait=True)
            reachy.goto_posture()

            event_listener.stop_episode()
            logging.info(f"[Task] Épisode {episode + 1} terminé.")
            reachy.mobile_base.turn_off()
            mujoco_ws.reset_env()
            time.sleep(2)
            reachy.mobile_base.turn_on()
            # reachy.goto_posture('default', wait = True)
            event_listener.stop_episode()


        event_listener.stop_recording()
        logging.info("[Task] Tous les épisodes terminés.")
    except Exception as e:
        logging.error(f"[Task] Erreur: {e}")
        event_listener.stop_recording()
